using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot.Courses;

// The forced-floor exit must remain upstream of the first water pulse.
// Ordinary geometry ends there; the water planner owns the timed continuation.
internal sealed class HitOpeningStep : Step
{
    private static readonly Lazy<PilotCourseGeometry> Geometry = new(() => PilotCourseGeometry.Load("hit-opening"));
    internal static PilotCourseGeometry InspectionGeometry => Geometry.Value;
    internal static void Prepare() => _ = Geometry.Value.Graph(6);
    private HitTimedRoutes.Plan? plan;
    private PathPassage? passage;
    private Vector3[] points = [];
    private double seenHit, nextRetry, progressAt, nextPositionCheck;
    private Vector2 progressPoint;
    private bool forced, waitingForForce;
    private double forceDeadline;
    internal const float ForcedExitZ = -194.5f;
    internal const float ForcedSeconds = 3;
    // Reserve one ordinary 50 ms movement interval before the safe exit.
    internal static bool NeedsForcedWait(float z) => z - Actuator.WalkSpeed * (ForcedSeconds + .05f) <= ForcedExitZ;

    public override bool ObserveMotion(PilotContext c)
    {
        if (c.Pose.Has(3737))
        {
            if (!forced)
            {
                var status = c.Pose.Raw.Statuses.First(s => s.Id == 3737);
                c.Log("hit-forced-start", new { at = c.Now, start = c.Pose.Feet,
                    remaining = status.Remaining, predictedZ = c.Pose.Feet.Z - Actuator.WalkSpeed * status.Remaining });
            }
            forced = true; waitingForForce = false;
        }
        else if (forced)
        {
            forced = false;
            c.Log("hit-forced-end", new { at = c.Now, feet = c.Pose.Feet });
            Select(c);
        }
        return false;
    }

    public override void Enter(PilotContext c) => Select(c);
    // The engine does not tick passages while 3737 owns movement. Its heading
    // must continue north, rather than turn back toward a passed graph vertex.
    public override Vector2? Aim(PilotContext c) => waitingForForce || c.Pose.Has(3737) ? new Vector2(0, -1) : passage?.Aim(c);

    public override bool Tick(PilotContext c)
    {
        if (c.Pose.Grounded)
        {
            foreach (var status in c.Pose.Raw.Statuses)
                if (status.Id is 3699 or 3700 or 3701 && NeedsForcedWait(c.Pose.Feet.Z))
                {
                    if (!waitingForForce)
                    {
                        waitingForForce = true;
                        forceDeadline = c.Now + Math.Max(0, status.Remaining) + .5;
                        c.Log("hit-forced-wait", new { at = c.Now, feet = c.Pose.Feet,
                            warning = status.Id, remaining = status.Remaining, predictedZ = c.Pose.Feet.Z - Actuator.WalkSpeed * ForcedSeconds });
                    }
                    break;
                }
            // Keep the hold across the warning-loss/gain packet boundary.
            if (waitingForForce && c.Now < forceDeadline)
            { c.Act.Hold(); progressAt = c.Now; return false; }
            waitingForForce = false;
        }
        if (c.Pose.Grounded && c.Pose.Feet.Z <= -194.8f) return true;
        bool displaced = false;
        if (c.Pose.Grounded && passage != null && c.Now >= nextPositionCheck)
        {
            nextPositionCheck = c.Now + .25;
            float nearest = float.PositiveInfinity;
            for (int i = 1; i < points.Length; ++i)
            {
                var a = points[i - 1].XZ(); var delta = points[i].XZ() - a;
                float t = delta.LengthSquared() == 0 ? 0 : Math.Clamp(Vector2.Dot(c.Pose.XZ - a, delta) / delta.LengthSquared(), 0, 1);
                nearest = Math.Min(nearest, Vector2.DistanceSquared(c.Pose.XZ, a + t * delta));
            }
            displaced = nearest > 1 || c.Now - progressAt > 2.5;
        }
        if (c.Pose.Grounded && c.Now >= nextRetry &&
            (plan is not { Found: true } || seenHit != c.LastHitAt || displaced)) Select(c);
        if (Vector2.DistanceSquared(progressPoint, c.Pose.XZ) > .01f)
        { progressPoint = c.Pose.XZ; progressAt = c.Now; }
        if (passage == null) { c.Act.Hold(); return false; }
        return passage.Follow(c);
    }

    private void Select(PilotContext c)
    {
        var watch = Stopwatch.GetTimestamp();
        var data = Geometry.Value;
        plan = data.Graph(6).Search(c.Hazards, c.Pose.XZ, c.Now, true,
            sourceEdges: data.Connectors(c.Pose, 6));
        seenHit = c.LastHitAt; nextRetry = c.Now + .25;
        progressPoint = c.Pose.XZ; progressAt = c.Now;
        var path = new List<Vector3>(); var jumps = new List<int>();
        foreach (var leg in plan.Legs)
            foreach (var move in (PilotCourseGeometry.Move[])leg.Edge.Motion!)
            {
                if (path.Count == 0 || path[^1] != move.Path[0]) path.Add(move.Path[0]);
                if (move.Kind == "jump") jumps.Add(path.Count - 1);
                path.AddRange(move.Path.Skip(1));
            }
        points = path.ToArray();
        passage = plan.Found && points.Length > 0 ? new PathPassage
            { Name = Name, Points = points, JumpFrom = jumps.ToArray(), ArriveRadius = .08f } : null;
        passage?.Begin(c);
        c.Log("hit-opening-plan", new { at = c.Now, start = c.Pose.Feet, plan.Found, plan.Arrival, plan.Expanded,
            planningMs = Stopwatch.GetElapsedTime(watch).TotalMilliseconds, points, jumps,
            route = plan.Legs.Select(l => new { l.Edge.From, l.Edge.To }).ToArray() });
    }
}
