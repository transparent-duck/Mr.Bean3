using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed class HitColumnStep : HitTimedStep
{
    private static readonly Lazy<HitTimedRoutes> Routes = new(() => new(
        [(float)Water.WaterRoutes.ExitZ, -254.5f, -259.55f, -267.35f, -269.4f],
        Enumerable.Range(0, 45).Select(i => -11f + i * .5f).ToArray(),
        _ => true, HitTimedRoutes.Walk));
    protected override HitTimedRoutes Graph => Routes.Value;
    internal static HitTimedRoutes InspectionGraph => Routes.Value;
    internal static void Prepare() => _ = Routes.Value;
}

internal abstract class HitTimedStep : Step
{
    protected abstract HitTimedRoutes Graph { get; }
    protected virtual bool CanWait(PilotPose pose) => true;
    protected virtual Func<Vector2, Vector2, HitTimedRoutes.Point[]>? ActualMotion(PilotPose pose) => null;
    protected virtual Passage CreatePassage(Vector3[] points) => new PathPassage
        { Name = Name, Points = points, ArriveRadius = .12f };
    private HitTimedRoutes.Plan plan = null!;
    private Passage? passage;
    private int firstLeg, nextLeg;
    private double departure, seenHit, nextRetry;
    private long plannedRevision;
    private bool begun;
    private bool nativePaused;
    private double nativeReadyAt = double.NaN;
    private Vector2 nativeReadyFeet;

    public override void Enter(PilotContext c) { seenHit = c.LastHitAt; Select(c); }
    public override bool ObserveMotion(PilotContext c)
    {
        if (c.Pose.Locked)
        {
            // 8514 owns input through native tick 65. The lock can begin one
            // frame after steering hands off, so an already-built wet route
            // must be planned for its future release, not resumed late.
            if (c.Pose.Raw.SteeringClock is { CurrentTimelineId: 8514,
                    CurrentTimestamp: { } ticks, ActualSchedulerSpeed: { } rate } clock && rate > .001f)
            {
                double ready = c.Now + Math.Max(0, 65 - ticks) / (30 * rate);
                if (!nativePaused || !double.IsFinite(nativeReadyAt) || Math.Abs(ready - nativeReadyAt) > .12 ||
                    Vector2.DistanceSquared(c.Pose.XZ, nativeReadyFeet) > .01f)
                {
                    nativeReadyAt = ready; nativeReadyFeet = c.Pose.XZ;
                    Select(c, ready);
                    c.Log("hit-native-plan", new { step = Name, at = c.Now, ready, feet = c.Pose.Feet, clock });
                }
            }
            else nativeReadyAt = double.NaN;
            nativePaused = true;
        }
        else if (nativePaused)
        {
            nativePaused = false;
            bool changed = !double.IsFinite(nativeReadyAt) || Math.Abs(c.Now - nativeReadyAt) > .12 ||
                Vector2.DistanceSquared(c.Pose.XZ, nativeReadyFeet) > .0225f;
            if (!changed && plan.Found && plannedRevision != c.Hazards.RouteRevision)
                changed = Graph.HasConflict(c.Hazards, plan, firstLeg, c.Pose.XZ, c.Now, plannedRevision);
            c.Log("hit-native-release", new { step = Name, at = c.Now,
                predicted = double.IsFinite(nativeReadyAt) ? nativeReadyAt : (double?)null, changed, feet = c.Pose.Feet });
            seenHit = c.LastHitAt;
            if (changed) Select(c);
        }
        return false;
    }
    public override bool Tick(PilotContext c)
    {
        if (plan.Found && c.Pose.Grounded && plannedRevision != c.Hazards.RouteRevision)
        {
            long previousRevision = plannedRevision;
            plannedRevision = c.Hazards.RouteRevision;
            if (Graph.HasConflict(c.Hazards, plan, firstLeg, c.Pose.XZ, c.Now, previousRevision))
            {
                c.Log("course-new-danger", new { at = c.Now, step = Name, feet = c.Pose.Feet });
                Select(c);
            }
        }
        if ((!plan.Found && (c.Now >= nextRetry || plannedRevision != c.Hazards.RouteRevision)) ||
            seenHit != c.LastHitAt && c.Pose.Grounded)
        { seenHit = c.LastHitAt; Select(c); }
        if (!plan.Found) { c.Act.Hold(); return false; }
        if (plan.Legs.Length == 0) return true;
        if (!begun && c.Pose.Grounded && c.Now > departure + .12)
        {
            c.Log("course-retime", new { at = c.Now, step = Name, late = c.Now - departure, feet = c.Pose.Feet });
            Select(c);
            if (!plan.Found) { c.Act.Hold(); return false; }
            if (plan.Legs.Length == 0) return true;
        }
        if (c.Now < departure) { c.Act.Hold(); return false; }
        if (!begun) { passage!.Begin(c); begun = true; }
        if (!passage!.Follow(c)) return false;
        if (nextLeg >= plan.Legs.Length) return true;
        SetPassage(c);
        if (c.Now >= departure) { passage!.Begin(c); begun = true; passage!.Follow(c); }
        else c.Act.Hold();
        return false;
    }
    public override Vector2? Aim(PilotContext c) => passage?.Aim(c);

    private void Select(PilotContext c, double? readyAt = null)
    {
        c.Act.ClearJump();
        long started = Stopwatch.GetTimestamp();
        double startAt = readyAt ?? c.Now;
        plan = Graph.Search(c.Hazards, c.Pose.XZ, startAt, CanWait(c.Pose), ActualMotion(c.Pose));
        nextRetry = c.Now + .25; plannedRevision = c.Hazards.RouteRevision;
        passage = null; firstLeg = nextLeg = 0;
        if (plan.Found && plan.Legs.Length > 0) SetPassage(c);
        c.Log("hit-ctc-plan", new { step = Name, at = c.Now, startAt, start = c.Pose.Feet,
            plan.Arrival, plan.Expanded, plan.Found, planningMs = Stopwatch.GetElapsedTime(started).TotalMilliseconds,
            nodes = Graph.Nodes.Count, edges = Graph.Outgoing.Sum(e => e.Count),
            route = plan.Legs.Select(l => new { l.Departure, duration = l.Edge.Duration,
                points = l.Edge.Path.Select(p => new { x = p.Position.X, z = p.Position.Y, t = p.Time }).ToArray() }).ToArray() });
    }

    private void SetPassage(PilotContext c)
    {
        begun = false;
        firstLeg = nextLeg;
        departure = plan.Legs[nextLeg].Departure;
        var points = new List<Vector3>();
        while (nextLeg < plan.Legs.Length)
        {
            var leg = plan.Legs[nextLeg];
            foreach (var point in leg.Edge.Path.Skip(1))
                points.Add(new(point.Position.X, HitGround.Loaded.Value.At(point.Position) ?? c.Pose.Feet.Y, point.Position.Y));
            ++nextLeg;
            if (nextLeg < plan.Legs.Length && plan.Legs[nextLeg].Departure > leg.Departure + leg.Edge.Duration + 1e-8) break;
        }
        passage = CreatePassage(points.ToArray());
    }
}
