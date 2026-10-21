using System;
using System.Collections.Generic;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot;

/// <summary>
/// A timed motion that starts from a waiting spot. Simulate and Follow share the same motion
/// law, so the planner's prediction is what the executor does. Every passage must be
/// executable from wherever the actor currently is (Follow never fails; it only steers).
/// </summary>
internal abstract class Passage
{
    public const double Dt = 0.05;
    public required string Name { get; init; }
    /// <summary>Positions at t = 0, Dt, 2Dt, ... relative to departure from start.</summary>
    public abstract void Simulate(Vector2 start, List<Vector2> output);
    public virtual void Begin(PilotContext c) { }
    /// <summary>Executes one frame; returns true when the passage has reached its end.</summary>
    public abstract bool Follow(PilotContext c);
    /// <summary>Heading native forced movement should take while this passage runs.</summary>
    public virtual Vector2? Aim(PilotContext c) => null;
}

/// <summary>
/// Ordinary running through points; Force(p) gives the external push at p (wet slopes).
/// Indices in JumpFrom are takeoff points: the actor jumps from there toward the next point
/// (jump horizontal speed equals running speed, so the simulation treats it as running).
/// </summary>
internal sealed class PathPassage : Passage
{
    public required Vector3[] Points { get; init; }
    public Func<Vector2, Vector2>? Force { get; init; }
    public int[] JumpFrom { get; init; } = [];
    public float ArriveRadius { get; init; } = 0.35f;
    public bool SkipStart { get; init; }
    public Courses.HitGround? Ground { get; init; }
    private int index;
    private Vector3? join;
    private bool launched, airborne;
    private double launchedAt;
    internal SharedRouteProgress Progress => new() { Index = index, Join = join };

    public override void Simulate(Vector2 start, List<Vector2> output)
    {
        var p = start; output.Add(p);
        foreach (var point in Points)
        {
            var target = point.XZ();
            for (int guard = 0; guard < 4000 && Vector2.Distance(p, target) > 0.05f; ++guard)
            {
                var dir = Geo.Toward(p, target);
                var force = Force?.Invoke(p) ?? Vector2.Zero;
                Geo.CompensateForce(dir, force, Actuator.WalkSpeed, out float speed);
                float step = MathF.Min(MathF.Max(speed, 0.5f) * (float)Dt, Vector2.Distance(p, target));
                p += dir * step; output.Add(p);
            }
        }
    }

    public override void Begin(PilotContext c)
    {
        var cursor = SkipStart ? new SharedRouteProgress { Index = 1 } :
            SharedRouteProgress.Begin(Points, c.Pose.Feet, Array.IndexOf(JumpFrom, 0) >= 0, false);
        if (Ground != null && !SkipStart && JumpFrom.Length == 0 &&
            c.Pose.Motion == MrBean.FallGuys.MotionKind.Ordinary && c.Pose.Force == Vector2.Zero)
        {
            float frame = Math.Clamp((float)(c.Pose.Raw.RealFrameSeconds ?? 1d / 60), 1f / 240, .1f);
            var checkedCursor = WalkHandoffCheck.Choose(cursor, Points, c.Pose.Feet, Ground,
                frame, c.Pose.Raw.Walk?.AvailableTargetSpeed ?? 6, ArriveRadius);
            if (checkedCursor.Index != cursor.Index)
                c.Log("course-handoff-approach", new { at = c.Now, passage = Name, feet = c.Pose.Feet, origin = Points[0] });
            cursor = checkedCursor;
        }
        index = cursor.Index; join = cursor.Join;
        launched = airborne = false; c.Act.ClearJump();
    }

    public override bool Follow(PilotContext c)
    {
        var pose = c.Pose; var p = pose.XZ;
        if (JumpFrom.Length == 0)
        {
            var cursor = Progress;
            bool moving = SharedRouteProgress.WalkTarget(Points, p, ref cursor, ArriveRadius, out var direction);
            index = cursor.Index; join = cursor.Join;
            if (!moving) return true;
            if (pose.Force != Vector2.Zero) c.Act.WalkNet(direction, pose.Force); else c.Act.Walk(direction);
            return false;
        }
        if (join is { } targetJoin)
        {
            if (Vector2.DistanceSquared(p, targetJoin.XZ()) > .12f * .12f)
            {
                var direction = Geo.Toward(p, targetJoin.XZ());
                if (pose.Force != Vector2.Zero) c.Act.WalkNet(direction, pose.Force); else c.Act.Walk(direction);
                return false;
            }
            join = null;
        }
        if (launched)
        {
            var dir = JumpDirection(index - 1);
            if (!pose.Grounded) { airborne = true; c.Act.Walk(dir); return false; }
            if (!airborne && c.Now - launchedAt < 0.5) { c.Act.Walk(dir); return false; }
            if (!airborne) --index; // never left the ground: approach the takeoff again
            launched = airborne = false;
            c.Act.ClearJump();
        }
        while (index < Points.Length)
        {
            var target = Points[index].XZ();
            bool takeoff = Array.IndexOf(JumpFrom, index) >= 0 && index + 1 < Points.Length;
            float distance = Vector2.Distance(p, target);
            bool passed = index > 0 && PassedSegmentEnd(Points[index - 1].XZ(), target, p);
            if (takeoff)
            {
                if (pose.Grounded && (distance <= 0.25f || passed && distance < 0.6f))
                {
                    if (distance > 0.05f) c.Act.Nudge(Points[index] with { Y = pose.Feet.Y }, c.Now, 0);
                    c.Act.Jump(JumpDirection(index), c.Now);
                    launched = true; airborne = false; launchedAt = c.Now; ++index;
                    c.Log("jump", new { passage = Name, at = c.Now, feet = pose.Feet, distance });
                    return false;
                }
                break;
            }
            if (distance <= ArriveRadius || passed) { ++index; continue; }
            break;
        }
        if (index >= Points.Length) return true;
        var heading = Geo.Toward(p, Points[index].XZ());
        if (pose.Force != Vector2.Zero) c.Act.WalkNet(heading, pose.Force); else c.Act.Walk(heading);
        return false;
    }

    private Vector2 JumpDirection(int takeoff) => Geo.Toward(Points[takeoff].XZ(), Points[takeoff + 1].XZ());

    private static bool PassedSegmentEnd(Vector2 a, Vector2 b, Vector2 p)
    {
        var ab = b - a;
        return ab.LengthSquared() > 0 && Vector2.Dot(p - a, ab) >= ab.LengthSquared();
    }

    public override Vector2? Aim(PilotContext c)
        => join is { } target ? Geo.Toward(c.Pose.XZ, target.XZ()) :
            index < Points.Length ? Geo.Toward(c.Pose.XZ, Points[index].XZ()) : null;
}

/// <summary>
/// Native limited-turn steering (Hit 3702): constant speed, heading turns toward the desired
/// heading at TurnRate. Before the status starts the actor runs at walking speed to the lane
/// start and then north into the steering floor.
/// </summary>
internal sealed class SteeringPassage : Passage
{
    public const float Speed = 3.6f, TurnRate = 0.7f, Lookahead = 5f;
    public required float LaneX { get; set; }
    public required float StartZ { get; init; }   // ordinary waiting line
    public required float EntryZ { get; init; }   // Z where native steering starts
    public required float EndZ { get; init; }     // Z where native steering ends
    /// <summary>Lanes the executor may switch to mid-pass when the current lane is predicted to be hit.</summary>
    public float[] AlternativeLanes { get; init; } = [];
    private bool started;
    private double nextReplan;
    private static readonly List<Vector2> replanSamples = [];

    public static float DesiredHeading(Vector2 p, float laneX)
        => Geo.Heading(Geo.Toward(p, new Vector2(laneX, p.Y - Lookahead)));

    public override void Simulate(Vector2 start, List<Vector2> output)
    {
        var p = start; output.Add(p);
        var laneStart = new Vector2(LaneX, StartZ);
        while (Vector2.Distance(p, laneStart) > 0.05f && output.Count < 2000)
        { p += Geo.Toward(p, laneStart) * MathF.Min(Actuator.WalkSpeed * (float)Dt, Vector2.Distance(p, laneStart)); output.Add(p); }
        while (p.Y > EntryZ && output.Count < 2000) { p.Y -= Actuator.WalkSpeed * (float)Dt; output.Add(p); }
        SimulateSteering(p, MathF.PI, LaneX, EndZ, output);
    }

    public static void SimulateSteering(Vector2 p, float heading, float laneX, float endZ, List<Vector2> output)
    {
        for (int guard = 0; guard < 2000 && p.Y > endZ; ++guard)
        {
            float desired = DesiredHeading(p, laneX);
            float delta = Math.Clamp(Geo.WrapAngle(desired - heading), -TurnRate * (float)Dt, TurnRate * (float)Dt);
            heading += delta;
            p += Geo.FromHeading(heading) * Speed * (float)Dt;
            output.Add(p);
        }
    }

    public override void Begin(PilotContext c) => started = false;

    public override bool Follow(PilotContext c)
    {
        var pose = c.Pose; var p = pose.XZ;
        if (pose.Motion == MrBean.FallGuys.MotionKind.Steering)
        {
            started = true;
            if (c.Now >= nextReplan && AlternativeLanes.Length > 0 && pose.Raw.ArrowDirection is { } heading)
            {
                nextReplan = c.Now + 0.5;
                ReplanLane(c, p, heading);
            }
            c.Act.Steer(pose.Feet, Geo.FromHeading(DesiredHeading(p, LaneX)));
            return false;
        }
        // Steering ended (normal release, or knocked off): the recovery lock is handled by the
        // engine and relocalization selects the next step from the actual position.
        if (started) return true;
        var laneStart = new Vector2(LaneX, StartZ);
        if (p.Y > StartZ + 0.5f || Math.Abs(p.X - LaneX) > 0.25f)
            c.Act.Walk(Geo.Toward(p, laneStart));
        else c.Act.Walk(new Vector2(0, -1));
        return false;
    }

    private void ReplanLane(PilotContext c, Vector2 p, float heading)
    {
        (int Hits, int Unknowns) Score(float lane)
        {
            replanSamples.Clear();
            SimulateSteering(p, heading, lane, EndZ, replanSamples);
            return Planner.Evaluate(c.Hazards, replanSamples, c.Now, PilotEngine.BodyRadius);
        }
        var current = Score(LaneX);
        if (current.Hits == 0) return;
        float bestLane = LaneX; var best = current;
        foreach (float lane in AlternativeLanes)
        {
            var s = Score(lane);
            if (s.Hits < best.Hits || s.Hits == best.Hits && s.Unknowns < best.Unknowns) { best = s; bestLane = lane; }
        }
        if (bestLane != LaneX) c.Log("steering-replan", new { from = LaneX, to = bestLane, current.Hits, bestHits = best.Hits });
        LaneX = bestLane;
    }
}

internal sealed record PlanChoice(Passage Passage, double Delay, int Hits, int Unknowns)
{
    public bool Clean => Hits == 0 && Unknowns == 0;
}

/// <summary>Chooses passage and departure delay against the live hazard field.</summary>
internal static class Planner
{
    private sealed record Exposed(HazardField.Station Station, double From, double To);
    private static readonly List<Vector2> samples = [];

    public static PlanChoice? Choose(HazardField field, IReadOnlyList<Passage> passages, Vector2 start,
        double now, double maxDelay, float radius, double delayStep = 0.1)
    {
        if (passages.Count == 0) return null;
        var exposures = new List<Exposed>[passages.Count];
        double longest = 0;
        for (int i = 0; i < passages.Count; ++i)
        {
            samples.Clear();
            passages[i].Simulate(start, samples);
            longest = Math.Max(longest, samples.Count * Passage.Dt);
            exposures[i] = Expose(field, samples, radius);
        }
        field.Prepare(now, maxDelay + longest + 1);
        var waiting = new List<HazardField.Station>();
        foreach (var s in field.Stations) if (s.Contains(start, s.Spec.Pad)) waiting.Add(s);

        for (double delay = 0; delay <= maxDelay + 1e-9; delay += delayStep)
        {
            int waitHits = 0, waitUnknown = 0;
            foreach (var s in waiting) Count(HazardField.Overlap(s, now, now + delay), ref waitHits, ref waitUnknown);
            for (int i = 0; i < passages.Count; ++i)
            {
                int hits = waitHits, unknown = waitUnknown;
                foreach (var e in exposures[i])
                    Count(HazardField.Overlap(e.Station, now + delay + e.From, now + delay + e.To), ref hits, ref unknown);
                var choice = new PlanChoice(passages[i], delay, hits, unknown);
                if (choice.Clean) return choice;
            }
        }
        return null;

        static void Count(Exposure e, ref int hits, ref int unknown)
        {
            if (e == Exposure.Hit) ++hits; else if (e == Exposure.Unknown) ++unknown;
        }
    }

    /// <summary>Hits/unknowns of a path that departs now (uses the field's current preparation horizon).</summary>
    public static (int Hits, int Unknowns) Evaluate(HazardField field, List<Vector2> path, double now, float radius)
    {
        field.Prepare(now, path.Count * Passage.Dt + 1);
        int hits = 0, unknown = 0;
        foreach (var e in Expose(field, path, radius))
        {
            var r = HazardField.Overlap(e.Station, now + e.From, now + e.To);
            if (r == Exposure.Hit) ++hits; else if (r == Exposure.Unknown) ++unknown;
        }
        return (hits, unknown);
    }

    private static List<Exposed> Expose(HazardField field, List<Vector2> path, float radius)
    {
        var result = new List<Exposed>();
        var min = new Vector2(float.MaxValue); var max = new Vector2(float.MinValue);
        foreach (var p in path) { min = Vector2.Min(min, p); max = Vector2.Max(max, p); }
        foreach (var station in field.Near(min, max, radius))
        {
            int first = -1;
            for (int i = 0; i <= path.Count; ++i)
            {
                bool inside = i < path.Count && station.Contains(path[i], station.Spec.Pad);
                if (inside && first < 0) first = i;
                else if (!inside && first >= 0)
                {
                    result.Add(new(station, (first - 0.5) * Passage.Dt, (i - 0.5) * Passage.Dt));
                    first = -1;
                }
            }
        }
        return result;
    }
}

/// <summary>Closed-loop point pursuit. Never fails: knockback or drift just changes the next heading.</summary>
internal sealed class PointFollower(Vector3[] points, float radius)
{
    public int Index { get; private set; }
    public Vector3? Current => Index < points.Length ? points[Index] : null;

    public bool Tick(PilotContext c)
    {
        var p = c.Pose.XZ;
        while (Index < points.Length)
        {
            var target = points[Index].XZ();
            if (Vector2.Distance(p, target) <= radius) { ++Index; continue; }
            if (Index > 0)
            {
                var a = points[Index - 1].XZ(); var ab = target - a;
                if (ab.LengthSquared() > 0 && Vector2.Dot(p - a, ab) >= ab.LengthSquared()) { ++Index; continue; }
            }
            break;
        }
        if (Index >= points.Length) return true;
        var dir = Geo.Toward(p, points[Index].XZ());
        if (c.Pose.Force != Vector2.Zero) c.Act.WalkNet(dir, c.Pose.Force);
        else c.Act.Walk(dir);
        return false;
    }

    public Vector2? Aim(PilotContext c) => Current is { } t ? Geo.Toward(c.Pose.XZ, t.XZ()) : null;
}
