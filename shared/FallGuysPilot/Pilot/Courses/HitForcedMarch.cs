using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed class HitForcedMarch
{
    internal const double Duration = 3;
    internal const float TurnRate = 2;
    internal ushort Warning { get; private set; }
    internal bool Native { get; private set; }
    internal bool Pending => Warning != 0 || Native;
    internal double StartsAt { get; private set; }
    internal double ReadyAt { get; private set; }
    internal Vector2 End { get; private set; }
    internal double Margin { get; private set; } = .3;
    private double lastAt = double.NaN;
    private bool grounded;

    internal static float Offset(ushort warning) => warning switch
    {
        3699 => MathF.PI,
        3700 => MathF.PI / 2,
        3701 => -MathF.PI / 2,
        _ => 0
    };

    internal bool Observe(PilotContext c)
    {
        double frame = c.Pose.Raw.RealFrameSeconds ?? (double.IsFinite(lastAt) ? c.Now - lastAt : 0);
        lastAt = c.Now;
        double margin = Math.Max(.3, 2 * Math.Clamp(frame, 0, 1));
        bool boundary = c.Pose.Grounded && !grounded || margin > Margin + .05;
        grounded = c.Pose.Grounded;
        Margin = margin;
        foreach (var status in c.Pose.Raw.Statuses)
            if (status.Id == 3737)
            {
                double ready = c.Now + Math.Max(0, status.Remaining);
                float offset = status.Param switch { 2 => MathF.PI, 4 => MathF.PI / 2, 8 => -MathF.PI / 2, _ => Offset(Warning) };
                float actual = c.Pose.Raw.FixedDirection ?? Geo.WrapAngle(c.Pose.Facing + offset);
                var end = c.Pose.XZ + Geo.FromHeading(actual) * (Actuator.WalkSpeed * (float)Math.Max(0, status.Remaining));
                bool changed = !Native || Math.Abs(ready - ReadyAt) > .12 || Vector2.DistanceSquared(end, End) > .0225f;
                Native = true;
                if (changed) { ReadyAt = ready; End = end; }
                return changed;
            }
        if (Native) { Native = false; Warning = 0; return true; }
        foreach (var status in c.Pose.Raw.Statuses)
            if (status.Id is 3699 or 3700 or 3701)
            {
                double start = c.Now + Math.Max(0, status.Remaining);
                if (Warning == status.Id && Math.Abs(start - StartsAt) <= .12) return boundary;
                Warning = status.Id; StartsAt = start;
                return true;
            }
        if (Warning != 0 && c.Now > StartsAt + .5) { Warning = 0; return true; }
        return false;
    }

    internal bool Matches(HitTimedRoutes.Plan? plan, Vector2? released = null)
    {
        if (plan is not { Found: true, Legs.Length: > 0 }) return false;
        var leg = plan.Legs[0];
        double at = leg.Departure;
        foreach (var move in (PilotCourseGeometry.Move[])leg.Edge.Motion!)
        {
            at += PilotCourseGeometry.Trajectory([move], Actuator.WalkSpeed)[^1].Time;
            if (move.Kind == "forced")
                return Math.Abs(at - ReadyAt) <= .12 &&
                    Vector2.DistanceSquared(move.Path[^1].XZ(), released ?? End) <= .0225f;
        }
        return false;
    }

    internal bool IncludesWarning(HitTimedRoutes.Plan? plan)
    {
        if (Warning == 0 || plan is not { Found: true, Legs.Length: > 0 }) return false;
        double at = plan.Legs[0].Departure;
        foreach (var move in (PilotCourseGeometry.Move[])plan.Legs[0].Edge.Motion!)
        {
            if (move.Kind == "forced")
                return Math.Abs(at - StartsAt) <= .12 && move.Facing is { } facing &&
                    Math.Abs(Geo.WrapAngle(Geo.Heading(Geo.Toward(move.Path[0].XZ(), move.Path[^1].XZ())) -
                        facing - Offset(Warning))) < .001f;
            at += PilotCourseGeometry.Trajectory([move], Actuator.WalkSpeed)[^1].Time;
        }
        return false;
    }

    internal HitForcedMarch Snapshot() => (HitForcedMarch)MemberwiseClone();

    internal HitTimedRoutes.Plan Retime(PilotCourseGeometry data, HitTimedRoutes.Plan plan, PilotPose pose, double now)
    {
        if (!plan.Found || plan.Legs.Length == 0) return plan;
        var edge = plan.Legs[0].Edge;
        var original = (PilotCourseGeometry.Move[])edge.Motion!;
        if (!original.Any(m => m.Kind == "forced")) return plan with { Found = false };
        var moves = new List<PilotCourseGeometry.Move>();
        double elapsed = 0;
        foreach (var move in original)
        {
            var next = move;
            if (moves.Count == 0 && move.Kind == "walk")
            {
                if (!data.Clear(pose.Feet, move.Path[^1].XZ())) return plan with { Found = false };
                next = PilotCourseGeometry.Move.Walk(pose.Feet, move.Path[^1]);
            }
            if (move.Kind == "forced-wait" && move.Facing != null)
            {
                double remaining = StartsAt - now - elapsed;
                if (remaining < 0) return plan with { Found = false };
                next = Timed(move.Kind, move.Path[0], move.Path[^1], remaining, move.Facing);
            }
            if (move.Kind == "forced" && Native)
                next = Timed(move.Kind, pose.Feet, move.Path[^1], Math.Max(0, ReadyAt - now), move.Facing);
            moves.Add(next);
            elapsed += PilotCourseGeometry.Trajectory([next], Actuator.WalkSpeed)[^1].Time;
        }
        var first = data.Edge(edge.From, edge.To, moves.ToArray(), Actuator.WalkSpeed);
        return plan with { Legs = [new(first, now), .. plan.Legs.Skip(1)],
            Arrival = plan.Legs.Length == 1 ? now + first.Duration : plan.Arrival };
    }

    internal IEnumerable<HitTimedRoutes.Edge> Connectors(PilotCourseGeometry data, PilotContext c, float speed)
        => Connectors(data, c.Pose, c.Now, speed);

    internal IEnumerable<HitTimedRoutes.Edge> Connectors(PilotCourseGeometry data, PilotPose pose, double now, float speed,
        HitTimedRoutes.Plan? approach = null)
    {
        if (Native)
        {
            double remaining = Math.Max(0, ReadyAt - now);
            var finish = Feet(End);
            var forced = Timed("forced", pose.Feet, finish, remaining);
            foreach (var edge in Continue([forced], finish)) yield return edge;
            yield break;
        }
        if (!pose.Grounded)
        {
            foreach (var edge in data.Connectors(pose, speed)) yield return edge;
            yield break;
        }
        double until = Math.Max(0, StartsAt - now);
        float offset = Offset(Warning);
        foreach (var preparation in Preparations())
        {
            var stand = preparation.Feet;
            float natural = Geo.WrapAngle(preparation.Facing + offset);
            var headings = Enumerable.Range(0, 12).Select(i => Geo.WrapAngle(MathF.PI + i * MathF.PI / 6)).Append(natural);
            foreach (float direction in headings.Distinct())
            {
                float facing = Geo.WrapAngle(direction - offset);
                double turn = Math.Abs(Geo.WrapAngle(facing - preparation.Facing)) / TurnRate;
                if (preparation.Seconds + turn + Margin > until &&
                    (preparation.Seconds > until || Math.Abs(Geo.WrapAngle(facing - preparation.Facing)) > .015f)) continue;
                var finish = Feet(stand.XZ() + Geo.FromHeading(direction) * (Actuator.WalkSpeed * (float)Duration));
                if (data.Height(finish.XZ()) == null || !data.Clear(stand, finish.XZ())) continue;
                var moves = new List<PilotCourseGeometry.Move>(preparation.Moves);
                if (turn > .001) moves.Add(Timed("prepare-turn", stand, stand, turn, facing));
                moves.Add(Timed("forced-wait", stand, stand, Math.Max(0, until - preparation.Seconds - turn), facing));
                moves.Add(Timed("forced", stand, finish, Duration, facing));
                foreach (var edge in Continue(moves, finish)) yield return edge;
            }
        }

        IEnumerable<(Vector3 Feet, float Facing, double Seconds, PilotCourseGeometry.Move[] Moves)> Preparations()
        {
            yield return (pose.Feet, pose.Facing, 0, []);
            if (approach is not { Found: true }) yield break;
            var prefix = new List<PilotCourseGeometry.Move>();
            double elapsed = 0;
            var feet = pose.Feet;
            float facing = pose.Facing;
            foreach (var leg in approach.Legs)
            {
                if (leg.Departure > now + elapsed)
                {
                    double wait = leg.Departure - now - elapsed;
                    prefix.Add(Timed("forced-wait", feet, feet, wait)); elapsed += wait;
                }
                foreach (var move in (PilotCourseGeometry.Move[])leg.Edge.Motion!)
                {
                    if (move.Kind is not ("walk" or "jump" or "join")) yield break;
                    var path = move.Path;
                    double duration = PilotCourseGeometry.Trajectory([move], speed)[^1].Time;
                    if (move.Kind == "walk")
                    {
                        // Sample the planned walk, including long graph edges.
                        // Candidates retain every preceding turn and jump.
                        double segmentAt = elapsed;
                        var segments = new List<PilotCourseGeometry.Move>();
                        for (int i = 1; i < path.Length; ++i)
                        {
                            float distance = Vector2.Distance(path[i - 1].XZ(), path[i].XZ());
                            float heading = distance > .001f ? Geo.Heading(Geo.Toward(path[i - 1].XZ(), path[i].XZ())) : facing;
                            int count = Math.Max(1, (int)Math.Ceiling(distance));
                            for (int j = 1; j <= count; ++j)
                            {
                                float fraction = (float)j / count;
                                double seconds = segmentAt + distance * fraction / speed;
                                if (seconds + Margin > until) break;
                                var point = Vector3.Lerp(path[i - 1], path[i], fraction);
                                yield return (point, heading, seconds,
                                    [.. prefix, .. segments, PilotCourseGeometry.Move.Walk(path[i - 1], point)]);
                            }
                            segments.Add(PilotCourseGeometry.Move.Walk(path[i - 1], path[i]));
                            segmentAt += distance / speed;
                        }
                    }
                    elapsed += duration;
                    if (elapsed + Margin > until) yield break;
                    prefix.Add(move); feet = path[^1];
                    if (path.Length > 1 && Vector2.DistanceSquared(path[^2].XZ(), feet.XZ()) > .000001f)
                        facing = Geo.Heading(Geo.Toward(path[^2].XZ(), feet.XZ()));
                    if (move.Kind != "walk") yield return (feet, facing, elapsed, prefix.ToArray());
                }
            }
        }

        Vector3 Feet(Vector2 p) => new(p.X, data.Height(p) ?? pose.Feet.Y, p.Y);
        IEnumerable<HitTimedRoutes.Edge> Continue(IEnumerable<PilotCourseGeometry.Move> prefix, Vector3 finish)
        {
            var continuation = pose with { Feet = finish, Motion = MrBean.FallGuys.MotionKind.Ordinary, Grounded = true };
            var first = prefix.ToArray();
            foreach (var edge in data.Connectors(continuation, speed).OrderBy(e => e.Duration).Take(24))
                yield return data.Edge(data.Nodes.Length, edge.To,
                    [.. first, .. (PilotCourseGeometry.Move[])edge.Motion!], speed);
        }
    }

    private static PilotCourseGeometry.Move Timed(string kind, Vector3 from, Vector3 to, double duration, float? facing = null)
        => new()
        {
            Kind = kind, Points = [[from.X, from.Y, from.Z], [to.X, to.Y, to.Z]], Facing = facing,
            TimedPath = [new(from.XZ(), 0), new(to.XZ(), duration)]
        };
}
