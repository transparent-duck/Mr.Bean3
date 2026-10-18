using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

// A short forecast of the route that still owns movement. Never splice into a
// native jump/slide. The actual position and state must match again at commit.
internal sealed record RouteContinuation(PilotPose Pose, double At, HitTimedRoutes.Plan Prefix,
    HitTimedRoutes.Edge? Connector = null, float? ArrowHeading = null)
{
    // Project only the executing edge. Its cached time samples include native
    // curves and force boundaries; no graph scan or allocation is needed.
    internal static double Progress(HitTimedRoutes.Edge edge, Vector2 feet)
    {
        double time = 0; float best = float.PositiveInfinity;
        for (int i = 1; i < edge.Path.Length; ++i)
        {
            var a = edge.Path[i - 1]; var b = edge.Path[i]; var d = b.Position - a.Position;
            if (d.LengthSquared() < 1e-10f) continue;
            float u = Math.Clamp(Vector2.Dot(feet - a.Position, d) / d.LengthSquared(), 0, 1);
            float error = Vector2.DistanceSquared(feet, a.Position + d * u);
            if (error >= best) continue;
            best = error; time = a.Time + (b.Time - a.Time) * u;
        }
        return time;
    }

    internal bool Matches(PilotPose pose, double now)
    {
        if (pose.Force != Pose.Force || pose.Motion != Pose.Motion &&
            pose.Motion is MotionKind.Steering or MotionKind.Wet or MotionKind.Arrow) return false;
        foreach (var leg in Prefix.Legs)
        {
            if (now > leg.Departure + leg.Edge.Duration) continue;
            var path = leg.Edge.Path;
            double elapsed = Math.Max(0, now - leg.Departure);
            for (int i = 1; i < path.Length; ++i)
            {
                if (elapsed > path[i].Time) continue;
                float u = (float)Math.Clamp((elapsed - path[i - 1].Time) /
                    Math.Max(1e-9, path[i].Time - path[i - 1].Time), 0, 1);
                return Vector2.DistanceSquared(pose.XZ,
                    Vector2.Lerp(path[i - 1].Position, path[i].Position, u)) <= .15f * .15f;
            }
        }
        return Vector2.DistanceSquared(pose.XZ, Pose.XZ) <= .15f * .15f;
    }

    internal static RouteContinuation? Forecast(PilotCourseGeometry data, HitTimedRoutes.Plan plan,
        int firstLeg, int firstMove, bool started, double moveAt, PilotPose pose, double now, double lead, float speed,
        float arrowRate = -MathF.Tau)
    {
        var legs = new List<HitTimedRoutes.Leg>();
        double at = now, target = now + lead; var feet = pose.Feet;
        float heading = pose.Raw.ArrowDirection ?? pose.Facing;
        var motion = pose.Motion;
        bool arrow = pose.Has(3694);
        int remainingLeg = firstLeg, remainingMove = firstMove, remainingPoint = 1;
        for (int l = firstLeg; l < plan.Legs.Length; ++l)
        {
            remainingLeg = l; remainingMove = l == firstLeg ? firstMove : 0; remainingPoint = 1;
            var leg = plan.Legs[l];
            var moves = (PilotCourseGeometry.Move[])leg.Edge.Motion!;
            bool canWait = leg.Edge.From < data.Nodes.Length ? data.Nodes[leg.Edge.From].CanWait :
                pose.Motion != MotionKind.Steering && pose.Force == Vector2.Zero;
            if (!(l == firstLeg && started) && canWait && at < leg.Departure)
            {
                double end = Math.Min(target, leg.Departure);
                Add(feet, feet, at, end, true); at = end;
                if (at >= target) return Finish();
            }
            for (int m = l == firstLeg ? firstMove : 0; m < moves.Length; ++m)
            {
                remainingMove = m; remainingPoint = 1;
                var move = moves[m]; string kind = move.Kind;
                if (kind == "join") continue;
                // Forced/recovery clocks remain owned by their native adapter.
                if (kind is not ("walk" or "jump" or "drop" or "ice" or "wet" or "steering" or "arrow")) return null;
                bool current = l == firstLeg && m == firstMove && started;
                if (current && kind == "jump" && pose.Grounded && now - moveAt > .5) kind = "walk";
                var points = move.Path; int next = 1;
                if (current && points.Length > 2)
                {
                    float best = float.PositiveInfinity;
                    for (int p = 1; p < points.Length; ++p)
                    {
                        var a = points[p - 1].XZ(); var d = points[p].XZ() - a;
                        float u = Math.Clamp(Vector2.Dot(feet.XZ() - a, d) / Math.Max(1e-12f, d.LengthSquared()), 0, 1);
                        float error = Vector2.DistanceSquared(feet.XZ(), a + d * u);
                        if (error < best) { best = error; next = p; }
                    }
                }
                if (Vector2.DistanceSquared(feet.XZ(), points[^1].XZ()) < .12f * .12f) continue;
                if (kind == "arrow" && !(current && pose.Speed >= .5f))
                {
                    float desired = Geo.Heading(Geo.Toward(points[0].XZ(), points[^1].XZ()));
                    double end = Math.Min(target, at + HitTimedRoutes.ArrowTurn(heading, desired, arrowRate));
                    Add(feet, feet, at, end, true); at = end;
                    if (at >= target) return Finish();
                    heading = desired;
                }
                if (kind == "ice")
                {
                    double elapsed = current ? Math.Clamp(now - moveAt, 0, 50d / 30) : 0;
                    double duration = 50d / 30 - elapsed;
                    if (duration <= .001) return null;
                    var path = new HitTimedRoutes.Point[51];
                    float fromU = (float)(elapsed / (50d / 30));
                    float fromProgress = fromU * fromU * (3 - 2 * fromU);
                    for (int i = 0; i <= 50; ++i)
                    {
                        float u = (float)((elapsed + duration * i / 50) / (50d / 30));
                        float progress = (u * u * (3 - 2 * u) - fromProgress) / Math.Max(.00001f, 1 - fromProgress);
                        path[i] = new(Vector2.Lerp(feet.XZ(), points[^1].XZ(), progress), duration * i / 50);
                    }
                    legs.Add(new(new(0, 0, path), at)); at += duration; feet = points[^1];
                    motion = MotionKind.Ordinary;
                    remainingMove = m + 1; remainingPoint = 1;
                    if (at >= target) return Finish();
                    continue;
                }
                bool native = kind is "jump" or "drop";
                for (int p = next; p < points.Length; ++p)
                {
                    remainingPoint = p;
                    var end = points[p];
                    var direction = Geo.Toward(feet.XZ(), end.XZ());
                    float travelSpeed = kind == "jump" ? 6 :
                        kind == "steering" && motion == MotionKind.Steering ? SteeringPassage.Speed : speed;
                    if (!native && pose.Force != Vector2.Zero)
                        Geo.CompensateForce(direction, pose.Force, speed, out travelSpeed);
                    if (travelSpeed <= .01f) return null;
                    double duration = Vector2.Distance(feet.XZ(), end.XZ()) / travelSpeed;
                    if (duration <= .000001) { feet = end; continue; }
                    if (!native && at + duration >= target)
                    {
                        end = Vector3.Lerp(feet, end, (float)Math.Clamp((target - at) / duration, 0, 1));
                        duration = target - at;
                    }
                    Add(feet, end, at, at + duration, !native); at += duration; feet = end;
                    if (kind == "steering") heading = Geo.Heading(direction);
                    if (!native && at >= target) return Finish();
                }
                remainingMove = m + 1; remainingPoint = 1;
                if (at >= target) return Finish();
            }
        }
        if (at < target) { Add(feet, feet, at, target, true); at = target; }
        return Finish();

        void Add(Vector3 a, Vector3 b, double start, double end, bool support)
        {
            legs.Add(new(new(0, 0, [new(a.XZ(), 0, support), new(b.XZ(), end - start)]), start));
            // Native arrows rotate only while stopped; movement holds heading.
            if (arrow && a == b) heading = Geo.WrapAngle(heading + arrowRate * (float)(end - start));
        }
        RouteContinuation Finish()
        {
            if (data.Height(feet.XZ()) is { } height) feet.Y = height;
            return new(pose with { Feet = feet, Grounded = true, Motion = motion,
                Raw = motion == MotionKind.Steering || arrow ? pose.Raw with { ArrowDirection = heading } : pose.Raw }, at,
                new(legs.ToArray(), at, 0, true), Remainder(), arrow ? heading : null);
        }
        HitTimedRoutes.Edge? Remainder()
        {
            if (remainingLeg >= plan.Legs.Length) return null;
            var edge = plan.Legs[remainingLeg].Edge;
            var moves = (PilotCourseGeometry.Move[])edge.Motion!;
            if (remainingMove >= moves.Length || moves[remainingMove].Kind is not ("walk" or "arrow")) return null;
            var current = moves[remainingMove];
            // A bent edge cannot be replaced by a straight connection to its
            // end. Keep its untraversed corners as one ordinary search option.
            var points = new[] { feet }.Concat(current.Path.Skip(remainingPoint));
            var first = new PilotCourseGeometry.Move { Kind = current.Kind,
                Points = points.Select(p => new[] { p.X, p.Y, p.Z }).ToArray() };
            return data.Edge(data.Nodes.Length, edge.To,
                new[] { first }.Concat(moves.Skip(remainingMove + 1)).ToArray(), speed);
        }
    }
}
