using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

// The environment adapter advances a copied pose under the controller's input.
// All waits, target changes, completion and native boundaries come from Advance.
internal static class RouteProjection
{
    internal static RouteContinuation? Follow(HitTimedRoutes.Plan plan, RouteExecution state,
        PilotPose pose, double now, double horizon, RouteExecution.Options options, bool handoff, bool captureMotion = false)
    {
        var timing = new RouteContinuation.Timing(plan.Legs.Length);
        if (captureMotion) timing.Motion = [];
        var legs = new List<HitTimedRoutes.Leg>();
        var path = new List<HitTimedRoutes.Point>();
        double pathAt = now;
        int pathLeg = state.Leg;
        double at = now, end = now + horizon;
        float frame = RouteExecution.Frame(pose);
        bool boundary = false;
        while (at < end - 1e-8)
        {
            var command = state.Advance(plan, pose, at, options, timing.Departures);
            if (captureMotion && state.Leg != pathLeg) { Flush(); pathLeg = state.Leg; }
            if (command.Boundary || command.Relocalize) { boundary = true; break; }
            if (command.Complete)
            {
                timing.Complete = true;
                if (!state.CanStop(pose, options)) { boundary = true; break; }
                if (handoff) at = end;
                break;
            }
            double seconds = Math.Min(frame, end - at);
            var before = pose;
            switch (command.Kind)
            {
                case RouteExecution.Input.Hold:
                case RouteExecution.Input.Face:
                    if (command.Until <= at) { boundary = true; break; }
                    seconds = Math.Min(Math.Ceiling((command.Until - at) / frame) * frame, end - at);
                    if (command.Kind == RouteExecution.Input.Face)
                        pose = pose with { Facing = Geo.Heading(command.Direction) };
                    break;
                case RouteExecution.Input.Arrow:
                    if (pose.Raw.ArrowDirection is not { } actual) { boundary = true; break; }
                    float desired = Geo.Heading(command.Direction);
                    if (!RouteExecution.ArrowWalk(ref state.ArrowLatched, actual, desired, at,
                        command.Until, frame, options.ArrowRate))
                    {
                        pose = RotateArrow(pose, options.ArrowRate, seconds);
                        break;
                    }
                    pose = Walk(pose, command with { Direction = Geo.FromHeading(actual) }, options, frame, end - at, out seconds);
                    break;
                case RouteExecution.Input.Walk:
                    pose = Walk(pose, command, options, frame, end - at, out seconds);
                    break;
                case RouteExecution.Input.Steer:
                    if (pose.Raw.ArrowDirection is not { } heading) { boundary = true; break; }
                    float turnStep = Math.Clamp(Geo.WrapAngle(Geo.Heading(command.Direction) - heading),
                        -SteeringPassage.TurnRate * (float)seconds, SteeringPassage.TurnRate * (float)seconds);
                    var delta = Geo.FromHeading(heading + turnStep * .5f) * (SteeringPassage.Speed * (float)seconds);
                    pose = pose with { Feet = pose.Feet + new Vector3(delta.X, 0, delta.Y),
                        Raw = pose.Raw with { ArrowDirection = Geo.WrapAngle(heading + turnStep) }, Speed = SteeringPassage.Speed };
                    break;
                default: boundary = true; break;
            }
            if (boundary) break;
            if (command.Kind is RouteExecution.Input.Hold or RouteExecution.Input.Face)
            {
                // A wait is a gap in the route, not a moving edge. This keeps
                // Crystal's supported wind waits under the normal hold rule.
                pose = RotateArrow(pose, options.ArrowRate, seconds) with { Speed = 0 };
            }
            if (pose.Feet == before.Feet) Flush();
            else
            {
                if (path.Count == 0) { pathAt = at; path.Add(new(before.XZ, 0)); }
                path.Add(new(pose.XZ, at + seconds - pathAt));
            }
            at += seconds;
            if (pose.Grounded && options.Data.Height(pose.XZ) is { } height)
                pose = pose with { Feet = pose.Feet with { Y = height } };
            pose = pose with { At = at };
        }
        Flush();
        if (handoff && (boundary || at < end - 1e-8)) return null;
        return new(pose, at, new(legs.ToArray(), at, 0, true), handoff ? Remainder() : null,
            pose.Has(3694) ? pose.Raw.ArrowDirection : null) { Schedule = timing, Boundary = boundary };

        void Flush()
        {
            if (path.Count > 1)
            {
                var edge = new HitTimedRoutes.Edge(0, 0, path.ToArray());
                legs.Add(new(edge, pathAt));
                timing.Motion?.Add((pathLeg, new(edge, pathAt)));
            }
            path.Clear();
        }

        HitTimedRoutes.Edge? Remainder()
        {
            if (state.Leg >= plan.Legs.Length) return null;
            var original = plan.Legs[state.Leg].Edge;
            var moves = (PilotCourseGeometry.Move[])original.Motion!;
            if (state.Move >= moves.Length || moves[state.Move].Kind is not ("walk" or "arrow")) return null;
            var current = moves[state.Move];
            var points = new List<Vector3> { pose.Feet };
            if (state.Started && state.Cursor.Join is { } join) points.Add(join);
            points.AddRange(current.Path.Skip(state.Started ? state.Cursor.Index : 0));
            if (points.Count < 2) return null;
            var first = new PilotCourseGeometry.Move { Kind = current.Kind,
                Points = points.Select(p => new[] { p.X, p.Y, p.Z }).ToArray() };
            if (options.Data.CrownStairBoost && first.Kind == "walk") first.TimedPath = PilotCourseGeometry.CrownWalkTrajectory(first.Path);
            var remainder = options.Data.Edge(options.Data.Nodes.Length, original.To,
                new[] { first }.Concat(moves.Skip(state.Move + 1)).ToArray(), options.Speed);
            remainder.Destination = original.Destination;
            return remainder;
        }
    }

    private static PilotPose RotateArrow(PilotPose pose, float rate, double seconds)
        => pose.Has(3694) && pose.Raw.ArrowDirection is { } heading
            ? pose with { Raw = pose.Raw with { ArrowDirection = Geo.WrapAngle(heading + rate * (float)seconds) } } : pose;

    private static PilotPose Walk(PilotPose pose, RouteExecution.Command command, RouteExecution.Options options,
        float frame, double remaining, out double seconds)
    {
        float speed = options.Data.CrownStairBoost && PilotCourseGeometry.CrownBoostRegion(pose.Feet)
            ? PilotCourseGeometry.CrownBoostSpeed : options.Speed;
        var input = Geo.CompensateForce(command.Direction, pose.Force, speed, out float net);
        seconds = Math.Min(frame, remaining);
        // A straight input remains unchanged up to the controller's next target
        // boundary. Skip those identical frames without skipping its decision.
        if (command.Target is { } target && net > .01f && !options.Data.CrownStairBoost &&
            pose.Motion is not (MotionKind.Wet or MotionKind.Arrow))
        {
            double distance = Math.Max(0, Vector2.Distance(pose.XZ, target.XZ()) - command.Radius);
            seconds = Math.Min(remaining, Math.Max(1, Math.Ceiling(distance / (net * frame))) * frame);
        }
        var delta = (input * speed + pose.Force) * (float)seconds;
        return pose with { Feet = pose.Feet + new Vector3(delta.X, 0, delta.Y), Speed = net };
    }
}
