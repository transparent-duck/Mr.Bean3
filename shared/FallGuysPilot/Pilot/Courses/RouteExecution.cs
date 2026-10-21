using System;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

// One controller for live input and projected input. Copying this value copies
// the entire route cursor; projection never owns an actuator or a worker.
internal struct RouteExecution
{
    internal int Leg, Move;
    internal bool Started, NativeSeen, Launched, Airborne;
    internal double StartedAt, LaunchedAt, EndsAt;
    internal SharedRouteProgress Cursor;
    internal PilotCourseGeometry.Move[] Moves;
    internal Vector2? Aim;
    internal bool ArrowLatched;
    internal float? ArrowTarget;

    internal readonly record struct Options(PilotCourseGeometry Data, CourseId Course, float Speed,
        float ArrowRate, HitTimedRoutes.Interval? Bomb = null);

    internal enum Input { None, Hold, Walk, Steer, Arrow, Jump, Face }
    internal readonly record struct Command(Input Kind, Vector2 Direction = default,
        Vector3? Target = null, float Radius = .12f, double Until = 0,
        bool Boundary = false, bool Complete = false, bool Relocalize = false, bool Started = false)
    {
        internal void Apply(PilotContext c)
        {
            switch (Kind)
            {
                case Input.Hold: c.Act.Hold(); break;
                case Input.Walk:
                    if (c.Pose.Force != Vector2.Zero) c.Act.WalkNet(Direction, c.Pose.Force);
                    else c.Act.Walk(Direction);
                    break;
                case Input.Steer: c.Act.Steer(c.Pose.Feet, Direction); break;
                case Input.Arrow: c.Act.Arrow(Direction, Until); break;
                case Input.Jump:
                    if (Target is { } takeoff && Vector2.Distance(c.Pose.XZ, takeoff.XZ()) > .05f)
                        c.Act.Nudge(takeoff with { Y = c.Pose.Feet.Y }, c.Now, 0);
                    c.Act.Jump(Direction, c.Now);
                    c.Log("jump", new { at = c.Now, feet = c.Pose.Feet, target = Target });
                    break;
                case Input.Face: c.Act.Face(Geo.Heading(Direction)); break;
            }
        }
    }

    internal readonly bool PendingGain(PilotPose pose) => Started && Moves != null && Move < Moves.Length &&
        (Moves[Move].Kind == "steering" && !pose.Has(3702) &&
            pose.Feet.Z is < HitSteeringRoutes.WaitingZ and > HitSteeringRoutes.ExitZ ||
         Moves[Move].Kind == "wet" && !pose.Has(3703) && !HitWetStep.CanWaitAt(pose.Feet.Z));

    internal readonly bool CanStop(PilotPose pose, Options options) => !PendingGain(pose) && !Launched &&
        !(Started && Moves != null && Move < Moves.Length && Moves[Move].Kind is "ice" or "forced") &&
        pose.Grounded && !pose.Locked && pose.Force == Vector2.Zero &&
        pose.Motion is not (MotionKind.Ice or MotionKind.Forced or MotionKind.Steering) &&
        (options.Course != CourseId.Hit || HitCourseGeometry.CanWait(pose));

    internal static bool AwaitsIceGain(PilotPose pose, Options options) => options.Course == CourseId.Knight &&
        pose.Grounded && PilotCourseGeometry.InIce(pose.Feet) && !pose.Has(3693);

    internal static Command? BombWait(PilotPose pose, double now, Options options)
        => options.Bomb is { } bomb && now >= bomb.Start && (now < bomb.End || pose.Has(3802))
            ? new(Input.Hold, Until: bomb.End, Boundary: now >= bomb.End) : null;

    internal void ClearMove()
    {
        Started = NativeSeen = Launched = Airborne = false;
        Cursor = default; Aim = null;
    }

    internal Command Advance(HitTimedRoutes.Plan plan, PilotPose pose, double now, Options options,
        double[]? departures = null)
    {
        var command = AdvanceCore(plan, pose, now, options, departures);
        // Native release and a change of owner require an observed receipt.
        // The live command continues; a projection ends at this same boundary.
        if (pose.Has(3703) && HitWetStep.CanWaitAt(pose.Feet.Z) ||
            pose.Has(3702) && pose.Feet.Z <= HitSteeringRoutes.ExitZ ||
            pose.Has(3694) && !PilotCourseGeometry.InArrow(pose.Feet))
            command = command with { Boundary = true };
        if (command.Kind != Input.Arrow) { ArrowLatched = false; ArrowTarget = null; }
        else
        {
            float target = Geo.Heading(command.Direction);
            if (ArrowTarget is not { } previous || Math.Abs(Geo.WrapAngle(target - previous)) > .0001f) ArrowLatched = false;
            ArrowTarget = target;
        }
        return command;
    }

    private Command AdvanceCore(HitTimedRoutes.Plan plan, PilotPose pose, double now, Options options,
        double[]? departures)
    {
        bool began = false;
        while (Leg < plan.Legs.Length)
        {
            var leg = plan.Legs[Leg];
            Moves = (PilotCourseGeometry.Move[])leg.Edge.Motion!;
            if (Move >= Moves.Length) { ++Leg; Move = 0; ClearMove(); continue; }
            var action = Moves[Move];
            var points = action.Path;
            string kind = action.Kind;

            // Native receipt/landing is an input to this controller, not an
            // estimated completion time in a second execution loop.
            if (pose.Motion == MotionKind.Ice)
            {
                if (kind == "ice") NativeSeen = true;
                return new(Input.Hold, Boundary: true);
            }
            if (pose.Motion == MotionKind.Forced || pose.Has(3737))
            {
                if (kind == "forced") NativeSeen = true;
                return new(Input.Hold, Boundary: true);
            }
            if (pose.Locked) return new(Input.Hold, Boundary: true);
            if (kind is "ice" or "forced" && NativeSeen)
            { FinishMove(); continue; }
            if (AwaitsIceGain(pose, options))
                return new(Input.Hold, Boundary: true);
            if (BombWait(pose, now, options) is { } bombWait) return bombWait;

            bool canWait = CanStop(pose, options) &&
                (leg.Edge.From >= options.Data.Nodes.Length || options.Data.Nodes[leg.Edge.From].CanWait);
            if (!Started && Move == 0 && canWait && now < leg.Departure)
            {
                if (pose.Has(3694) && kind == "arrow")
                    return new(Input.Arrow, Geo.Toward(points[0].XZ(), points[^1].XZ()), Until: leg.Departure);
                return new(Input.Hold, Until: leg.Departure);
            }
            if (!Started)
            {
                double duration = Duration(action, options.Speed);
                if (options.Bomb is { } stop && kind is "jump" or "ice" &&
                    now < stop.End && now + duration >= stop.Start)
                    return new(Input.Hold, Until: stop.End);
                StartedAt = now; EndsAt = leg.Departure;
                for (int i = 0; i <= Move; ++i) EndsAt += Duration(Moves[i], options.Speed);
                bool skip = options.Data.CrownStairBoost && kind == "walk" && points.Length > 1 &&
                    pose.Feet.Z <= PilotCourseGeometry.CrownBoostStartZ &&
                    Vector3.DistanceSquared(pose.Feet, points[0]) < 1 &&
                    options.Data.Ground.ClearFromGrounded(pose.Feet, points[1].XZ());
                Cursor = skip ? new() { Index = 1 } : SharedRouteProgress.Begin(points, pose.Feet,
                    kind == "jump", kind == "steering", kind == "steering" && (Leg > 0 || Move > 0));
                if (kind == "walk" && !skip && pose.Motion == MotionKind.Ordinary && pose.Force == Vector2.Zero)
                    Cursor = WalkHandoffCheck.Choose(Cursor, points, pose.Feet, options.Data.Ground,
                        Frame(pose), options.Speed, Radius(leg, options));
                Started = true; began = true;
                if (departures != null && Move == 0) departures[Leg] = now;
            }

            if (kind == "join") { FinishMove(); continue; }
            if (kind == "recovery")
            {
                if (now < EndsAt) return new(Input.Hold, Until: EndsAt, Started: began);
                FinishMove(); continue;
            }
            if (kind is "prepare-turn" or "forced-wait" or "forced")
            {
                if (kind != "forced" && now >= EndsAt &&
                    (action.Facing == null || Math.Abs(Geo.WrapAngle(pose.Facing - action.Facing.Value)) <= .02f))
                { FinishMove(); continue; }
                return new(action.Facing == null ? Input.Hold : Input.Face,
                    action.Facing is { } facing ? Geo.FromHeading(facing) : default,
                    Until: EndsAt, Boundary: kind == "forced", Started: began);
            }
            if (kind == "ice")
            {
                if (Vector2.Distance(pose.XZ, points[^1].XZ()) < .15f) { FinishMove(); continue; }
                if (!pose.Has(3693)) return new(Input.Hold, Boundary: true, Started: began);
                if (now - StartedAt > .5) return new(Input.Hold, Relocalize: true);
                Aim = Geo.Toward(pose.XZ, points[^1].XZ());
                return new(Input.Walk, Aim.Value, points[^1], Boundary: true, Started: began);
            }
            if (kind == "arrow")
            {
                var delta = points[^1].XZ() - points[0].XZ();
                if (Vector2.Distance(pose.XZ, points[^1].XZ()) < .2f ||
                    delta.LengthSquared() > 0 && Vector2.Dot(pose.XZ - points[^1].XZ(), delta) >= 0)
                { FinishMove(); continue; }
                Aim = Geo.Toward(points[0].XZ(), points[^1].XZ());
                return new(pose.Has(3694) ? Input.Arrow : Input.Walk, Aim.Value, points[^1], .2f,
                    leg.Departure, Boundary: !pose.Has(3694), Started: began);
            }
            if (kind == "steering" && pose.Has(3702))
            {
                int index = Cursor.Index;
                if (!SharedRouteProgress.SteeringTarget(points, pose.XZ, ref index, out var direction))
                { FinishMove(); continue; }
                Cursor.Index = index; Aim = direction;
                return new(Input.Steer, direction, Started: began);
            }
            if (kind == "jump")
            {
                if (Launched)
                {
                    Aim = Geo.Toward(points[0].XZ(), points[1].XZ());
                    if (!pose.Grounded) { Airborne = true; return new(Input.Walk, Aim.Value, Boundary: true); }
                    if (!Airborne && now - LaunchedAt < .5)
                        return new(Input.Walk, Aim.Value, Boundary: true);
                    if (!Airborne) --Cursor.Index;
                    Launched = Airborne = false;
                }
                if (Cursor.Index == 0)
                {
                    float distance = Vector2.Distance(pose.XZ, points[0].XZ());
                    bool passed = Vector2.Dot(pose.XZ - points[0].XZ(), points[1].XZ() - points[0].XZ()) >= 0;
                    Aim = Geo.Toward(points[0].XZ(), points[1].XZ());
                    if (pose.Grounded && (distance <= .25f || passed && distance < .6f))
                    {
                        Launched = true; LaunchedAt = now; ++Cursor.Index;
                        return new(Input.Jump, Aim.Value, points[0], Boundary: true, Started: began);
                    }
                    return new(Input.Walk, Geo.Toward(pose.XZ, points[0].XZ()), points[0], .25f, Started: began);
                }
            }
            bool moving = SharedRouteProgress.WalkTarget(points, pose.XZ, ref Cursor, Radius(leg, options), out var walk);
            if (kind == "steering" && Cursor.Index >= points.Length - 1 &&
                points[0].Z >= HitSteeringRoutes.WaitingZ && points[^1].Z < HitSteeringRoutes.WaitingZ &&
                pose.Feet.Z is < HitSteeringRoutes.WaitingZ and > HitSteeringRoutes.ExitZ)
                return new(Input.Walk, Geo.FromHeading(HitSteeringRoutes.EntryHeading), Boundary: true, Started: began);
            if (!moving)
            {
                if (kind == "drop" && !pose.Grounded) return new(Input.Hold, Boundary: true);
                FinishMove(); continue;
            }
            Aim = walk;
            var target = Cursor.Join ?? points[Cursor.Index];
            return new(Input.Walk, walk, target, Cursor.Join != null ? .12f : Radius(leg, options),
                Boundary: kind == "wet" && !pose.Has(3703) || kind == "steering" && !pose.Has(3702) || !pose.Grounded,
                Started: began);
        }
        Aim = null;
        return new(Input.Hold, Complete: true);
    }

    private void FinishMove()
    {
        ++Move; ClearMove();
        if (Moves != null && Move >= Moves.Length) { ++Leg; Move = 0; Moves = []; }
    }
    internal static float Frame(PilotPose pose) => Math.Clamp((float)(pose.Raw.RealFrameSeconds ?? 1d / 60), 1f / 240, .1f);
    private static float Radius(HitTimedRoutes.Leg leg, Options options) => options.Course == CourseId.Crown &&
        (leg.Edge.Destination?.Goal ?? (leg.Edge.To < options.Data.Nodes.Length && options.Data.Nodes[leg.Edge.To].Goal)) ? .03f : .12f;
    private static double Duration(PilotCourseGeometry.Move move, float speed)
    {
        if (move.TimedPath is { Length: > 0 } timed) return timed[^1].Time;
        if (move.Kind == "ice") return 50d / 30;
        double duration = 0;
        var points = move.Path;
        for (int i = 1; i < points.Length; ++i) duration += Vector2.Distance(points[i - 1].XZ(), points[i].XZ()) / (move.Kind == "jump" ? 6 : speed);
        return duration;
    }

    internal static bool ArrowWalk(ref bool latched, float actual, float desired, double now,
        double notBefore, float frame, float rate)
    {
        float error = Math.Abs(Geo.WrapAngle(desired - actual));
        if (error > .15f) latched = false;
        latched = now + frame * .5 >= notBefore &&
            (latched || error <= Math.Max(.01f, Math.Abs(rate) * frame * .55f));
        return latched;
    }
}
