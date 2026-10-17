using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed class KnightCourse : GraphCourse { public override CourseId Course => CourseId.Knight; }
internal sealed class RingCourse : GraphCourse { public override CourseId Course => CourseId.Ring; }
internal sealed class CrystalCourse : GraphCourse { public override CourseId Course => CourseId.Crystal; }
internal sealed class CrownCourse : GraphCourse { public override CourseId Course => CourseId.Crown; }

internal abstract class GraphCourse : CourseScript
{
    public override HazardSpec[] Hazards => CourseHazards.Specs(Course);
    public override Step[] Build(PilotPose start) => [new CourseRouteStep(Course)
        { Name = Course.ToString().ToLowerInvariant() + "-ctc", Zone = new(-1000, 1000, -1000, 1000) }];
}

// Each search starts at the actual feet and ends at the current course goal.
// Receipts do not replace an executing race route. Hits, displaced feet and
// changed Crystal carry state relocalize; Ring advances its goal on arrival.
internal sealed class CourseRouteStep(CourseId course) : Step
{
    private readonly PilotCourseGeometry data = PilotCourseGeometry.Load(course.ToString().ToLowerInvariant());
    private HitTimedRoutes graph = null!;
    private HitTimedRoutes.Plan? plan;
    private PilotCourseGeometry.Move[] moves = [];
    private Passage? passage;
    private int legIndex, moveIndex, goal = -1;
    private double seenHit, progressAt, nextRetry, moveAt, moveDuration;
    private Vector2 progressPoint;
    private bool moveStarted;
    private bool nativeIce, wasOnIce;
    private bool hadBomb, bombHolding;
    private double bombExpires = double.NaN;
    // A local stop 0.220 s before 34711 still exploded in Knight-003358.
    // Reserve one second for the server's movement judgment, not just the
    // local frame that clears input. Continue moving during the first 4 s.
    private const double BombLead = 1, BombTail = .1;
    private float speed = 6;
    private long plannedRevision;
    private double windHit = double.NegativeInfinity;
    private bool windPending, windNative;
    private double windExecution = 1.5;
    private int ringDirection = -1;
    private double seenDelivery = double.NegativeInfinity;
    private bool returnToLower;
    private HashSet<int>? lowerReturnGoals;

    public override bool ObserveMotion(PilotContext c)
    {
        if (c.Pose.Motion == MotionKind.Ice) nativeIce = true;
        if (course != CourseId.Crystal) return false;
        if (c.LastWindHitAt > windHit)
        {
            windHit = c.LastWindHitAt; windPending = true; windNative = false;
            c.Log("crystal-wind-pending", new { at = c.Now, receipt = windHit, feet = c.Pose.Feet });
        }
        if (!windPending) return false;
        bool native = c.Pose.Transport.InputLocked || c.Pose.Speed > 6.5f || !c.Pose.Grounded;
        if (native && !windNative)
            c.Log("crystal-wind-native", new { at = c.Now, feet = c.Pose.Feet, c.Pose.Transport, c.Pose.Speed });
        windNative |= native;
        if (native || !windNative && c.Now - windHit < 2)
        { c.Act.Hold(); progressAt = c.Now; return true; }
        c.Log("crystal-wind-end", new { at = c.Now, feet = c.Pose.Feet, nativeObserved = windNative,
            carry = Goal(c.Pose), c.Pose.Transport });
        if (windNative) windExecution = Math.Clamp(c.Now - windHit, .25, 2);
        windPending = false; plan = null;
        return false;
    }

    public override void Enter(PilotContext c)
    {
        CourseHazards.Install(course, c.Hazards);
        c.Act.KeepJumpSpeed = course == CourseId.Crystal;
        c.Act.ImmediateJump = course == CourseId.Ring;
        graph = data.Graph(speed); seenHit = c.LastHitAt; plan = null;
        if (course == CourseId.Crystal)
            lowerReturnGoals = Enumerable.Range(0, data.Nodes.Length)
                .Where(i => data.Nodes[i].ReturnFloor).ToHashSet();
        progressAt = c.Now; progressPoint = c.Pose.XZ;
        wasOnIce = c.Pose.Has(3693);
    }
    private int Goal(PilotPose pose)
    {
        if (course != CourseId.Crystal) return -1;
        foreach (var status in pose.Raw.Statuses)
            if (status.Id == 3704) return status.Param switch { 35 => 1, 36 => 2, 37 => 3, _ => 0 };
        return 0;
    }
    public override bool Tick(PilotContext c)
    {
        // Complete short connectors and hand off the next leg in the same
        // frame. In particular, holding for one frame would rotate an arrow
        // that was meant to continue straight without a turn.
        for (int i = 0; i < 8; ++i) if (!TickOnce(c)) return false;
        c.Act.Hold(); return false;
    }
    private bool TickOnce(PilotContext c)
    {
        bool onIce = c.Pose.Has(3693);
        if (course == CourseId.Knight && c.Pose.Grounded && onIce != wasOnIce)
        {
            c.Log("course-motion-boundary", new { at = c.Now, ice = onIce, feet = c.Pose.Feet });
            plan = null;
        }
        if (c.Pose.Grounded) wasOnIce = onIce;
        if (nativeIce)
        {
            nativeIce = false;
            // Unexpected native movement invalidates a walk/jump continuation.
            // A planned slide may continue only from its actual endpoint.
            bool expected = moveStarted && moves.Length > moveIndex && moves[moveIndex].Kind == "ice";
            float error = expected ? Vector2.Distance(c.Pose.XZ, moves[moveIndex].Path[^1].XZ()) : float.PositiveInfinity;
            c.Log("course-ice-land", new { at = c.Now, feet = c.Pose.Feet, expected, error = expected ? error : (float?)null });
            if (!expected || error > .15f) plan = null;
        }
        if (course == CourseId.Crown && c.Pose.Feet.Y > 40 && c.Pose.Feet.Z < 122)
        {
            var goalPoint = new Vector2(-.015258789f, 119.98169f);
            if (Vector2.Distance(c.Pose.XZ, goalPoint) > .15f) c.Act.Walk(Geo.Toward(c.Pose.XZ, goalPoint));
            else c.Act.Hold();
            return false;
        }
        if (course == CourseId.Knight)
        {
            bool bomb = c.Pose.Has(3802);
            if (bomb != hadBomb)
            {
                plan = null; hadBomb = bomb; bombHolding = false;
                bombExpires = double.NaN;
                if (bomb)
                    foreach (var status in c.Pose.Raw.Statuses)
                        if (status.Id == 3802) { bombExpires = c.Now + Math.Max(0, status.Remaining); break; }
                c.Log("course-bomb", new { at = c.Now, active = bomb, expires = double.IsFinite(bombExpires) ? bombExpires : (double?)null });
            }
            if (bomb && c.Now >= bombExpires - BombLead)
            {
                if (!bombHolding) c.Log("course-bomb-hold", new { at = c.Now, expires = bombExpires, feet = c.Pose.Feet });
                bombHolding = true; c.Act.Hold(); progressAt = c.Now; plan = null; return false;
            }
            foreach (var status in c.Pose.Raw.Statuses)
                if (status.Id == 3694 && status.Param != 0) c.Act.ArrowRate = -unchecked((short)status.Param) * MathF.PI / 180;
        }
        int currentGoal = Goal(c.Pose);
        if (course == CourseId.Crystal)
        {
            if (c.LastDeliveryAt > seenDelivery)
            { seenDelivery = c.LastDeliveryAt; returnToLower = true; plan = null; }
            if (returnToLower && c.Pose.Grounded && c.Pose.Feet.Y < 3.3f)
            {
                returnToLower = false; plan = null;
                c.Log("crystal-return-floor", new { at = c.Now, feet = c.Pose.Feet });
            }
        }
        if (course == CourseId.Ring && c.Hazards.RingWind is { Rate: > .001f or < -.001f } axis &&
            Math.Sign(axis.Rate) != ringDirection)
        { ringDirection = Math.Sign(axis.Rate); plan = null; }
        if (course is CourseId.Ring or CourseId.Crystal or CourseId.Crown && plan is { Found: true } && c.Pose.Grounded &&
            plannedRevision != c.Hazards.RouteRevision)
        {
            long previousRevision = plannedRevision;
            plannedRevision = c.Hazards.RouteRevision;
            if (graph.HasConflict(c.Hazards, plan, legIndex, c.Pose.XZ, c.Now, previousRevision))
            {
                c.Log("course-new-danger", new { at = c.Now, course, feet = c.Pose.Feet });
                plan = null;
            }
        }
        float available = c.Pose.Raw.Walk?.AvailableTargetSpeed ?? 6;
        float actualSpeed = course == CourseId.Crystal && available > 6 ? MathF.Round(available, 2) : 6;
        bool displaced = c.Now - progressAt > 2.5 && Vector2.Distance(progressPoint, c.Pose.XZ) < .1f;
        if (plan == null || c.Pose.Grounded && (seenHit != c.LastHitAt || goal != currentGoal || speed != actualSpeed || displaced))
        {
            if (c.Now < nextRetry && plannedRevision == c.Hazards.RouteRevision) { c.Act.Hold(); return false; }
            if (course == CourseId.Crystal && currentGoal != goal)
                c.Log("crystal-goal", new { at = c.Now, carry = currentGoal, previous = goal,
                    destination = currentGoal switch { 1 => 10060979, 2 => 10060978, 3 => 10060980, _ => 10060977 },
                    feet = c.Pose.Feet });
            goal = currentGoal; speed = actualSpeed; Select(c);
        }
        if (Vector2.Distance(progressPoint, c.Pose.XZ) > .1f) { progressPoint = c.Pose.XZ; progressAt = c.Now; }
        if (!plan!.Found)
        {
            c.Act.Hold(); plan = null; nextRetry = c.Now + .25;
            return false;
        }
        if (plan.Legs.Length == 0 || legIndex >= plan.Legs.Length)
        {
            if (course == CourseId.Ring && plan.Legs.Length > 0)
            { plan = null; return true; }
            c.Act.Hold(); progressAt = c.Now;
            return false;
        }
        var leg = plan.Legs[legIndex];
        if (moves.Length == 0) { moves = (PilotCourseGeometry.Move[])leg.Edge.Motion!; moveIndex = 0; }
        var move = moves[moveIndex];
        if (!moveStarted && moveIndex == 0 && c.Now > leg.Departure + .12 && c.Pose.Grounded)
        {
            c.Log("course-retime", new { at = c.Now, late = c.Now - leg.Departure, feet = c.Pose.Feet });
            Select(c); return true;
        }
        if (c.Now < leg.Departure)
        {
            if (c.Pose.Has(3694) && move.Kind == "arrow") c.Act.Arrow(Geo.Toward(move.Path[0].XZ(), move.Path[^1].XZ()), leg.Departure);
            else c.Act.Hold();
            progressAt = c.Now; return false;
        }
        if (!moveStarted)
        {
            moveDuration = PilotCourseGeometry.Trajectory([move], speed)[^1].Time;
            if (hadBomb && move.Kind is "jump" or "ice" && c.Now < bombExpires + BombTail &&
                c.Now + moveDuration >= bombExpires - BombLead)
            { c.Act.Hold(); progressAt = c.Now; return false; }
            moveAt = c.Now;
            var points = move.Path;
            passage = new PathPassage { Name = Name + ":" + move.Kind, Points = points,
                JumpFrom = move.Kind == "jump" ? [0] : [],
                // Crown-045749 stopped just outside the rotated launch box
                // at the ordinary 0.12 m radius. Its endpoint is inside it.
                ArriveRadius = course == CourseId.Crown && data.Nodes[leg.Edge.To].Goal ? .03f : .12f };
            passage.Begin(c); moveStarted = true;
            c.Log("course-motion", new { course, at = c.Now, legIndex, moveIndex, move.Kind, feet = c.Pose.Feet });
        }
        bool done;
        if (move.Kind == "join") done = true;
        else if (move.Kind == "arrow")
        {
            var points = move.Path; var target = points[^1].XZ();
            var delta = target - points[0].XZ();
            done = Vector2.Distance(c.Pose.XZ, target) < .2f ||
                delta.LengthSquared() > 0 && Vector2.Dot(c.Pose.XZ - target, delta) >= 0;
            if (!done)
            {
                // The search times this fixed native heading. Pursuing the
                // endpoint changed it near arrival and could demand another
                // full rotation after the planned turn had already completed.
                var direction = Geo.Toward(points[0].XZ(), target);
                c.Act.ArrowPreference = Geo.Heading(direction);
                if (c.Pose.Has(3694)) c.Act.Arrow(direction, leg.Departure);
                else c.Act.Walk(direction);
            }
        }
        else if (move.Kind == "ice")
        {
            var target = move.Path[^1].XZ();
            done = Vector2.Distance(c.Pose.XZ, target) < .15f;
            if (!done)
            {
                if (c.Now - moveAt > .5) { plan = null; return false; }
                c.Act.Walk(Geo.Toward(c.Pose.XZ, target));
                // Engine holds while the native timeline runs. Once it returns,
                // the actual endpoint decides whether the next slide can start.
                return false;
            }
        }
        else
        {
            done = passage!.Follow(c);
            if (move.Kind == "drop" && done && !c.Pose.Grounded) { c.Act.Hold(); return false; }
        }
        if (!done) return false;
        moveStarted = false; passage = null;
        if (++moveIndex == moves.Length) { ++legIndex; moves = []; }
        progressAt = c.Now;
        return true;
    }
    public override Vector2? Aim(PilotContext c) => passage?.Aim(c);

    private void Select(PilotContext c)
    {
        long started = Stopwatch.GetTimestamp();
        graph = data.Graph(speed, course == CourseId.Crystal && goal > 0);
        graph.UseWaitingSearch = course == CourseId.Crown;
        var connectors = data.Connectors(c.Pose, speed).ToArray();
        plan = graph.Search(c.Hazards, c.Pose.XZ, c.Now, true,
            sourceEdges: connectors, goals: course == CourseId.Ring ? RingGoals(c) :
                course == CourseId.Crystal && goal == 0 && returnToLower ? lowerReturnGoals :
                goal >= 0 ? new HashSet<int> { goal } : null,
            immobile: hadBomb ? new(bombExpires - BombLead, bombExpires + BombTail) : null,
            arrowHeading: c.Pose.Has(3694) ? c.Pose.Raw.ArrowDirection : null, arrowRate: c.Act.ArrowRate,
            // Crown spans entrance circles, hammers and the upper waves. In
            // Crown-034151 a valid 46.21 s route exceeded shortest + 12 s.
            windExecution: windExecution, waitHorizon: course == CourseId.Crown ? 45 : 12);
        if (course == CourseId.Crystal && plan.Found)
            for (int i = 1; i < plan.Legs.Length; ++i)
            {
                var previous = plan.Legs[i - 1]; var next = plan.Legs[i];
                double arrived = previous.Departure + previous.Edge.Duration;
                if (data.Nodes[next.Edge.From].WindBrace && next.Departure > arrived + .1)
                    c.Log("crystal-brace-plan", new { at = c.Now, node = next.Edge.From,
                        target = data.Nodes[next.Edge.From].Feet, arrived, depart = next.Departure, windExecution });
            }
        seenHit = c.LastHitAt; plannedRevision = c.Hazards.RouteRevision;
        nextRetry = 0;
        legIndex = moveIndex = 0; moves = []; moveStarted = false; passage = null;
        c.Log("course-ctc-plan", new { course, at = c.Now, start = c.Pose.Feet, goal, speed, returnToLower,
            arrowHeading = c.Pose.Raw.ArrowDirection, arrowRate = c.Act.ArrowRate,
            plan.Arrival, plan.Found, plan.Expanded, connectors = connectors.Length,
            planningMs = Stopwatch.GetElapsedTime(started).TotalMilliseconds,
            route = plan.Legs.Select(l => new { l.Edge.From, l.Edge.To, l.Departure, duration = l.Edge.Duration }).ToArray() });
    }
    private HashSet<int> RingGoals(PilotContext c)
    {
        var center = new Vector2(200, -125);
        float angle = Geo.Heading(c.Pose.XZ - center);
        float advance = MathF.PI / 3;
        float? frontGap = null;
        if (c.Hazards.RingWind is { } axis)
        {
            float heading = axis.Heading + axis.Rate * (float)(c.Now - axis.At);
            float gap = (heading - angle) * ringDirection % MathF.Tau;
            if (gap < 0) gap += MathF.Tau;
            frontGap = gap;
            // Chase the leading boundary while retaining angular clearance.
            // A short forward target follows its motion; CTC decides whether
            // the actual walk/jump can reach it without crossing the wind.
            advance = Math.Clamp(gap + Math.Abs(axis.Rate) * 2 - .45f, .35f, MathF.PI / 3);
        }
        float target = angle + ringDirection * advance;
        var goals = new HashSet<int>();
        int nearest = 0; float nearestError = float.PositiveInfinity;
        for (int i = 0; i < data.Nodes.Length; ++i)
        {
            float error = Math.Abs(Geo.WrapAngle(Geo.Heading(data.Nodes[i].Feet.XZ() - center) - target));
            if (error < nearestError) { nearestError = error; nearest = i; }
            if (error < .13f) goals.Add(i);
        }
        if (goals.Count == 0) goals.Add(nearest);
        c.Log("ring-target", new { at = c.Now, feet = c.Pose.Feet, angle, target, advance,
            direction = ringDirection, frontGap, wind = c.Hazards.RingWind, candidates = goals.Count });
        return goals;
    }
}
