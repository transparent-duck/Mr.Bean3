using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed class KnightCourse : GraphCourse { public override CourseId Course => CourseId.Knight; }
internal sealed class RingCourse : GraphCourse { public override CourseId Course => CourseId.Ring; }
internal sealed class CrystalCourse : GraphCourse { public override CourseId Course => CourseId.Crystal; }
internal sealed class CrownCourse(bool stairBoost = false) : GraphCourse
{
    public override CourseId Course => CourseId.Crown;
    protected override bool CrownStairBoost => stairBoost;
}

internal abstract class GraphCourse : CourseScript
{
    public override HazardSpec[] Hazards => CourseHazards.Specs(Course);
    protected virtual bool CrownStairBoost => false;
    public override Step[] Build(PilotPose start) => [new CourseRouteStep(Course, CrownStairBoost)
        { Name = Course.ToString().ToLowerInvariant() + "-ctc", Zone = new(-1000, 1000, -1000, 1000) }];
}

// Each search starts at the actual feet and ends at the current course goal.
// Receipts do not replace an executing race route. Hits, displaced feet and
// changed Crystal carry state relocalize; Ring follows the observed wind angle.
internal sealed partial class CourseRouteStep(CourseId course, bool crownStairBoost = false) : Step, IDisposable
{
    private PilotCourseGeometry data = PilotCourseGeometry.Load(course.ToString().ToLowerInvariant(), crownStairBoost);
    private Dictionary<int, PilotCourseGeometry>? crownProfiles;
    internal int CrownFrameRate { get; private set; } = 30;
    private double crownFrameSeconds = 1d / 30;
    private int pendingCrownRate;
    private double crownRateSince;
    private HitTimedRoutes graph = null!;
    private HitTimedRoutes.Plan? activePlan;
    private HitTimedRoutes.Plan? plan
    {
        get => activePlan;
        set
        {
            if (value is { Found: true } && !ReferenceEquals(value, activePlan)) ++RouteNumber;
            activePlan = value;
            execution = default;
        }
    }
    internal HitTimedRoutes.Plan? ActivePlan => plan;
    internal long RouteNumber { get; private set; }
    internal long SearchNumber { get; private set; }
    internal int ActiveLeg => legIndex;
    internal int ActiveMove => moveIndex;
    internal bool Searching => sections?.Pending == true || background?.Pending == true;
    internal bool AwaitingHandoff => routeQuery != null && preparedRoute != null;
    internal bool Forecasting => sections?.Pending == true && requestedForecast;
    internal double WaitUntil => plan is { Found: true } && legIndex < plan.Legs.Length && !moveStarted
        ? plan.Legs[legIndex].Departure : 0;
    internal string SectionName => sections == null ? course.ToString() : data.Sections[sectionIndex].Name;
    private RouteExecution execution;
    private PilotCourseGeometry.Move[] moves { get => execution.Moves ?? []; set => execution.Moves = value; }
    private int legIndex { get => execution.Leg; set => execution.Leg = value; }
    private int moveIndex { get => execution.Move; set => execution.Move = value; }
    private int goal = -1;
    private double seenHit, progressAt, nextRetry;
    private RouteExecution.Options MotionOptions => new(data, course, speed,
        arrowRate, hadBomb ? new(bombExpires - BombLead, bombExpires + BombTail) : null);
    private float arrowRate = -MathF.Tau;
    private Vector2 progressPoint;
    private bool moveStarted { get => execution.Started; set => execution.Started = value; }
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
    private float ringGoalHeading = float.NaN;
    private RingGoalSelection? ringGoals;
    private double seenDelivery = double.NegativeInfinity;
    private bool returnToLower;
    private HashSet<int>? lowerReturnGoals;
    private readonly HitForcedMarch march = new();
    private bool nativePaused;
    private double nativeReadyAt = double.NaN;
    private Vector2 nativeReadyFeet;

    public override bool ObserveMotion(PilotContext c)
    {
        arrowRate = c.Act.ArrowRate;
        if (plan is { Found: true } && (c.Pose.Motion is MotionKind.Ice or MotionKind.Forced || c.Pose.Has(3737)))
            execution.Advance(plan, c.Pose, c.Now, MotionOptions);
        if (ObserveIceEntry(c)) return true;
        UpdateCrownTiming(c);
        PollSections(c);
        PollBackground(c, false);
        if (course == CourseId.Hit)
        {
            c.Water.Observe(c);
            bool wasForced = march.Native;
            if (march.Observe(c))
            {
                if (wasForced && !march.Native)
                {
                    if (seenHit == c.LastHitAt && march.Matches(plan, c.Pose.XZ) && Math.Abs(c.Now - march.ReadyAt) <= .12)
                    {
                        moves = (PilotCourseGeometry.Move[])plan!.Legs[0].Edge.Motion!;
                        legIndex = 0;
                        moveIndex = Array.FindIndex(moves, m => m.Kind == "forced") + 1;
                        if (moveIndex >= moves.Length) { legIndex = 1; moves = []; moveIndex = 0; }
                        moveStarted = false; execution.ClearMove();
                        seenHit = c.LastHitAt;
                        if (plannedRevision != c.Hazards.RouteRevision &&
                            graph.HasConflict(c.Hazards, plan, legIndex, c.Pose.XZ, c.Now, plannedRevision)) plan = null;
                    }
                    else plan = null;
                    c.Log("hit-forced-release", new { at = c.Now, feet = c.Pose.Feet, reused = plan != null });
                }
                else if (march.Native)
                {
                    if (seenHit != c.LastHitAt || !march.Matches(plan)) { plan = null; Select(c); }
                }
                // A warning changes a future motion; it must not cancel a jump
                // already in flight. Observe reports the landing boundary again.
                else if (c.Pose.Grounded && !march.IncludesWarning(plan)) plan = null;
            }
            if (march.Native)
            {
                if (plan is { Found: true } && plannedRevision != c.Hazards.RouteRevision)
                {
                    long previous = plannedRevision;
                    plannedRevision = c.Hazards.RouteRevision;
                    if (graph.HasConflict(c.Hazards, plan, legIndex, c.Pose.XZ, c.Now, previous)) Select(c);
                }
                return false;
            }
            if (c.Pose.Locked)
            {
                if (c.Pose.Raw.SteeringClock is { CurrentTimelineId: 8514,
                        CurrentTimestamp: { } ticks, ActualSchedulerSpeed: { } rate } && rate > .001f)
                {
                    double ready = c.Now + Math.Max(0, 65 - ticks) / (30 * rate);
                    if (!nativePaused || !double.IsFinite(nativeReadyAt) || Math.Abs(ready - nativeReadyAt) > .12 ||
                        Vector2.DistanceSquared(c.Pose.XZ, nativeReadyFeet) > .01f)
                    {
                        nativeReadyAt = ready; nativeReadyFeet = c.Pose.XZ;
                        if (!TrackRecovery(c)) Select(c, ready);
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
                if (!changed && plan is { Found: true } && plannedRevision != c.Hazards.RouteRevision)
                    changed = graph.HasConflict(c.Hazards, plan, legIndex, c.Pose.XZ, c.Now, plannedRevision);
                seenHit = c.LastHitAt;
                if (changed) plan = null;
            }
            PrepareNextSection(c);
        }
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
        c.Act.ArrowEntryHeading = course == CourseId.Knight ? Random.Shared.NextSingle() * MathF.Tau - MathF.PI : null;
        if (c.Act.ArrowEntryHeading is { } entry)
            c.Log("knight-arrow-entry-heading", new { at = c.Now, heading = entry });
        startingCrown = course == CourseId.Crown && !double.IsFinite(c.ReleasedAt);
        preparedCrownStart = false;
        CourseHazards.Install(course, c.Hazards);
        if (course == CourseId.Hit) c.Water.Attach(c.Hazards);
        c.Act.ImmediateJump = course == CourseId.Ring;
        if (course == CourseId.Crown && crownStairBoost)
        {
            crownProfiles ??= new() { [30] = data };
            foreach (int fps in new[] { 60 })
                if (!crownProfiles.ContainsKey(fps)) crownProfiles[fps] = PilotCourseGeometry.Load("crown", true, fps);
            crownFrameSeconds = c.Pose.Raw.RealFrameSeconds ?? 1d / 30;
            CrownFrameRate = PilotCourseGeometry.CrownFrameRate(crownFrameSeconds);
            data = crownProfiles[CrownFrameRate];
            c.Log("crown-speed-profile", new { at = c.Now, fps = CrownFrameRate, frameSeconds = crownFrameSeconds });
        }
        graph = data.Graph(speed); seenHit = c.LastHitAt; plan = null;
        sections?.Dispose();
        if (data.Sections.Length > 0) sections = new(data);
        background?.Dispose();
        background = new(course, crownStairBoost, CrownFrameRate);
        CancelBackground(); replanWanted = false; requestAfter = 0;
        if (course == CourseId.Crystal)
            lowerReturnGoals = Enumerable.Range(0, data.Nodes.Length)
                .Where(i => data.Nodes[i].ReturnFloor).ToHashSet();
        progressAt = c.Now; progressPoint = c.Pose.XZ;
        wasOnIce = c.Pose.Has(3693);
    }
    private void UpdateCrownTiming(PilotContext c)
    {
        if (crownProfiles == null) return;
        crownFrameSeconds += ((c.Pose.Raw.RealFrameSeconds ?? crownFrameSeconds) - crownFrameSeconds) * .125;
        int fps = PilotCourseGeometry.CrownFrameRate(crownFrameSeconds);
        if (fps != pendingCrownRate) { pendingCrownRate = fps; crownRateSince = c.Now; }
        if (fps == CrownFrameRate || c.Now - crownRateSince < .5 || !c.Pose.Grounded || c.Pose.Locked ||
            moveStarted && moveIndex < moves.Length && moves[moveIndex].Kind == "jump") return;
        // Change only at a controllable boundary, keeping the worker and live
        // graph on the same baked native-startup profile. Never interrupt flight.
        CancelBackground(); background?.Dispose();
        if (startingCrown) preparedCrownStart = false;
        CrownFrameRate = fps; data = crownProfiles[fps]; graph = data.Graph(speed);
        graph.UseWaitingSearch = true; background = new(course, true, fps);
        plan = null; moves = []; moveStarted = false; execution.ClearMove();
        c.Log("crown-speed-profile", new { at = c.Now, fps, frameSeconds = crownFrameSeconds });
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
        PollSections(c);
        bool onIce = c.Pose.Has(3693);
        if (course == CourseId.Knight && c.Pose.Grounded && onIce != wasOnIce)
        {
            c.Log("course-motion-boundary", new { at = c.Now, ice = onIce, feet = c.Pose.Feet });
            clockDirty = true;
        }
        if (c.Pose.Grounded) wasOnIce = onIce;
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
                plan = null; sections?.Cancel(); preparedSection = null;
                hadBomb = bomb; bombHolding = false;
                bombExpires = double.NaN;
                if (bomb)
                    foreach (var status in c.Pose.Raw.Statuses)
                        if (status.Id == 3802) { bombExpires = c.Now + Math.Max(0, status.Remaining); break; }
                c.Log("course-bomb", new { at = c.Now, active = bomb, expires = double.IsFinite(bombExpires) ? bombExpires : (double?)null });
            }
            if (RouteExecution.BombWait(c.Pose, c.Now, MotionOptions) is { } bombWait)
            {
                if (!bombHolding) c.Log("course-bomb-hold", new { at = c.Now, expires = bombExpires, feet = c.Pose.Feet });
                bombHolding = true; bombWait.Apply(c); progressAt = c.Now; return false;
            }
            foreach (var status in c.Pose.Raw.Statuses)
                if (status.Id == 3694 && status.Param != 0) c.Act.ArrowRate = -unchecked((short)status.Param) * MathF.PI / 180;
        }
        int currentGoal = Goal(c.Pose);
        arrowRate = c.Act.ArrowRate;
        execution.ArrowLatched = c.Act.ArrowLatched;
        if (TickArrowRegions(c)) return false;
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
        if (course == CourseId.Ring && c.Hazards.RingWind is { } wind &&
            float.IsFinite(ringGoalHeading) && Math.Abs(Geo.WrapAngle(wind.Heading - ringGoalHeading)) > .01f)
            RequestBackground(c);
        ObserveRouteClock(c);
        if (!clockOwnsValidation && plan is { Found: true } && c.Pose.Grounded &&
            plannedRevision != c.Hazards.RouteRevision)
        {
            long previousRevision = plannedRevision;
            plannedRevision = c.Hazards.RouteRevision;
            if (graph.HasConflict(c.Hazards, plan, legIndex, c.Pose.XZ, c.Now, previousRevision))
            {
                c.Log("course-new-danger", new { at = c.Now, course, feet = c.Pose.Feet });
                RequestBackground(c);
            }
        }
        float available = c.Pose.Raw.Walk?.AvailableTargetSpeed ?? 6;
        float actualSpeed = course == CourseId.Crystal && available > 6 ? MathF.Round(available, 2) : 6;
        bool displaced = !startingCrown && c.Now - progressAt > 2.5 && Vector2.Distance(progressPoint, c.Pose.XZ) < .1f;
        if (plan == null || c.Pose.Grounded && (seenHit != c.LastHitAt || goal != currentGoal || speed != actualSpeed || displaced))
        {
            if (c.Now < nextRetry && plannedRevision == c.Hazards.RouteRevision) { c.Act.Hold(); return false; }
            if (course == CourseId.Crystal && currentGoal != goal)
                c.Log("crystal-goal", new { at = c.Now, carry = currentGoal, previous = goal,
                    destination = currentGoal switch { 1 => 10060979, 2 => 10060978, 3 => 10060980, _ => 10060977 },
                    feet = c.Pose.Feet });
            if (seenHit != c.LastHitAt || goal != currentGoal || speed != actualSpeed || displaced)
            { CancelBackground(); plan = null; }
            goal = currentGoal; speed = actualSpeed; Select(c);
        }
        if (background != null)
        {
            PollBackground(c, true);
            if (replanWanted && routeQuery == null && c.Now >= requestAfter) BeginBackground(c, plan is { Found: true });
        }
        if (plan == null) { c.Act.Hold(); progressAt = c.Now; return false; }
        if (Vector2.Distance(progressPoint, c.Pose.XZ) > .1f) { progressPoint = c.Pose.XZ; progressAt = c.Now; }
        if (!plan!.Found)
        {
            c.Act.Hold(); plan = null; nextRetry = c.Now + .25;
            return false;
        }
        if (plan.Legs.Length == 0 || legIndex >= plan.Legs.Length)
        {
            if (sections != null && sectionIndex + 1 < data.Sections.Length)
                return AdvanceSection(c);
            if (course == CourseId.Ring && plan.Legs.Length > 0)
            { plan = null; return true; }
            c.Act.Hold(); progressAt = c.Now;
            return false;
        }
        PrepareNextSection(c);
        int previousLeg = legIndex, previousMove = moveIndex;
        arrowRate = c.Act.ArrowRate;
        var command = execution.Advance(plan, c.Pose, c.Now, MotionOptions);
        if (command.Started)
            c.Log("course-motion", new { course, at = c.Now, legIndex, moveIndex,
                Kind = moveIndex < moves.Length ? moves[moveIndex].Kind : "complete", feet = c.Pose.Feet });
        if (previousLeg != legIndex || previousMove != moveIndex)
        { clockDirty = true; progressAt = c.Now; }
        if (command.Relocalize) { plan = null; return true; }
        command.Apply(c);
        if (command.Kind is RouteExecution.Input.Hold or RouteExecution.Input.Face) progressAt = c.Now;
        return command.Complete && sections != null && sectionIndex + 1 < data.Sections.Length && AdvanceSection(c);
    }
    public override Vector2? Aim(PilotContext c) => execution.Aim;

    private void Select(PilotContext c, double? readyAt = null)
    {
        if (sections != null) { if (routeQuery == null) SelectSection(c, readyAt); return; }
        if (background != null)
        {
            graph = data.Graph(speed, course == CourseId.Crystal && goal > 0);
            graph.UseWaitingSearch = course == CourseId.Crown;
            if (routeQuery != null || c.Now >= requestAfter) BeginBackground(c, false, readyAt);
            return;
        }
    }
    private HashSet<int> RingGoals(PilotContext c)
    {
        var center = new Vector2(200, -125);
        float angle = Geo.Heading(c.Pose.XZ - center);
        float advance = MathF.PI / 3;
        float target = angle + ringDirection * advance;
        float? frontGap = null;
        if (c.Hazards.RingWind is { } axis)
        {
            ringGoalHeading = axis.Heading;
            float gap = (axis.Heading - angle) * ringDirection % MathF.Tau;
            if (gap < 0) gap += MathF.Tau;
            frontGap = gap;
            // Follow the latest received angle, without advancing it to a
            // guessed arrival time. Timed hazards still check the whole route.
            advance = Math.Min(gap - .45f, MathF.PI / 3);
            target = angle + ringDirection * advance;
        }
        var goals = (ringGoals ??= new(data)).Select(target);
        c.Log("ring-target", new { at = c.Now, feet = c.Pose.Feet, angle, target, advance,
            direction = ringDirection, frontGap, wind = c.Hazards.RingWind, candidates = goals.Count });
        return goals;
    }
}
