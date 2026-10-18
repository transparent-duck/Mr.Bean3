using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed partial class CourseRouteStep
{
    private CourseRoutePlanner? background;
    private CourseRoutePlanner.Result? preparedRoute;
    private CourseRoutePlanner.Query? routeQuery;
    private HitTimedRoutes.Plan? retainedRoute;
    private RouteContinuation? continuation;
    private long requestClock, checkedRouteRevision, checkedPrefixRevision;
    private double requestHit, requestDelivery, resultSeconds;
    private bool requestReturn, replanWanted, sampledResult;
    private double requestAfter;
    private bool continuingExposure;

    private static double FrameSeconds(PilotContext c)
        => Math.Clamp(c.Pose.Raw.RealFrameSeconds ?? 1d / 60, 1d / 240, .1);

    private void CancelBackground()
    {
        background?.Cancel(); preparedRoute = null; routeQuery = null;
        retainedRoute = null; continuation = null; continuingExposure = false;
    }

    private void BeginBackground(PilotContext c, bool retain, double? readyAt = null)
    {
        if (background == null) return;
        if (course == CourseId.Hit && march.Pending) { replanWanted = true; return; }
        if (routeQuery != null && requestHit == c.LastHitAt && requestDelivery == c.LastDeliveryAt &&
            requestReturn == returnToLower && routeQuery.Speed == speed &&
            (retainedRoute == plan || !retain && retainedRoute == null && plan == null)) return;
        CancelBackground();
        double frame = FrameSeconds(c);
        double lead = c.PlanningTimes.Lead(course, !background.Warm, frame);
        lead = Math.Ceiling(lead / frame) * frame;
        double at = Math.Max(c.Now + lead, readyAt ?? c.Now);
        RouteContinuation? forecast = null;
        if (retain && plan is { Found: true })
            forecast = RouteContinuation.Forecast(data, plan, legIndex, moveIndex, moveStarted, moveAt,
                c.Pose, c.Now, at - c.Now, speed, c.Act.ArrowRate);
        // A worker deadline cannot authorize walking into an already known
        // hazard. If there is no safe continuation, search from the real feet.
        if (forecast != null && graph.HasConflict(c.Hazards, forecast.Prefix, 0, c.Pose.XZ, c.Now))
        {
            c.Log("course-background-prefix-unsafe", new { at = c.Now, handoff = forecast.At, feet = c.Pose.Feet });
            // Stopping is an alternative only if the actual motion can stop
            // and this position stays clear until the worker can take over.
            var hold = new HitTimedRoutes.Plan([new(new(0, 0,
                [new(c.Pose.XZ, 0), new(c.Pose.XZ, at - c.Now)]), c.Now)], at, 0, true);
            if (CanStop(c.Pose) && !graph.HasConflict(c.Hazards, hold, 0, c.Pose.XZ, c.Now)) forecast = null;
            else continuingExposure = true;
        }
        if (forecast == null)
        {
            // Do not interrupt an airborne/native action to manufacture a
            // stationary start. Retry at its controllable boundary instead.
            if (!CanStop(c.Pose))
            { replanWanted = true; return; }
            plan = null; moves = []; moveStarted = false; passage = null;
            c.Act.ClearJump();
        }
        continuation = forecast; retainedRoute = plan;
        if (sections != null && !requestedForecast) sections.Cancel();
        var pose = forecast?.Pose ?? c.Pose;
        at = forecast?.At ?? at;
        float? arrow = forecast?.ArrowHeading ?? (pose.Has(3694) && pose.Raw.ArrowDirection is { } heading
            ? Geo.WrapAngle(heading + c.Act.ArrowRate * (float)(at - c.Now)) : null);
        var goals = course == CourseId.Ring ? RingGoals(c) :
            course == CourseId.Crystal && goal == 0 && returnToLower ? lowerReturnGoals :
            goal >= 0 ? new HashSet<int> { goal } : null;
        routeQuery = new(pose, at, speed, course == CourseId.Crystal && goal > 0,
            goals == null ? null : new(goals), hadBomb ? new(bombExpires - BombLead, bombExpires + BombTail) : null,
            arrow, c.Act.ArrowRate, windExecution, course == CourseId.Crown ? 45 : 12,
            sections != null ? sectionIndex : null, forecast?.Connector);
        requestHit = c.LastHitAt; requestDelivery = c.LastDeliveryAt; requestReturn = returnToLower;
        requestClock = Stopwatch.GetTimestamp(); sampledResult = false; resultSeconds = 0;
        ++SearchNumber;
        background.Start(routeQuery, c.Hazards.Snapshot(c.Now));
        replanWanted = true; nextRetry = 0;
        seenHit = c.LastHitAt;
        checkedPrefixRevision = checkedRouteRevision = plannedRevision = c.Hazards.RouteRevision;
        c.Log("course-background-start", new { at = c.Now, course, handoff = at, lead = at - c.Now,
            start = pose.Feet, retained = retainedRoute != null, continuingExposure, revision = checkedRouteRevision });
    }

    private static bool CanStop(PilotPose pose) => pose.Grounded && !pose.Locked && pose.Force == Vector2.Zero &&
        pose.Motion is not (MotionKind.Ice or MotionKind.Forced or MotionKind.Steering);

    private void RejectBackground(PilotContext c, string reason)
    {
        c.Log("course-background-reject", new { at = c.Now, course, reason, handoff = routeQuery?.At,
            feet = c.Pose.Feet, planned = routeQuery?.Pose.Feet });
        CancelBackground(); replanWanted = true; requestAfter = c.Now;
    }

    private void PollBackground(PilotContext c, bool commit)
    {
        if (background == null || routeQuery == null) return;
        if (requestHit != c.LastHitAt || requestDelivery != c.LastDeliveryAt || requestReturn != returnToLower ||
            goal != Goal(c.Pose) || retainedRoute != plan || hadBomb != c.Pose.Has(3802) ||
            routeQuery.Pose.Has(3693) != c.Pose.Has(3693) || routeQuery.Pose.Has(3694) != c.Pose.Has(3694))
        { RejectBackground(c, "state-changed"); return; }
        if (course == CourseId.Hit && march.Pending)
        { RejectBackground(c, "forced-motion"); return; }
        if (continuation != null && !continuation.Matches(c.Pose, Math.Min(c.Now, continuation.At)))
        { RejectBackground(c, "motion-changed"); return; }
        // This check also runs while the worker is busy. Waiting for its result
        // before checking a newly dangerous old prefix is too late.
        if (continuation != null && checkedPrefixRevision != c.Hazards.RouteRevision)
        {
            checkedPrefixRevision = c.Hazards.RouteRevision;
            bool conflict = graph.HasConflict(c.Hazards, continuation.Prefix, 0, c.Pose.XZ, c.Now);
            if (conflict && !continuingExposure)
            {
                RejectBackground(c, "retained-prefix-changed");
                if (CanStop(c.Pose))
                { plan = null; moves = []; moveStarted = false; passage = null; }
                return;
            }
        }
        if (background.TryTake(out var result) && result != null)
        {
            preparedRoute = result;
            resultSeconds = Stopwatch.GetElapsedTime(requestClock).TotalSeconds;
            c.Log("course-background-result", new { at = c.Now, course, result.Milliseconds,
                elapsedMs = resultSeconds * 1000, result.Cold, result.Connectors, result.Plan.Found, result.Plan.Expanded, result.Error });
            if (!result.Plan.Found)
            {
                c.PlanningTimes.Observe(course, result.Cold, resultSeconds);
                RejectBackground(c, "no-route");
                requestAfter = c.Now + .1;
                return;
            }
            // First validation checks all stations, not only revisions since
            // launch. Published edges do not carry the worker's contact cache.
            checkedRouteRevision = 0;
        }
        if (c.Now > routeQuery.At + Math.Max(.04, FrameSeconds(c)) ||
            preparedRoute == null && c.Now + FrameSeconds(c) * .5 >= routeQuery.At)
        {
            if (preparedRoute == null)
                c.PlanningTimes.Observe(course, !background.Warm, Stopwatch.GetElapsedTime(requestClock).TotalSeconds);
            else if (!sampledResult)
                c.PlanningTimes.Observe(course, preparedRoute.Cold, resultSeconds);
            RejectBackground(c, "missed-handoff"); return;
        }
        if (preparedRoute is not { } ready) return;
        if (checkedRouteRevision != c.Hazards.RouteRevision || !sampledResult)
        {
            long began = Stopwatch.GetTimestamp();
            bool conflict = graph.HasConflict(c.Hazards, ready.Plan, 0, routeQuery.Pose.XZ, routeQuery.At, checkedRouteRevision);
            if (!conflict && continuation != null && !continuingExposure)
                conflict = graph.HasConflict(c.Hazards, continuation.Prefix, 0, c.Pose.XZ, c.Now);
            double validation = Stopwatch.GetElapsedTime(began).TotalSeconds;
            if (!sampledResult)
            {
                c.PlanningTimes.Observe(course, ready.Cold, resultSeconds + validation);
                sampledResult = true;
            }
            checkedRouteRevision = c.Hazards.RouteRevision;
            if (conflict) { RejectBackground(c, "new-hazard"); return; }
            if (c.Now + validation + FrameSeconds(c) > routeQuery.At)
            { RejectBackground(c, "validation-deadline"); return; }
        }
        if (!commit || c.Now + FrameSeconds(c) * .5 < routeQuery.At ||
            !c.Pose.Grounded || c.Pose.Locked || c.Pose.Motion is MotionKind.Ice or MotionKind.Forced) return;
        if (Vector2.DistanceSquared(c.Pose.XZ, routeQuery.Pose.XZ) > .15f * .15f ||
            Math.Abs(c.Pose.Feet.Y - routeQuery.Pose.Feet.Y) > .4f)
        { RejectBackground(c, "position-mismatch"); return; }
        if (routeQuery.ArrowHeading is { } expected && c.Pose.Raw.ArrowDirection is { } actual &&
            Math.Abs(Geo.WrapAngle(actual - expected)) > Math.Max(.1, Math.Abs(routeQuery.ArrowRate) * FrameSeconds(c)))
        { RejectBackground(c, "arrow-phase-mismatch"); return; }
        plan = ready.Plan;
        if (sections != null && !requestedForecast) sections.Cancel();
        legIndex = moveIndex = 0; moves = []; passage = null; moveStarted = false;
        seenHit = c.LastHitAt; plannedRevision = c.Hazards.RouteRevision;
        progressAt = c.Now; progressPoint = c.Pose.XZ; nextRetry = 0;
        c.Log("course-ctc-plan", new { course, at = c.Now, selectedAt = routeQuery.At,
            start = routeQuery.Pose.Feet, actual = c.Pose.Feet, goal, speed, returnToLower,
            arrowHeading = routeQuery.ArrowHeading, arrowRate = routeQuery.ArrowRate,
            plan.Arrival, plan.Found, plan.Expanded, connectors = ready.Connectors,
            planningMs = ready.Milliseconds, background = true,
            route = plan.Legs.Select(l => new { l.Edge.From, l.Edge.To, l.Departure, duration = l.Edge.Duration }).ToArray() });
        CancelBackground(); replanWanted = false;
    }

    private void RequestBackground(PilotContext c)
    {
        replanWanted = true;
        if (c.Now >= requestAfter) BeginBackground(c, true);
    }
}
