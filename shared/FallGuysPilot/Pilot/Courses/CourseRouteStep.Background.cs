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
    private bool requestReturn, requestBeforeRelease, replanWanted, sampledResult;
    private double requestAfter;
    private bool continuingExposure;
    private bool startingCrown, preparedCrownStart;
    // A committed native entry already owns movement before its status packet
    // arrives. Replacing or cancelling a search cannot turn that into a hold.
    private bool PendingHitMotion(PilotPose pose) => execution.PendingGain(pose);

    public override void PrepareStart(PilotContext c)
    {
        if (!startingCrown || !c.Pose.Has(3782)) return;
        progressAt = c.Now; progressPoint = c.Pose.XZ;
        UpdateCrownTiming(c);
        if (!preparedCrownStart && double.IsFinite(c.ExpectedReleaseAt) && CanStop(c.Pose))
        {
            preparedCrownStart = true;
            BeginBackground(c, false, c.ExpectedReleaseAt);
        }
        PollBackground(c, false);
    }

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
        if (PendingHitMotion(c.Pose))
        {
            // The gain time is not known yet. Keep the committed controller
            // until the observed native phase supplies a valid moving root.
            replanWanted = true; return;
        }
        if (routeQuery != null && requestHit == c.LastHitAt && requestDelivery == c.LastDeliveryAt &&
            requestReturn == returnToLower && routeQuery.Speed == speed &&
            (retainedRoute == plan || !retain && retainedRoute == null && plan == null)) return;
        CancelBackground();
        double frame = FrameSeconds(c);
        double lead = c.PlanningTimes.Lead(course, !background.Warm, frame);
        lead = Math.Ceiling(lead / frame) * frame;
        // A stationary start has no moving prefix to meet at a future time.
        // Rebase its result when usable; do not reserve another worker window.
        double at = Math.Max(c.Now + (startingCrown ? 0 : lead), readyAt ?? c.Now);
        RouteContinuation? forecast = null;
        if (retain && plan is { Found: true })
        {
            forecast = RouteContinuation.Forecast(plan, execution, c.Pose, c.Now, at - c.Now, MotionOptions);
            // An unobserved native boundary is not a stationary search root.
            // Keep the controller that owns the approach and retry on receipt.
            if (forecast == null) { replanWanted = true; return; }
        }
        // A worker deadline cannot authorize walking into an already known
        // hazard. If there is no safe continuation, search from the real feet.
        if (forecast != null && graph.HasConflict(c.Hazards, forecast.Prefix, 0, c.Pose.XZ, c.Now))
        {
            c.Log("course-background-prefix-unsafe", new { at = c.Now, handoff = forecast.At, feet = c.Pose.Feet });
            if (course == CourseId.Hit)
            {
                // Find a handoff before the first contact, using the same
                // executed prefix. A slow historical query cannot reserve an
                // unsafe several-second approach to a safe replacement.
                double safe = 0, unsafeAt = at - c.Now;
                for (int i = 0; i < 8 && unsafeAt - safe > frame; ++i)
                {
                    double span = (safe + unsafeAt) * .5;
                    var prefix = RouteContinuation.Forecast(plan!, execution, c.Pose, c.Now, span, MotionOptions);
                    if (prefix == null || graph.HasConflict(c.Hazards, prefix.Prefix, 0, c.Pose.XZ, c.Now)) unsafeAt = span;
                    else safe = span;
                }
                double earlier = Math.Floor((safe - 2 * frame) / frame) * frame;
                if (earlier >= frame)
                {
                    var shortened = RouteContinuation.Forecast(plan!, execution, c.Pose, c.Now, earlier, MotionOptions);
                    if (shortened != null)
                    {
                        forecast = shortened; at = forecast.At;
                        c.Log("course-background-deadline", new { at = c.Now, handoff = at, contactWithin = unsafeAt });
                    }
                }
            }
            // Stopping is an alternative only if the actual motion can stop
            // and this position stays clear until the worker can take over.
            var hold = new HitTimedRoutes.Plan([], at, 0, true);
            if (CanStop(c.Pose) && !graph.HasConflict(c.Hazards, hold, 0, c.Pose.XZ, c.Now)) forecast = null;
            else continuingExposure = forecast != null && graph.HasConflict(c.Hazards, forecast.Prefix, 0, c.Pose.XZ, c.Now);
        }
        if (forecast == null)
        {
            // Do not interrupt an airborne/native action to manufacture a
            // stationary start. Retry at its controllable boundary instead.
            if (!CanStop(c.Pose))
            { replanWanted = true; return; }
            plan = null; moves = []; moveStarted = false; execution.ClearMove();
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
            sections != null ? sectionIndex : null, forecast?.Connector, c.Act.ArrowEntryHeading);
        requestHit = c.LastHitAt; requestDelivery = c.LastDeliveryAt; requestReturn = returnToLower;
        requestBeforeRelease = !double.IsFinite(c.ReleasedAt);
        requestClock = Stopwatch.GetTimestamp(); sampledResult = false; resultSeconds = 0;
        ++SearchNumber;
        background.Start(routeQuery, c.Hazards.Snapshot(c.Now));
        replanWanted = true; nextRetry = 0;
        seenHit = c.LastHitAt;
        checkedPrefixRevision = checkedRouteRevision = plannedRevision = c.Hazards.RouteRevision;
        c.Log("course-background-start", new { at = c.Now, course, handoff = at, lead = at - c.Now,
            start = pose.Feet, retained = retainedRoute != null, continuingExposure,
            beforeRelease = requestBeforeRelease, revision = checkedRouteRevision });
    }

    private bool CanStop(PilotPose pose) => execution.CanStop(pose, MotionOptions);

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
        // A stationary query becomes stale as soon as the feet move or the
        // terrain/status no longer permits a stop, even while the worker runs.
        if (continuation == null && (!CanStop(c.Pose) ||
            Vector2.DistanceSquared(c.Pose.XZ, routeQuery.Pose.XZ) > .15f * .15f ||
            Math.Abs(c.Pose.Feet.Y - routeQuery.Pose.Feet.Y) > .4f))
        { RejectBackground(c, "stationary-start-changed"); return; }
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
                { plan = null; moves = []; moveStarted = false; execution.ClearMove(); }
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
                if (startingCrown && !double.IsFinite(c.ReleasedAt)) return;
                FailedRoute(c);
                c.PlanningTimes.Observe(course, result.Cold, resultSeconds);
                RejectBackground(c, "no-route");
                requestAfter = c.Now + .1;
                return;
            }
            // First validation checks all stations, not only revisions since
            // launch. Published edges do not carry the worker's contact cache.
            checkedRouteRevision = 0;
            if (startingCrown && !double.IsFinite(c.ReleasedAt))
            {
                // Warm the published path's contact cache while locked. New
                // observations are checked at release, without repeated searches.
                RouteSchedule.HasConflict(graph, c.Hazards, result.Plan, routeQuery.Pose, routeQuery.At, MotionOptions);
                return;
            }
        }
        if (startingCrown)
        {
            if (!double.IsFinite(c.ReleasedAt) || !commit || preparedRoute == null) return;
            if (!AlignCrownStart(c)) return;
        }
        else if (continuation == null)
        {
            if (preparedRoute == null || !commit) return;
            if (!AlignStationaryStart(c)) return;
        }
        else
        {
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
                bool conflict = RouteSchedule.HasConflict(graph, c.Hazards, ready.Plan,
                    routeQuery.Pose, routeQuery.At, MotionOptions, checkedRouteRevision);
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
                // Validation can finish in the adoption frame. Reserving one
                // more frame here rejects an already ready, checked result
                // whenever a hazard observation arrives just before handoff.
                if (c.Now + validation > routeQuery.At + FrameSeconds(c) * .5)
                { RejectBackground(c, "validation-deadline"); return; }
            }
        }
        var accepted = preparedRoute!;
        if (!commit || c.Now + FrameSeconds(c) * .5 < routeQuery.At ||
            !c.Pose.Grounded || c.Pose.Locked || c.Pose.Motion is MotionKind.Ice or MotionKind.Forced) return;
        if (Vector2.DistanceSquared(c.Pose.XZ, routeQuery.Pose.XZ) > .15f * .15f ||
            Math.Abs(c.Pose.Feet.Y - routeQuery.Pose.Feet.Y) > .4f)
        { RejectBackground(c, "position-mismatch"); return; }
        if (routeQuery.ArrowHeading is { } expected && c.Pose.Raw.ArrowDirection is { } actual &&
            Math.Abs(Geo.WrapAngle(actual - expected)) > Math.Max(.1, Math.Abs(routeQuery.ArrowRate) * FrameSeconds(c)))
        { RejectBackground(c, "arrow-phase-mismatch"); return; }
        plan = accepted.Plan;
        noRouteSince = double.NaN;
        startingCrown = false;
        if (sections != null && !requestedForecast) sections.Cancel();
        legIndex = moveIndex = 0; moves = []; execution.ClearMove(); moveStarted = false;
        seenHit = c.LastHitAt; plannedRevision = c.Hazards.RouteRevision;
        progressAt = c.Now; progressPoint = c.Pose.XZ; nextRetry = 0;
        c.Log("course-ctc-plan", new { course, at = c.Now, selectedAt = routeQuery.At,
            start = routeQuery.Pose.Feet, actual = c.Pose.Feet, goal, speed, returnToLower,
            arrowHeading = routeQuery.ArrowHeading, arrowRate = routeQuery.ArrowRate,
            plan.Arrival, plan.Found, plan.Expanded, connectors = accepted.Connectors,
            planningMs = accepted.Milliseconds, background = true,
            route = plan.Legs.Select(l => new { l.Edge.From, l.Edge.To, l.Departure, duration = l.Edge.Duration }).ToArray() });
        CancelBackground(); replanWanted = false;
    }

    private bool AlignStationaryStart(PilotContext c)
    {
        var ready = preparedRoute!;
        long began = Stopwatch.GetTimestamp();
        float? arrow = c.Pose.Has(3694) ? c.Pose.Raw.ArrowDirection : null;
        // Only retime this published path. Its legal waits, bomb interval and
        // arrow phase still apply; moving roots retain their exact handoff.
        var aligned = graph.Reschedule(c.Hazards, ready.Plan, 0, c.Pose.XZ, c.Now, true,
            immobile: routeQuery!.Immobile, arrowHeading: arrow, arrowRate: routeQuery.ArrowRate,
            windExecution: routeQuery.WindExecution, waitHorizon: routeQuery.WaitHorizon, entryArrowHeading: routeQuery.EntryArrowHeading);
        aligned = RouteSchedule.Retime(graph, c.Hazards, aligned, default, c.Pose, c.Now, MotionOptions,
            routeQuery.WindExecution, routeQuery.WaitHorizon) ?? aligned;
        bool conflict = !aligned.Found || RouteSchedule.HasConflict(graph, c.Hazards, aligned, c.Pose, c.Now, MotionOptions);
        // A failed earlier start does not invalidate a future schedule. Adopt
        // that schedule with its departure waits instead of searching again.
        if (conflict && routeQuery.At > c.Now &&
            !RouteSchedule.HasConflict(graph, c.Hazards, ready.Plan, c.Pose, c.Now, MotionOptions))
        {
            aligned = ready.Plan;
            conflict = false;
        }
        double seconds = Stopwatch.GetElapsedTime(began).TotalSeconds;
        if (!sampledResult)
        {
            c.PlanningTimes.Observe(course, ready.Cold, resultSeconds + seconds);
            sampledResult = true;
        }
        c.Log("course-stationary-align", new { at = c.Now, saved = routeQuery.At - c.Now,
            seconds, conflict, aligned.Arrival });
        if (conflict) { RejectBackground(c, "stationary-hazard"); return false; }
        preparedRoute = ready with { Plan = aligned };
        routeQuery = routeQuery with { At = c.Now, ArrowHeading = arrow };
        checkedRouteRevision = c.Hazards.RouteRevision;
        return true;
    }

    private bool AlignCrownStart(PilotContext c)
    {
        var ready = preparedRoute!;
        // A failed countdown query is already old; retry at release now.
        // Fresh post-release failures keep the normal retry backoff above.
        if (!ready.Plan.Found) { RejectBackground(c, "no-route"); return false; }
        long began = Stopwatch.GetTimestamp();
        double shift = c.Now - routeQuery!.At;
        var aligned = ready.Plan with
        {
            Legs = ready.Plan.Legs.Select(l => l with { Departure = l.Departure + shift }).ToArray(),
            Arrival = ready.Plan.Arrival + shift,
        };
        aligned = RouteSchedule.Retime(graph, c.Hazards, aligned, default, c.Pose, c.Now, MotionOptions,
            routeQuery.WindExecution, routeQuery.WaitHorizon) ?? aligned;
        bool conflict = RouteSchedule.HasConflict(graph, c.Hazards, aligned, c.Pose, c.Now, MotionOptions);
        double seconds = Stopwatch.GetElapsedTime(began).TotalSeconds;
        c.PlanningTimes.Observe(course, ready.Cold, ready.Milliseconds / 1000 + seconds);
        c.Log("course-start-align", new { at = c.Now, shift, seconds, conflict,
            prepared = requestBeforeRelease, revision = c.Hazards.RouteRevision });
        if (conflict) { RejectBackground(c, "start-hazard"); return false; }
        preparedRoute = ready with { Plan = aligned };
        routeQuery = routeQuery with { At = c.Now };
        checkedRouteRevision = c.Hazards.RouteRevision;
        return true;
    }

    private void RequestBackground(PilotContext c)
    {
        replanWanted = true;
        if (c.Now >= requestAfter) BeginBackground(c, true);
    }
}
