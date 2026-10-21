using System;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed partial class CourseRouteStep
{
    private HitTimedRoutes.Plan? clockPlan;
    private RouteContinuation? clockRoute;
    private Vector2 clockForce;
    private MotionKind clockMotion;
    private int clockStatuses;
    private long clockRevision;
    private double nextClockCheck;
    private bool clockDirty, clockOwnsValidation, clockDanger;

    private static int TimingStatuses(PilotPose pose) => (pose.Has(3693) ? 1 : 0) |
        (pose.Has(3694) ? 2 : 0) | (pose.Has(3702) ? 4 : 0) | (pose.Has(3703) ? 8 : 0);

    private void ObserveRouteClock(PilotContext c, bool force = false)
    {
        clockOwnsValidation = false;
        if (plan is not { Found: true } || legIndex >= plan.Legs.Length ||
            !c.Pose.Grounded || c.Pose.Locked || course == CourseId.Hit && march.Pending) return;
        clockOwnsValidation = true;
        int statuses = TimingStatuses(c.Pose);
        bool changed = clockForce != c.Pose.Force || clockMotion != c.Pose.Motion || clockStatuses != statuses;
        if (clockPlan != plan)
        {
            clockRoute = null; clockDirty = true; clockDanger = false;
            clockPlan = plan;
        }
        if (changed && routeQuery != null)
        {
            // A query rooted in the old movement state is stale. That says
            // nothing about whether the committed spatial route is unsafe.
            CancelBackground(); replanWanted = false;
        }
        if (routeQuery != null || !force && !changed && !clockDirty && c.Now < nextClockCheck) return;
        nextClockCheck = c.Now + .05;
        clockForce = c.Pose.Force; clockMotion = c.Pose.Motion; clockStatuses = statuses;
        bool retimed = force || changed || clockDirty || clockRoute == null ||
            c.Now >= clockRoute.At || !clockRoute.Matches(c.Pose, c.Now);
        if (retimed)
        {
            clockRoute = RouteContinuation.Retime(plan, execution, c.Pose, c.Now, MotionOptions);
            clockDirty = false; clockRevision = 0;
            if (clockRoute?.Schedule is { } schedule)
            {
                var legs = (HitTimedRoutes.Leg[])plan.Legs.Clone();
                for (int i = legIndex; i < legs.Length; ++i)
                    if (double.IsFinite(schedule.Departures[i])) legs[i] = legs[i] with { Departure = schedule.Departures[i] };
                // Timing changes do not reset the controller, its cursor, the
                // current native action, or the displayed spatial route id.
                activePlan = plan with { Legs = legs, Arrival = schedule.Complete ? clockRoute.At : plan.Arrival };
            }
            clockPlan = plan;
        }
        if (clockRoute == null || clockRoute.At <= c.Now) return;
        if (!retimed && clockRevision == c.Hazards.RouteRevision) return;
        // An incremental check can establish a new conflict, but cannot clear
        // an existing one: the station responsible may not have changed. Keep
        // retrying until the complete remaining trajectory is actually clear.
        bool danger = graph.HasConflict(c.Hazards, clockRoute.Prefix, 0, c.Pose.XZ, c.Now,
            clockDanger ? 0 : clockRevision);
        clockDanger = danger;
        clockRevision = plannedRevision = c.Hazards.RouteRevision;
        if (retimed) c.Log("course-route-retimed", new { at = c.Now, course, legIndex, moveIndex,
            until = clockRoute.At, complete = clockRoute.Schedule?.Complete, clockRoute.Boundary, danger, feet = c.Pose.Feet });
        if (danger) RequestBackground(c);
        else replanWanted = false;
    }
}
