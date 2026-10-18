using System;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed partial class CourseRouteStep
{
    private HitTimedRoutes.Plan? clockPlan;
    private Vector2 clockForce;
    private MotionKind clockMotion;
    private double nextClockCheck;

    private void ObserveRouteArrival(PilotContext c, HitTimedRoutes.Leg completed)
    {
        if (plan == null || legIndex >= plan.Legs.Length || routeQuery != null ||
            course == CourseId.Hit && march.Pending) return;
        double error = c.Now - completed.Departure - completed.Edge.Duration;
        if (Math.Abs(error) <= Math.Max(.08, 2 * FrameSeconds(c))) return;
        c.Log("course-clock-arrival", new { at = c.Now, course, legIndex, error, feet = c.Pose.Feet });
        RequestBackground(c);
    }

    private void ObserveRouteClock(PilotContext c)
    {
        if (plan is not { Found: true } || !moveStarted || legIndex >= plan.Legs.Length ||
            moveIndex >= moves.Length || moves[moveIndex].Kind is not ("walk" or "wet" or "steering" or "arrow") ||
            !c.Pose.Grounded || c.Pose.Locked || course == CourseId.Hit && march.Pending) return;
        if (clockPlan != plan)
        {
            clockPlan = plan; clockForce = c.Pose.Force; clockMotion = c.Pose.Motion;
            nextClockCheck = 0;
        }
        if (c.Now < nextClockCheck || routeQuery != null) return;
        nextClockCheck = c.Now + .05;
        var leg = plan.Legs[legIndex];
        double expected = leg.Departure + RouteContinuation.Progress(leg.Edge, c.Pose.XZ);
        double error = c.Now - expected;
        bool changed = clockForce != c.Pose.Force || clockMotion != c.Pose.Motion;
        if (!changed && Math.Abs(error) <= Math.Max(.08, 2 * FrameSeconds(c))) return;
        clockForce = c.Pose.Force; clockMotion = c.Pose.Motion;
        c.Log("course-clock-drift", new { at = c.Now, course, legIndex, error,
            changed, c.Pose.Force, c.Pose.Motion, feet = c.Pose.Feet });
        RequestBackground(c);
    }
}
