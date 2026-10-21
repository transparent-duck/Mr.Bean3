using System;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed partial class CourseRouteStep
{
    private bool iceEntryReady, waitingForIce;
    private double noRouteSince = double.NaN;

    internal bool NoRouteExpired(PilotContext c)
    {
        if (plan is { Found: true } || !CanStop(c.Pose) || waitingForIce || bombHolding)
            noRouteSince = double.NaN;
        return double.IsFinite(noRouteSince) && c.Now - noRouteSince > 10;
    }

    private void FailedRoute(PilotContext c)
    {
        if (plan is { Found: true } || !CanStop(c.Pose) || waitingForIce || bombHolding) return;
        if (!double.IsFinite(noRouteSince)) noRouteSince = c.Now;
    }

    // Landing and status delivery are separate events. Never let the ordinary
    // walk input move the slide origin while waiting for status 3693.
    private bool ObserveIceEntry(PilotContext c)
    {
        if (course != CourseId.Knight) return false;
        if (!c.Pose.Grounded) { iceEntryReady = waitingForIce = false; return false; }
        if (c.Pose.Motion == MotionKind.Ice)
        {
            iceEntryReady = true;
            waitingForIce = false;
            if (!nativeIce)
            {
                c.Log("course-ice-native", new { at = c.Now, feet = c.Pose.Feet,
                    clock = c.Pose.Raw.IceClock, movement = c.Pose.Raw.TimelineMove });
                clockDirty = true;
            }
            ObserveRouteClock(c);
            return false;
        }
        if (nativeIce)
        {
            bool expected = moveStarted && moves.Length > moveIndex && moves[moveIndex].Kind == "ice";
            float? error = expected ? System.Numerics.Vector2.Distance(c.Pose.XZ, moves[moveIndex].Path[^1].XZ()) : null;
            c.Log("course-ice-land", new { at = c.Now, feet = c.Pose.Feet, expected, error });
            nativeIce = false; iceEntryReady = false;
            // RouteExecution observes the native phase and consumes its move
            // at release, identically for live control and projected control.
            if (!expected) plan = null;
            iceEntryReady = c.Pose.Has(3693);
            clockDirty = true;
        }
        if (!PilotCourseGeometry.InIce(c.Pose.Feet))
        { iceEntryReady = waitingForIce = false; return false; }
        if (iceEntryReady && c.Pose.Has(3693)) return false;
        if (!waitingForIce)
        {
            c.Log("course-ice-entry", new { at = c.Now, feet = c.Pose.Feet, buff = c.Pose.Has(3693) });
            if (routeQuery != null && !c.Pose.Has(3693)) { CancelBackground(); replanWanted = false; }
            if (sections?.Pending == true && !requestedForecast) sections.Cancel();
            clockDirty = true;
            waitingForIce = true;
        }
        progressAt = c.Now;
        if (RouteExecution.AwaitsIceGain(c.Pose, MotionOptions))
        { new RouteExecution.Command(RouteExecution.Input.Hold, Boundary: true).Apply(c); return true; }
        iceEntryReady = true; waitingForIce = false; wasOnIce = true;
        nextRetry = requestAfter = 0;
        c.Log("course-ice-ready", new { at = c.Now, feet = c.Pose.Feet });
        // The accepted route survives the delayed gain. Its actual remaining
        // timing is checked below; only a conflict requests another route.
        return false;
    }
}
