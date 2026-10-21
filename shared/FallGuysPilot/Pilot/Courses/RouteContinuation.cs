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
    internal sealed class Timing(int count)
    {
        internal readonly double[] Departures = Enumerable.Repeat(double.NaN, count).ToArray();
        internal bool Complete;
        internal List<(int Index, HitTimedRoutes.Leg Leg)>? Motion;
    }
    internal Timing? Schedule { get; init; }
    internal bool Boundary { get; init; }

    internal static RouteContinuation? Retime(HitTimedRoutes.Plan plan, RouteExecution state,
        PilotPose pose, double now, RouteExecution.Options options)
        => RouteProjection.Follow(plan, state, pose, now, Math.Max(60, plan.Arrival - now + 10), options, false);

    internal static RouteContinuation? Forecast(HitTimedRoutes.Plan plan, RouteExecution state,
        PilotPose pose, double now, double lead, RouteExecution.Options options)
        => RouteProjection.Follow(plan, state, pose, now, lead, options, true);

    internal bool Matches(PilotPose pose, double now)
    {
        if (pose.Force != Pose.Force || pose.Motion != Pose.Motion || pose.Grounded != Pose.Grounded) return false;
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

}
