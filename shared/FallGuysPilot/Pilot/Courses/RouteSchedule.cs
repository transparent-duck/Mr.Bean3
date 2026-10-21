using System;
using System.Linq;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

// The graph chooses the spatial route. Its final ordinary-motion schedule uses
// the controller's traversals and stopping positions, including corner cuts.
internal static class RouteSchedule
{
    internal static bool HasConflict(HitTimedRoutes graph, HazardField field, HitTimedRoutes.Plan plan,
        PilotPose pose, double now, RouteExecution.Options options, long afterRevision = 0)
    {
        var projected = RouteContinuation.Retime(plan, default, pose, now, options);
        return projected?.Schedule?.Complete == true
            ? graph.HasConflict(field, projected.Prefix, 0, pose.XZ, now, afterRevision)
            : graph.HasConflict(field, plan, 0, pose.XZ, now, afterRevision);
    }

    internal static HitTimedRoutes.Plan? Retime(HitTimedRoutes graph, HazardField field,
        HitTimedRoutes.Plan plan, RouteExecution state, PilotPose pose, double now,
        RouteExecution.Options options, double windExecution = 1.5, double waitHorizon = 12)
    {
        if (!plan.Found || state.Leg >= plan.Legs.Length || pose.Force != default ||
            pose.Motion != MotionKind.Ordinary) return null;
        // Native gains and flights are observation boundaries. Their existing
        // owner continues until the shared controller can project the next part.
        for (int i = state.Leg; i < plan.Legs.Length; ++i)
            if (plan.Legs[i].Edge.Motion is not PilotCourseGeometry.Move[] moves ||
                moves.Any(m => m.Kind != "walk")) return null;
        var projection = RouteProjection.Follow(plan, state, pose, now,
            Math.Max(60, plan.Arrival - now + 10), options, false, true);
        if (projection?.Schedule is not { Complete: true, Motion: { Count: > 0 } motion }) return null;
        // A walk edge has one continuous traversal. Keep indices explicit so
        // zero-length, already consumed connectors do not shift the schedule.
        if (motion.Select(m => m.Index).Distinct().Count() != motion.Count) return null;
        double frame = RouteExecution.Frame(pose);
        var actual = motion.Select(m =>
        {
            var original = plan.Legs[m.Index].Edge;
            var destination = original.Destination ?? graph.Nodes[original.To];
            var position = m.Leg.Edge.Path[^1].Position;
            var edge = new HitTimedRoutes.Edge(original.From, original.To, m.Leg.Edge.Path)
            {
                Motion = original.Motion,
                ContactTimeMargin = frame,
                Destination = destination with { Position = position,
                    WindBrace = destination.WindBrace &&
                        System.Numerics.Vector2.DistanceSquared(destination.Position, position) <= .12f * .12f }
            };
            // A scheduled input is applied on an update frame. Reserve that
            // frame in contact time rather than pretending it starts exactly
            // on the continuous search boundary.
            return new HitTimedRoutes.Leg(edge, m.Leg.Departure);
        }).ToArray();
        var scheduled = graph.Reschedule(field, new(actual, projection.At, 0, true), 0, pose.XZ, now,
            !state.Started && state.CanStop(pose, options), immobile: options.Bomb,
            windExecution: windExecution, waitHorizon: waitHorizon);
        if (!scheduled.Found || scheduled.Legs.Length != motion.Count) return null;
        var legs = (HitTimedRoutes.Leg[])plan.Legs.Clone();
        for (int i = 0; i < motion.Count; ++i)
            if (motion[i].Index != state.Leg || !state.Started)
                legs[motion[i].Index] = legs[motion[i].Index] with { Departure = scheduled.Legs[i].Departure };
        // A connector already inside the completion radius has no traversal.
        // Its old gate must not delay the newly scheduled following traversal.
        int next = motion.Count - 1;
        double nextDeparture = scheduled.Arrival;
        for (int i = legs.Length - 1; i >= state.Leg; --i)
        {
            if (next >= 0 && motion[next].Index == i)
            { nextDeparture = legs[i].Departure; --next; }
            else if (i != state.Leg || !state.Started)
                legs[i] = legs[i] with { Departure = Math.Min(legs[i].Departure, nextDeparture) };
        }
        return plan with { Legs = legs, Arrival = scheduled.Arrival };
    }
}
