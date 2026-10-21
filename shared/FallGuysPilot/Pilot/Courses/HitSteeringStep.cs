using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal static class HitSteeringStep
{
    private static readonly Lazy<HitSteeringRoutes> Routes = new(() =>
    {
        using var stream = typeof(HitSteeringStep).Assembly.GetManifestResourceStream("MrBean.Pilot.HitSteering")!;
        return HitSteeringRoutes.Read(stream);
    });
    internal static HitTimedRoutes InspectionGraph => Routes.Value.Graph;
    internal static void Prepare() => _ = Routes.Value;
    internal static IEnumerable<HitTimedRoutes.Edge> Connectors(PilotPose pose) =>
        Routes.Value.Connectors(pose.XZ, pose.Raw.ArrowDirection ?? pose.Facing, pose.Motion == MotionKind.Steering);
    internal static bool TimingChanged(HazardField hazards, long afterRevision)
    {
        foreach (var station in hazards.Stations)
            if (station.ObservedRevision > afterRevision &&
                station.Origin.Y + station.Extent + station.Spec.Pad >= HitSteeringRoutes.ExitZ &&
                station.Origin.Y - station.Extent - station.Spec.Pad <= HitSteeringRoutes.WaitingZ) return true;
        return false;
    }
}

internal sealed class HitSteeringPath : Passage
{
    public required Vector3[] Points { get; init; }
    public bool OriginConsumed { get; init; }
    private int index;
    internal int ProgressIndex => index;
    public override void Begin(PilotContext c)
        => index = SharedRouteProgress.Begin(Points, c.Pose.Feet, false, true, OriginConsumed).Index;
    public override void Simulate(Vector2 start, List<Vector2> output)
    {
        // This passage executes the already timed CTC trajectory; callers do not
        // resimulate it with the old lane-selection planner.
        output.Add(start); output.AddRange(Points.Select(p => p.XZ()));
    }
    public override bool Follow(PilotContext c)
    {
        var p = c.Pose.XZ;
        if (c.Pose.Motion == MotionKind.Steering)
        {
            if (!SharedRouteProgress.SteeringTarget(Points, p, ref index, out var direction)) return true;
            c.Act.Steer(c.Pose.Feet, direction);
            return false;
        }
        while (index < Points.Length)
        {
            var target = Points[index].XZ();
            bool passed = index > 0 && Vector2.Dot(p - target, target - Points[index - 1].XZ()) >= 0;
            if (Vector2.DistanceSquared(p, target) > .12f * .12f && !passed) break;
            ++index;
        }
        if (index >= Points.Length - 1 && Points[0].Z >= HitSteeringRoutes.WaitingZ &&
            Points[^1].Z < HitSteeringRoutes.WaitingZ && p.Y < HitSteeringRoutes.WaitingZ &&
            p.Y > HitSteeringRoutes.ExitZ)
        {
            // Entry position does not establish the native gain time. Keep
            // the forward approach until the actual reset has been observed.
            c.Act.Walk(Geo.FromHeading(HitSteeringRoutes.EntryHeading));
            return false;
        }
        if (index == Points.Length) return true;
        c.Act.Walk(Geo.Toward(p, Points[index].XZ()));
        return false;
    }
    public override Vector2? Aim(PilotContext c) => index < Points.Length ? Geo.Toward(c.Pose.XZ, Points[index].XZ()) : null;
}
