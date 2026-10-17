using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed class HitSteeringStep : HitTimedStep
{
    private const float WaitingZ = -269.4f, EntryZ = -270.6f;
    private static readonly Lazy<HitTimedRoutes> Routes = new(() => new(
        [WaitingZ, EntryZ, -279f, -286f, -296f, -306f, -316f, -326f, -332.6f],
        Enumerable.Range(0, 22).Select(i => -6.35f + i * .6f).ToArray(),
        p => p.Y >= WaitingZ, Motion));
    protected override HitTimedRoutes Graph => Routes.Value;
    internal static HitTimedRoutes InspectionGraph => Routes.Value;
    internal static void Prepare() => _ = Routes.Value;
    protected override bool CanWait(PilotPose pose) => pose.Motion != MotionKind.Steering && pose.Feet.Z > EntryZ;
    protected override Func<Vector2, Vector2, HitTimedRoutes.Point[]> ActualMotion(PilotPose pose)
        => pose.Motion == MotionKind.Steering
            ? (a, b) => Curve(a, b, pose.Raw.ArrowDirection ?? pose.Facing)
            : Motion;
    protected override Passage CreatePassage(Vector3[] points) => new HitSteeringPath { Name = Name, Points = points };

    private static HitTimedRoutes.Point[] Motion(Vector2 start, Vector2 end)
    {
        if (end.Y >= WaitingZ) return HitTimedRoutes.Walk(start, end);
        if (start.Y <= EntryZ) return Curve(start, end, MathF.PI);
        var approach = new List<HitTimedRoutes.Point> { new(start, 0) };
        // Choose the entrance lane in this same search, accounting for the actual
        // approach distance. A short northbound approach gives the entry heading.
        var align = new Vector2(end.X, Math.Min(start.Y, WaitingZ));
        double time = Vector2.Distance(start, align) / Actuator.WalkSpeed;
        if (start != align) approach.Add(new(align, time));
        var entrance = new Vector2(end.X, EntryZ);
        time += Vector2.Distance(align, entrance) / Actuator.WalkSpeed;
        approach.Add(new(entrance, time));
        if (end == entrance) return approach.ToArray();
        var curve = Curve(entrance, end, MathF.PI);
        if (curve.Length == 0) return [];
        approach.AddRange(curve.Skip(1).Select(p => p with { Time = p.Time + time }));
        return approach.ToArray();
    }

    // Cubic x(progress) joins north-facing lattice states with continuous heading.
    // The derivative at an actual steering start preserves its current heading.
    // Curvature * native speed must fit the native turn rate; this excludes
    // physically impossible bends, not model-mismatch execution.
    private static HitTimedRoutes.Point[] Curve(Vector2 start, Vector2 end, float heading)
    {
        double distance = start.Y - end.Y;
        if (distance <= 0) return [];
        var direction = Geo.FromHeading(heading);
        if (direction.Y >= 0) return []; // no curve in this control family for the current heading
        double slope = direction.X / -direction.Y;
        double dx = end.X - start.X;
        double a = slope * distance - 2 * dx, b = 3 * dx - 2 * slope * distance, c = slope * distance;
        int count = Math.Max(1, (int)Math.Ceiling(distance / .2));
        var result = new List<HitTimedRoutes.Point> { new(start, 0) };
        double time = 0; var previous = start;
        for (int i = 0; i <= count; ++i)
        {
            double t = (double)i / count;
            double derivative = (3 * a * t * t + 2 * b * t + c) / distance;
            double second = (6 * a * t + 2 * b) / (distance * distance);
            double turn = Math.Abs(second) / Math.Pow(1 + derivative * derivative, 1.5) * SteeringPassage.Speed;
            if (turn > SteeringPassage.TurnRate) return [];
            if (i == 0) continue;
            var position = new Vector2((float)(start.X + ((a * t + b) * t + c) * t), (float)(start.Y - distance * t));
            time += Vector2.Distance(previous, position) / SteeringPassage.Speed;
            result.Add(new(position, time)); previous = position;
        }
        return result.ToArray();
    }
}

internal sealed class HitSteeringPath : Passage
{
    public required Vector3[] Points { get; init; }
    private int index;
    public override void Begin(PilotContext c) => index = 0;
    public override void Simulate(Vector2 start, List<Vector2> output)
    {
        // This passage executes the already timed CTC trajectory; callers do not
        // resimulate it with the old lane-selection planner.
        output.Add(start); output.AddRange(Points.Select(p => p.XZ()));
    }
    public override bool Follow(PilotContext c)
    {
        var p = c.Pose.XZ;
        while (index < Points.Length)
        {
            var target = Points[index].XZ();
            bool passed = index > 0 && Vector2.Dot(p - target, target - Points[index - 1].XZ()) >= 0;
            if (Vector2.DistanceSquared(p, target) > .08f * .08f && !passed) break;
            ++index;
        }
        if (index == Points.Length) return true;
        int aim = index;
        if (c.Pose.Motion == MotionKind.Steering)
        {
            // Follow the selected curve, with a short geometric lookahead;
            // this never changes lanes or searches again during the crossing.
            while (aim + 1 < Points.Length && Vector2.DistanceSquared(p, Points[aim].XZ()) < .5f * .5f) ++aim;
            c.Act.Steer(c.Pose.Feet, Geo.Toward(p, Points[aim].XZ()));
        }
        else c.Act.Walk(Geo.Toward(p, Points[aim].XZ()));
        return false;
    }
    public override Vector2? Aim(PilotContext c) => index < Points.Length ? Geo.Toward(c.Pose.XZ, Points[index].XZ()) : null;
}
