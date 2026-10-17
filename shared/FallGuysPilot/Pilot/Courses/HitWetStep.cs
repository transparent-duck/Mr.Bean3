using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed class HitWetStep : HitTimedStep
{
    private const float StartZ = -338.9f, EndZ = -367.2f;
    private static readonly Lazy<HitTimedRoutes> Routes = new(() => new(
        [-332.6f, -336.05f, StartZ, -344f, -351.8f, -359.8f, EndZ, -372.6f],
        Enumerable.Range(0, 26).Select(i => -6.25f + i * .5f).ToArray(),
        p => p.Y > StartZ || p.Y < EndZ, Motion,
        Enumerable.Range(0, 17).Select(i => -334f - i * 2).ToArray()));
    protected override HitTimedRoutes Graph => Routes.Value;
    internal static HitTimedRoutes InspectionGraph => Routes.Value;
    internal static void Prepare() => _ = Routes.Value;
    protected override bool CanWait(PilotPose pose) => pose.Force == Vector2.Zero && (pose.Feet.Z > StartZ || pose.Feet.Z < EndZ);

    // Split at force boundaries so every CTC segment has constant velocity.
    // The runtime PathPassage uses WalkNet with the measured external force.
    private static HitTimedRoutes.Point[] Motion(Vector2 start, Vector2 end)
    {
        var points = new List<Vector2> { start };
        if (end.Y != start.Y)
            foreach (float z in new[] { StartZ, EndZ })
            {
                float t = (z - start.Y) / (end.Y - start.Y);
                if (t > 0 && t < 1) points.Add(Vector2.Lerp(start, end, t));
            }
        points.Add(end);
        points = points.OrderBy(p => Vector2.DistanceSquared(start, p)).ToList();
        var result = new List<HitTimedRoutes.Point> { new(start, 0) }; double at = 0;
        for (int i = 1; i < points.Count; ++i)
        {
            var a = points[i - 1]; var b = points[i];
            float z = (a.Y + b.Y) * .5f;
            var force = z < StartZ && z > EndZ ? new Vector2(0, 4) : Vector2.Zero;
            Geo.CompensateForce(Geo.Toward(a, b), force, Actuator.WalkSpeed, out float speed);
            at += Vector2.Distance(a, b) / speed;
            result.Add(new(b, at));
        }
        return result.ToArray();
    }
}
