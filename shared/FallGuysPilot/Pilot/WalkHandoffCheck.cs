using System;
using System.Numerics;
using MrBean.Modules.FallGuysPilot.Courses;

namespace MrBean.Modules.FallGuysPilot;

internal static class WalkHandoffCheck
{
    public static SharedRouteProgress Choose(SharedRouteProgress proposed, Vector3[] points, Vector3 feet,
        HitGround ground, float dt, float speed, float radius)
    {
        // Identical first commands do not introduce a new approach to validate.
        if (proposed.Index == 0 || proposed.Join == null &&
            Vector2.DistanceSquared(feet.XZ(), points[0].XZ()) <= radius * radius) return proposed;
        if (Clear(proposed, points, feet, ground, dt, speed, radius) == false &&
            Clear(default, points, feet, ground, dt, speed, radius) == true) return default;
        return proposed;
    }

    private static bool? Clear(SharedRouteProgress cursor, Vector3[] points, Vector3 feet,
        HitGround ground, float dt, float speed, float radius)
    {
        // Once per entry, replay at most four seconds of local walking. A budget
        // limit keeps the proposed approach; it never stops the pilot or searches.
        int limit = (int)Math.Ceiling(4 / dt);
        for (int tick = 0; tick < limit; ++tick)
        {
            if (!SharedRouteProgress.WalkTarget(points, feet.XZ(), ref cursor, radius, out var direction)) return true;
            var delta = direction * (speed * dt);
            var next = feet + new Vector3(delta.X, 0, delta.Y);
            if (!ground.ClearFromGrounded(feet, next.XZ())) return false;
            next.Y = ground.At(next.XZ()) ?? feet.Y;
            feet = next;
        }
        return null;
    }
}
