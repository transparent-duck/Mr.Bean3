using System;
using System.Collections.Generic;
using System.Numerics;
using MrBean.Modules.FallGuysPilot.Courses;

namespace MrBean.Modules.FallGuysPilot;

// A consumed origin and the local approach to its segment are separate states.
internal struct SharedRouteProgress
{
    public int Index;
    public Vector3? Join;

    public static SharedRouteProgress Begin(Vector3[] points, Vector3 feet, bool takeoff, bool turning, bool originConsumed = false)
    {
        if (takeoff || points.Length < 2) return new();
        var a = points[0].XZ(); var d = points[1].XZ() - a;
        float u = d.LengthSquared() > 1e-10f ? Vector2.Dot(feet.XZ() - a, d) / d.LengthSquared() : 0;
        if (u < -.001f && !originConsumed) return new();
        var projection = Vector3.Lerp(points[0], points[1], Math.Clamp(u, 0, 1));
        return new()
        {
            Index = 1,
            Join = !turning && Vector2.DistanceSquared(feet.XZ(), projection.XZ()) > .12f * .12f ? projection : null
        };
    }

    public static bool Passed(Vector2 a, Vector2 b, Vector2 feet)
        => (b - a).LengthSquared() > 1e-10f && Vector2.Dot(feet - b, b - a) >= 0;

    public static bool WalkTarget(Vector3[] points, Vector2 feet, ref SharedRouteProgress cursor,
        float radius, out Vector2 direction)
    {
        if (cursor.Join is { } join)
        {
            if (Vector2.DistanceSquared(feet, join.XZ()) > .12f * .12f)
            { direction = Geo.Toward(feet, join.XZ()); return true; }
            cursor.Join = null;
        }
        while (cursor.Index < points.Length &&
            (Vector2.Distance(feet, points[cursor.Index].XZ()) <= radius ||
             cursor.Index > 0 && Passed(points[cursor.Index - 1].XZ(), points[cursor.Index].XZ(), feet)))
            ++cursor.Index;
        if (cursor.Index >= points.Length) { direction = default; return false; }
        direction = Geo.Toward(feet, points[cursor.Index].XZ()); return true;
    }

    public static bool SteeringTarget(Vector3[] points, Vector2 feet, ref int index, out Vector2 direction)
    {
        while (index < points.Length && (Vector2.DistanceSquared(feet, points[index].XZ()) <= .12f * .12f ||
            index > 0 && Passed(points[index - 1].XZ(), points[index].XZ(), feet))) ++index;
        if (index >= points.Length) { direction = default; return false; }
        int aim = index;
        while (aim + 1 < points.Length && Vector2.DistanceSquared(feet, points[aim].XZ()) < .5f * .5f) ++aim;
        direction = Geo.Toward(feet, points[aim].XZ());
        return true;
    }

}
