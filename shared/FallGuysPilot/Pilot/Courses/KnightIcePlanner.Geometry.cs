using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Interval = MrBean.Modules.FallGuysPilot.Courses.HitTimedRoutes.Interval;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed partial class KnightIcePlanner
{
    private Interval[] Contact(Arc arc, HazardField.Station station)
    {
        var a = arc.From.Feet.XZ(); var b = arc.To.Feet.XZ();
        var spatial = GrowingContacts(a, b, station, arc.Ice ? PositionError : 0);
        var result = new List<Interval>();
        foreach (var part in spatial)
        {
            double lo = arc.Ice ? Inverse(part.Start) : part.Start;
            double hi = arc.Ice ? Inverse(part.End) : part.End;
            // Cover the continuous curve and the executor's sampled trajectory.
            if (arc.Ice) { lo = Math.Min(lo, Approx(part.Start, lo)); hi = Math.Max(hi, Approx(part.End, hi)); }
            result.Add(new(Math.Max(0, lo * arc.NativeSeconds - .0001),
                Math.Min(arc.NativeSeconds, hi * arc.NativeSeconds + .0001) + (arc.Ice ? startup : 0)));
        }
        if (arc.Ice && station.Contains(a, station.Spec.Pad)) result.Add(new(0, startup));
        if (arc.Ice && station.Contains(b, station.Spec.Pad + PositionError))
            result.Add(new(arc.NativeSeconds, arc.NativeSeconds + startup));
        return HitTimedRoutes.Merge(result);
    }
    private static double Inverse(double p) => .5 - Math.Sin(Math.Asin(Math.Clamp(1 - 2 * p, -1, 1)) / 3);
    private static double Approx(double p, double t)
    {
        int cell = Math.Min(49, (int)Math.Floor(t * 50));
        double a = cell / 50d, b = (cell + 1) / 50d;
        double x = a * a * (3 - 2 * a), y = b * b * (3 - 2 * b);
        return a + (b - a) * (p - x) / (y - x);
    }

    // Spatial contact fractions for the mechanism's footprint plus a position
    // uncertainty that grows from zero at the observed slide origin.
    private static Interval[] GrowingContacts(Vector2 a, Vector2 b, HazardField.Station station, float growth)
    {
        var result = new List<Interval>();
        double body = station.Spec.Pad, radius = station.Spec.Size + body;
        double dx = (double)b.X - a.X, dz = (double)b.Y - a.Y;
        void Circle(Vector2 center)
        {
            double x = (double)a.X - center.X, z = (double)a.Y - center.Y;
            double aa = dx * dx + dz * dz - growth * growth;
            double bb = 2 * (x * dx + z * dz - radius * growth), cc = x * x + z * z - radius * radius;
            if (Math.Abs(aa) < 1e-12)
            {
                double lo = 0, hi = 1;
                if (Clip(-cc, -bb, ref lo, ref hi)) result.Add(new(lo, hi));
                return;
            }
            double discriminant = bb * bb - 4 * aa * cc;
            if (discriminant < 0) { if (aa < 0) result.Add(new(0, 1)); return; }
            double root = Math.Sqrt(Math.Max(0, discriminant));
            double x0 = (-bb - root) / (2 * aa), x1 = (-bb + root) / (2 * aa);
            if (x0 > x1) (x0, x1) = (x1, x0);
            void Add(double lo, double hi)
            { lo = Math.Max(0, lo); hi = Math.Min(1, hi); if (lo <= hi) result.Add(new(lo, hi)); }
            if (aa > 0) Add(x0, x1); else { Add(double.NegativeInfinity, x0); Add(x1, double.PositiveInfinity); }
        }
        static bool Clip(double c, double slope, ref double lo, ref double hi)
        {
            if (Math.Abs(slope) < 1e-14) return c >= 0;
            double crossing = -c / slope;
            if (slope > 0) lo = Math.Max(lo, crossing); else hi = Math.Min(hi, crossing);
            return lo <= hi;
        }
        void Box(Vector2 origin, Vector2 forward, double back, double front, bool growEnds)
        {
            var side = new Vector2(forward.Y, -forward.X);
            double px = (double)a.X - origin.X, pz = (double)a.Y - origin.Y;
            double x = px * side.X + pz * side.Y, y = px * forward.X + pz * forward.Y;
            double vx = dx * side.X + dz * side.Y, vy = dx * forward.X + dz * forward.Y;
            double lo = 0, hi = 1, g = growEnds ? growth : 0;
            if (Clip(radius + x, growth + vx, ref lo, ref hi) && Clip(radius - x, growth - vx, ref lo, ref hi) &&
                Clip(y - back, vy + g, ref lo, ref hi) && Clip(front - y, g - vy, ref lo, ref hi))
                result.Add(new(lo, hi));
        }
        switch (station.Spec.Shape)
        {
            case HazardShape.Circle: Circle(station.Origin); break;
            case HazardShape.Square: Box(station.Origin, new(0, 1), -radius, radius, true); break;
            case HazardShape.Rect:
                Box(station.Origin, Geo.FromHeading(station.Rotation), station.Spec.ForwardMin - body, station.Spec.Length + body, true);
                break;
            case HazardShape.HalfDisc:
                Circle(station.Origin);
                var f = Geo.FromHeading(station.Rotation);
                double start = ((double)a.X - station.Origin.X) * f.X + ((double)a.Y - station.Origin.Y) * f.Y + body;
                double velocity = dx * f.X + dz * f.Y + growth;
                for (int i = result.Count - 1; i >= 0; --i)
                {
                    double lo = result[i].Start, hi = result[i].End;
                    if (!Clip(start, velocity, ref lo, ref hi)) result.RemoveAt(i); else result[i] = new(lo, hi);
                }
                break;
            default:
                Circle(station.Origin); Circle(station.Endpoint);
                var axis = station.Endpoint - station.Origin;
                if (axis.LengthSquared() > 0) Box(station.Origin, Vector2.Normalize(axis), 0, axis.Length(), false);
                break;
        }
        return HitTimedRoutes.Merge(result);
    }

    private bool TubeClear(Vector2 a, Vector2 b)
    {
        var points = new List<Vector2>(17) { a };
        for (int i = 0; i < 16; ++i)
        {
            float angle = i * MathF.Tau / 16;
            points.Add(b + new Vector2(MathF.Cos(angle), MathF.Sin(angle)) * (PositionError / MathF.Cos(MathF.PI / 16)));
        }
        points.Sort((p, q) => p.X != q.X ? p.X.CompareTo(q.X) : p.Y.CompareTo(q.Y));
        static double Cross(Vector2 p, Vector2 q, Vector2 r)
            => ((double)q.X - p.X) * (r.Y - p.Y) - ((double)q.Y - p.Y) * (r.X - p.X);
        var hull = new List<Vector2>();
        foreach (var p in points)
        {
            while (hull.Count >= 2 && Cross(hull[^2], hull[^1], p) <= 0) hull.RemoveAt(hull.Count - 1);
            hull.Add(p);
        }
        int lower = hull.Count;
        for (int i = points.Count - 2; i >= 0; --i)
        {
            var p = points[i];
            while (hull.Count > lower && Cross(hull[^2], hull[^1], p) <= 0) hull.RemoveAt(hull.Count - 1);
            hull.Add(p);
        }
        hull.RemoveAt(hull.Count - 1);
        var ground = source.Ground;
        var min = new Vector2(hull.Min(p => p.X), hull.Min(p => p.Y));
        var max = new Vector2(hull.Max(p => p.X), hull.Max(p => p.Y));
        int x0 = (int)Math.Floor((min.X - ground.MinX) / ground.Resolution), x1 = (int)Math.Floor((max.X - ground.MinX) / ground.Resolution);
        int z0 = (int)Math.Floor((min.Y - ground.MinZ) / ground.Resolution), z1 = (int)Math.Floor((max.Y - ground.MinZ) / ground.Resolution);
        if (x0 < 0 || z0 < 0 || x1 >= ground.Width || z1 >= ground.Height) return false;
        for (int z = z0; z <= z1; ++z)
        for (int x = x0; x <= x1; ++x)
        {
            cancellation.ThrowIfCancellationRequested();
            if (ground.Heights[z * ground.Width + x] != null) continue;
            var lo = new Vector2(ground.MinX + x * ground.Resolution, ground.MinZ + z * ground.Resolution);
            var hi = lo + new Vector2(ground.Resolution);
            bool separated = false;
            for (int i = 0; i < hull.Count; ++i)
            {
                var edge = hull[(i + 1) % hull.Count] - hull[i]; var axis = new Vector2(-edge.Y, edge.X);
                double low = double.PositiveInfinity, high = double.NegativeInfinity;
                foreach (var p in hull) { double dot = Vector2.Dot(p, axis); low = Math.Min(low, dot); high = Math.Max(high, dot); }
                double boxLow = axis.X * (axis.X >= 0 ? lo.X : hi.X) + axis.Y * (axis.Y >= 0 ? lo.Y : hi.Y);
                double boxHigh = axis.X * (axis.X >= 0 ? hi.X : lo.X) + axis.Y * (axis.Y >= 0 ? hi.Y : lo.Y);
                if (high < boxLow - 1e-6 || low > boxHigh + 1e-6) { separated = true; break; }
            }
            if (!separated) return false;
        }
        if (ground.CollisionFaces.Length == 0) return true;
        mesh ??= new(ground.CollisionFaces);
        for (int i = 0; i < 20; ++i)
        {
            cancellation.ThrowIfCancellationRequested();
            var p = Vector2.Lerp(a, b, i / 20f); var q = Vector2.Lerp(a, b, (i + 1) / 20f);
            if (ground.At(p) is not { } py || ground.At(q) is not { } qy ||
                !mesh.Clear(new(p.X, py + 1.01f, p.Y), new(q.X, qy + 1.01f, q.Y),
                    ground.BodyRadius + PositionError * (i + 1) / 20 + .10f)) return false;
        }
        return true;
    }
}
