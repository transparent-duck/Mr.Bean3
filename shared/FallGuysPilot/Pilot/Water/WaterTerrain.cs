using System;

namespace MrBean.Modules.FallGuysPilot.Water;

// Compact XZ collision geometry for this level deck, shared by offline edges
// and runtime connectors. It contains no retired runtime or admission policy.
internal sealed class WaterTerrain
{
    public string Source { get; set; } = "";
    public string CapturedUtc { get; set; } = "";
    public double BodyRadius { get; set; }
    public double MinX { get; set; }
    public double MaxX { get; set; }
    public double MinZ { get; set; }
    public double MaxZ { get; set; }
    public WaterPoint[][] Walls { get; set; } = [];
    public WaterPoint[][] Solids { get; set; } = [];

    public bool Clear(WaterPoint a, WaterPoint b)
    {
        if (!OnDeck(a) || !OnDeck(b)) return false;
        foreach (var polygon in Solids)
            if (Inside(a, polygon) || Inside(b, polygon)) return false;
        double r2 = BodyRadius * BodyRadius;
        foreach (var wall in Walls)
        {
            var c = wall[0]; var d = wall[1];
            if (Math.Max(a.X, b.X) + BodyRadius < Math.Min(c.X, d.X) ||
                Math.Min(a.X, b.X) - BodyRadius > Math.Max(c.X, d.X) ||
                Math.Max(a.Z, b.Z) + BodyRadius < Math.Min(c.Z, d.Z) ||
                Math.Min(a.Z, b.Z) - BodyRadius > Math.Max(c.Z, d.Z)) continue;
            if (Cross(a, b, c) * Cross(a, b, d) <= 0 && Cross(c, d, a) * Cross(c, d, b) <= 0) return false;
            if (Distance2(a, c, d) < r2 || Distance2(b, c, d) < r2 ||
                Distance2(c, a, b) < r2 || Distance2(d, a, b) < r2) return false;
        }
        return true;
    }

    private bool OnDeck(WaterPoint p) => p.X >= MinX + BodyRadius && p.X <= MaxX - BodyRadius &&
        p.Z >= MinZ + BodyRadius && p.Z <= MaxZ - BodyRadius;
    private static double Cross(WaterPoint a, WaterPoint b, WaterPoint c) =>
        (b.X - a.X) * (c.Z - a.Z) - (b.Z - a.Z) * (c.X - a.X);
    private static double Distance2(WaterPoint p, WaterPoint a, WaterPoint b)
    {
        double dx = b.X - a.X, dz = b.Z - a.Z, length = dx * dx + dz * dz;
        double t = length == 0 ? 0 : Math.Clamp(((p.X - a.X) * dx + (p.Z - a.Z) * dz) / length, 0, 1);
        double x = p.X - a.X - t * dx, z = p.Z - a.Z - t * dz;
        return x * x + z * z;
    }
    private static bool Inside(WaterPoint p, WaterPoint[] polygon)
    {
        bool inside = false;
        for (int i = 0, j = polygon.Length - 1; i < polygon.Length; j = i++)
        {
            var a = polygon[i]; var b = polygon[j];
            if ((a.Z > p.Z) != (b.Z > p.Z) && p.X < (b.X - a.X) * (p.Z - a.Z) / (b.Z - a.Z) + a.X)
                inside = !inside;
        }
        return inside;
    }
}
