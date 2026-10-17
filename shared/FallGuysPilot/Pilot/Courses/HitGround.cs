using System;
using System.IO.Compression;
using System.Numerics;
using System.Text.Json;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Captured main-deck support, independent of the retired runtime. The source
// cells were generated with the physical 0.5 m body and contain no lower catwalk.
internal sealed class HitGround
{
    public float MinX { get; set; }
    public float MinZ { get; set; }
    public float Resolution { get; set; }
    public int Width { get; set; }
    public int Height { get; set; }
    public float?[] Heights { get; set; } = [];
    public float[][] Walls { get; set; } = [];
    public float[][] CollisionFaces { get; set; } = [];
    public float[][] ExcludedSupportFaces { get; set; } = [];
    public float SupportMargin { get; set; }
    public float[] FloorSlopes { get; set; } = [];
    private PilotCollisionMesh? collision;
    private PilotCollisionMesh? excludedSupport;
    public void PrepareCollision()
    {
        if (CollisionFaces.Length > 0) collision ??= new(CollisionFaces);
        if (ExcludedSupportFaces.Length > 0) excludedSupport ??= new(ExcludedSupportFaces);
    }
    public float BodyRadius { get; set; }
    public float WalkableStepHeight { get; set; } = .05f;
    public static readonly Lazy<HitGround> Loaded = new(() =>
    {
        using var stream = typeof(HitGround).Assembly.GetManifestResourceStream("MrBean.Pilot.HitGround")!;
        using var gzip = new GZipStream(stream, CompressionMode.Decompress);
        return JsonSerializer.Deserialize<HitGround>(gzip)!;
    });

    public float? At(Vector2 p)
    {
        int x = (int)MathF.Floor((p.X - MinX) / Resolution), z = (int)MathF.Floor((p.Y - MinZ) / Resolution);
        if (x < 0 || x >= Width || z < 0 || z >= Height) return null;
        int cell = z * Width + x;
        var y = Heights[cell];
        return y == null || FloorSlopes.Length == 0 ? y : y +
            FloorSlopes[cell * 2] * (p.X - MinX - (x + .5f) * Resolution) +
            FloorSlopes[cell * 2 + 1] * (p.Y - MinZ - (z + .5f) * Resolution);
    }

    public bool Clear(Vector2 a, Vector2 b) => Clear(a, b, null);
    public bool ClearFromGrounded(Vector3 a, Vector2 b) => Clear(new(a.X,a.Z), b, a.Y);
    private bool Clear(Vector2 a, Vector2 b, float? sourceHeight)
    {
        // Sample support along the segment; wall clearance below is continuous.
        // Height continuity rejects jumping between decks.
        int steps = Math.Max(1, (int)Math.Ceiling(Vector2.Distance(a, b) / (Resolution * .5)));
        float? previous = At(a);
        if (previous == null) return false;
        PrepareCollision();
        // Ring's material-marked fall floor can cut through an otherwise
        // walkable grid cell. Check the continuous feet corridor as well.
        if (excludedSupport != null && !excludedSupport.Clear(
            new(a.X, previous.Value, a.Y), new(b.X, At(b) ?? previous.Value, b.Y), SupportMargin)) return false;
        var last = new Vector3(a.X, (sourceHeight ?? previous.Value) + 1.01f, a.Y);
        var anchor = last;
        Vector3? contactSource = sourceHeight.HasValue ? last : null;
        for (int i = 1; i <= steps; ++i)
        {
            var p = Vector2.Lerp(a, b, (float)i / steps);
            var height = At(p);
            if (height == null || Math.Abs(height.Value - previous.Value) > .6f) return false;
            var next = new Vector3(p.X, height.Value + 1.01f, p.Y);
            if (collision != null)
            {
                var span = next - anchor;
                float ratio = span.LengthSquared() > 1e-12f ? Vector3.Dot(last-anchor,span)/span.LengthSquared() : 0;
                if (Vector3.DistanceSquared(last,anchor+span*ratio) > 1e-8f)
                {
                    if (!collision.Clear(anchor,last,BodyRadius + .0001f,contactSource)) return false;
                    anchor = last;
                }
                if (i == steps && !collision.Clear(anchor,next,BodyRadius + .0001f,contactSource)) return false;
            }
            last = next;
            previous = height;
        }
        if (collision != null) return true;
        foreach (var wall in Walls)
        {
            var c = new Vector2(wall[0], wall[1]); var d = new Vector2(wall[2], wall[3]);
            if (Math.Max(a.X, b.X) + BodyRadius < Math.Min(c.X, d.X) ||
                Math.Min(a.X, b.X) - BodyRadius > Math.Max(c.X, d.X) ||
                Math.Max(a.Y, b.Y) + BodyRadius < Math.Min(c.Y, d.Y) ||
                Math.Min(a.Y, b.Y) - BodyRadius > Math.Max(c.Y, d.Y)) continue;
            var midpoint = (Vector2.Clamp((c + d) * .5f, Vector2.Min(a, b), Vector2.Max(a, b)));
            float height = At(midpoint) ?? previous.Value;
            if (wall[5] <= height + WalkableStepHeight || wall[4] >= height + 1.8f) continue;
            static float Cross(Vector2 p, Vector2 q, Vector2 r) => (q.X - p.X) * (r.Y - p.Y) - (q.Y - p.Y) * (r.X - p.X);
            if (Cross(a, b, c) * Cross(a, b, d) <= 0 && Cross(c, d, a) * Cross(c, d, b) <= 0) return false;
            float r2 = BodyRadius * BodyRadius;
            if (Distance2(a, c, d) < r2 || Distance2(b, c, d) < r2 || Distance2(c, a, b) < r2 || Distance2(d, a, b) < r2) return false;
        }
        return true;
    }

    private static float Distance2(Vector2 p, Vector2 a, Vector2 b)
    {
        var ab = b - a;
        float t = ab.LengthSquared() == 0 ? 0 : Math.Clamp(Vector2.Dot(p - a, ab) / ab.LengthSquared(), 0, 1);
        return Vector2.DistanceSquared(p, a + t * ab);
    }
}
