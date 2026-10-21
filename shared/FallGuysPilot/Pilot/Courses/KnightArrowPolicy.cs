using System;
using System.Collections.Generic;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Native arrow input is binary: walk on the actual heading, or stop and rotate.
// Cross broad floor strips instead of pursuing point/heading/time triples.
internal sealed class KnightArrowPolicy(HitGround ground)
{
    internal static readonly float[] Rows = [-254, -274, -287, -295];
    internal const float FrameBudget = 1f / 30;
    private const float Speed = 6, StopDistance = Speed * FrameBudget;
    private static readonly Vector2[] Directions = MakeDirections();
    private readonly Dictionary<(Vector2, int), bool> continuations = [];
    private HazardField.Station[] stations = [];
    private HazardField? owner;
    private int stationCount = -1, target = -1;
    private int headingSamples = 49;
    private Vector2 departure;
    private float departureHeading;
    private double waitSince = double.NaN;
    private long revision = -1, movingRevision;
    internal int Target => target;
    internal bool Moving { get; private set; }
    internal bool Forced { get; private set; }
    internal bool NeedsRecovery { get; private set; }
    internal long Decisions { get; private set; }
    internal long Changes { get; private set; }
    internal double ChangedAt { get; private set; }
    internal Vector3 ChangedFeet { get; private set; }
    internal float ChangedHeading { get; private set; }

    internal void Prepare(HazardField field, double now, float rotationRate = -MathF.Tau)
    {
        int samples = Math.Max(2, (int)Math.Ceiling(Math.Abs(rotationRate) * FrameBudget * 720 / MathF.PI - 1e-5f) + 1);
        if (samples != headingSamples) { headingSamples = samples; continuations.Clear(); }
        field.Prepare(now, 12);
        revision = field.Revision;
        if (owner == field && stationCount == field.Stations.Count) return;
        owner = field;
        stationCount = field.Stations.Count;
        var selected = new List<HazardField.Station>();
        foreach (var station in field.Stations)
            if (station.Spec.ActionId == 34718) selected.Add(station);
        stations = selected.ToArray();
    }

    internal void Reset()
    {
        Moving = Forced = NeedsRecovery = false; target = -1; waitSince = double.NaN;
    }

    internal bool Next(Vector3 feet, float heading, double now)
    {
        ++Decisions;
        var p = feet.XZ();
        if (!float.IsFinite(heading) || p.Y <= Rows[^1] || p.Y > -244 ||
            ground.At(p) is not { } height || Math.Abs(feet.Y - height) > .8f)
        { Reset(); return false; }
        var direction = Geo.FromHeading(heading);
        int first = 0;
        while (first < Rows.Length && p.Y <= Rows[first]) ++first;
        // Keep a valid crossing through its interior. Recheck from the actual
        // feet after a heading/displacement change or a new hazard observation.
        var offset = p - departure;
        float sideways = Math.Abs(offset.X * direction.Y - offset.Y * direction.X);
        if (Moving && target >= first && Math.Abs(Geo.WrapAngle(heading - departureHeading)) < .01f && sideways < .15f)
        {
            if (movingRevision == revision) return true;
            float distance = (Rows[target] - p.Y) / direction.Y;
            if (Exposure(p, p + direction * (distance + StopDistance), now, distance / Speed + FrameBudget) == 0)
            { movingRevision = revision; return true; }
        }

        if (double.IsNaN(waitSince)) waitSince = now;
        bool force = now - waitSince >= 12;
        int selected = -1;
        double bestExposure = double.PositiveInfinity;
        if (direction.Y < -.1f)
        for (int g = Rows.Length - 1; g >= first; --g)
        {
            float distance = (Rows[g] - p.Y) / direction.Y;
            // Reserve the full 30 FPS stopping distance even at 60 FPS.
            var end = p + direction * (distance + StopDistance);
            if (!ground.ClearSupported(p, end)) continue;
            double exposure = Exposure(p, end, now, distance / Speed + FrameBudget);
            if (exposure > 0 && !force) continue;
            if (!force && g < Rows.Length - 1 &&
                (!CanContinue(p + direction * distance, g) || !CanContinue(end, g))) continue;
            if (exposure < bestExposure) { selected = g; bestExposure = exposure; }
            if (exposure == 0) break;
        }
        bool walk = selected >= 0;
        if (walk != Moving || walk && selected != target)
        {
            ++Changes; ChangedAt = now; ChangedFeet = feet; ChangedHeading = heading;
        }
        Moving = walk; Forced = walk && bestExposure > 0;
        // A displaced stop may have no reachable forward ray at all. Let the
        // course planner relocalize and take a lateral/backward connector.
        NeedsRecovery = force && !walk;
        if (walk)
        {
            target = selected; departure = p; departureHeading = heading;
            movingRevision = revision; waitSince = double.NaN;
        }
        return walk;
    }

    private bool CanContinue(Vector2 p, int from)
    {
        var key = (p, from);
        if (continuations.TryGetValue(key, out bool result)) return result;
        int run = 0;
        for (int i = 0; i < Directions.Length; ++i)
        {
            bool clear = ExitClear(p, from, Directions[i]);
            run = clear ? run + 1 : 0;
            // At 360 degrees/s, 49 samples span 12 degrees. This is a sampled margin;
            // live timing is checked again when that next crossing starts.
            if (run >= headingSamples) { result = true; break; }
        }
        if (continuations.Count >= 1024) continuations.Clear();
        continuations[key] = result;
        return result;
    }

    private bool ExitClear(Vector2 p, int from, Vector2 direction)
    {
        if (direction.Y >= -.1f) return false;
        for (int g = from + 1; g < Rows.Length; ++g)
        {
            float distance = (Rows[g] - p.Y) / direction.Y;
            if (distance > 0 && ground.ClearSupported(p, p + direction * (distance + StopDistance))) return true;
        }
        return false;
    }

    private double Exposure(Vector2 a, Vector2 b, double now, double duration)
    {
        double total = 0;
        foreach (var s in stations)
        {
            var forward = Geo.FromHeading(s.Rotation);
            var side = new Vector2(forward.Y, -forward.X);
            var p = a - s.Origin; var delta = b - a;
            double lo = 0, hi = 1;
            float radius = s.Spec.Pad;
            if (!Clip(Vector2.Dot(p, side), Vector2.Dot(delta, side), -s.Spec.Size - radius, s.Spec.Size + radius, ref lo, ref hi) ||
                !Clip(Vector2.Dot(p, forward), Vector2.Dot(delta, forward), s.Spec.ForwardMin - radius, s.Spec.Length + radius, ref lo, ref hi)) continue;
            double start = now + lo * duration, end = now + hi * duration;
            foreach (var window in s.Windows)
                if (window.Start <= end && window.End >= start)
                    total += Math.Max(1e-6, Math.Min(end, window.End) - Math.Max(start, window.Start));
        }
        return total;
    }

    private static bool Clip(double x, double dx, double min, double max, ref double lo, ref double hi)
    {
        if (Math.Abs(dx) < 1e-12) return x >= min && x <= max;
        double a = (min - x) / dx, b = (max - x) / dx;
        if (a > b) (a, b) = (b, a);
        lo = Math.Max(lo, a); hi = Math.Min(hi, b);
        return lo <= hi;
    }

    private static Vector2[] MakeDirections()
    {
        var result = new Vector2[721];
        for (int i = 0; i < result.Length; ++i) result[i] = Geo.FromHeading(MathF.PI / 2 + i * MathF.PI / 720);
        return result;
    }
}
