using System;
using System.Collections.Generic;
using System.Numerics;
using MrBean.FallGuys.Mechanics;

namespace MrBean.Modules.FallGuysPilot;

internal enum HazardShape
{
    /// <summary>Size = radius around the receipt origin.</summary>
    Circle,
    /// <summary>Size = half side of an axis-aligned square centred on the origin.</summary>
    Square,
    /// <summary>Size = half width; Length = forward extent from the origin along the receipt rotation.</summary>
    Rect,
    /// <summary>Size = half width of the corridor from the origin to the packet target position.</summary>
    Charge,
    HalfDisc,
}

/// <summary>
/// One AoE family of a course. Stations (fixed geometry instances) are discovered from live
/// ActionEffect receipts; each station repeats its last observed cycle with Period.
/// This table is course DATA: fill and tune it from recordings (see doc/FallGuys-Pilot.md).
/// </summary>
/// <param name="Period">Prior cycle length of one station in seconds; 0 = not periodic (casts/receipts only).</param>
/// <param name="Lead">Modeled danger before a predicted receipt, excluding the shared timing padding.</param>
/// <param name="Tail">Modeled danger after a predicted receipt, excluding the shared timing padding.</param>
/// <param name="Area">Optional origin filter (XMin, XMax, ZMin, ZMax) when one action has different families.</param>
/// <param name="CastWarning">When true, a cast start predicts its activation after the packet cast duration.</param>
/// <param name="Quantum">Origin grid (m) that identifies one station; coarser for noisy moving sources.</param>
/// <param name="EffectRotationOffset">Converts effect-packet facing to the cast footprint direction.</param>
/// <param name="Pad">Metres the footprint inflates for planning checks, per mechanism.</param>
internal sealed record HazardSpec(uint ActionId, HazardShape Shape, float Size, float Length = 0,
    double Period = 0, double Lead = 0, double Tail = 0, Vector4? Area = null,
    bool CastWarning = false, float Quantum = 0.25f, string Note = "", float ForwardMin = 0, double EndpointDwell = 0,
    float EffectRotationOffset = 0, float Pad = 0)
{
    public bool Accepts(Vector2 origin) => Area is not { } a ||
        origin.X >= a.X && origin.X <= a.Y && origin.Y >= a.Z && origin.Y <= a.W;
}

internal enum Exposure { Clear, Hit, Unknown }

/// <summary>Live AoE timing model. Cheap per-frame update; planning reads prepared station windows.</summary>
internal sealed class HazardField
{
    private const double Jitter = 0.03; // added uncertainty per predicted cycle
    private const int MaxReceipts = 48;

    internal sealed class Station
    {
        public required HazardSpec Spec;
        public Vector2 Origin, Endpoint;
        public float Rotation;
        public double Period;
        public readonly List<double> Receipts = [];
        public double? CastEnd;
        public float Extent; // bounding radius from Origin
        // Prepared for one planning query.
        public readonly List<(double Start, double End)> Windows = [];
        public void AddWindow(double start, double end)
            => Windows.Add((start - HazardTiming.Padding, end + HazardTiming.Padding));
        public double KnownUntil;
        public long ChangedAt;
        // Receipt freshness also drives optional route improvements.
        public long ObservedRevision;

        public bool Contains(Vector2 p, float radius)
        {
            var d = p - Origin;
            switch (Spec.Shape)
            {
                case HazardShape.Circle: return d.LengthSquared() <= (Spec.Size + radius) * (Spec.Size + radius);
                case HazardShape.HalfDisc: return d.LengthSquared() <= (Spec.Size + radius) * (Spec.Size + radius)
                    && Vector2.Dot(d, Geo.FromHeading(Rotation)) >= -radius;
                case HazardShape.Square: return MathF.Max(MathF.Abs(d.X), MathF.Abs(d.Y)) <= Spec.Size + radius;
                case HazardShape.Rect:
                {
                    var f = Geo.FromHeading(Rotation);
                    float forward = Vector2.Dot(d, f), side = MathF.Abs(d.X * f.Y - d.Y * f.X);
                    return forward >= Spec.ForwardMin - radius && forward <= Spec.Length + radius && side <= Spec.Size + radius;
                }
                default:
                {
                    var axis = Endpoint - Origin; float l2 = axis.LengthSquared();
                    float t = l2 > 0 ? Math.Clamp(Vector2.Dot(d, axis) / l2, 0, 1) : 0;
                    return Vector2.Distance(p, Origin + axis * t) <= Spec.Size + radius;
                }
            }
        }
    }

    private readonly HazardSpec[] specs;
    private readonly Dictionary<(uint, int, int, int, int, int), Station> stations = [];
    public IReadOnlyCollection<Station> Stations => stations.Values;
    public long Revision { get; private set; }
    public long RouteRevision { get; private set; }
    public void Changed(Station station) => station.ChangedAt = ++RouteRevision;
    public double PreparedAt { get; private set; } = double.NaN;
    public Func<MechanicObservation, bool>? ExtraObservation;
    public Action<double, double>? ExtraWindows;
    public Func<double, double?>? NextWind;
    public (double At, float Heading, float Rate)? RingWind;

    public HazardField(HazardSpec[] specs) => this.specs = specs;

    // Installed action durations exceed ActorCast durations by 0.300 s for
    // every recorded cast family. Keep this attack anchor separate from padding.
    internal static double CastActivation(MechanicCastObservation cast)
        => cast.Seconds + cast.Duration + .3;

    private bool frozen;
    private double frozenStart, frozenEnd;
    internal bool Covers(double start, double end) => !frozen || start >= frozenStart && end <= frozenEnd + 1e-6;

    internal HazardField Snapshot(double now, double horizon = 180)
    {
        double start = Math.Floor(now / 30) * 30;
        Prepare(start, horizon);
        var copy = new HazardField(specs);
        copy.ApplySnapshot(this, start, start + horizon);
        foreach (var station in copy.stations.Values)
            for (int i = 0; i < station.Windows.Count; ++i)
            {
                var window = station.Windows[i];
                station.Windows[i] = (window.Start - 1e-6, window.End + 1e-6);
            }
        return copy;
    }

    internal bool ApplySnapshot(HazardField snapshot)
        => ApplySnapshot(snapshot, snapshot.frozenStart, snapshot.frozenEnd);

    private bool ApplySnapshot(HazardField source, double start, double end)
    {
        bool geometryChanged = false;
        foreach (var (key, value) in source.stations)
        {
            if (!stations.TryGetValue(key, out var target))
                stations.Add(key, target = new() { Spec = value.Spec });
            else geometryChanged |= target.Spec != value.Spec || target.Origin != value.Origin ||
                target.Endpoint != value.Endpoint || target.Rotation != value.Rotation;
            target.Spec = value.Spec; target.Origin = value.Origin; target.Endpoint = value.Endpoint;
            target.Rotation = value.Rotation; target.Extent = value.Extent; target.Period = value.Period;
            target.ChangedAt = value.ChangedAt; target.ObservedRevision = value.ObservedRevision;
            target.KnownUntil = value.KnownUntil;
            target.Windows.Clear(); target.Windows.AddRange(value.Windows);
        }
        frozen = true; frozenStart = start; frozenEnd = end;
        RouteRevision = source.RouteRevision; Revision = source.Revision;
        return geometryChanged;
    }

    // Combo/resource metadata (27/62), misses and status 529 alone do not
    // establish a hazard hit. Use positive damage, stun or displacement.
    internal static bool IsHit(MechanicEffectRow effect) =>
        effect.Type == 14 && effect.Value == 149 ||
        effect.Type is 3 or 5 or 6 && effect.Value > 0 || effect.Type is >= 31 and <= 36;

    private HazardSpec? SpecFor(uint action, Vector2 origin)
    {
        foreach (var spec in specs) if (spec.ActionId == action && spec.Accepts(origin)) return spec;
        return null;
    }

    public void Observe(MechanicObservation observation)
    {
        if (ExtraObservation?.Invoke(observation) == true) { ++Revision; return; }
        switch (observation)
        {
            case MechanicEffectObservation { Origin: { } o } effect:
            {
                var origin = new Vector2(o.X, o.Z);
                if (SpecFor(effect.ActionId, origin) is not { } spec) return;
                var end = effect.PacketTargetPosition is { } e ? new Vector2(e.X, e.Z) : origin;
                var station = StationFor(spec, origin, effect.Rotation + spec.EffectRotationOffset, end);
                AddReceipt(station, effect.Seconds);
                // Adding a receipt can change both the phase and fitted period.
                // Padding is not a threshold for notifying retained routes.
                Changed(station);
                if (spec.EndpointDwell > spec.Tail && effect.PacketTargetPosition != null)
                {
                    // The same column remains at the packet destination until
                    // its next move. Keep this occupancy separate from the sweep.
                    var body = StationFor(new(spec.ActionId | 0xE0000000, HazardShape.Circle, spec.Size,
                        Period: spec.Period, Lead: 0, Tail: spec.EndpointDwell - spec.Tail, Note: "column-body"), end, 0, end);
                    AddReceipt(body, effect.Seconds + spec.Tail);
                    Changed(body);
                }
                station.CastEnd = null;
                ++Revision;
                break;
            }
            case MechanicCastObservation cast:
            {
                var origin = new Vector2(cast.Position.X, cast.Position.Z);
                if (SpecFor(cast.ActionId, origin) is not { CastWarning: true } spec) return;
                var station = StationFor(spec, origin, cast.Rotation, origin);
                station.CastEnd = CastActivation(cast);
                Changed(station);
                ++Revision;
                break;
            }
        }
    }

    internal Station StationFor(HazardSpec spec, Vector2 origin, float rotation, Vector2 endpoint)
    {
        // Geometry identity: origin, and for directional shapes the direction/endpoint as well.
        bool directional = spec.Shape is HazardShape.Rect or HazardShape.Charge or HazardShape.HalfDisc;
        int Q(float v) => (int)MathF.Round(v / spec.Quantum);
        var key = (spec.ActionId, Q(origin.X), Q(origin.Y),
            directional ? (int)MathF.Round(Geo.WrapAngle(rotation) * 8) : 0,
            spec.Shape == HazardShape.Charge ? Q(endpoint.X) : 0, spec.Shape == HazardShape.Charge ? Q(endpoint.Y) : 0);
        if (stations.TryGetValue(key, out var station)) return station;
        station = new Station { Spec = spec, Origin = origin, Endpoint = endpoint, Rotation = rotation, Period = spec.Period };
        station.Extent = spec.Shape switch
        {
            HazardShape.Circle or HazardShape.HalfDisc => spec.Size,
            HazardShape.Square => spec.Size * 1.4143f,
            HazardShape.Rect => MathF.Sqrt(MathF.Max(spec.Length * spec.Length, spec.ForwardMin * spec.ForwardMin) + spec.Size * spec.Size),
            _ => Vector2.Distance(origin, endpoint) + spec.Size,
        };
        stations.Add(key, station);
        return station;
    }

    internal static void AddReceipt(Station s, double at)
    {
        var receipts = s.Receipts;
        if (receipts.Count > 0 && at - receipts[^1] < 0.02) return; // duplicate packet
        // Refine the period from the earlier receipt closest to one period ago. The tolerance must
        // stay below the intra-burst spacing (water pulses are 0.4 s apart), otherwise a wrong
        // pulse pairs up and the period drifts (observed live in Hit-20261016-195419).
        if (s.Period > 0)
        {
            double bestLag = double.NaN, bestError = 0.15;
            for (int i = receipts.Count - 1; i >= 0; --i)
            {
                double lag = at - receipts[i];
                if (lag > s.Period + 0.5) break;
                double error = Math.Abs(lag - s.Period);
                if (error < bestError) { bestError = error; bestLag = lag; }
            }
            if (!double.IsNaN(bestLag)) s.Period = 0.9 * s.Period + 0.1 * bestLag;
        }
        receipts.Add(at);
        if (receipts.Count > MaxReceipts) receipts.RemoveAt(0);
    }

    /// <summary>
    /// Build danger windows for [now, now+horizon]. A station with a known period and a complete
    /// observed cycle repeats that cycle; otherwise its future is Unknown after its last receipt.
    /// </summary>
    public void Prepare(double now, double horizon)
    {
        PreparedAt = now;
        if (frozen)
        {
            if (now < frozenStart || now + horizon > frozenEnd + 1e-6)
                throw new InvalidOperationException("Planning exceeded its hazard snapshot.");
            return;
        }
        double end = now + horizon + HazardTiming.Padding;
        double start = now - HazardTiming.Padding;
        foreach (var s in stations.Values)
        {
            s.Windows.Clear();
            var r = s.Receipts;
            double lead = s.Spec.Lead, tail = s.Spec.Tail;
            foreach (double t in r)
                if (t + tail >= start - 1) s.AddWindow(t - lead, t + tail);
            if (s.CastEnd is { } castEnd) s.AddWindow(castEnd - lead, castEnd + tail);
            s.KnownUntil = r.Count > 0 ? r[^1] : now;
            if (s.Period <= 0 || r.Count == 0 || r[^1] - r[0] < s.Period * 0.9) continue;
            double last = r[^1], cycleStart = last - s.Period + 0.05;
            for (int i = r.Count - 1; i >= 0 && r[i] > cycleStart; --i)
                for (int n = 1; ; ++n)
                {
                    double t = r[i] + n * s.Period;
                    double u = n * Jitter;
                    if (t - lead - u > end) break;
                    if (t + tail + u >= start) s.AddWindow(t - lead - u, t + tail + u);
                }
            s.KnownUntil = double.PositiveInfinity;
        }
        ExtraWindows?.Invoke(start, end);
    }

    /// <summary>Spatial exposure is static per station, so callers precompute it once per candidate.</summary>
    public IEnumerable<Station> Near(Vector2 min, Vector2 max, float radius)
    {
        foreach (var s in stations.Values)
        {
            float pad = s.Extent + radius;
            if (s.Origin.X + pad < min.X || s.Origin.X - pad > max.X || s.Origin.Y + pad < min.Y || s.Origin.Y - pad > max.Y) continue;
            yield return s;
        }
    }

    public static Exposure Overlap(Station s, double start, double end)
    {
        foreach (var (a, b) in s.Windows) if (a <= end && b >= start) return Exposure.Hit;
        return end > s.KnownUntil ? Exposure.Unknown : Exposure.Clear;
    }

    /// <summary>Point query for diagnostics/overlays.</summary>
    public Exposure At(Vector2 p, float radius, double t)
    {
        var result = Exposure.Clear;
        foreach (var s in stations.Values)
        {
            if (!s.Contains(p, s.Spec.Pad)) continue;
            var e = Overlap(s, t, t);
            if (e == Exposure.Hit) return e;
            if (e == Exposure.Unknown) result = e;
        }
        return result;
    }
}
