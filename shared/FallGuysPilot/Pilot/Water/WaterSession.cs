using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MrBean.Modules.FallGuysPilot.Courses;

namespace MrBean.Modules.FallGuysPilot.Water;

/// <summary>One phase, one plan and one clock for observation, manual loading and drawing.</summary>
internal sealed class WaterSession
{
    internal const double PhaseTolerance = .15;
    private static readonly Lazy<(WaterRoutes Cache, double LoadMs)> Loaded = new(() =>
    {
        long began = Stopwatch.GetTimestamp();
        using var input = typeof(WaterSession).Assembly.GetManifestResourceStream("MrBean.Pilot.HitWater")!;
        var cache = WaterRoutes.Read(input);
        return (cache, Stopwatch.GetElapsedTime(began).TotalMilliseconds);
    });
    public WaterRoutes Cache => Loaded.Value.Cache;
    internal static WaterRoutes SharedCache => Loaded.Value.Cache;
    public double LoadMs => Loaded.Value.LoadMs;
    public bool Manual { get; private set; }
    public bool HasPhase { get; private set; }
    public int Pattern { get; private set; }
    public double Epoch { get; private set; }
    public double ObservedEpoch { get; private set; }
    public double PlanEpoch { get; private set; }
    public long Revision { get; private set; }
    public double MeanError { get; private set; }
    public int ReceiptCount { get; private set; }
    public double IdentifyMs { get; private set; }
    public double QueryMs { get; private set; }
    public double IntervalBuildMs { get; private set; }
    public bool Reused { get; private set; }
    public bool SearchedGraph { get; private set; }
    public bool ModelMiss { get; private set; }
    public WaterPlan? Plan { get; private set; }
    public WaterPoint PlanStart { get; private set; }
    public double PlannedAt { get; private set; }
    public string Source { get; private set; } = "尚未載入";
    private long observedRevision = -1;
    private double latestReceipt = double.NegativeInfinity;
    private HitTimedRoutes.Plan? hazardPlan;
    private int firstHazardLeg;
    private HazardField? field;
    private HazardField.Station[] stations = [];

    internal void Attach(HazardField hazards)
    {
        if (ReferenceEquals(field, hazards))
        {
            hazards.ExtraWindows -= PrepareWindows;
            hazards.ExtraWindows += PrepareWindows;
            return;
        }
        field = hazards;
        var model = Cache.Model;
        var spec = new HazardSpec(34773, HazardShape.Circle, (float)model.Radius - PilotEngine.BodyRadius,
            Period: model.Period, Lead: model.Lead, Tail: model.Tail);
        stations = model.Stations.Select(p =>
        {
            var station = hazards.StationFor(spec, p.Vector, 0, p.Vector);
            station.Spec = spec; station.Extent = spec.Size;
            hazards.Changed(station);
            return station;
        }).ToArray();
        hazards.ExtraWindows += PrepareWindows;
    }

    private void PrepareWindows(double now, double end)
    {
        if (!HasPhase) return;
        var model = Cache.Model;
        for (int i = 0; i < stations.Length; ++i)
        {
            var station = stations[i];
            station.Windows.Clear();
            foreach (double pulse in model.Patterns[Pattern].Pulses[i])
                for (double cycle = Math.Ceiling((now - Epoch - pulse - model.Tail) / model.Period); ; ++cycle)
                {
                    double at = Epoch + pulse + cycle * model.Period;
                    if (at - model.Lead > end) break;
                    station.Windows.Add((at - model.Lead, at + model.Tail));
                }
            station.KnownUntil = double.PositiveInfinity;
        }
    }

    // Both inputs end here. The planner and renderer only consume this state.
    public void LoadPattern(int pattern, double epoch, bool manual)
    {
        if (pattern < 0 || pattern >= Cache.Model.Patterns.Length) throw new ArgumentOutOfRangeException(nameof(pattern));
        if (!HasPhase || Pattern != pattern)
        {
            long began = Stopwatch.GetTimestamp();
            Cache.PreparePattern(pattern);
            IntervalBuildMs = Stopwatch.GetElapsedTime(began).TotalMilliseconds;
        }
        // Keep the clock used by plans fixed within the tolerance. Compare to
        // that published clock, so a series of small corrections cannot hide
        // a larger displacement. Manual loading still selects the exact time.
        ObservedEpoch = epoch;
        if (!HasPhase || Pattern != pattern || (manual ? Epoch != epoch :
            Math.Abs(Math.IEEERemainder(epoch - Epoch, Cache.Model.Period)) > PhaseTolerance))
        {
            Epoch = epoch;
            ++Revision;
            if (field != null) foreach (var station in stations) field.Changed(station);
        }
        Pattern = pattern; Manual = manual; HasPhase = true;
        Source = manual ? "手動載入" : "現場觀測";
        if (manual) { Plan = null; ReceiptCount = 0; MeanError = IdentifyMs = 0; }
    }

    public void UseObservation()
    {
        Manual = false; HasPhase = false; Plan = null; hazardPlan = null;
        field = null; stations = [];
        observedRevision = -1; latestReceipt = double.NegativeInfinity; Source = "等待觀測";
    }

    public void Observe(PilotContext c)
    {
        if (Manual || HasPhase && observedRevision == c.Hazards.Revision) return;
        observedRevision = c.Hazards.Revision;
        int count = 0; double latest = double.NegativeInfinity;
        foreach (var station in c.Hazards.Stations)
            if (station.Spec.ActionId == 34773)
            {
                count += station.Receipts.Count;
                if (station.Receipts.Count > 0) latest = Math.Max(latest, station.Receipts[^1]);
            }
        if (HasPhase && count == ReceiptCount && latest == latestReceipt) return;
        latestReceipt = latest;
        long began = Stopwatch.GetTimestamp();
        double origin = double.IsNaN(c.ReleasedAt) ? c.Now : c.ReleasedAt;
        var receipts = new List<WaterReceipt>();
        foreach (var station in c.Hazards.Stations)
        {
            if (station.Spec.ActionId != 34773) continue;
            int id = Cache.Station(new(station.Origin.X, station.Origin.Y));
            foreach (double time in station.Receipts) receipts.Add(new(id, time - origin));
        }
        var clock = Cache.Identify(receipts);
        LoadPattern(clock.Pattern, origin + clock.Offset, false);
        MeanError = clock.MeanError; ReceiptCount = receipts.Count;
        IdentifyMs = Stopwatch.GetElapsedTime(began).TotalMilliseconds;
    }

    public void Calculate(WaterPoint start, double worldNow)
    {
        long began = Stopwatch.GetTimestamp();
        double now = worldNow - Epoch;
        WaterPlan? selected = Cache.Search(Pattern, now, start, connectFromActualPosition: true);
        Reused = false;
        SearchedGraph = true;
        ModelMiss = selected == null;
        Plan = selected; PlanStart = start; PlannedAt = worldNow; PlanEpoch = Epoch;
        firstHazardLeg = 0; hazardPlan = null;
        if (selected != null)
        {
            var legs = new HitTimedRoutes.Leg[selected.Edges.Length];
            var point = start;
            for (int i = 0; i < selected.Edges.Length; ++i)
            {
                var edge = selected.Edges[i];
                var path = new HitTimedRoutes.Point[edge.Points.Length + 1];
                path[0] = new(point.Vector, 0);
                double elapsed = 0;
                for (int j = 0; j < edge.Points.Length; ++j)
                {
                    elapsed += point.Distance(edge.Points[j]) / Cache.Model.Speed;
                    point = edge.Points[j];
                    path[j + 1] = new(point.Vector, elapsed);
                }
                legs[i] = new(new(i, i + 1, path), selected.Departures[i] + PlanEpoch);
            }
            hazardPlan = new(legs, selected.Arrival + PlanEpoch, selected.Expanded, true);
        }
        QueryMs = Stopwatch.GetElapsedTime(began).TotalMilliseconds;
    }

    public bool HasConflict(PilotContext c, long afterRevision, bool clockChanged)
        => Plan != null && (clockChanged && Cache.HasConflict(Plan, new(c.Pose.Feet.X, c.Pose.Feet.Z),
                c.Now, PlanEpoch, Pattern, Epoch) || hazardPlan != null && afterRevision != c.Hazards.RouteRevision &&
            HitTimedRoutes.CheckConflict(c.Hazards, hazardPlan, firstHazardLeg, c.Pose.XZ, c.Now,
                afterRevision, ignoredAction: 34773));

    public void Advance(int count, WaterPoint position, double now)
    {
        if (Plan != null) Plan = Plan with { Edges = Plan.Edges[count..], Departures = Plan.Departures[count..] };
        firstHazardLeg += count;
        PlanStart = position;
        PlannedAt = now;
    }

    public object Snapshot(double now) => new
    {
        HasPhase, Manual, Source, Pattern, phase = WaterRoutes.Mod(now - Epoch, Cache.Model.Period),
        Epoch, ObservedEpoch, PhaseTolerance, ReceiptCount, MeanError, IdentifyMs, IntervalBuildMs, QueryMs, Reused, SearchedGraph, ModelMiss,
        at = now, start = PlanStart, arrival = Plan?.Arrival + PlanEpoch,
        route = Plan?.Edges.Select((e, i) => new { e.Points, e.Jump, departure = Plan.Departures[i] + PlanEpoch }).ToArray(),
    };
}
