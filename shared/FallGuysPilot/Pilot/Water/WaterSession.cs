using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace MrBean.Modules.FallGuysPilot.Water;

/// <summary>One phase, one plan and one clock for observation, manual loading and drawing.</summary>
internal sealed class WaterSession
{
    private static readonly Lazy<(WaterRoutes Cache, double LoadMs)> Loaded = new(() =>
    {
        long began = Stopwatch.GetTimestamp();
        using var input = typeof(WaterSession).Assembly.GetManifestResourceStream("MrBean.Pilot.HitWater")!;
        var cache = WaterRoutes.Read(input);
        return (cache, Stopwatch.GetElapsedTime(began).TotalMilliseconds);
    });
    public WaterRoutes Cache => Loaded.Value.Cache;
    public double LoadMs => Loaded.Value.LoadMs;
    public bool Manual { get; private set; }
    public bool HasPhase { get; private set; }
    public int Pattern { get; private set; }
    public double Epoch { get; private set; }
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
        Pattern = pattern; Epoch = epoch; Manual = manual; HasPhase = true;
        Source = manual ? "手動載入" : "現場觀測";
        if (manual) { Plan = null; ReceiptCount = 0; MeanError = IdentifyMs = 0; }
    }

    public void UseObservation() { Manual = false; HasPhase = false; Plan = null; Source = "等待觀測"; }

    public void Observe(PilotContext c)
    {
        if (Manual) return;
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
        Plan = selected; PlanStart = start; PlannedAt = worldNow;
        QueryMs = Stopwatch.GetElapsedTime(began).TotalMilliseconds;
    }

    public void Advance(int count, WaterPoint position, double now)
    {
        if (Plan != null) Plan = Plan with { Edges = Plan.Edges[count..], Departures = Plan.Departures[count..] };
        PlanStart = position;
        PlannedAt = now;
    }

    public object Snapshot(double now) => new
    {
        HasPhase, Manual, Source, Pattern, phase = WaterRoutes.Mod(now - Epoch, Cache.Model.Period),
        Epoch, ReceiptCount, MeanError, IdentifyMs, IntervalBuildMs, QueryMs, Reused, SearchedGraph, ModelMiss,
        at = now, start = PlanStart, arrival = Plan?.Arrival + Epoch,
        route = Plan?.Edges.Select((e, i) => new { e.Points, e.Jump, departure = Plan.Departures[i] + Epoch }).ToArray(),
    };
}
