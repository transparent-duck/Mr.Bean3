using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Numerics;
using System.Text.Json;

namespace MrBean.Modules.FallGuysPilot.Water;

// Pure planning code shared by the offline compiler and the active Pilot. It does
// not reference the retired compiled-program runtime or any native game service.
internal sealed class WaterModel
{
    public double Period { get; set; }
    public double Radius { get; set; }
    public double Lead { get; set; }
    public double Tail { get; set; }
    // Padding already applied to serialized edge intervals; older assets omit it.
    public double WindowPadding { get; set; }
    public double Speed { get; set; }
    public double LaneSpacing { get; set; }
    // Refinement preserves the original physical search horizon; inserting
    // vertices along the same walk does not require more periodic cycles.
    public double IntervalHorizon { get; set; }
    public string Evidence { get; set; } = "";
    public WaterTerrain Terrain { get; set; } = new();
    public WaterPoint[] Stations { get; set; } = [];
    public WaterPattern[] Patterns { get; set; } = [];
    public static readonly JsonSerializerOptions Json = new() { PropertyNameCaseInsensitive = true };
}

internal sealed class WaterPattern
{
    public int[] Recordings { get; set; } = [];
    public double[][] Pulses { get; set; } = [];
}

internal readonly record struct WaterPoint(double X, double Z)
{
    public double Distance(WaterPoint b) => Math.Sqrt((X - b.X) * (X - b.X) + (Z - b.Z) * (Z - b.Z));
    internal Vector2 Vector => new((float)X, (float)Z);
}

internal readonly record struct WaterInterval(double Start, double End);
internal readonly record struct WaterExposure(int Station, double Start, double End);

internal sealed class WaterEdge
{
    public int From, To;
    public WaterPoint[] Points = [];
    public int Jump = -1;
    public double Duration;
    public WaterExposure[] Exposures = [];
    public WaterInterval[][] Forbidden = [];
}

internal sealed class WaterRoute
{
    public int Entry;
    public int Pattern;
    public double SamplePhase;
    public int[] Edges = [];
    public double[] Departures = [];
    public double Duration;
}

internal sealed record WaterPlan(WaterEdge[] Edges, double[] Departures, double Arrival, int Expanded = 0);
internal readonly record struct WaterReceipt(int Station, double At);
internal readonly record struct WaterClock(int Pattern, double Offset, double MeanError);

internal sealed class WaterRoutes
{
    public WaterModel Model = new();
    public WaterPoint[] Nodes = [];
    public WaterEdge[] Edges = [];
    public WaterRoute[] Routes = [];
    public int[][] Outgoing = [];
    public WaterInterval[][][] NodeForbidden = [];
    private WaterInterval[][][] stationForbidden = [];
    public int[] Goals = [];
    public int[] Entries = [];
    private int preparedPattern = -1;
    private bool storedIntervals;
    private Dictionary<int, WaterRoute[]> routesByEntry = [];
    public int PrecomputeSearches;
    public long PrecomputeExpanded;
    private double horizon;
    internal double IntervalHorizon => horizon;
    private double[] remainingTime = [];
    private WaterPoint? connectorStart;
    private WaterEdge[] connectorCache = [];
    public const double ExitZ = -245.68;
    private static readonly double[] Rows = [-197, -202, -207, -212, -217, -222, -227, -232, -237];

    public static double Mod(double t, double period) => t - Math.Floor(t / period) * period;

    public static WaterRoutes Build(WaterModel model, int phaseSamples, double exitZ = ExitZ)
    {
        var cache = new WaterRoutes { Model = model };
        var nodes = new List<WaterPoint>();
        foreach (double z in new[] { -194.9 }.Concat(Rows))
            for (double x = Math.Ceiling((model.Terrain.MinX + model.Terrain.BodyRadius) / model.LaneSpacing) * model.LaneSpacing;
                 x <= model.Terrain.MaxX - model.Terrain.BodyRadius; x += model.LaneSpacing)
                // Keep the original fine lanes. Intermediate rows need only
                // 1 m lateral choices to bend between pulses without growing
                // the graph to the full quarter-metre cross product.
                if ((z is -194.9 or -207 or -217 or -227 or -237 || Math.Abs(x - Math.Round(x)) < .001) &&
                    model.Terrain.Clear(new(x, z), new(x, z))) nodes.Add(new(x, z));
        cache.Entries = Enumerable.Range(0, nodes.Count).Where(n => nodes[n].Z == -194.9).ToArray();
        // Tangent corner candidates let routes bend around solids between the
        // sampled rows instead of requiring a straight crossing of each strip.
        foreach (var polygon in model.Terrain.Solids)
        {
            double cx = polygon.Average(p => p.X), cz = polygon.Average(p => p.Z);
            foreach (var vertex in polygon)
            {
                double dx = vertex.X - cx, dz = vertex.Z - cz, length = Math.Sqrt(dx * dx + dz * dz);
                // These retained solid outlines are square diamonds: sqrt(2)*r
                // offsets a corner so both incident walls clear the body disc.
                double offset = Math.Sqrt(2) * model.Terrain.BodyRadius + .01; // capture outline weld precision
                var point = new WaterPoint(vertex.X + dx / length * offset, vertex.Z + dz / length * offset);
                if (point.Z >= -237 && point.Z <= -194.9 && model.Terrain.Clear(point, point)) nodes.Add(point);
            }
        }
        nodes = nodes.Distinct().ToList();
        cache.Goals = [nodes.Count, nodes.Count + 1];
        nodes.Add(new(-5.52, exitZ)); nodes.Add(new(5.59, exitZ));
        cache.Nodes = nodes.ToArray();
        var edges = new List<WaterEdge>();
        int ordinary = cache.Goals[0];
        for (int from = 0; from < ordinary; ++from)
            for (int to = 0; to < ordinary; ++to)
            {
                double advance = nodes[from].Z - nodes[to].Z;
                bool lateral = advance == 0 && Math.Abs(nodes[from].X - nodes[to].X) <= model.LaneSpacing * 1.01;
                if (!(advance > 0 && advance <= 10 || lateral)) continue;
                if (model.Terrain.Clear(nodes[from], nodes[to])) edges.Add(cache.MakeEdge(from, to, [nodes[to]]));
            }
        for (int from = 0; from < ordinary; ++from)
        {
            if (nodes[from].Z > -237) continue;
            foreach (var edge in new[] {
                cache.MakeEdge(from, cache.Goals[0], [new(-7.51, -239.19), new(-5.54, -243.95), new(-5.52, -245.68), nodes[cache.Goals[0]]], 0),
                cache.MakeEdge(from, cache.Goals[1], [new(7.53, -239.22), new(5.695, -243.645), nodes[cache.Goals[1]]], 0) })
                if (cache.Clear(edge)) edges.Add(edge);
        }
        cache.Edges = edges.ToArray();
        cache.Index();
        cache.Precompute(phaseSamples);
        return cache;
    }

    internal void Index()
    {
        connectorStart = null; connectorCache = [];
        var outgoing = Enumerable.Range(0, Nodes.Length).Select(_ => new List<int>()).ToArray();
        for (int i = 0; i < Edges.Length; ++i) outgoing[Edges[i].From].Add(i);
        Outgoing = outgoing.Select(l => l.ToArray()).ToArray();
        var incoming = Enumerable.Range(0, Nodes.Length).Select(_ => new List<int>()).ToArray();
        for (int i = 0; i < Edges.Length; ++i) incoming[Edges[i].To].Add(i);
        remainingTime = Enumerable.Repeat(double.PositiveInfinity, Nodes.Length).ToArray();
        var distances = new PriorityQueue<int, double>();
        foreach (int goal in Goals) { remainingTime[goal] = 0; distances.Enqueue(goal, 0); }
        while (distances.TryDequeue(out int target, out double distance))
        {
            if (distance != remainingTime[target]) continue;
            foreach (int id in incoming[target])
            {
                var edge = Edges[id]; double candidate = distance + edge.Duration;
                if (candidate >= remainingTime[edge.From]) continue;
                remainingTime[edge.From] = candidate; distances.Enqueue(edge.From, candidate);
            }
        }
        Entries = Entries.Where(n => double.IsFinite(remainingTime[n])).ToArray();
        // Forward levels plus lateral traversal of a full deck width. Used only
        // to materialize periodic intervals, not as a runtime wait/stop policy.
        var remaining = new double[Nodes.Length];
        foreach (int node in Enumerable.Range(0, Nodes.Length).OrderBy(n => Nodes[n].Z))
            foreach (int id in Outgoing[node])
                if (Nodes[Edges[id].To].Z < Nodes[node].Z)
                    remaining[node] = Math.Max(remaining[node], Edges[id].Duration + Model.Period + remaining[Edges[id].To]);
        horizon = Model.IntervalHorizon > 0 ? Model.IntervalHorizon
            : remaining.Max() + (Model.Terrain.MaxX - Model.Terrain.MinX) / Model.Speed * Rows.Length;
        NodeForbidden = new WaterInterval[Model.Patterns.Length][][];
        stationForbidden = new WaterInterval[Model.Patterns.Length][][];
        routesByEntry = Routes.GroupBy(r => r.Entry).ToDictionary(g => g.Key, g => g.ToArray());
        preparedPattern = -1;
    }

    // Offline graph generation: these edges cannot depart at any phase in any
    // retained arrangement. Removing them does not depend on sampled queries.
    internal int PruneBlockedEdges()
    {
        int before = Edges.Length;
        Edges = Edges.Where(edge =>
        {
            if (edge.From == edge.To) return false;
            bool usable = false;
            for (int pattern = 0; pattern < Model.Patterns.Length; ++pattern)
            {
                var spans = edge.Forbidden[pattern] ??= Forbid(edge.Exposures, pattern);
                usable |= !(spans.Length == 1 && spans[0].Start == 0 && spans[0].End == Model.Period);
            }
            return usable;
        }).ToArray();
        Index();
        return before - Edges.Length;
    }

    public void PreparePattern(int pattern)
    {
        if (preparedPattern == pattern) return;
        if (preparedPattern >= 0)
        {
            NodeForbidden[preparedPattern] = null!;
            stationForbidden[preparedPattern] = null!;
            if (!storedIntervals) foreach (var edge in Edges) edge.Forbidden[preparedPattern] = null!;
        }
        stationForbidden[pattern] = new WaterInterval[Model.Stations.Length][];
        for (int station = 0; station < Model.Stations.Length; ++station)
            stationForbidden[pattern][station] = Forbid([new WaterExposure(station, 0, 0)], pattern);
        NodeForbidden[pattern] = new WaterInterval[Nodes.Length][];
        for (int node = 0; node < Nodes.Length; ++node)
        {
            var exposures = new List<WaterExposure>();
            for (int station = 0; station < Model.Stations.Length; ++station)
                if (Nodes[node].Distance(Model.Stations[station]) <= Model.Radius) exposures.Add(new(station, 0, 0));
            NodeForbidden[pattern][node] = Forbid(exposures, pattern);
        }
        foreach (var edge in Edges) edge.Forbidden[pattern] ??= Forbid(edge.Exposures, pattern);
        preparedPattern = pattern;
    }

    public void Precompute(int phaseSamples)
    {
        var routes = new List<WaterRoute>();
        var ids = Edges.Select((edge, index) => (edge, index)).ToDictionary(x => x.edge, x => x.index);
        var unique = new HashSet<string>();
        for (int pattern = 0; pattern < Model.Patterns.Length && phaseSamples > 0; ++pattern)
        {
            PreparePattern(pattern);
            foreach (int entry in Entries)
                for (int sample = 0; sample < phaseSamples; ++sample)
                {
                    double at = Model.Period * sample / phaseSamples;
                    var plan = Search(pattern, at, Nodes[entry]);
                    ++PrecomputeSearches;
                    if (PrecomputeSearches % 4096 == 0) Console.WriteLine($"Water precompute: {PrecomputeSearches} searches.");
                    if (plan == null) continue;
                    PrecomputeExpanded += plan.Expanded;
                    var routeIds = plan.Edges.Select(e => ids[e]).ToArray();
                    if (!unique.Add(entry + ":" + string.Join(',', routeIds))) continue;
                    routes.Add(new WaterRoute { Entry = entry, Pattern = pattern, SamplePhase = at, Edges = routeIds,
                        Departures = plan.Departures.Select(d => d - at).ToArray(), Duration = plan.Arrival - at });
                }
            Console.WriteLine($"Water pattern {pattern + 1}: {PrecomputeSearches} searches, {routes.Count} routes.");
        }
        Routes = routes.ToArray();
        routesByEntry = Routes.GroupBy(r => r.Entry).ToDictionary(g => g.Key, g => g.ToArray());
    }

    internal bool HasConflict(WaterPlan plan, WaterPoint feet, double worldNow, double planEpoch,
        int pattern, double epoch)
    {
        PreparePattern(pattern);
        bool Overlap(WaterInterval[] spans, double start, double end)
        {
            if (end < start) return false;
            foreach (var span in spans)
            {
                double cycle = Math.Ceiling((start - span.End) / Model.Period);
                if (span.Start + cycle * Model.Period <= end) return true;
            }
            return false;
        }
        bool Hold(WaterPoint position, double start, double end)
        {
            if (end < start) return false;
            for (int station = 0; station < Model.Stations.Length; ++station)
                if (position.Distance(Model.Stations[station]) <= Model.Radius &&
                    Overlap(stationForbidden[pattern][station], start - epoch, end - epoch)) return true;
            return false;
        }
        double at = worldNow;
        var point = feet;
        for (int i = 0; i < plan.Edges.Length; ++i)
        {
            var edge = plan.Edges[i];
            double departure = plan.Departures[i] + planEpoch;
            if (Hold(point, at, departure)) return true;
            foreach (var exposure in edge.Exposures)
                if (Overlap(stationForbidden[pattern][exposure.Station],
                    Math.Max(worldNow, departure + exposure.Start) - epoch,
                    departure + exposure.End - epoch)) return true;
            at = Math.Max(worldNow, departure + edge.Duration);
            point = edge.Points[^1];
        }
        return Hold(point, at, plan.Arrival + planEpoch);
    }

    /// <summary>Drawing uses the same periodic interval compiler as path collision checks.</summary>
    public (bool Active, double Seconds) StationState(int pattern, int station, double at)
    {
        PreparePattern(pattern);
        double phase = Mod(at, Model.Period), next = double.PositiveInfinity;
        var spans = stationForbidden[pattern][station];
        foreach (var span in spans)
        {
            if (span.Start <= phase && phase <= span.End)
            {
                double end = span.End;
                if (end == Model.Period && spans[0].Start == 0) end += spans[0].End;
                return (true, end - phase);
            }
            next = Math.Min(next, Mod(span.Start - phase, Model.Period));
        }
        return (false, next);
    }

    public WaterEdge MakeEdge(int from, int to, WaterPoint[] points, int jump = -1, WaterPoint? actualStart = null)
    {
        // A recovery beyond the takeoff continues on the landing side; do not
        // walk backwards and launch the same jump again.
        if (actualStart is { } actual && jump >= 0 && actual.Z < points[jump].Z)
        {
            points = points.Where(p => p.Z < actual.Z).ToArray();
            if (points.Length == 0) points = [Nodes[to]];
            jump = -1;
        }
        var edge = new WaterEdge { From = from, To = to, Points = points, Jump = jump };
        var exposures = new List<WaterExposure>();
        var a = actualStart ?? Nodes[from];
        double elapsed = 0;
        foreach (var b in points)
        {
            double length = a.Distance(b), duration = length / Model.Speed;
            if (length > 0)
            {
                double ux = (b.X - a.X) / length, uz = (b.Z - a.Z) / length;
                for (int s = 0; s < Model.Stations.Length; ++s)
                {
                    var center = Model.Stations[s];
                    double dx = center.X - a.X, dz = center.Z - a.Z;
                    double along = dx * ux + dz * uz;
                    double h2 = Model.Radius * Model.Radius - (dx * dx + dz * dz - along * along);
                    if (h2 < 0) continue;
                    double h = Math.Sqrt(h2), enter = Math.Max(0, along - h), exit = Math.Min(length, along + h);
                    if (enter <= exit) exposures.Add(new(s, elapsed + enter / Model.Speed, elapsed + exit / Model.Speed));
                }
            }
            elapsed += duration; a = b;
        }
        edge.Duration = elapsed; edge.Exposures = exposures.ToArray();
        edge.Forbidden = new WaterInterval[Model.Patterns.Length][];
        return edge;
    }

    public bool Clear(WaterEdge edge, WaterPoint? start = null)
    {
        var from = start ?? Nodes[edge.From];
        foreach (var point in edge.Points)
        {
            if (!Model.Terrain.Clear(from, point)) return false;
            from = point;
        }
        return true;
    }

    private WaterInterval[] Forbid(IEnumerable<WaterExposure> exposures, int pattern)
    {
        var intervals = new List<WaterInterval>();
        foreach (var e in exposures)
            foreach (double pulse in Model.Patterns[pattern].Pulses[e.Station])
                AddPeriodic(intervals, pulse - Model.Lead - HazardTiming.Padding - e.End,
                    pulse + Model.Tail + HazardTiming.Padding - e.Start);
        return Merge(intervals);
    }

    private void AddPeriodic(List<WaterInterval> result, double start, double end)
    {
        double length = end - start, period = Model.Period;
        if (length >= period) { result.Add(new(0, period)); return; }
        start = Mod(start, period); end = start + length;
        result.Add(new(start, Math.Min(end, period)));
        if (end > period) result.Add(new(0, end - period));
    }

    private static WaterInterval[] Merge(List<WaterInterval> intervals)
    {
        intervals.Sort((a, b) => a.Start.CompareTo(b.Start));
        var result = new List<WaterInterval>();
        foreach (var span in intervals)
            if (result.Count > 0 && span.Start <= result[^1].End)
                result[^1] = new(result[^1].Start, Math.Max(result[^1].End, span.End));
            else result.Add(span);
        return result.ToArray();
    }

    // Strictly leave the closed danger interval. BitIncrement is floating-point
    // endpoint handling, not an added physical timing margin.
    private double NextDeparture(WaterInterval[] forbidden, double at, double until)
    {
        if (forbidden.Length == 1 && forbidden[0].Start == 0 && forbidden[0].End == Model.Period)
            return double.PositiveInfinity;
        while (at <= until)
        {
            double cycle = Math.Floor(at / Model.Period) * Model.Period;
            bool changed = false;
            foreach (var interval in forbidden)
                if (at >= cycle + interval.Start && at <= cycle + interval.End)
                {
                    at = Math.BitIncrement(cycle + interval.End); changed = true; break;
                }
            if (!changed) return at;
        }
        return double.PositiveInfinity;
    }

    private WaterInterval[] Safe(WaterInterval[] forbidden, double start, double end)
    {
        var unsafeSpans = new List<WaterInterval>();
        for (int n = (int)Math.Floor(start / Model.Period) - 1; n <= Math.Ceiling(end / Model.Period); ++n)
            foreach (var f in forbidden)
                unsafeSpans.Add(new(f.Start + n * Model.Period, f.End + n * Model.Period));
        var result = new List<WaterInterval>();
        double at = start;
        foreach (var f in Merge(unsafeSpans))
        {
            if (f.End < at) continue;
            if (f.Start > end) break;
            if (f.Start > at) result.Add(new(at, Math.BitDecrement(f.Start)));
            at = Math.Max(at, Math.BitIncrement(f.End));
        }
        if (at <= end) result.Add(new(at, end));
        return result.ToArray();
    }

    private sealed record Label(int Node, int Interval, double Arrival, double Departure, Label? Previous, WaterEdge? Edge);

    private static WaterPlan Unpack(Label goal, int expanded = 0)
    {
        var edges = new List<WaterEdge>(); var departures = new List<double>();
        for (var label = goal; label.Edge != null; label = label.Previous!)
        { edges.Add(label.Edge); departures.Add(label.Departure); }
        edges.Reverse(); departures.Reverse();
        return new(edges.ToArray(), departures.ToArray(), goal.Arrival, expanded);
    }

    // All states here are ordinary walking on the level water deck. They may wait
    // within a safe interval. Jump edges are indivisible; there is no midair wait.
    public WaterPlan? Search(int pattern, double at, WaterPoint? actualStart = null, bool connectFromActualPosition = false)
    {
        PreparePattern(pattern);
        var start = actualStart ?? Nodes[Entries[0]];
        double maximum = at + horizon + start.Distance(Nodes[Entries[0]]) / Model.Speed;
        // Runtime starts use the same visible connectors even if their feet
        // happen to equal a graph vertex. Exact equality must not remove route
        // choices when more intermediate vertices are added. Offline complete
        // route precomputation keeps graph edges so it can store their IDs.
        int source = connectFromActualPosition ? -1 : Array.IndexOf(Nodes, start);
        if (source < 0 && SafeAt(start, pattern, at, at).Length == 0) return null;
        var connectors = source < 0 ? Connectors(start) : [];
        foreach (var edge in connectors) edge.Forbidden[pattern] ??= Forbid(edge.Exposures, pattern);
        double shortest = source >= 0 ? remainingTime[source]
            : connectors.Select(e => e.Duration + remainingTime[e.To]).DefaultIfEmpty(double.PositiveInfinity).Min();
        if (!double.IsFinite(shortest)) return null;
        int totalExpanded = 0, expanded = 0;
        bool invalidStart = false;
        // Most crossings need only a small part of the conservative interval
        // horizon. A goal found in a shorter window is still globally earliest:
        // every earlier route is contained in that same window. If none exists,
        // expand until the original full horizon; no feasible route is dropped.
        for (double slack = Model.Period; ; slack *= 2)
        {
            double end = Math.Min(maximum, at + shortest + slack);
            var result = SearchWindow(end);
            totalExpanded += expanded;
            if (result != null) return result with { Expanded = totalExpanded };
            if (invalidStart || end >= maximum) return null;
        }

        WaterPlan? SearchWindow(double end)
        {
            var safe = new WaterInterval[Nodes.Length][];
            WaterInterval[] SafeNode(int node) => safe[node] ??= Safe(NodeForbidden[pattern][node], at, end);
            var startSafe = source >= 0 ? SafeNode(source) : SafeAt(start, pattern, at, end);
            var labels = new Dictionary<(int, int), Label>();
            var queue = new PriorityQueue<Label, double>();
            expanded = 0;
            for (int j = 0; j < startSafe.Length; ++j)
                if (startSafe[j].Start <= at && at <= startSafe[j].End) Offer(new(source, j, at, at, null, null));
            invalidStart = queue.Count == 0;
            while (queue.TryDequeue(out var current, out _))
            {
                if (!ReferenceEquals(labels[(current.Node, current.Interval)], current)) continue;
                ++expanded;
                if (Array.IndexOf(Goals, current.Node) >= 0) return Unpack(current, expanded);
                foreach (var edge in current.Node < 0 ? connectors : Outgoing[current.Node].Select(id => Edges[id]))
                {
                    var forbidden = edge.Forbidden[pattern];
                    if (forbidden.Length == 1 && forbidden[0].Start == 0 && forbidden[0].End == Model.Period) continue;
                    for (int j = 0; j < SafeNode(edge.To).Length; ++j)
                    {
                        var target = SafeNode(edge.To)[j];
                        double sourceEnd = (current.Node < 0 ? startSafe : SafeNode(current.Node))[current.Interval].End;
                        if (target.Start > sourceEnd + edge.Duration) break;
                        if (target.End < current.Arrival + edge.Duration) continue;
                        double first = Math.Max(current.Arrival, target.Start - edge.Duration);
                        double last = Math.Min((current.Node < 0 ? startSafe : SafeNode(current.Node))[current.Interval].End, target.End - edge.Duration);
                        if (first > last) continue;
                        double departure = NextDeparture(edge.Forbidden[pattern], first, last);
                        if (departure <= last) Offer(new(edge.To, j, departure + edge.Duration, departure, current, edge));
                    }
                }
            }
            return null;

            void Offer(Label label)
            {
                var key = (label.Node, label.Interval);
                if (labels.TryGetValue(key, out var old) && old.Arrival <= label.Arrival) return;
                labels[key] = label;
                double remaining = label.Node < 0 ? connectors.Select(e => e.Duration + remainingTime[e.To]).DefaultIfEmpty(double.PositiveInfinity).Min() : remainingTime[label.Node];
                queue.Enqueue(label, label.Arrival + remaining);
            }
        }
    }

    internal WaterEdge[] Connectors(WaterPoint start)
    {
        // Geometry and phase-relative contacts do not change while the feet
        // stay at this exact point. Reuse them for retries or phase queries.
        if (connectorStart == start) return connectorCache;
        var result = new List<WaterEdge>();
        for (int node = 0; node < Goals[0]; ++node)
        {
            double advance = start.Z - Nodes[node].Z;
            if (advance < 0 || !Model.Terrain.Clear(start, Nodes[node])) continue;
            result.Add(MakeEdge(-1, node, [Nodes[node]], actualStart: start));
        }
        if (start.Z <= -237)
            foreach (int goal in Goals)
            {
                var template = Edges.First(e => e.To == goal);
                var edge = MakeEdge(-1, goal, template.Points, template.Jump, start);
                if (Clear(edge, start)) result.Add(edge);
            }
        connectorStart = start;
        return connectorCache = result.ToArray();
    }

    public WaterPlan? GeometryRoute(WaterPoint start, double at)
    {
        var queue = new PriorityQueue<Label, double>();
        var best = new Dictionary<int, double>();
        queue.Enqueue(new(-1, 0, at, at, null, null), at);
        while (queue.TryDequeue(out var current, out _))
        {
            if (best.TryGetValue(current.Node, out double old) && old <= current.Arrival) continue;
            best[current.Node] = current.Arrival;
            if (Array.IndexOf(Goals, current.Node) >= 0) return Unpack(current);
            var choices = current.Node < 0 ? Connectors(start) : Outgoing[current.Node].Select(id => Edges[id]);
            foreach (var edge in choices)
                queue.Enqueue(new(edge.To, 0, current.Arrival + edge.Duration, current.Arrival, current, edge), current.Arrival + edge.Duration);
        }
        return null;
    }

    /// <summary>Retimes each cached complete route at the exact supplied phase; no time bins.</summary>
    public WaterPlan? Query(int pattern, double at, WaterPoint start)
    {
        PreparePattern(pattern);
        double end = at + horizon + start.Distance(Nodes[0]) / Model.Speed;
        var safe = new WaterInterval[Nodes.Length][];
        var startSafe = SafeAt(start, pattern, at, end);
        WaterPlan? best = null;
        var nearest = Entries.OrderBy(n => Math.Abs(Nodes[n].X - start.X)).Take(2);
        foreach (var route in nearest.Where(routesByEntry.ContainsKey).SelectMany(n => routesByEntry[n]))
        {
            // Recovery can join the remaining suffix from the actual position.
            int first = 0;
            while (first < route.Edges.Length && Nodes[Edges[route.Edges[first]].To].Z >= start.Z) ++first;
            if (first == route.Edges.Length) continue;
            var candidate = Schedule(route.Edges.Skip(first).Select(id => Edges[id]).ToArray(), pattern, at, start,
                safe, startSafe, best?.Arrival ?? double.PositiveInfinity);
            if (candidate != null && (best == null || candidate.Arrival < best.Arrival)) best = candidate;
        }
        return best;
    }

    private WaterInterval[] SafeAt(WaterPoint point, int pattern, double at, double end)
    {
        if (!Model.Terrain.Clear(point, point)) return [];
        var exposures = new List<WaterExposure>();
        for (int s = 0; s < Model.Stations.Length; ++s)
            if (point.Distance(Model.Stations[s]) <= Model.Radius) exposures.Add(new(s, 0, 0));
        return Safe(Forbid(exposures, pattern), at, end);
    }

    private WaterPlan? Schedule(WaterEdge[] edges, int pattern, double at, WaterPoint start,
        WaterInterval[][] safe, WaterInterval[] startSafe, double bestArrival)
    {
        double end = at + horizon + start.Distance(Nodes[0]) / Model.Speed;
        WaterInterval[] SafeNode(int node) => safe[node] ??= Safe(NodeForbidden[pattern][node], at, end);
        var labels = new List<Label>();
        for (int j = 0; j < startSafe.Length; ++j)
            if (startSafe[j].Start <= at && at <= startSafe[j].End) labels.Add(new(-1, j, at, at, null, null));
        for (int step = 0; step < edges.Length && labels.Count > 0; ++step)
        {
            var edge = edges[step];
            if (step == 0 && (edge.From < 0 || start != Nodes[edge.From]))
                edge = MakeEdge(-1, edge.To, edge.Points, edge.Jump, start);
            if (step == 0 && !Clear(edge, start)) return null;
            edge.Forbidden[pattern] ??= Forbid(edge.Exposures, pattern);
            var next = new Dictionary<int, Label>();
            foreach (var current in labels)
            {
                double sourceEnd = current.Node == -1 ? startSafe[current.Interval].End : SafeNode(current.Node)[current.Interval].End;
                for (int j = 0; j < SafeNode(edge.To).Length; ++j)
                {
                    var target = SafeNode(edge.To)[j];
                    double lower = Math.Max(current.Arrival, target.Start - edge.Duration);
                    double upper = Math.Min(sourceEnd, target.End - edge.Duration);
                    upper = Math.Min(upper, bestArrival - edge.Duration);
                    if (lower > upper) continue;
                    double departure = NextDeparture(edge.Forbidden[pattern], lower, upper);
                    double arrival = departure + edge.Duration;
                    if (departure <= upper && (!next.TryGetValue(j, out var previous) || arrival < previous.Arrival))
                        next[j] = new(edge.To, j, arrival, departure, current, edge);
                }
            }
            labels = next.Values.ToList();
        }
        var goal = labels.Where(l => Array.IndexOf(Goals, l.Node) >= 0).MinBy(l => l.Arrival);
        return goal == null ? null : Unpack(goal);
    }

    /// <summary>Fit one area clock and arrangement to current receipts, without an admission threshold.</summary>
    public WaterClock Identify(IReadOnlyList<WaterReceipt> receipts)
    {
        var best = new WaterClock(0, 0, double.PositiveInfinity);
        for (int p = 0; p < Model.Patterns.Length; ++p)
        {
            double shift = 0;
            var residuals = new double[receipts.Count];
            for (int iteration = 0; iteration < 3 && receipts.Count > 0; ++iteration)
            {
                for (int r = 0; r < receipts.Count; ++r) residuals[r] = Residual(receipts[r], p, shift);
                Array.Sort(residuals); shift += residuals[residuals.Length / 2];
            }
            double error = receipts.Count == 0 ? 0 : receipts.Sum(r => Math.Abs(Residual(r, p, shift))) / receipts.Count;
            if (error < best.MeanError) best = new(p, shift, error);
        }
        return best;

        double Residual(WaterReceipt receipt, int pattern, double shift)
        {
            double residual = double.PositiveInfinity;
            foreach (double pulse in Model.Patterns[pattern].Pulses[receipt.Station])
            {
                double d = Mod(receipt.At - shift - pulse + Model.Period / 2, Model.Period) - Model.Period / 2;
                if (Math.Abs(d) < Math.Abs(residual)) residual = d;
            }
            return residual;
        }
    }

    public int Station(WaterPoint point)
    {
        int nearest = 0;
        for (int i = 1; i < Model.Stations.Length; ++i)
            if (point.Distance(Model.Stations[i]) < point.Distance(Model.Stations[nearest])) nearest = i;
        return nearest;
    }

    public void Write(Stream stream, bool includeRoutes = true)
    {
        using var zip = new GZipStream(stream, CompressionLevel.SmallestSize, leaveOpen: true);
        using var w = new BinaryWriter(zip);
        Model.WindowPadding = HazardTiming.Padding;
        w.Write(JsonSerializer.Serialize(Model));
        w.Write(includeRoutes);
        w.Write(Nodes.Length); foreach (var p in Nodes) { w.Write(p.X); w.Write(p.Z); }
        w.Write(Entries.Length); foreach (int entry in Entries) w.Write(entry);
        w.Write(Goals.Length); foreach (int g in Goals) w.Write(g);
        w.Write(Edges.Length);
        foreach (var e in Edges)
        {
            w.Write(e.From); w.Write(e.To); w.Write(e.Jump); w.Write(e.Duration);
            w.Write(e.Points.Length); foreach (var p in e.Points) { w.Write(p.X); w.Write(p.Z); }
            w.Write(e.Exposures.Length);
            foreach (var x in e.Exposures) { w.Write(x.Station); w.Write(x.Start); w.Write(x.End); }
            if (includeRoutes)
                for (int pattern = 0; pattern < Model.Patterns.Length; ++pattern)
                {
                    var spans = e.Forbidden[pattern] ?? Forbid(e.Exposures, pattern);
                    w.Write(spans.Length); foreach (var span in spans) { w.Write(span.Start); w.Write(span.End); }
                }

        }
        w.Write(includeRoutes ? Routes.Length : 0);
        foreach (var r in includeRoutes ? Routes : [])
        {
            w.Write(r.Entry); w.Write(r.Pattern); w.Write(r.SamplePhase); w.Write(r.Duration); w.Write(r.Edges.Length);
            for (int i = 0; i < r.Edges.Length; ++i) { w.Write(r.Edges[i]); w.Write(r.Departures[i]); }
        }
    }

    public static WaterRoutes Read(Stream stream)
    {
        using var zip = new GZipStream(stream, CompressionMode.Decompress, leaveOpen: true);
        using var r = new BinaryReader(zip);
        var cache = new WaterRoutes { Model = JsonSerializer.Deserialize<WaterModel>(r.ReadString())!, storedIntervals = r.ReadBoolean() };
        cache.Nodes = new WaterPoint[r.ReadInt32()];
        for (int i = 0; i < cache.Nodes.Length; ++i) cache.Nodes[i] = new(r.ReadDouble(), r.ReadDouble());
        cache.Entries = new int[r.ReadInt32()]; for (int i = 0; i < cache.Entries.Length; ++i) cache.Entries[i] = r.ReadInt32();
        cache.Goals = new int[r.ReadInt32()]; for (int i = 0; i < cache.Goals.Length; ++i) cache.Goals[i] = r.ReadInt32();
        cache.Edges = new WaterEdge[r.ReadInt32()];
        for (int i = 0; i < cache.Edges.Length; ++i)
        {
            var e = new WaterEdge { From = r.ReadInt32(), To = r.ReadInt32(), Jump = r.ReadInt32(), Duration = r.ReadDouble() };
            e.Points = new WaterPoint[r.ReadInt32()]; for (int j = 0; j < e.Points.Length; ++j) e.Points[j] = new(r.ReadDouble(), r.ReadDouble());
            e.Exposures = new WaterExposure[r.ReadInt32()];
            for (int j = 0; j < e.Exposures.Length; ++j) e.Exposures[j] = new(r.ReadInt32(), r.ReadDouble(), r.ReadDouble());
            e.Forbidden = new WaterInterval[cache.Model.Patterns.Length][];
            if (cache.storedIntervals)
                for (int pattern = 0; pattern < e.Forbidden.Length; ++pattern)
                {
                    e.Forbidden[pattern] = new WaterInterval[r.ReadInt32()];
                    for (int j = 0; j < e.Forbidden[pattern].Length; ++j) e.Forbidden[pattern][j] = new(r.ReadDouble(), r.ReadDouble());
                    double padding = HazardTiming.Padding - cache.Model.WindowPadding;
                    if (padding > 0)
                    {
                        // Expand the union once on load, including cycle wrap.
                        // Live queries continue to use precomputed intervals.
                        var spans = new List<WaterInterval>();
                        foreach (var span in e.Forbidden[pattern])
                            cache.AddPeriodic(spans, span.Start - padding, span.End + padding);
                        e.Forbidden[pattern] = Merge(spans);
                    }
                    else if (padding < 0) e.Forbidden[pattern] = cache.Forbid(e.Exposures, pattern);
                }
            cache.Edges[i] = e;
        }
        cache.Routes = new WaterRoute[r.ReadInt32()];
        for (int i = 0; i < cache.Routes.Length; ++i)
        {
            var route = new WaterRoute { Entry = r.ReadInt32(), Pattern = r.ReadInt32(), SamplePhase = r.ReadDouble(), Duration = r.ReadDouble() };
            route.Edges = new int[r.ReadInt32()]; route.Departures = new double[route.Edges.Length];
            for (int j = 0; j < route.Edges.Length; ++j) { route.Edges[j] = r.ReadInt32(); route.Departures[j] = r.ReadDouble(); }
            cache.Routes[i] = route;
        }
        cache.Model.WindowPadding = HazardTiming.Padding;
        cache.Index();
        return cache;
    }
}
