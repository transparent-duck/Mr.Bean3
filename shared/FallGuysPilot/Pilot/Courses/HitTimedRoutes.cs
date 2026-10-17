using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Hit's remaining areas: cache spatial contact intervals (CTC), then propagate
// continuous reachable time intervals through the whole area. No delay/time bins.
// A node's state includes its movement law: walking may wait; forced motion may
// not. Keeping whole arrival intervals is essential for non-stopping sections:
// reaching a steering node earlier does not dominate reaching it later.
internal sealed class HitTimedRoutes
{
    internal readonly record struct Interval(double Start, double End);
    internal readonly record struct Point(Vector2 Position, double Time, bool NeedsSupport = true);
    internal sealed record Node(Vector2 Position, bool CanWait, bool Goal = false, bool Arrow = false, bool WindBrace = false);
    internal sealed class Edge(int from, int to, Point[] path)
    {
        public int From = from, To = to;
        public Point[] Path = path;
        public double Duration => Path[^1].Time;
        public readonly Dictionary<HazardField.Station, Interval[]> Contacts = [];
        public object? Motion;
        public float? ArrowStart, ArrowEnd;
    }
    internal sealed record Leg(Edge Edge, double Departure);
    internal sealed record Plan(Leg[] Legs, double Arrival, int Expanded, bool Found);
    private sealed record Label(int Node, Interval Times, double IncomingEnd, Label? Parent, Edge? Edge,
        float? Heading = null, double? TurnDelay = null);

    public readonly List<Node> Nodes = [];
    public readonly List<Edge>[] Outgoing;
    public readonly float EndZ;
    // Crown's ordinary vertices can all wait. It can keep one earliest label
    // per safe interval instead of propagating general forced-motion ranges.
    public bool UseWaitingSearch;
    private readonly double[] remaining;
    private readonly List<HazardField.Station> knownStations = [];
    private readonly Dictionary<HazardField.Station, bool[]> nodeContacts = [];
    private HazardField? geometryOwner;
    private readonly HitGround ground;
    private readonly Func<Vector2, Vector2, Point[]> motion;
    private readonly Node[] windBraces;
    private readonly int[] nonWaiting;
    private readonly int timingLevels;
    private readonly Dictionary<float, Dictionary<(int Node, float? Heading), double>> turnRemaining = [];

    private bool AtWindBrace(Vector2 position)
    {
        foreach (var node in windBraces)
            if (node.WindBrace && Vector2.DistanceSquared(node.Position, position) <= .12f * .12f) return true;
        return false;
    }

    public HitTimedRoutes(float[] rows, float[] lanes, Func<Vector2, bool> canWait,
        Func<Vector2, Vector2, Point[]> motion, float[]? extraRows = null)
    {
        ground = HitGround.Loaded.Value;
        EndZ = rows[^1]; this.motion = motion;
        timingLevels = rows.Length;
        var levels = extraRows == null ? rows : rows.Concat(extraRows).Distinct().OrderDescending().ToArray();
        var byRow = new Dictionary<float, List<int>>();
        foreach (float z in levels)
        {
            var ids = byRow[z] = [];
            foreach (float x in lanes)
                if (ground.At(new(x, z)) != null)
                { ids.Add(Nodes.Count); Nodes.Add(new(new(x, z), canWait(new(x, z)), z == EndZ)); }
        }
        Outgoing = Enumerable.Range(0, Nodes.Count).Select(_ => new List<Edge>()).ToArray();
        for (int row = 0; row < levels.Length; ++row)
        {
            var ids = byRow[levels[row]];
            int original = Array.IndexOf(rows, levels[row]);
            foreach (int a in ids)
            {
                if (row + 1 < levels.Length)
                    foreach (int b in byRow[levels[row + 1]]) Add(b);
                // Refinement adds steering choices without deleting a direct
                // connection or changing its original constant-motion path.
                if (original >= 0 && original + 1 < rows.Length && rows[original + 1] != levels[row + 1])
                    foreach (int b in byRow[rows[original + 1]]) Add(b);
                if (Nodes[a].CanWait)
                    foreach (int b in ids)
                        if (Math.Abs(a - b) == 1) Add(b);
                void Add(int b)
                {
                    var edge = Connect(a, b, Nodes[a].Position);
                    if (edge != null) Outgoing[a].Add(edge);
                }
            }
        }
        windBraces = Nodes.Where(n => n.WindBrace).ToArray();
        nonWaiting = Enumerable.Range(0, Nodes.Count).Where(i => !Nodes[i].CanWait).ToArray();
        remaining = Remaining(null);
    }

    // Explicit course graphs share only the continuous-time search. They own
    // their terrain, goals and native motion descriptions.
    public HitTimedRoutes(Node[] nodes, Edge[] edges, HitGround terrain)
    {
        ground = terrain; motion = Walk; EndZ = nodes.Min(n => n.Position.Y);
        timingLevels = nodes.Select(n => n.Position.Y).Distinct().Count();
        Nodes.AddRange(nodes);
        Outgoing = Enumerable.Range(0, Nodes.Count).Select(_ => new List<Edge>()).ToArray();
        foreach (var edge in edges) Outgoing[edge.From].Add(edge);
        windBraces = Nodes.Where(n => n.WindBrace).ToArray();
        nonWaiting = Enumerable.Range(0, Nodes.Count).Where(i => !Nodes[i].CanWait).ToArray();
        remaining = Remaining(null);
    }

    private double[] Remaining(IReadOnlySet<int>? goals)
    {
        var result = Enumerable.Repeat(double.PositiveInfinity, Nodes.Count).ToArray();
        var incoming = Enumerable.Range(0, Nodes.Count).Select(_ => new List<Edge>()).ToArray();
        foreach (var edges in Outgoing) foreach (var edge in edges) incoming[edge.To].Add(edge);
        var queue = new PriorityQueue<int, double>();
        for (int i = 0; i < Nodes.Count; ++i)
            if (goals?.Contains(i) ?? Nodes[i].Goal) { result[i] = 0; queue.Enqueue(i, 0); }
        while (queue.TryDequeue(out int at, out double cost))
        {
            if (result[at] != cost) continue;
            foreach (var edge in incoming[at])
                if (cost + edge.Duration < result[edge.From])
                { result[edge.From] = cost + edge.Duration; queue.Enqueue(edge.From, result[edge.From]); }
        }
        return result;
    }

    // Arrow headings continue rotating while stationary. Ignoring the required
    // turns makes A* expand thousands of distinct arrival phases before it can
    // reach a goal. Reverse Dijkstra on (vertex, incoming heading) includes those
    // unavoidable turns, but no hazards, so it remains a lower bound.
    private Dictionary<(int Node, float? Heading), double> TurnRemaining(float rate, IReadOnlySet<int>? goals)
    {
        if (goals == null && turnRemaining.TryGetValue(rate, out var cached)) return cached;
        var states = Nodes.Select(_ => new HashSet<float?>()).ToArray();
        for (int i = 0; i < Nodes.Count; ++i) if (!Nodes[i].Arrow) states[i].Add(null);
        foreach (var edges in Outgoing)
            foreach (var edge in edges) states[edge.To].Add(Nodes[edge.To].Arrow ? edge.ArrowEnd : null);
        var incoming = new Dictionary<(int Node, float? Heading), List<((int Node, float? Heading) From, double Cost)>>();
        var result = new Dictionary<(int Node, float? Heading), double>();
        for (int node = 0; node < Nodes.Count; ++node)
            foreach (var heading in states[node])
            { result[(node, heading)] = double.PositiveInfinity; incoming[(node, heading)] = []; }
        for (int node = 0; node < Nodes.Count; ++node)
            foreach (var heading in states[node])
                foreach (var edge in Outgoing[node])
                {
                    var target = (edge.To, Nodes[edge.To].Arrow ? edge.ArrowEnd : null);
                    double turn = heading is { } from && edge.ArrowStart is { } to ? ArrowTurn(from, to, rate) : 0;
                    incoming[target].Add(((node, heading), turn + edge.Duration));
                }
        var queue = new PriorityQueue<(int Node, float? Heading), double>();
        foreach (var key in result.Keys)
            if (goals?.Contains(key.Node) ?? Nodes[key.Node].Goal)
            { result[key] = 0; queue.Enqueue(key, 0); }
        while (queue.TryDequeue(out var key, out double cost))
        {
            if (result[key] != cost) continue;
            foreach (var previous in incoming[key])
                if (cost + previous.Cost < result[previous.From])
                { result[previous.From] = cost + previous.Cost; queue.Enqueue(previous.From, cost + previous.Cost); }
        }
        if (goals == null) turnRemaining.Add(rate, result);
        return result;
    }

    private Edge? Connect(int from, int to, Vector2 position, Func<Vector2, Vector2, Point[]>? actualMotion = null)
    {
        var path = (actualMotion ?? motion)(position, Nodes[to].Position);
        if (path.Length < 2) return null;
        for (int i = 1; i < path.Length; ++i)
            if (!ground.Clear(path[i - 1].Position, path[i].Position)) return null;
        return new(from, to, path);
    }

    // Static edges retain CTC when only receipt times change. Actual-position
    // connectors are calculated once for that query, with the same geometry law.
    private void PrepareGeometry(HazardField field)
    {
        if (!ReferenceEquals(geometryOwner, field))
        {
            geometryOwner = field; knownStations.Clear(); nodeContacts.Clear();
            foreach (var edges in Outgoing) foreach (var edge in edges) edge.Contacts.Clear();
        }
        float minZ = Nodes.Min(n => n.Position.Y), maxZ = Nodes.Max(n => n.Position.Y);
        foreach (var station in field.Stations)
        {
            if (nodeContacts.ContainsKey(station)) continue;
            if (station.Origin.Y + station.Extent + PilotEngine.BodyRadius < minZ ||
                station.Origin.Y - station.Extent - PilotEngine.BodyRadius > maxZ) continue;
            knownStations.Add(station);
            nodeContacts.Add(station, Nodes.Select(n => station.Contains(n.Position, PilotEngine.BodyRadius)).ToArray());
            foreach (var edges in Outgoing)
                foreach (var edge in edges)
                {
                    var contacts = Contacts(edge.Path, station);
                    if (contacts.Length > 0) edge.Contacts.Add(station, contacts);
                }
        }
    }

    // Reuse the route's CTC when a new cast or wind observation arrives.
    // Only a conflict requests another search; unrelated receipts keep the route.
    public bool HasConflict(HazardField field, Plan plan, int firstLeg, Vector2 feet, double now, long afterRevision = 0)
    {
        int previousStations = knownStations.Count;
        PrepareGeometry(field);
        foreach (var leg in plan.Legs)
            if (leg.Edge.From == Nodes.Count)
                for (int i = previousStations; i < knownStations.Count; ++i)
                {
                    var station = knownStations[i];
                    var contacts = Contacts(leg.Edge.Path, station);
                    if (contacts.Length > 0) leg.Edge.Contacts.TryAdd(station, contacts);
                }
        double end = Math.Max(now + 6, plan.Arrival);
        field.Prepare(now, end - now);
        bool HoldConflict(Vector2 position, double start, double finish)
        {
            if (finish < start) return false;
            bool braced = AtWindBrace(position);
            foreach (var station in knownStations)
                if (!(braced && station.Spec.Note == "crystal-wind") &&
                    station.ChangedAt > afterRevision && station.Contains(position, PilotEngine.BodyRadius))
                    foreach (var window in station.Windows)
                        if (window.Start <= finish && window.End >= start) return true;
            return false;
        }
        var point = feet; double at = now;
        for (int i = firstLeg; i < plan.Legs.Length; ++i)
        {
            var leg = plan.Legs[i];
            if (HoldConflict(point, at, leg.Departure)) return true;
            foreach (var pair in leg.Edge.Contacts)
                if (pair.Key.ChangedAt > afterRevision)
                foreach (var contact in pair.Value)
                {
                    double start = Math.Max(now, leg.Departure + contact.Start);
                    double finish = leg.Departure + contact.End;
                    if (finish < start) continue;
                    foreach (var window in pair.Key.Windows)
                        if (window.Start <= finish && window.End >= start) return true;
                }
            at = Math.Max(now, leg.Departure + leg.Edge.Duration);
            point = Nodes[leg.Edge.To].Position;
        }
        return HoldConflict(point, at, firstLeg >= plan.Legs.Length ? end : plan.Arrival);
    }

    public Plan Search(HazardField field, Vector2 start, double now, bool canWait,
        Func<Vector2, Vector2, Point[]>? actualMotion = null,
        IEnumerable<Edge>? sourceEdges = null, IReadOnlySet<int>? goals = null, double? surviveUntil = null,
        Interval? immobile = null, float? arrowHeading = null, float arrowRate = -MathF.Tau, double windExecution = 1.5,
        double waitHorizon = 12)
    {
        if (!UseWaitingSearch || surviveUntil != null) return Run(waitHorizon);
        PrepareGeometry(field);
        field.Prepare(now, 0);
        if (knownStations.Any(s => s.Contains(start, PilotEngine.BodyRadius) &&
            s.Windows.Any(w => w.Start <= now && now <= w.End))) return new([], now, 0, false);
        sourceEdges = sourceEdges?.ToArray();
        int expanded = 0;
        for (double slack = Math.Min(6, waitHorizon); ; slack = Math.Min(slack * 2, waitHorizon))
        {
            var plan = Run(slack); expanded += plan.Expanded;
            // A route found in the shorter time window is still globally
            // earliest. A miss expands to the caller's original full horizon.
            if (plan.Found || slack >= waitHorizon) return plan with { Expanded = expanded };
        }
        Plan Run(double horizon) => SearchCore(field, start, now, canWait, actualMotion, sourceEdges, goals,
            surviveUntil, immobile, arrowHeading, arrowRate, windExecution, horizon);
    }

    private Plan SearchCore(HazardField field, Vector2 start, double now, bool canWait,
        Func<Vector2, Vector2, Point[]>? actualMotion = null,
        IEnumerable<Edge>? sourceEdges = null, IReadOnlySet<int>? goals = null, double? surviveUntil = null,
        Interval? immobile = null, float? arrowHeading = null, float arrowRate = -MathF.Tau, double windExecution = 1.5,
        double waitHorizon = 12)
    {
        PrepareGeometry(field);
        var remaining = surviveUntil != null ? new double[Nodes.Count] : goals == null ? this.remaining : Remaining(goals);
        int source = Nodes.Count;
        var connectors = new List<Edge>();
        IEnumerable<Edge> ActualConnectors()
        {
            if (sourceEdges != null) { foreach (var edge in sourceEdges) yield return edge; yield break; }
            for (int to = 0; to < Nodes.Count; ++to)
                if (Nodes[to].Position.Y <= start.Y && Connect(source, to, start, actualMotion) is { } edge) yield return edge;
        }
        foreach (var edge in ActualConnectors())
        {
            foreach (var station in knownStations)
            {
                if (edge.Contacts.ContainsKey(station)) continue;
                var contacts = Contacts(edge.Path, station);
                // Cache misses too: a wider time horizon does not change
                // an actual-position connector's spatial intersection.
                edge.Contacts.Add(station, contacts);
            }
            connectors.Add(edge);
        }
        double shortest = connectors.Select(e => e.Duration + remaining[e.To]).DefaultIfEmpty(0).Min();
        if (UseWaitingSearch && surviveUntil == null && !double.IsFinite(shortest))
            return new([], now, 0, false);
        var turns = surviveUntil == null && Nodes.Any(n => n.Arrow) &&
            Outgoing.All(edges => edges.All(e => !Nodes[e.To].Arrow || e.ArrowEnd != null))
            ? TurnRemaining(arrowRate, goals) : null;
        double Heuristic(int node, float? heading)
        {
            if (turns == null) return remaining[node];
            if (turns.TryGetValue((node, heading), out double exact)) return exact;
            if (goals?.Contains(node) ?? Nodes[node].Goal) return 0;
            // Actual-position connectors may produce an incoming heading that
            // is absent from the static graph. Only this first turn is new.
            double best = double.PositiveInfinity;
            foreach (var edge in Outgoing[node])
            {
                double turn = heading is { } from && edge.ArrowStart is { } to ? ArrowTurn(from, to, arrowRate) : 0;
                double onward = turns.GetValueOrDefault((edge.To, Nodes[edge.To].Arrow ? edge.ArrowEnd : null), remaining[edge.To]);
                best = Math.Min(best, turn + edge.Duration + onward);
            }
            return best;
        }
        double sourceRemaining = turns == null ? shortest : connectors.Select(e => e.Duration +
            (arrowHeading is { } from && e.ArrowStart is { } to ? ArrowTurn(from, to, arrowRate) : 0) +
            Heuristic(e.To, Nodes[e.To].Arrow ? e.ArrowEnd : null)).DefaultIfEmpty(0).Min();
        double period = knownStations.Select(s => s.Period).DefaultIfEmpty(0).Max();
        // Materialization extent, not a runtime stop condition. It covers a
        // crossing plus one full cycle per forward level. A miss returns no route;
        // it never retries without hazard windows.
        double end = surviveUntil ?? now + (double.IsFinite(shortest) ? shortest : Math.Abs(start.Y - EndZ) / 2) +
            (sourceEdges != null ? waitHorizon : period * (timingLevels + 1) + 1);
        field.Prepare(now, end - now);
        bool sourceBraced = AtWindBrace(start);
        var windLocks = knownStations.Where(s => s.Spec.Note == "crystal-wind")
            .SelectMany(s => s.Windows.Select(w => new Interval(w.Start, w.End + windExecution))).ToArray();
        var safe = new Interval[Nodes.Count + 1][];
        Interval[] SafeNode(int id)
        {
            if (safe[id] is { } cached) return cached;
            bool braced = id == source ? sourceBraced : Nodes[id].WindBrace;
            return safe[id] = Complement(Merge(knownStations.Where(s => !(braced && s.Spec.Note == "crystal-wind") && (id == source
                    ? s.Contains(start, PilotEngine.BodyRadius) : nodeContacts[s][id]))
                .SelectMany(s => s.Windows.Select(w => new Interval(w.Start, w.End)))), now, end);
        }
        var forbidden = new Dictionary<Edge, Interval[]>();
        Interval[] Forbidden(Edge edge)
        {
            if (forbidden.TryGetValue(edge, out var cached)) return cached;
            var spans = edge.Contacts.SelectMany(pair => pair.Value.SelectMany(contact =>
                pair.Key.Windows.Select(w => new Interval(w.Start - contact.End, w.End - contact.Start))));
            if (immobile is { } stop && edge.Duration > 0)
                spans = spans.Append(new(stop.Start - edge.Duration, stop.End));
            // A supported wall stops the displacement, not its native motion
            // phase. Only stationary occupancy accepts this specific wind;
            // every approach/departure still checks the wind footprint.
            if (edge.From == source ? sourceBraced : Nodes[edge.From].WindBrace)
                spans = spans.Concat(windLocks);
            return forbidden[edge] = Merge(spans);
        }

        if (UseWaitingSearch && canWait && arrowHeading == null && surviveUntil == null && immobile == null &&
            nonWaiting.All(i => goals?.Contains(i) ?? Nodes[i].Goal))
            return SearchWaiting();

        Plan SearchWaiting()
        {
            var best = new Dictionary<(int Node, int Interval), Label>();
            var queue = new PriorityQueue<(Label Label, int Interval), double>();
            var initial = SafeNode(source);
            int expanded = 0;
            for (int i = 0; i < initial.Length; ++i)
                if (initial[i].Start <= now && now <= initial[i].End)
                    OfferWait(new(source, new(now, initial[i].End), now, null, null), i);
            while (queue.TryDequeue(out var item, out _))
            {
                var current = item.Label;
                if (!ReferenceEquals(best[(current.Node, item.Interval)], current)) continue;
                ++expanded;
                if (current.Node != source && (goals?.Contains(current.Node) ?? Nodes[current.Node].Goal))
                    return Unpack(current, current.Times.Start, expanded);
                foreach (var edge in current.Node == source ? connectors : Outgoing[current.Node])
                {
                    if (!double.IsFinite(remaining[edge.To])) continue;
                    var target = SafeNode(edge.To);
                    var blocked = Forbidden(edge);
                    for (int i = 0; i < target.Length; ++i)
                    {
                        if (target[i].Start > current.Times.End + edge.Duration) break;
                        if (target[i].End < current.Times.Start + edge.Duration) continue;
                        double departure = Math.Max(current.Times.Start, target[i].Start - edge.Duration);
                        double last = Math.Min(current.Times.End, target[i].End - edge.Duration);
                        foreach (var span in blocked)
                        {
                            if (span.End < departure) continue;
                            if (span.Start > departure) break;
                            departure = Math.BitIncrement(span.End);
                            if (departure > last) break;
                        }
                        if (departure <= last)
                            OfferWait(new(edge.To, new(departure + edge.Duration, target[i].End), departure + edge.Duration, current, edge), i);
                    }
                }
            }
            return new([], now, expanded, false);

            void OfferWait(Label label, int interval)
            {
                var key = (label.Node, interval);
                if (best.TryGetValue(key, out var previous) && previous.Times.Start <= label.Times.Start) return;
                best[key] = label;
                queue.Enqueue((label, interval), label.Times.Start + (label.Node == source ? shortest : remaining[label.Node]));
            }
        }

        var covered = new Dictionary<(int Node, float? Heading), List<Interval>>();
        var queue = new PriorityQueue<Label, double>();
        foreach (var interval in SafeNode(source))
            if (interval.Start <= now && now <= interval.End)
                Offer(new(source, new(now, canWait && arrowHeading == null ? interval.End : now), now, null, null, arrowHeading));
        int expanded = 0;
        while (queue.TryDequeue(out var label, out _))
        {
            ++expanded;
            if (surviveUntil is { } until && label.Times.End >= until)
                return Unpack(label, label.Times.Start, expanded) with { Arrival = until };
            if (surviveUntil == null && label.Node != source && (goals?.Contains(label.Node) ?? Nodes[label.Node].Goal))
                return Unpack(label, label.Times.Start, expanded);
            foreach (var edge in label.Node == source ? connectors : Outgoing[label.Node])
                foreach (var opportunity in Departures(label, edge))
                foreach (var departure in Subtract(opportunity.Times, Forbidden(edge)))
                    foreach (var target in SafeNode(edge.To))
                    {
                        double a = Math.Max(departure.Start + edge.Duration, target.Start);
                        double b = Math.Min(departure.End + edge.Duration, target.End);
                        if (a > b) continue;
                        float? heading = Nodes[edge.To].Arrow ? edge.ArrowEnd ?? label.Heading : null;
                        Offer(new(edge.To, new(a, Nodes[edge.To].CanWait && heading == null ? target.End : b),
                            b, label, edge, heading, opportunity.TurnDelay));
                    }
        }
        return new([], now, expanded, false);

        IEnumerable<(Interval Times, double? TurnDelay)> Departures(Label label, Edge edge)
        {
            if (label.Heading is not { } heading || edge.ArrowStart is not { } desired)
            { yield return (label.Times, null); yield break; }
            double period = Math.Tau / Math.Abs(arrowRate);
            double turn = ArrowTurn(heading, desired, arrowRate);
            double holdEnd = SafeNode(label.Node).First(s => s.Start <= label.Times.Start && s.End >= label.Times.Start).End;
            for (double delay = turn; label.Times.Start + delay <= holdEnd; delay += period)
                yield return (new(label.Times.Start + delay, Math.Min(label.Times.End + delay, holdEnd)), delay);
        }

        void Offer(Label label)
        {
            // Dominance is coverage of reachable times at the same physical
            // state, not simply earliest arrival at a node that cannot stop.
            var key = (label.Node, label.Heading);
            if (!covered.TryGetValue(key, out var coverage)) covered.Add(key, coverage = []);
            var fresh = Uncovered(label.Times, coverage).ToArray();
            foreach (var span in fresh)
            {
                var part = label with { Times = span };
                queue.Enqueue(part, span.Start + (label.Node == source ? sourceRemaining : Heuristic(label.Node, label.Heading)));
            }
            if (fresh.Length > 0)
            {
                foreach (var span in fresh) Cover(coverage, span);
            }
        }
    }

    // Coverage stays sorted and disjoint. Point arrival phases can be numerous;
    // do not rescan and sort the whole accumulated list on every offered label.
    private static int CoverageStart(List<Interval> coverage, double time)
    {
        int lo = 0, hi = coverage.Count;
        while (lo < hi)
        {
            int mid = lo + (hi - lo) / 2;
            if (coverage[mid].End < time) lo = mid + 1; else hi = mid;
        }
        return lo;
    }
    private static IEnumerable<Interval> Uncovered(Interval source, List<Interval> coverage)
    {
        double at = source.Start;
        for (int i = CoverageStart(coverage, at); i < coverage.Count; ++i)
        {
            var span = coverage[i];
            if (span.Start > source.End) break;
            if (span.Start > at) yield return new(at, Math.BitDecrement(span.Start));
            at = Math.Max(at, Math.BitIncrement(span.End));
            if (at > source.End) yield break;
        }
        if (at <= source.End) yield return new(at, source.End);
    }
    private static void Cover(List<Interval> coverage, Interval span)
    {
        int start = CoverageStart(coverage, Math.BitDecrement(span.Start)), end = start;
        while (end < coverage.Count && coverage[end].Start <= Math.BitIncrement(span.End))
        {
            span = new(Math.Min(span.Start, coverage[end].Start), Math.Max(span.End, coverage[end].End));
            ++end;
        }
        if (end > start) coverage.RemoveRange(start, end - start);
        coverage.Insert(start, span);
    }

    public static double ArrowTurn(float from, float to, float rate)
    {
        double angle = (to - from) * Math.Sign(rate) % Math.Tau;
        if (angle < 0) angle += Math.Tau;
        // Identical float headings can differ at the wrap boundary.
        if (angle < 1e-6 || Math.Tau - angle < 1e-6) return 0;
        return angle / Math.Abs(rate);
    }

    private static Plan Unpack(Label goal, double arrival, int expanded)
    {
        var legs = new List<Leg>(); double time = arrival;
        for (var label = goal; label.Edge != null; label = label.Parent!)
        {
            double depart = Math.Min(time, label.IncomingEnd) - label.Edge.Duration;
            legs.Add(new(label.Edge, depart)); time = depart - (label.TurnDelay ?? 0);
        }
        legs.Reverse(); return new(legs.ToArray(), arrival, expanded, true);
    }

    internal static Interval[] Merge(IEnumerable<Interval> spans)
    {
        var result = new List<Interval>();
        foreach (var span in spans.OrderBy(s => s.Start))
            if (result.Count > 0 && span.Start <= Math.BitIncrement(result[^1].End))
                result[^1] = new(result[^1].Start, Math.Max(result[^1].End, span.End));
            else result.Add(span);
        return result.ToArray();
    }

    private static IEnumerable<Interval> Subtract(Interval source, IEnumerable<Interval> forbidden)
    {
        double at = source.Start;
        foreach (var span in forbidden)
        {
            if (span.End < at) continue;
            if (span.Start > source.End) break;
            if (span.Start > at) yield return new(at, Math.BitDecrement(span.Start));
            at = Math.Max(at, Math.BitIncrement(span.End));
            if (at > source.End) yield break;
        }
        if (at <= source.End) yield return new(at, source.End);
    }
    private static Interval[] Complement(Interval[] forbidden, double start, double end)
        => Subtract(new(start, end), forbidden).ToArray();

    public static Point[] Walk(Vector2 start, Vector2 end)
        => [new(start, 0), new(end, Vector2.Distance(start, end) / Actuator.WalkSpeed)];

    // Exact contact interval on each constant-velocity segment. The capsule is
    // the union of its rectangle and endpoint discs; their line intersection is
    // convex. Curved motion uses its stored piecewise-linear trajectory.
    private static Interval[] Contacts(Point[] path, HazardField.Station station)
    {
        var result = new List<Interval>();
        float extent = station.Extent + PilotEngine.BodyRadius;
        var min = path[0].Position; var max = min;
        foreach (var point in path) { min = Vector2.Min(min, point.Position); max = Vector2.Max(max, point.Position); }
        if (station.Origin.X + extent < min.X || station.Origin.X - extent > max.X ||
            station.Origin.Y + extent < min.Y || station.Origin.Y - extent > max.Y) return [];
        for (int i = 1; i < path.Length; ++i)
        {
            var a = path[i - 1]; var b = path[i];
            if (station.Spec.Note == "support" && !a.NeedsSupport) continue;
            var parts = new List<Interval>();
            double radius = station.Spec.Size + PilotEngine.BodyRadius;
            switch (station.Spec.Shape)
            {
                case HazardShape.Circle: Circle(station.Origin, radius); break;
                case HazardShape.HalfDisc:
                {
                    Circle(station.Origin, radius);
                    var f = Geo.FromHeading(station.Rotation);
                    double start = Vector2.Dot(a.Position - station.Origin, f), velocity = Vector2.Dot(b.Position - a.Position, f);
                    for (int j = parts.Count - 1; j >= 0; --j)
                    {
                        var part = parts[j];
                        if (velocity == 0) { if (start < -PilotEngine.BodyRadius) parts.RemoveAt(j); continue; }
                        double crossing = (-PilotEngine.BodyRadius - start) / velocity;
                        var clipped = velocity > 0 ? part with { Start = Math.Max(part.Start, crossing) }
                            : part with { End = Math.Min(part.End, crossing) };
                        if (clipped.Start > clipped.End) parts.RemoveAt(j); else parts[j] = clipped;
                    }
                    break;
                }
                case HazardShape.Square: Box(a.Position - station.Origin, b.Position - station.Origin,
                    -radius, radius, -radius, radius); break;
                case HazardShape.Rect:
                {
                    var f = Geo.FromHeading(station.Rotation); var side = new Vector2(f.Y, -f.X);
                    Vector2 Local(Vector2 p) => new(Vector2.Dot(p - station.Origin, side), Vector2.Dot(p - station.Origin, f));
                    Box(Local(a.Position), Local(b.Position), -radius, radius, station.Spec.ForwardMin - PilotEngine.BodyRadius,
                        station.Spec.Length + PilotEngine.BodyRadius); break;
                }
                default:
                {
                    var axis = station.Endpoint - station.Origin; double length = axis.Length();
                    Circle(station.Origin, radius); Circle(station.Endpoint, radius);
                    if (length > 0)
                    {
                        var f = axis / (float)length; var side = new Vector2(f.Y, -f.X);
                        Vector2 Local(Vector2 p) => new(Vector2.Dot(p - station.Origin, side), Vector2.Dot(p - station.Origin, f));
                        Box(Local(a.Position), Local(b.Position), -radius, radius, 0, length);
                    }
                    break;
                }
            }
            foreach (var part in parts)
                result.Add(new(a.Time + (b.Time - a.Time) * part.Start, a.Time + (b.Time - a.Time) * part.End));

            void Circle(Vector2 center, double r)
            {
                // Float dot products can move a long connector's contact
                // boundary by several microseconds. Preserve double precision
                // from the coordinate differences through the quadratic.
                double dx = (double)b.Position.X - a.Position.X, dz = (double)b.Position.Y - a.Position.Y;
                double qx = (double)a.Position.X - center.X, qz = (double)a.Position.Y - center.Y;
                double aa = dx * dx + dz * dz, bb = qx * dx + qz * dz, cc = qx * qx + qz * qz - r * r;
                if (aa == 0) { if (cc <= 0) parts.Add(new(0, 1)); return; }
                double discriminant = bb * bb - aa * cc;
                if (discriminant < 0) return;
                double root = Math.Sqrt(discriminant), from = Math.Max(0, (-bb - root) / aa), to = Math.Min(1, (-bb + root) / aa);
                if (from <= to) parts.Add(new(from, to));
            }
            void Box(Vector2 p, Vector2 q, double xmin, double xmax, double zmin, double zmax)
            {
                double from = 0, to = 1;
                if (Clip(p.X, q.X - p.X, xmin, xmax) && Clip(p.Y, q.Y - p.Y, zmin, zmax)) parts.Add(new(from, to));
                bool Clip(double value, double velocity, double min, double max)
                {
                    if (velocity == 0) return value >= min && value <= max;
                    double t0 = (min - value) / velocity, t1 = (max - value) / velocity;
                    from = Math.Max(from, Math.Min(t0, t1)); to = Math.Min(to, Math.Max(t0, t1)); return from <= to;
                }
            }
        }
        return Merge(result);
    }
}
