using System;
using System.Collections.Generic;
using System.Linq;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed partial class HitTimedRoutes
{
    private TimingCache? timing;

    internal void PrepareTimings(HazardField field, double now, double end)
    {
        PrepareGeometry(field);
        (timing ??= new(this)).Prepare(field, now, end);
    }

    private static Interval[] DepartureWindows(Edge edge)
    {
        var spans = new List<Interval>();
        foreach (var (station, contacts) in edge.Contacts)
            foreach (var contact in contacts)
                foreach (var window in station.Windows)
                    spans.Add(new(window.Start - contact.End, window.End - contact.Start));
        return MergePrepared(spans);
    }

    private static Interval[] MergePrepared(List<Interval> spans)
    {
        if (spans.Count == 0) return [];
        spans.Sort(static (a, b) => a.Start.CompareTo(b.Start));
        int count = 0;
        for (int i = 0; i < spans.Count; ++i)
        {
            var span = spans[i];
            if (count > 0 && span.Start <= Math.BitIncrement(spans[count - 1].End))
                spans[count - 1] = new(spans[count - 1].Start, Math.Max(spans[count - 1].End, span.End));
            else spans[count++] = span;
        }
        var result = new Interval[count];
        spans.CopyTo(0, result, 0, count);
        return result;
    }

    private sealed class PreparedStation
    {
        internal (double Start, double End)[] Windows = [];
        internal long Changed;
    }

    private sealed class PreparedNode
    {
        internal readonly List<PreparedStation> Stations = [];
        internal long Built = -1;
        internal Interval[] Forbidden = [];
    }

    private sealed class PreparedEdge(Edge edge)
    {
        internal readonly Edge Edge = edge;
        internal (PreparedStation Station, Interval[] Contacts)[] Links = [];
        internal int ContactCount = -1;
        internal long Built = -1;
        internal Interval[] Forbidden = [];
    }

    private sealed class TimingCache
    {
        private const double CoverageStep = 30;
        private readonly HitTimedRoutes graph;
        private readonly Dictionary<HazardField.Station, PreparedStation> stations = [];
        internal readonly Dictionary<Edge, PreparedEdge> Edges = [];
        private readonly PreparedEdge[] edges;
        internal readonly PreparedNode[] Nodes;
        private readonly List<Interval> spans = [];
        private int linkedStations;
        private long revision;
        private double start = double.NaN, end;

        internal TimingCache(HitTimedRoutes graph)
        {
            this.graph = graph;
            Nodes = Enumerable.Range(0, graph.Nodes.Count).Select(_ => new PreparedNode()).ToArray();
            edges = graph.Outgoing.SelectMany(e => e).Select(e => new PreparedEdge(e)).ToArray();
            foreach (var edge in edges) Edges.Add(edge.Edge, edge);
        }

        internal void Prepare(HazardField field, double now, double finish)
        {
            if (!(now >= start && finish <= end) || !field.Covers(start, end))
            {
                start = Math.Floor(now / CoverageStep) * CoverageStep;
                end = Math.Max(start + CoverageStep, Math.Ceiling(finish / CoverageStep) * CoverageStep);
            }
            try
            {
                field.Prepare(start, end - start);
                foreach (var station in field.Stations)
                {
                    if (!stations.TryGetValue(station, out var prepared)) stations.Add(station, prepared = new());
                    var windows = station.Windows;
                    bool changed = windows.Count != prepared.Windows.Length;
                    for (int i = 0; !changed && i < windows.Count; ++i)
                        changed = windows[i] != prepared.Windows[i];
                    if (!changed) continue;
                    prepared.Windows = windows.ToArray();
                    prepared.Changed = ++revision;
                }
                for (; linkedStations < graph.knownStations.Count; ++linkedStations)
                {
                    var station = graph.knownStations[linkedStations];
                    var contacts = graph.nodeContacts[station];
                    for (int node = 0; node < Nodes.Length; ++node)
                        if (contacts[node] && !(graph.Nodes[node].WindBrace && station.Spec.Note == "crystal-wind"))
                        {
                            Nodes[node].Stations.Add(stations[station]);
                            Nodes[node].Built = -1;
                        }
                }
                foreach (var node in Nodes)
                {
                    bool changed = node.Built < 0;
                    foreach (var station in node.Stations) changed |= station.Changed > node.Built;
                    if (!changed) continue;
                    spans.Clear();
                    foreach (var station in node.Stations)
                        foreach (var window in station.Windows) spans.Add(new(window.Start, window.End));
                    node.Forbidden = MergePrepared(spans);
                    node.Built = revision;
                }
                foreach (var prepared in edges)
                {
                    var edge = prepared.Edge;
                    if (prepared.ContactCount != edge.Contacts.Count)
                    {
                        prepared.Links = edge.Contacts.Where(p => p.Value.Length > 0)
                            .Select(p => (stations[p.Key], p.Value)).ToArray();
                        prepared.ContactCount = edge.Contacts.Count;
                        prepared.Built = -1;
                    }
                    bool changed = prepared.Built < 0;
                    foreach (var link in prepared.Links) changed |= link.Station.Changed > prepared.Built;
                    if (!changed) continue;
                    spans.Clear();
                    foreach (var link in prepared.Links)
                        foreach (var contact in link.Contacts)
                            foreach (var window in link.Station.Windows)
                                spans.Add(new(window.Start - contact.End, window.End - contact.Start));
                    prepared.Forbidden = MergePrepared(spans);
                    prepared.Built = revision;
                }
            }
            finally { field.Prepare(now, finish - now); }
        }
    }
}
