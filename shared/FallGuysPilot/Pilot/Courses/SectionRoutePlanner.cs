using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed class SectionRoutePlanner(PilotCourseGeometry data) : IDisposable
{
    internal sealed record Result(int Section, Vector2 Start, double At, double HitAt, long Revision,
        bool Forecast, HitTimedRoutes.Plan Plan, double Milliseconds, string? Error = null,
        KnightColumnPlanner.Metrics? Columns = null);

    private sealed record Area(HitTimedRoutes Graph, int[] Global, Dictionary<int, int> Local);
    private readonly Dictionary<int, Area> areas = [];
    private readonly HazardField field = new([]);
    private KnightColumnPlanner? columns;
    private readonly object worker = new();
    private CancellationTokenSource? cancellation;
    private Task<Result>? pending;
    internal bool Pending => pending != null;
    internal double LeadSeconds { get; private set; } = 2;
    internal double ReadySeconds { get; private set; } = .25;

    internal int Locate(Vector3 feet)
    {
        if (!data.IsKnight && feet.Y < 9 && feet.Z is > -335 and < -262 && Math.Abs(feet.X) > 9.5f) return 3;
        for (int i = data.Sections.Length - 1; i > 0; --i)
            if (feet.Z <= data.Sections[i].Entries.Max(n => data.Nodes[n].Feet.Z) + .001f) return i;
        return 0;
    }

    internal void Start(int section, HazardField hazards, Vector2 feet, double at, bool canWait,
        HitTimedRoutes.Edge[] connectors, double hitAt, bool forecast,
        HitForcedMarch? forced = null, PilotPose? pose = null,
        HitTimedRoutes.Interval? immobile = null, float? arrowHeading = null, float arrowRate = -MathF.Tau, float? entryArrowHeading = null)
    {
        Cancel();
        var snapshot = hazards.Snapshot(at);
        var source = cancellation = new();
        var token = source.Token;
        pending = Task.Run(() =>
        {
            lock (worker)
            {
                token.ThrowIfCancellationRequested();
                return Search(section, snapshot, feet, at, canWait, connectors, hitAt, forecast, token, forced, pose,
                    immobile, arrowHeading, arrowRate, entryArrowHeading);
            }
        }, token);
    }

    internal Result Search(int section, HazardField snapshot, Vector2 feet, double at, bool canWait,
        HitTimedRoutes.Edge[] connectors, double hitAt = 0, bool forecast = false, CancellationToken token = default,
        HitForcedMarch? forced = null, PilotPose? pose = null,
        HitTimedRoutes.Interval? immobile = null, float? arrowHeading = null, float arrowRate = -MathF.Tau, float? entryArrowHeading = null)
    {
        long began = Stopwatch.GetTimestamp();
        try
        {
            if (forced != null && pose is { } actual)
            {
                var approach = forced.Native ? null : Search(section, snapshot, feet, at,
                    HitCourseGeometry.CanWait(actual), connectors, hitAt, false, token).Plan;
                token.ThrowIfCancellationRequested();
                var prepared = forced.Connectors(data, actual, at, 6, approach).ToArray();
                token.ThrowIfCancellationRequested();
                var result = Search(section, snapshot, feet, at, false, prepared, hitAt, forecast, token);
                return result with { Milliseconds = Stopwatch.GetElapsedTime(began).TotalMilliseconds };
            }
            bool geometryChanged = field.ApplySnapshot(snapshot);
            if (geometryChanged) { areas.Clear(); columns?.InvalidateContacts(); }
            if (data.IsKnight && section == 1)
            {
                columns ??= new(data);
                var start = pose?.Feet ?? new Vector3(feet.X, data.Ground.At(feet) ?? 17.48449f, feet.Y);
                var selected = columns.Search(field, start, pose?.Has(3693) == true, at, canWait, connectors, token, immobile,
                    pose?.Raw.RealFrameSeconds ?? 1d / 30);
                return new(section, feet, at, hitAt, snapshot.RouteRevision, forecast, selected,
                    Stopwatch.GetElapsedTime(began).TotalMilliseconds, Columns: columns.LastMetrics);
            }
            if (!areas.TryGetValue(section, out var area))
            {
                var definition = data.Sections[section];
                var global = definition.Nodes.Distinct().ToArray();
                var local = global.Select((id, index) => (id, index)).ToDictionary(p => p.id, p => p.index);
                var exits = definition.Exits.ToHashSet();
                var template = data.Graph(6);
                var nodes = global.Select(n => template.Nodes[n] with { Goal = exits.Contains(n) }).ToArray();
                var edges = data.Edges.Where(e => local.ContainsKey(e.From) && local.ContainsKey(e.To))
                    .Select(e => data.Edge(local[e.From], local[e.To], e.Moves, 6)).ToArray();
                area = new(new(nodes, edges, data.Ground), global, local);
                areas.Add(section, area);
            }
            area.Graph.SearchCancellation = token;
            var sources = connectors.Where(e => area.Local.ContainsKey(e.To)).Select(e =>
                new HitTimedRoutes.Edge(area.Global.Length, area.Local[e.To], e.Path)
                { Motion = e.Motion, ArrowStart = e.ArrowStart, ArrowEnd = e.ArrowEnd }).ToArray();
            var plan = area.Graph.Search(field, feet, at, canWait, sourceEdges: sources,
                immobile: immobile, arrowHeading: arrowHeading, arrowRate: arrowRate, entryArrowHeading: entryArrowHeading);
            plan = plan with { Legs = plan.Legs.Select(l => new HitTimedRoutes.Leg(new(
                l.Edge.From == area.Global.Length ? data.Nodes.Length : area.Global[l.Edge.From],
                area.Global[l.Edge.To], l.Edge.Path)
                { Motion = l.Edge.Motion, ArrowStart = l.Edge.ArrowStart, ArrowEnd = l.Edge.ArrowEnd }, l.Departure)).ToArray() };
            return new(section, feet, at, hitAt, snapshot.RouteRevision, forecast, plan, Stopwatch.GetElapsedTime(began).TotalMilliseconds);
        }
        catch (OperationCanceledException) { throw; }
        catch (Exception error)
        {
            return new(section, feet, at, hitAt, snapshot.RouteRevision, forecast, new([], at, 0, false),
                Stopwatch.GetElapsedTime(began).TotalMilliseconds, error.ToString());
        }
    }

    internal bool TryTake(out Result? result)
    {
        result = null;
        if (pending is not { IsCompleted: true } done) return false;
        pending = null;
        try { result = done.GetAwaiter().GetResult(); }
        catch (OperationCanceledException) { return false; }
        LeadSeconds = Math.Clamp(Math.Max(LeadSeconds * .9, result.Milliseconds / 1000 * 3 + .25), .5, 4);
        ReadySeconds = Math.Clamp(result.Milliseconds / 1000 * 2 + .05, .1, 1);
        return true;
    }

    internal void Cancel()
    {
        cancellation?.Cancel(); cancellation?.Dispose(); cancellation = null; pending = null;
    }
    public void Dispose() => Cancel();
}
