using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Owns its graph, contact caches and hazard field on one serialized worker.
// Published paths have separate contact dictionaries: live validation must not
// mutate a graph that the next background search is using.
internal sealed class CourseRoutePlanner(CourseId course, bool crownStairBoost = false, int crownFrameRate = 30) : IDisposable
{
    internal sealed record Query(PilotPose Pose, double At, float Speed, bool Carrying,
        HashSet<int>? Goals, HitTimedRoutes.Interval? Immobile, float? ArrowHeading,
        float ArrowRate, double WindExecution, double WaitHorizon, int? Section = null,
        HitTimedRoutes.Edge? Continuation = null, float? EntryArrowHeading = null);
    internal sealed record Result(Query Query, HitTimedRoutes.Plan Plan, int Connectors,
        long Revision, bool Cold, double Milliseconds, string? Error = null);
    private readonly object worker = new();
    private readonly HazardField field = new([]);
    private readonly Dictionary<(float Speed, bool Carrying), HitTimedRoutes> graphs = [];
    private PilotCourseGeometry? data;
    private SectionRoutePlanner? sections;
    private CancellationTokenSource? cancellation;
    private Task<Result>? pending;
    internal bool Pending => pending != null;
    internal bool Warm { get; private set; }

    internal void Start(Query query, HazardField snapshot)
    {
        Cancel();
        var token = (cancellation = new()).Token;
        pending = Task.Run(() =>
        {
            lock (worker)
            {
                token.ThrowIfCancellationRequested();
                return Search(query, snapshot, token);
            }
        }, token);
    }
    internal Result Search(Query query, HazardField snapshot, CancellationToken token = default)
    {
        long began = Stopwatch.GetTimestamp(); bool cold = false;
        try
        {
            data ??= PilotCourseGeometry.Load(course.ToString().ToLowerInvariant(), crownStairBoost, crownFrameRate);
            if (query.Section is { } section)
            {
                cold = sections == null;
                sections ??= new(data);
                var sources = data.IsKnight && section == 1
                    ? query.Continuation is { } remainder ? new[] { remainder } : []
                    : Connectors(query);
                var result = sections.Search(section, snapshot, query.Pose.XZ, query.At,
                    course != CourseId.Hit || HitCourseGeometry.CanWait(query.Pose), sources, token: token, pose: query.Pose,
                    immobile: query.Immobile, arrowHeading: query.ArrowHeading, arrowRate: query.ArrowRate, entryArrowHeading: query.EntryArrowHeading);
                return new(query, Copy(result.Plan), sources.Length, snapshot.RouteRevision, cold,
                    Stopwatch.GetElapsedTime(began).TotalMilliseconds, result.Error);
            }
            if (field.ApplySnapshot(snapshot)) graphs.Clear();
            if (!graphs.TryGetValue((query.Speed, query.Carrying), out var graph))
            {
                cold = true;
                var template = data.Graph(query.Speed, query.Carrying);
                graph = new(template.Nodes.ToArray(), template.Outgoing.SelectMany(e => e).Select(Copy).ToArray(), data.Ground)
                    { UseWaitingSearch = course is CourseId.Crown or CourseId.Ring };
                graphs.Add((query.Speed, query.Carrying), graph);
            }
            graph.SearchCancellation = token;
            token.ThrowIfCancellationRequested();
            var connectors = Connectors(query);
            var plan = graph.Search(field, query.Pose.XZ, query.At, true,
                sourceEdges: connectors, goals: query.Goals, immobile: query.Immobile,
                arrowHeading: query.ArrowHeading, arrowRate: query.ArrowRate,
                windExecution: query.WindExecution, waitHorizon: query.WaitHorizon, entryArrowHeading: query.EntryArrowHeading);
            plan = RouteSchedule.Retime(graph, field, plan, default, query.Pose, query.At,
                new(data, course, query.Speed, query.ArrowRate, query.Immobile),
                query.WindExecution, query.WaitHorizon) ?? plan;
            token.ThrowIfCancellationRequested();
            return new(query, Copy(plan), connectors.Length, snapshot.RouteRevision, cold,
                Stopwatch.GetElapsedTime(began).TotalMilliseconds);
        }
        catch (OperationCanceledException) { throw; }
        catch (Exception error)
        {
            return new(query, new([], query.At, 0, false), 0, snapshot.RouteRevision, cold,
                Stopwatch.GetElapsedTime(began).TotalMilliseconds, error.ToString());
        }
    }
    private HitTimedRoutes.Edge[] Connectors(Query query)
    {
        var connectors = query.Section is { } section
            ? data!.EntryConnectors(query.Pose, query.Speed, data.Sections[section].Entries).AsEnumerable()
            : data!.Connectors(query.Pose, query.Speed);
        if (query.Continuation is { } remainder) connectors = connectors.Append(remainder);
        return connectors.ToArray();
    }
    internal static HitTimedRoutes.Edge Copy(HitTimedRoutes.Edge edge) => new(edge.From, edge.To, edge.Path)
        { Motion = edge.Motion, Destination = edge.Destination, ArrowStart = edge.ArrowStart, ArrowEnd = edge.ArrowEnd };
    internal static HitTimedRoutes.Plan Copy(HitTimedRoutes.Plan plan) => plan with
        { Legs = plan.Legs.Select(l => new HitTimedRoutes.Leg(Copy(l.Edge), l.Departure)).ToArray() };
    internal bool TryTake(out Result? result)
    {
        result = null;
        if (pending is not { IsCompleted: true } done) return false;
        pending = null;
        try { result = done.GetAwaiter().GetResult(); }
        catch (OperationCanceledException) { return false; }
        Warm = true;
        return true;
    }
    internal void Cancel()
    {
        cancellation?.Cancel(); cancellation?.Dispose(); cancellation = null; pending = null;
    }
    public void Dispose() { Cancel(); sections?.Dispose(); }
}
