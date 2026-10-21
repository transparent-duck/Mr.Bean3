using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Threading;
using Move = MrBean.Modules.FallGuysPilot.Courses.PilotCourseGeometry.Move;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Owned by the section worker. The approach lattice is built once; a stopped
// ice position uses an implicit native-action search, never the dense approach.
internal sealed class KnightColumnPlanner(PilotCourseGeometry source)
{
    private Area? approach, actual;
    private Vector3 actualFeet;
    private bool actualIce;
    private KnightIcePlanner? ice;
    private readonly PilotCollisionMesh mesh = new(source.Ground.CollisionFaces);
    internal sealed record Metrics(double BuildMs, double ConnectMs, double SearchMs, int Nodes, int Edges,
        bool Implicit = false, bool Refined = false, bool BudgetHit = false);
    internal Metrics? LastMetrics { get; private set; }

    internal void InvalidateContacts()
    {
        approach?.Graph.InvalidateGeometry();
        actual?.Graph.InvalidateGeometry();
        ice?.InvalidateContacts();
    }

    internal HitTimedRoutes.Plan Search(HazardField field, Vector3 feet, bool onIce, double at,
        bool canWait, HitTimedRoutes.Edge[] supplied, CancellationToken token, HitTimedRoutes.Interval? immobile = null,
        double frameSeconds = 1d / 30)
    {
        if (onIce)
        {
            ice ??= new(source);
            var selected = ice.Search(field, feet, at, frameSeconds, token, immobile);
            LastMetrics = ice.LastMetrics;
            return selected;
        }
        long began = Stopwatch.GetTimestamp();
        token.ThrowIfCancellationRequested();
        Area area;
        if (onIce || feet.Z is > -180 and < -175)
        {
            if (actual == null || actualFeet != feet || actualIce != onIce)
            {
                Area? prefix = onIce ? null : approach ??= BuildApproach(token);
                if (prefix != null)
                {
                    prefix.Graph.SearchCancellation = token;
                    prefix.Graph.PrepareGeometry(field);
                }
                var builder = prefix == null ? new Builder(source, mesh, token) :
                    new Builder(source, mesh, token, prefix);
                builder.FromActual(feet, onIce);
                area = builder.Finish(prefix, field);
                // Publish only after a complete build. Cancelled work cannot
                // leave a partial graph in the cache.
                actual = area; actualFeet = feet; actualIce = onIce;
            }
            else area = actual;
        }
        else area = approach ??= BuildApproach(token);
        double built = Stopwatch.GetElapsedTime(began).TotalMilliseconds;
        area.Graph.SearchCancellation = token;
        var connectors = area.Connectors(feet, onIce, token).ToList();
        foreach (var edge in supplied)
        {
            // A continuation may end at a vertex created by another worker.
            // Resolve its coordinates, not that worker's local numeric ID.
            if (area.Positions.TryGetValue(Key(((Move[])edge.Motion!)[^1].Path[^1]), out int to))
                connectors.Add(new(area.Nodes.Length, to, edge.Path) { Motion = edge.Motion });
        }
        double connected = Stopwatch.GetElapsedTime(began).TotalMilliseconds;
        var plan = area.Graph.Search(field, feet.XZ(), at, canWait, sourceEdges: connectors.ToArray(), immobile: immobile);
        token.ThrowIfCancellationRequested();
        LastMetrics = new(built, connected - built, Stopwatch.GetElapsedTime(began).TotalMilliseconds - connected,
            area.Nodes.Length, area.Data.Edges.Length);
        return plan with { Legs = plan.Legs.Select(l => new HitTimedRoutes.Leg(new(
            l.Edge.From == area.Nodes.Length ? source.Nodes.Length : area.Global[l.Edge.From],
            area.Global[l.Edge.To], l.Edge.Path)
        {
            Motion = l.Edge.Motion, Destination = area.Graph.Nodes[l.Edge.To],
        }, l.Departure)).ToArray() };
    }

    private Area BuildApproach(CancellationToken token)
    {
        var builder = new Builder(source, mesh, token);
        builder.Approach(); return builder.Finish();
    }

    private static (int, int, int) Key(Vector3 p)
        => ((int)MathF.Round(p.X * 10000), (int)MathF.Round(p.Y * 10000), (int)MathF.Round(p.Z * 10000));
    private static bool Ice(Vector3 p) => p.X is > -215 and < -185 && p.Z is > -204 and < -180;

    private sealed class Area
    {
        internal readonly PilotCourseGeometry Data;
        internal readonly PilotCourseGeometry.Vertex[] Nodes;
        internal readonly HitTimedRoutes Graph;
        internal readonly HitTimedRoutes.Edge[] Edges;
        internal readonly int[] Global;
        internal readonly Dictionary<(int, int, int), int> Positions;
        private readonly int actualRoot;

        internal Area(PilotCourseGeometry data, int[] global, Dictionary<(int, int, int), int> positions,
            int actualRoot, Area? prefix, HazardField? field)
        {
            Data = data; Nodes = data.Nodes; Global = global; Positions = positions; this.actualRoot = actualRoot;
            int oldCount = prefix?.Edges.Length ?? 0;
            Edges = new HitTimedRoutes.Edge[data.Edges.Length];
            for (int i = 0; i < Edges.Length; ++i)
            {
                if (i < oldCount)
                {
                    Edges[i] = CourseRoutePlanner.Copy(prefix!.Edges[i]);
                    foreach (var pair in prefix.Edges[i].Contacts) Edges[i].Contacts.Add(pair.Key, pair.Value);
                }
                else
                {
                    var edge = data.Edges[i]; Edges[i] = data.Edge(edge.From, edge.To, edge.Moves, 6);
                }
            }
            Graph = new(Nodes.Select(n => new HitTimedRoutes.Node(n.Feet.XZ(), n.CanWait, n.Goal)).ToArray(), Edges, data.Ground);
            if (prefix != null) Graph.InheritGeometry(prefix.Graph, field!, Edges[oldCount..]);
        }

        internal IEnumerable<HitTimedRoutes.Edge> Connectors(Vector3 feet, bool onIce, CancellationToken token)
        {
            if (onIce && actualRoot >= 0)
            {
                var join = Move.Walk(feet, feet); join.Kind = "join";
                yield return Data.Edge(Nodes.Length, actualRoot, [join], 6);
                yield break;
            }
            for (int to = 0; to < Nodes.Length; ++to)
            {
                token.ThrowIfCancellationRequested();
                var target = Nodes[to].Feet;
                float distance = Vector2.Distance(feet.XZ(), target.XZ());
                if (distance > 20 || Math.Abs(target.Y - feet.Y) > Math.Max(.8f, distance) ||
                    PilotCourseGeometry.CrossesIce(feet, target) || !Data.Ground.ClearFromGrounded(feet, target.XZ())) continue;
                yield return Data.Edge(Nodes.Length, to, [Move.Walk(feet, target)], 6);
            }
        }
    }

    private sealed class Builder
    {
        private readonly PilotCourseGeometry source;
        private readonly PilotCollisionMesh mesh;
        private readonly CancellationToken token;
        private readonly List<PilotCourseGeometry.Vertex> nodes = [];
        private readonly List<PilotCourseGeometry.Route> edges = [];
        private readonly List<int> global = [];
        private readonly Dictionary<(int, int, int), int> positions = [];
        private readonly HashSet<(int, int, string)> known = [];
        private readonly Dictionary<(int, int), bool> clear = [];
        private readonly int[] exits, seeds;
        private readonly Move[] jumps;
        private int actualRoot = -1;

        internal Builder(PilotCourseGeometry source, PilotCollisionMesh mesh, CancellationToken token)
        {
            this.source = source; this.mesh = mesh; this.token = token;
            var map = new Dictionary<int, int>();
            foreach (int id in source.Sections[1].Nodes)
            {
                map.Add(id, nodes.Count); global.Add(id);
                var old = source.Nodes[id];
                positions.TryAdd(Key(old.Feet), nodes.Count);
                nodes.Add(new() { Position = old.Position, CanWait = old.CanWait,
                    Goal = source.Sections[1].Exits.Contains(id) });
            }
            foreach (var edge in source.Edges)
                if (map.TryGetValue(edge.From, out int a) && map.TryGetValue(edge.To, out int b))
                {
                    edges.Add(new() { From = a, To = b, Moves = edge.Moves });
                    known.Add((a, b, edge.Moves[0].Kind));
                }
            exits = Enumerable.Range(0, nodes.Count).Where(i => nodes[i].Feet.Z <= -204).ToArray();
            seeds = edges.Where(e => e.Moves.Any(m => m.Kind == "ice") && nodes[e.From].Feet.Z > -184)
                .Select(e => e.From).Distinct().ToArray();
            jumps = edges.Where(e => e.Moves.Length == 1 && e.Moves[0].Kind == "jump" &&
                Ice(nodes[e.To].Feet) && !Ice(nodes[e.From].Feet)).Select(e => e.Moves[0]).ToArray();
        }
        internal Builder(PilotCourseGeometry source, PilotCollisionMesh mesh, CancellationToken token, Area prepared)
            : this(source, mesh, token)
        {
            nodes.Clear(); nodes.AddRange(prepared.Nodes);
            edges.Clear(); edges.AddRange(prepared.Data.Edges);
            global.Clear(); global.AddRange(prepared.Global);
            positions.Clear(); foreach (var pair in prepared.Positions) positions.Add(pair.Key, pair.Value);
            known.Clear(); foreach (var edge in edges) known.Add((edge.From, edge.To, edge.Moves[0].Kind));
        }
        private Vector3? Feet(float x, float z)
            => source.Ground.At(new(x, z)) is { } y ? new(x, y, z) : null;
        private int Add(Vector3 p)
        {
            if (positions.TryGetValue(Key(p), out int old)) return old;
            int id = nodes.Count; positions.Add(Key(p), id);
            nodes.Add(new() { Position = [p.X, p.Y, p.Z], CanWait = true });
            global.Add(source.Nodes.Length + 1 + id); return id;
        }
        private bool Clear(int a, int b)
        {
            var key = a < b ? (a, b) : (b, a);
            if (!clear.TryGetValue(key, out bool ok))
            {
                token.ThrowIfCancellationRequested();
                clear.Add(key, ok = source.Ground.Clear(nodes[a].Feet.XZ(), nodes[b].Feet.XZ()));
            }
            return ok;
        }
        private void Edge(int a, int b, string kind)
        {
            if (!known.Add((a, b, kind))) return;
            var move = Move.Walk(nodes[a].Feet, nodes[b].Feet); move.Kind = kind;
            edges.Add(new() { From = a, To = b, Moves = [move] });
        }
        internal void Approach()
        {
            int oldCount = nodes.Count;
            for (int z = -164; z >= -178; --z)
                for (int x = -212; x <= -188; ++x)
                    if (Feet(x, z) is { } p && source.Ground.Clear(p.XZ(), p.XZ())) Add(p);
            var landings = new List<int>(seeds);
            for (int z = -176; z >= -179; --z)
                for (int x = -212; x <= -188; ++x) AddJumps(new(x, z), landings);
            WalkConnections(oldCount);
            Slides(landings, five: true);
        }
        internal void FromActual(Vector3 feet, bool onIce)
        {
            if (onIce)
            {
                actualRoot = Add(feet);
                Slides([actualRoot], five: false);
            }
            else
            {
                int oldCount = nodes.Count;
                var landings = new List<int>(); AddJumps(feet.XZ(), landings);
                WalkConnections(oldCount); Slides(landings, five: true);
            }
        }
        private void AddJumps(Vector2 takeoff, List<int> landings)
        {
            foreach (var jump in jumps)
            {
                token.ThrowIfCancellationRequested();
                var a = jump.Path[0]; var b = jump.Path[^1];
                if (Feet(takeoff.X, takeoff.Y) is not { } p ||
                    Feet(takeoff.X + b.X - a.X, takeoff.Y + b.Z - a.Z) is not { } q ||
                    !Ice(q) || Math.Abs(p.Y - a.Y) > .1 || Math.Abs(q.Y - b.Y) > .1 || !JumpClear(p, q)) continue;
                int from = Add(p), to = Add(q); Edge(from, to, "jump"); landings.Add(to);
            }
        }
        private bool JumpClear(Vector3 a, Vector3 b)
        {
            if (!source.Ground.Clear(a.XZ(), a.XZ()) || !source.Ground.Clear(b.XZ(), b.XZ())) return false;
            double duration = Vector2.Distance(a.XZ(), b.XZ()) / 6;
            double startup = Math.Max(0, duration - 2 * 10.4 / 30);
            var previous = a + Vector3.UnitY * 1.01f;
            int count = (int)Math.Ceiling(duration * 120);
            for (int i = 1; i <= count; ++i)
            {
                double t = duration * i / count, u = Math.Max(0, t - startup);
                var p = Vector3.Lerp(a, b, (float)(t / duration));
                p.Y += (float)Math.Max(0, 10.4 * u - 15 * u * u) + 1.01f;
                if (!mesh.Clear(previous, p, source.Ground.BodyRadius + .0001f)) return false;
                previous = p;
            }
            return true;
        }
        private void WalkConnections(int oldCount)
        {
            // Four-metre buckets bound neighbor enumeration. Original edges
            // remain, including longer connections authored before refinement.
            var buckets = new Dictionary<(int, int), List<int>>();
            for (int i = 0; i < nodes.Count; ++i)
            {
                var p = nodes[i].Feet; if (p.Z < -180) continue;
                var key = ((int)MathF.Floor(p.X / 4), (int)MathF.Floor(p.Z / 4));
                if (!buckets.TryGetValue(key, out var ids)) buckets.Add(key, ids = []);
                ids.Add(i);
            }
            foreach (var (key, ids) in buckets)
            foreach (int a in ids)
            for (int dx = -1; dx <= 1; ++dx)
            for (int dz = -1; dz <= 1; ++dz)
            {
                if (!buckets.TryGetValue((key.Item1 + dx, key.Item2 + dz), out var nearby)) continue;
                foreach (int b in nearby)
                {
                    if (a == b || a < oldCount && b < oldCount) continue;
                    var p = nodes[a].Feet; var q = nodes[b].Feet;
                    if (q.Z > p.Z + .001f || Vector2.DistanceSquared(p.XZ(), q.XZ()) > 16 || !Clear(a, b)) continue;
                    Edge(a, b, "walk");
                }
            }
        }
        private void Slides(IEnumerable<int> roots, bool five)
        {
            (int X, int Z)[] directions = five ? [(0, -5), (-3, -4), (3, -4), (-4, -3), (4, -3)]
                : [(0, -5), (-3, -4), (3, -4)];
            var visited = new HashSet<int>(); var pending = new Queue<int>(roots);
            while (pending.TryDequeue(out int a))
            {
                token.ThrowIfCancellationRequested();
                if (!visited.Add(a)) continue;
                var p = nodes[a].Feet;
                if (p.Z <= -204)
                {
                    foreach (int b in exits)
                        if (a != b && nodes[b].Feet.Z <= p.Z + .001f &&
                            Vector2.DistanceSquared(p.XZ(), nodes[b].Feet.XZ()) <= 64 && Clear(a, b)) Edge(a, b, "walk");
                    continue;
                }
                foreach (var (dx, dz) in directions)
                {
                    if (p.X + dx is <= -215 or >= -185 || Feet(p.X + dx, p.Z + dz) is not { } q) continue;
                    int b = Add(q); if (!Clear(a, b)) continue;
                    Edge(a, b, "ice"); pending.Enqueue(b);
                }
            }
        }
        internal Area Finish(Area? prefix = null, HazardField? field = null)
        {
            token.ThrowIfCancellationRequested();
            if (actualRoot >= 0)
            {
                // A source anchored on ice cannot reach the ordinary approach
                // or the other baked entry lattices. Do not compute CTC for them.
                var outgoing = edges.GroupBy(e => e.From).ToDictionary(g => g.Key, g => g.ToArray());
                var reachable = new HashSet<int>(); var pending = new Queue<int>(); pending.Enqueue(actualRoot);
                while (pending.TryDequeue(out int id))
                {
                    if (!reachable.Add(id) || !outgoing.TryGetValue(id, out var next)) continue;
                    foreach (var edge in next) pending.Enqueue(edge.To);
                }
                var ids = reachable.Order().ToArray();
                var map = ids.Select((id, index) => (id, index)).ToDictionary(p => p.id, p => p.index);
                var keptNodes = ids.Select(i => nodes[i]).ToArray();
                var keptEdges = edges.Where(e => reachable.Contains(e.From)).Select(e => new PilotCourseGeometry.Route
                    { From = map[e.From], To = map[e.To], Moves = e.Moves }).ToArray();
                var keptPositions = new Dictionary<(int, int, int), int>();
                for (int i = 0; i < keptNodes.Length; ++i) keptPositions.TryAdd(Key(keptNodes[i].Feet), i);
                return new(new() { Nodes = keptNodes, Edges = keptEdges, Ground = source.Ground },
                    ids.Select(i => global[i]).ToArray(), keptPositions, map[actualRoot], null, null);
            }
            return new(new() { Nodes = nodes.ToArray(), Edges = edges.ToArray(), Ground = source.Ground },
                global.ToArray(), positions, actualRoot, prefix, field);
        }
    }
}
