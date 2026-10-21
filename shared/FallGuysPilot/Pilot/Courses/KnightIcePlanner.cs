using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Threading;
using Interval = MrBean.Modules.FallGuysPilot.Courses.HitTimedRoutes.Interval;
using Move = MrBean.Modules.FallGuysPilot.Courses.PilotCourseGeometry.Move;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Owned by the section worker. Generate native actions only at expanded states;
// measured feet are an exact root, never a correction to a baked ice node.
internal sealed partial class KnightIcePlanner(PilotCourseGeometry source)
{
    private const double SlideSeconds = 50d / 30;
    private const float PositionError = .24f;
    private static readonly (int X, int Z)[] Coarse = [(0, -50), (-30, -40), (30, -40), (-40, -30), (40, -30)];
    private static readonly (int X, int Z)[] Extra = [(-14, -48), (14, -48), (-48, -14), (48, -14),
        (-50, 0), (50, 0), (-30, 40), (30, 40)];

    private sealed class Node(Vector3 feet, int id, bool goal)
    {
        internal readonly Vector3 Feet = feet;
        internal readonly int Id = id;
        internal readonly bool Goal = goal;
        internal readonly List<Arc> Out = [];
        internal int Expanded = -1, SafeQuery = -1;
        internal Interval[] Safe = [];
    }
    private sealed class Arc(Node from, Node to, bool ice)
    {
        internal readonly Node From = from, To = to;
        internal readonly bool Ice = ice;
        internal readonly double NativeSeconds = ice ? SlideSeconds : Vector2.Distance(from.Feet.XZ(), to.Feet.XZ()) / 6;
        internal bool? Clear;
        internal int Geometry = -1, Query = -1;
        internal readonly Dictionary<HazardField.Station, Interval[]> Contacts = [];
        internal Interval[] Blocked = [];
    }
    private sealed record Label(Node Node, int Safe, double At, Label? Parent, Arc? Arc, double Departure);

    private readonly Dictionary<Vector3, Node> nodes = [];
    private readonly Dictionary<Node, (Vector3 Origin, int X, int Z)> coordinates = [];
    private readonly Dictionary<(Vector3 Origin, int X, int Z), Node> lattice = [];
    private Node[] exits = [];
    private HazardField.Station[] stations = [];
    private HazardField? owner;
    private int geometry, query, edges, level, expanded;
    private double frame, startup;
    private long began;
    private CancellationToken cancellation;
    private bool budgetHit;
    private PilotCollisionMesh? mesh;
    internal KnightColumnPlanner.Metrics? LastMetrics { get; private set; }

    internal void InvalidateContacts() => ++geometry;

    internal HitTimedRoutes.Plan Search(HazardField field, Vector3 feet, double at, double frameSeconds,
        CancellationToken token, Interval? immobile = null)
    {
        began = Stopwatch.GetTimestamp(); cancellation = token; token.ThrowIfCancellationRequested();
        // Bound retained geometry across displaced roots without changing feet.
        if (nodes.Count > 12000 && !nodes.ContainsKey(feet))
        { nodes.Clear(); coordinates.Clear(); lattice.Clear(); exits = []; edges = 0; }
        double nextFrame = Math.Clamp(frameSeconds, 1d / 240, .2);
        var current = field.Stations.Where(s => s.Origin.X + s.Extent + s.Spec.Pad > -216 &&
            s.Origin.X - s.Extent - s.Spec.Pad < -184 &&
            s.Origin.Y + s.Extent + s.Spec.Pad > -212 &&
            s.Origin.Y - s.Extent - s.Spec.Pad < -179).ToArray();
        if (!ReferenceEquals(owner, field) || !stations.SequenceEqual(current) || nextFrame != frame) ++geometry;
        owner = field; stations = current; frame = nextFrame; startup = .10 + 2 * frame;
        field.Prepare(at, 60); ++query;
        if (exits.Length == 0)
            exits = source.Sections[1].Exits.Select(id => Local(source.Nodes[id].Feet, id)).ToArray();
        var root = Root(feet);
        double built = Stopwatch.GetElapsedTime(began).TotalMilliseconds;
        expanded = 0; level = 0; budgetHit = false;
        var plan = Search(root, at, immobile);
        bool refined = false;
        if (!budgetHit && (!plan.Found || plan.Arrival - at > Remaining(root) + .25))
        {
            refined = true; level = 1;
            var alternative = Search(root, at, immobile);
            if (alternative.Found && (!plan.Found || alternative.Arrival < plan.Arrival)) plan = alternative;
        }
        token.ThrowIfCancellationRequested();
        LastMetrics = new(built, 0, Stopwatch.GetElapsedTime(began).TotalMilliseconds - built,
            nodes.Count, edges, Implicit: true, Refined: refined, BudgetHit: budgetHit);
        return plan;
    }

    private Node Local(Vector3 feet, int? exit = null)
    {
        if (!nodes.TryGetValue(feet, out var node))
            nodes.Add(feet, node = new(feet, exit ?? source.Nodes.Length + 1 + nodes.Count, exit != null));
        return node;
    }
    private Node Root(Vector3 feet)
    {
        var node = Local(feet);
        if (coordinates.TryAdd(node, (feet, 0, 0))) lattice[(feet, 0, 0)] = node;
        return node;
    }
    private Node? Target(Node from, int x, int z)
    {
        var key = coordinates[from];
        var next = (key.Origin, key.X + x, key.Z + z);
        if (lattice.TryGetValue(next, out var found)) return found;
        var p = key.Origin + new Vector3(next.Item2 / 10f, 0, next.Item3 / 10f);
        if (p.X is <= -214.75f or >= -185.25f || p.Z >= -180.25f ||
            source.Ground.At(p.XZ()) is not { } y) return null;
        p.Y = y;
        var node = Local(p); lattice[next] = node; coordinates.TryAdd(node, next);
        return node;
    }
    private void Expand(Node node)
    {
        if (node.Goal || node.Expanded >= level) return;
        if (node.Feet.Z <= -204)
        {
            if (node.Expanded < 0)
                foreach (var exit in exits)
                    if (!ReferenceEquals(node, exit) && Vector2.DistanceSquared(node.Feet.XZ(), exit.Feet.XZ()) <= 64)
                    { node.Out.Add(new(node, exit, false)); ++edges; }
            node.Expanded = level; return;
        }
        void Add((int X, int Z)[] directions)
        {
            foreach (var (x, z) in directions)
                if (Target(node, x, z) is { } target)
                { node.Out.Add(new(node, target, true)); ++edges; }
        }
        if (node.Expanded < 0) Add(Coarse);
        if (level > 0) Add(Extra);
        node.Expanded = level;
    }
    private Interval[] Safe(Node node)
    {
        if (node.SafeQuery == query) return node.Safe;
        var blocked = HitTimedRoutes.Merge(stations.Where(s => s.Contains(node.Feet.XZ(), s.Spec.Pad))
            .SelectMany(s => s.Windows.Select(w => new Interval(w.Start, w.End))));
        var result = new List<Interval>(); double at = double.NegativeInfinity;
        foreach (var b in blocked)
        {
            if (b.Start > at) result.Add(new(at, Math.BitDecrement(b.Start)));
            at = Math.Max(at, Math.BitIncrement(b.End));
        }
        result.Add(new(at, double.PositiveInfinity));
        node.SafeQuery = query; return node.Safe = result.ToArray();
    }
    private bool Prepare(Arc arc, Interval? immobile)
    {
        cancellation.ThrowIfCancellationRequested();
        arc.Clear ??= source.Ground.ClearSupported(arc.From.Feet.XZ(), arc.To.Feet.XZ()) &&
            (!arc.Ice || TubeClear(arc.From.Feet.XZ(), arc.To.Feet.XZ()));
        if (!arc.Clear.Value) return false;
        if (arc.Geometry != geometry)
        {
            arc.Contacts.Clear();
            foreach (var station in stations)
            {
                var contact = Contact(arc, station);
                if (contact.Length > 0) arc.Contacts.Add(station, contact);
            }
            arc.Geometry = geometry;
        }
        if (arc.Query == query) return true;
        var blocked = new List<Interval>();
        foreach (var (station, contacts) in arc.Contacts)
            foreach (var c in contacts)
                foreach (var w in station.Windows) blocked.Add(new(w.Start - c.End, w.End - c.Start));
        if (immobile is { } stop) blocked.Add(new(stop.Start - Duration(arc), stop.End));
        arc.Blocked = HitTimedRoutes.Merge(blocked); arc.Query = query; return true;
    }
    private double Duration(Arc arc) => arc.NativeSeconds + (arc.Ice ? startup : 0);
    private double Remaining(Node node)
    {
        if (node.Goal) return 0;
        if (node.Feet.Z <= -204) return exits.Min(n => Vector2.Distance(node.Feet.XZ(), n.Feet.XZ())) / 6;
        int slides = Math.Max(0, (int)Math.Ceiling((node.Feet.Z + 204) / 5));
        return slides * (SlideSeconds + startup) + Math.Max(0, node.Feet.Z - slides * 5 + 210) / 6;
    }
    private HitTimedRoutes.Plan Search(Node root, double now, Interval? immobile)
    {
        var best = new Dictionary<(Node, int), Label>();
        var queue = new PriorityQueue<Label, double>();
        void Offer(Label label)
        {
            var key = (label.Node, label.Safe);
            if (best.TryGetValue(key, out var old) && old.At <= label.At) return;
            best[key] = label; queue.Enqueue(label, label.At + Remaining(label.Node));
        }
        double Align(double at) => now + Math.Ceiling((at - now) / frame - 1e-8) * frame;
        var first = Safe(root);
        for (int i = 0; i < first.Length; ++i)
            if (first[i].Start <= now && now <= first[i].End) Offer(new(root, i, now, null, null, now));
        while (queue.TryDequeue(out var current, out _))
        {
            cancellation.ThrowIfCancellationRequested();
            if (!ReferenceEquals(best[(current.Node, current.Safe)], current)) continue;
            ++expanded;
            if (current.Node.Goal) return Unpack(current);
            if (expanded > 8000 || Stopwatch.GetElapsedTime(began).TotalMilliseconds > 200)
            { budgetHit = true; break; }
            Expand(current.Node);
            double end = Math.Min(Safe(current.Node)[current.Safe].End, now + 60);
            foreach (var arc in current.Node.Out)
            {
                if (!Prepare(arc, immobile)) continue;
                double duration = Duration(arc);
                var intervals = Safe(arc.To);
                for (int i = 0; i < intervals.Length; ++i)
                {
                    var safe = intervals[i];
                    if (safe.End < current.At + duration) continue;
                    if (safe.Start > end + duration) break;
                    double departure = Align(Math.Max(current.At, safe.Start - duration));
                    double last = Math.Min(end, safe.End - duration);
                    foreach (var blocked in arc.Blocked)
                    {
                        if (blocked.End < departure) continue;
                        if (blocked.Start > departure) break;
                        departure = Align(Math.BitIncrement(blocked.End) + 1e-8);
                        if (departure > last) break;
                    }
                    if (departure <= last) Offer(new(arc.To, i, departure + duration, current, arc, departure));
                }
            }
        }
        return new([], now, expanded, false);
    }
    private HitTimedRoutes.Plan Unpack(Label final)
    {
        var legs = new List<HitTimedRoutes.Leg>();
        for (var label = final; label.Arc is { } arc; label = label.Parent!)
        {
            var move = Move.Walk(arc.From.Feet, arc.To.Feet); move.Kind = arc.Ice ? "ice" : "walk";
            var path = PilotCourseGeometry.Trajectory([move], 6).ToList();
            if (Duration(arc) > path[^1].Time) path.Add(new(path[^1].Position, Duration(arc)));
            // Live validation fills its own contacts. Even station keys must
            // not escape: the worker updates those objects on the next query.
            var edge = new HitTimedRoutes.Edge(label.Parent!.Parent == null ? source.Nodes.Length : arc.From.Id,
                arc.To.Id, path.ToArray())
            {
                Motion = new[] { move }, Destination = new(arc.To.Feet.XZ(), true, arc.To.Goal),
            };
            legs.Add(new(edge, label.Departure));
        }
        legs.Reverse(); return new(legs.ToArray(), final.At, expanded, true);
    }
}
