using System;
using System.Collections.Generic;
using System.IO.Compression;
using System.Linq;
using System.Numerics;
using System.Text.Json;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Pure terrain and motion data. No dependency on compiled programs or their
// execution owner. Native ice uses row 3788's 5 m smoothstep over 50/30 s.
internal sealed class PilotCourseGeometry
{
    public sealed class Vertex
    {
        public float[] Position { get; set; } = [];
        public bool CanWait { get; set; }
        public bool Goal { get; set; }
        public bool WindBrace { get; set; }
        public bool ReturnFloor { get; set; }
        public Vector3 Feet => new(Position[0], Position[1], Position[2]);
    }
    public sealed class Move
    {
        public string Kind { get; set; } = "walk";
        public float[][] Points { get; set; } = [];
        internal HitTimedRoutes.Point[]? TimedPath;
        internal float? Facing;
        private Vector3[]? path;
        public Vector3[] Path => path ??= Points.Select(p => new Vector3(p[0], p[1], p[2])).ToArray();
        public static Move Walk(Vector3 a, Vector3 b) => new()
            { Points = [[a.X, a.Y, a.Z], [b.X, b.Y, b.Z]] };
    }
    public sealed class Route
    {
        public int From { get; set; }
        public int To { get; set; }
        public Move[] Moves { get; set; } = [];
        public bool EmptyOnly { get; set; }
    }
    public Vertex[] Nodes { get; set; } = [];
    public Route[] Edges { get; set; } = [];
    internal sealed record Section(string Name, int[] Nodes, int[] Entries, int[] Exits);
    internal Section[] Sections = [];
    public HitGround Ground = null!;
    private bool knight;
    internal Func<PilotPose, float, IEnumerable<HitTimedRoutes.Edge>>? CustomConnectors;
    internal Func<Vector2, float?>? CustomHeight;
    internal Func<Vector3, Vector2, bool>? CustomClear;
    internal float? Height(Vector2 point) => CustomHeight != null ? CustomHeight(point) : Ground.At(point);
    internal bool Clear(Vector3 from, Vector2 to) => CustomClear != null ? CustomClear(from, to) : Ground.ClearFromGrounded(from, to);
    private readonly Dictionary<(float, bool), HitTimedRoutes> graphs = [];

    public static T Read<T>(string name)
    {
        using var stream = typeof(PilotCourseGeometry).Assembly.GetManifestResourceStream($"MrBean.Pilot.{name}")!;
        using var gzip = new GZipStream(stream, CompressionMode.Decompress);
        return JsonSerializer.Deserialize<T>(gzip)!;
    }
    public static PilotCourseGeometry Load(string name)
    {
        if (name == "hit") return HitCourseGeometry.Load();
        var data = Read<PilotCourseGeometry>($"{name}-routes.json.gz");
        data.Ground = Read<HitGround>($"{name}-ground.json.gz");
        data.Ground.PrepareCollision();
        data.knight = name == "knight";
        return data;
    }
    public HitTimedRoutes Graph(float speed, bool carrying = false)
    {
        if (graphs.TryGetValue((speed, carrying), out var graph)) return graph;
        graph = new(Nodes.Select(n => new HitTimedRoutes.Node(n.Feet.XZ(), n.CanWait, n.Goal, knight && InArrow(n.Feet), n.WindBrace)).ToArray(),
            Edges.Where(e => !carrying || !e.EmptyOnly).Select(e => Edge(e.From, e.To, e.Moves, speed)).ToArray(), Ground);
        graphs.Add((speed, carrying), graph); return graph;
    }
    internal HitTimedRoutes.Edge Edge(int from, int to, Move[] moves, float speed)
    {
        var first = moves[0].Path; var last = moves[^1].Path;
        return new(from, to, Trajectory(moves, speed))
        {
            Motion = moves,
            ArrowStart = knight && moves[0].Kind == "arrow" && first.Length > 1 && first[0] != first[1]
                ? Geo.Heading(Geo.Toward(first[0].XZ(), first[1].XZ())) : null,
            ArrowEnd = knight && InArrow(last[^1]) && last.Length > 1 && last[^1] != last[^2]
                ? Geo.Heading(Geo.Toward(last[^2].XZ(), last[^1].XZ())) : null,
        };
    }
    public static bool InArrow(Vector3 p) => p.X is > -209 and < -191 && p.Z is > -295 and < -246;
    public static HitTimedRoutes.Point[] Trajectory(Move[] moves, float speed)
    {
        var result = new List<HitTimedRoutes.Point>(); double time = 0;
        foreach (var move in moves)
        {
            if (move.TimedPath is { } timed)
            {
                if (result.Count == 0) result.Add(timed[0] with { Time = time });
                foreach (var point in timed.Skip(1)) result.Add(point with { Time = time + point.Time });
                time += timed[^1].Time;
                continue;
            }
            var path = move.Path;
            if (result.Count == 0) result.Add(new(path[0].XZ(), time));
            if (move.Kind == "ice")
            {
                for (int i = 1; i <= 50; ++i)
                {
                    float u = i / 50f, progress = u * u * (3 - 2 * u);
                    result.Add(new(Vector2.Lerp(path[0].XZ(), path[^1].XZ(), progress), time + i / 30d));
                }
                time += 50d / 30;
            }
            else
            {
                if (move.Kind is "jump" or "drop") result[^1] = result[^1] with { NeedsSupport = false };
                for (int i = 1; i < path.Length; ++i)
                {
                    time += Vector2.Distance(path[i - 1].XZ(), path[i].XZ()) / (move.Kind == "jump" ? 6 : speed);
                    result.Add(new(path[i].XZ(), time));
                }
            }
        }
        return result.ToArray();
    }
    public IEnumerable<HitTimedRoutes.Edge> Connectors(PilotPose pose, float speed)
    {
        if (CustomConnectors != null)
        {
            foreach (var edge in CustomConnectors(pose, speed)) yield return edge;
            yield break;
        }
        for (int to = 0; to < Nodes.Length; ++to)
        {
            var target = Nodes[to].Feet;
            float distance = Vector2.Distance(pose.XZ, target.XZ());
            if (distance > 20 || Math.Abs(target.Y - pose.Feet.Y) > Math.Max(.8f, distance)) continue;
            Move move;
            if (pose.Has(3693))
            {
                foreach (var sequence in IceConnections(pose.Feet, target))
                    yield return Edge(Nodes.Length, to, sequence, speed);
                continue;
            }
            else
            {
                if (knight && CrossesIce(pose.Feet, target) || !(pose.Grounded && Ground.CollisionFaces.Length > 0
                    ? Ground.ClearFromGrounded(pose.Feet, target.XZ()) : Ground.Clear(pose.XZ, target.XZ()))) continue;
                move = Move.Walk(pose.Feet, target);
                if (pose.Has(3694) || knight && (InArrow(pose.Feet) || InArrow(target))) move.Kind = "arrow";
            }
            yield return Edge(Nodes.Length, to, [move], speed);
        }
    }

    // EventRange 10048823. Ordinary connectors may approach the ice but must
    // not silently turn a walk into a native 5 m / 50-frame slide.
    private static bool InIce(Vector3 p) => p.X is > -215 and < -185 && p.Z is > -204 and < -180;
    private static bool CrossesIce(Vector3 a, Vector3 b)
    {
        float lo = 0, hi = 1;
        bool Clip(float start, float delta, float min, float max)
        {
            if (Math.Abs(delta) < 1e-6f) return start > min && start < max;
            float x = (min - start) / delta, y = (max - start) / delta;
            lo = Math.Max(lo, Math.Min(x, y)); hi = Math.Min(hi, Math.Max(x, y));
            return lo < hi;
        }
        return Clip(a.X, b.X - a.X, -215, -185) && Clip(a.Z, b.Z - a.Z, -204, -180);
    }

    private IEnumerable<Move[]> IceConnections(Vector3 start, Vector3 target)
    {
        float distance = Vector2.Distance(start.XZ(), target.XZ());
        if (distance < .02f)
        {
            var join = Move.Walk(start, start); join.Kind = "join";
            yield return [join]; yield break;
        }
        if (distance > 10 || target.Z > start.Z + .1f) yield break;
        Move Slide(Vector3 a, Vector3 b)
        { var slide = Move.Walk(a, b); slide.Kind = "ice"; return slide; }
        Vector3 Feet(Vector2 p) => new(p.X, Ground.At(p) ?? start.Y, p.Y);
        if (Math.Abs(distance - 5) < .02f && Ground.Clear(start.XZ(), target.XZ()))
            yield return [Slide(start, target)];
        // Two radius-5 circles provide exact native steps from an arbitrary
        // stopped position. Both lateral solutions are candidates for CTC.
        var delta = (target.XZ() - start.XZ()) / distance;
        var middle = (start.XZ() + target.XZ()) * .5f;
        float height = MathF.Sqrt(Math.Max(0, 25 - distance * distance * .25f));
        foreach (float side in new[] { -1f, 1f })
        {
            var via = Feet(middle + new Vector2(-delta.Y, delta.X) * (height * side));
            if (!InIce(via) || !Ground.Clear(start.XZ(), via.XZ()) || !Ground.Clear(via.XZ(), target.XZ())) continue;
            yield return [Slide(start, via), Slide(via, target)];
        }
        // Near the exit the final full slide can pass a graph vertex. Once
        // outside the ice, an ordinary connector reaches that vertex.
        if (distance < 5 && !InIce(target))
        {
            var end = Feet(start.XZ() + delta * 5);
            if (!InIce(end) && Ground.Clear(start.XZ(), end.XZ()) && Ground.Clear(end.XZ(), target.XZ()))
                yield return [Slide(start, end), Move.Walk(end, target)];
        }
    }
}
