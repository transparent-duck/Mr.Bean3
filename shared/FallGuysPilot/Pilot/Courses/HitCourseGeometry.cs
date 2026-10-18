using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using MrBean.FallGuys;
using MrBean.Modules.FallGuysPilot.Water;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed class HitCourseGeometry
{
    private static readonly Lazy<HitCourseGeometry> Loaded = new(() => new());
    internal static PilotCourseGeometry Load() => Loaded.Value.data;
    private readonly PilotCourseGeometry data = new() { Ground = HitGround.Loaded.Value };
    private readonly PilotCourseGeometry opening = HitOpeningStep.InspectionGeometry;
    private readonly WaterRoutes water = WaterSession.SharedCache;
    private readonly HitTimedRoutes column = HitColumnStep.InspectionGraph;
    private readonly HitTimedRoutes steering = HitSteeringStep.InspectionGraph;
    private readonly HitTimedRoutes wet = HitWetStep.InspectionGraph;
    private readonly int waterOffset, columnOffset, steeringOffset, wetOffset, finish;
    private readonly int[] steeringExitTargets;
    private readonly int[] catwalk = new int[2];
    private const double RecoverySeconds = 65d / 30;
    private const float HandoffRange = 4.5f;

    private HitCourseGeometry()
    {
        var nodes = opening.Nodes.Select(n => new PilotCourseGeometry.Vertex
            { Position = n.Position, CanWait = n.CanWait }).ToList();
        var edges = new List<PilotCourseGeometry.Route>(opening.Edges);
        waterOffset = nodes.Count;
        foreach (var node in water.Nodes) AddNode(Feet(node.Vector), true);
        foreach (var edge in water.Edges)
            AddEdge(waterOffset + edge.From, waterOffset + edge.To, WaterMoves(edge, water.Nodes[edge.From]));
        columnOffset = AddTimed(column, "walk");
        steeringOffset = AddTimed(steering, "steering");
        wetOffset = AddTimed(wet, "wet");
        finish = AddNode(new(0, 18.94f, -380), true);
        nodes[finish].Goal = true;
        Join(opening.Nodes.Select((n, i) => (n, i)).Where(p => p.n.Goal).Select(p => p.i),
            water.Entries.Select(i => i + waterOffset));
        Join(water.Goals.Select(i => i + waterOffset), Row(column, columnOffset, true));
        Join(Row(column, columnOffset, false), Row(steering, steeringOffset, true));
        Join(Row(steering, steeringOffset, false), Row(wet, wetOffset, true), true);
        Join(Row(wet, wetOffset, false), [finish]);
        for (int side = 0; side < 2; ++side)
        {
            float sign = side == 0 ? -1 : 1;
            Vector3[] points = [new(sign * 13.6f, 6, -271.5f), new(sign * 13.2f, 6, -267.5f),
                new(sign * 7.95f, 9.5f, -268.5f), new(sign * 3.3f, 10, -269)];
            catwalk[side] = nodes.Count;
            foreach (var point in points) AddNode(point, true);
            for (int i = 0; i + 1 < points.Length; ++i)
                AddEdge(catwalk[side] + i, catwalk[side] + i + 1, [PilotCourseGeometry.Move.Walk(points[i], points[i + 1])]);
            Join([catwalk[side] + points.Length - 1], Row(steering, steeringOffset, true));
        }
        data.Nodes = nodes.ToArray(); data.Edges = edges.ToArray(); data.CustomConnectors = Connectors;
        int[] Range(int start, int end) => Enumerable.Range(start, end - start).ToArray();
        int[] First(HitTimedRoutes graph, int offset) => Row(graph, offset, true).ToArray();
        var waterEntry = water.Entries.Select(i => i + waterOffset).ToArray();
        var columnEntry = First(column, columnOffset);
        var steeringEntry = First(steering, steeringOffset);
        var wetEntry = First(wet, wetOffset);
        steeringExitTargets = wetEntry;
        data.Sections = [
            new("opening", Range(0, waterOffset).Concat(waterEntry).ToArray(), [0], waterEntry),
            new("water", Range(waterOffset, columnOffset).Concat(columnEntry).ToArray(), waterEntry, columnEntry),
            new("column", Range(columnOffset, steeringOffset).Concat(steeringEntry).ToArray(), columnEntry, steeringEntry),
            new("steering", Range(steeringOffset, wetOffset).Concat(wetEntry).Concat(Range(finish + 1, nodes.Count)).ToArray(), steeringEntry, wetEntry),
            new("wet", Range(wetOffset, finish + 1), wetEntry, [finish])];
        foreach (var edge in data.Edges) foreach (var move in edge.Moves) _ = move.Path;
        data.CustomHeight = Height;
        data.CustomClear = Clear;

        int AddNode(Vector3 p, bool canWait)
        {
            int id = nodes.Count;
            nodes.Add(new() { Position = [p.X, p.Y, p.Z], CanWait = canWait });
            return id;
        }
        void AddEdge(int from, int to, PilotCourseGeometry.Move[] moves)
            => edges.Add(new() { From = from, To = to, Moves = moves });
        int AddTimed(HitTimedRoutes graph, string kind)
        {
            int offset = nodes.Count;
            foreach (var node in graph.Nodes) AddNode(Feet(node.Position), node.CanWait);
            foreach (var outgoing in graph.Outgoing)
                foreach (var edge in outgoing) AddEdge(offset + edge.From, offset + edge.To, [TimedMove(edge.Path, kind)]);
            return offset;
        }
        void Join(IEnumerable<int> from, IEnumerable<int> to, bool recovery = false)
        {
            var targets = to.ToArray();
            foreach (int a in from)
                foreach (int b in targets)
                {
                    var start = nodes[a].Feet; var end = nodes[b].Feet;
                    if (Vector2.Distance(start.XZ(), end.XZ()) > (b == finish ? 12 : HandoffRange)) continue;
                    if (b != finish && !(start.Z > -242.9f && end.Z > -242.9f
                        ? water.Model.Terrain.Clear(new(start.X, start.Z), new(end.X, end.Z))
                        : data.Ground.Clear(start.XZ(), end.XZ()))) continue;
                    var walk = PilotCourseGeometry.Move.Walk(start, end);
                    AddEdge(a, b, recovery ? [new()
                    {
                        Kind = "recovery", Points = [[start.X, start.Y, start.Z], [start.X, start.Y, start.Z]],
                        TimedPath = [new(start.XZ(), 0), new(start.XZ(), RecoverySeconds)]
                    }, walk] : [walk]);
                }
        }
    }

    private Vector3 Feet(Vector2 p) => new(p.X, data.Ground.At(p) ?? 10, p.Y);
    private float? Height(Vector2 p) => p.Y > -194.9f ? opening.Ground.At(p) : p.Y >= WaterRoutes.ExitZ
        ? water.Model.Terrain.Clear(new(p.X, p.Y), new(p.X, p.Y)) ? 10 : null : data.Ground.At(p);

    private bool Clear(Vector3 from, Vector2 to)
    {
        var start = from.XZ();
        Span<float> cuts = stackalloc float[4];
        cuts[0] = 0; cuts[1] = 1;
        int count = 2;
        if (Math.Abs(to.Y - start.Y) > 1e-6f)
            foreach (float boundary in new[] { -194.9f, WaterRoutes.ExitZ })
            {
                float t = (boundary - start.Y) / (to.Y - start.Y);
                if (t > 0 && t < 1) cuts[count++] = t;
            }
        cuts[..count].Sort();
        for (int i = 1; i < count; ++i)
        {
            var a = Vector2.Lerp(start, to, cuts[i - 1]);
            var b = Vector2.Lerp(start, to, cuts[i]);
            float z = (a.Y + b.Y) * .5f;
            bool clear = z > -194.9f ? (i == 1 ? opening.Ground.ClearFromGrounded(from, b) : opening.Ground.Clear(a, b))
                : z >= WaterRoutes.ExitZ ? water.Model.Terrain.Clear(new(a.X, a.Y), new(b.X, b.Y)) : data.Ground.Clear(a, b);
            if (!clear) return false;
        }
        return true;
    }
    private static IEnumerable<int> Row(HitTimedRoutes graph, int offset, bool first)
    {
        float z = first ? graph.Nodes.Max(n => n.Position.Y) : graph.EndZ;
        return graph.Nodes.Select((n, i) => (n, i)).Where(p => p.n.Position.Y == z).Select(p => p.i + offset);
    }
    private PilotCourseGeometry.Move TimedMove(HitTimedRoutes.Point[] path, string kind) => new()
    {
        Kind = kind, TimedPath = path,
        Points = path.Select(p => Feet(p.Position)).Select(p => new[] { p.X, p.Y, p.Z }).ToArray()
    };
    private PilotCourseGeometry.Move[] WaterMoves(WaterEdge edge, WaterPoint start)
    {
        var moves = new List<PilotCourseGeometry.Move>();
        for (int i = 0; i < edge.Points.Length; ++i)
        {
            var move = PilotCourseGeometry.Move.Walk(Feet(start.Vector), Feet(edge.Points[i].Vector));
            if (i > 0 && edge.Jump == i - 1) move.Kind = "jump";
            moves.Add(move); start = edge.Points[i];
        }
        return moves.ToArray();
    }
    private IEnumerable<HitTimedRoutes.Edge> Connectors(PilotPose pose, float speed)
    {
        if (pose.Feet.Y < 9 && pose.Feet.Z is > -335 and < -262 && Math.Abs(pose.Feet.X) > 9.5f)
        {
            int target = catwalk[pose.Feet.X < 0 ? 0 : 1];
            yield return data.Edge(data.Nodes.Length, target, [PilotCourseGeometry.Move.Walk(pose.Feet, data.Nodes[target].Feet)], speed);
            yield break;
        }
        if (pose.Feet.Z > -194.8f)
        {
            foreach (var edge in opening.Connectors(pose, speed))
                yield return data.Edge(data.Nodes.Length, edge.To, (PilotCourseGeometry.Move[])edge.Motion!, speed);
            yield break;
        }
        if (pose.Feet.Z >= WaterRoutes.ExitZ)
        {
            var start = new WaterPoint(pose.Feet.X, pose.Feet.Z);
            foreach (var edge in water.Connectors(start))
                yield return data.Edge(data.Nodes.Length, waterOffset + edge.To, WaterMoves(edge, start), speed);
            yield break;
        }
        if (pose.Feet.Z < -372.6f)
        {
            yield return data.Edge(data.Nodes.Length, finish, [PilotCourseGeometry.Move.Walk(pose.Feet, data.Nodes[finish].Feet)], speed);
            yield break;
        }
        bool recovering = pose.Raw.SteeringClock?.CurrentTimelineId == 8514 && pose.Feet.Z < -332;
        var graph = pose.Feet.Z >= -269.4f ? column : pose.Feet.Z >= -332.6f && !recovering ? steering : wet;
        // Steering can release before the last graph row. Join the next area's
        // entries with the observed ordinary motion, not a residual steering curve.
        if (ReferenceEquals(graph, steering) && pose.Motion == MotionKind.Ordinary &&
            pose.Grounded && !pose.Locked && pose.Force == Vector2.Zero && pose.Feet.Z - steering.EndZ <= HandoffRange)
        {
            foreach (int target in steeringExitTargets)
            {
                var feet = data.Nodes[target].Feet;
                if (Vector2.DistanceSquared(pose.XZ, feet.XZ()) > HandoffRange * HandoffRange || !data.Clear(pose.Feet, feet.XZ())) continue;
                yield return data.Edge(data.Nodes.Length, target, [PilotCourseGeometry.Move.Walk(pose.Feet, feet)], speed);
            }
            yield break;
        }
        int offset = ReferenceEquals(graph, column) ? columnOffset : ReferenceEquals(graph, steering) ? steeringOffset : wetOffset;
        string kind = ReferenceEquals(graph, steering) ? "steering" : ReferenceEquals(graph, wet) ? "wet" : "walk";
        Func<Vector2, Vector2, HitTimedRoutes.Point[]>? motion = pose.Motion == MotionKind.Steering
            ? (a, b) => HitSteeringStep.Curve(a, b, pose.Raw.ArrowDirection ?? pose.Facing) : null;
        foreach (var edge in graph.Connectors(pose.XZ, motion))
            yield return data.Edge(data.Nodes.Length, offset + edge.To, [TimedMove(edge.Path, kind)], speed);
    }

    internal static bool CanWait(PilotPose pose) => pose.Motion != MotionKind.Steering &&
        pose.Force == Vector2.Zero && HitWetStep.CanWaitAt(pose.Feet.Z);
}
