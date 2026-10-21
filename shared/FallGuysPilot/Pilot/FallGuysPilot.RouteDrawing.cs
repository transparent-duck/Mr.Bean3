using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using MrBean.Host;

namespace MrBean.Modules.FallGuysPilot;

public sealed partial class FallGuysPilot
{
    private readonly record struct RouteDrawingPath(Vector3[] Points, int From, int To,
        bool Jump, bool Recorded, bool Script, bool Steering);
    private sealed class RouteDrawingCell
    {
        internal readonly List<int> Lines = [], Nodes = [];
    }
    private const float RouteCellSize = 8, RouteDrawingError = .1f;
    private RouteDrawingPath[] routePaths = [];
    private Dictionary<(int X, int Z), RouteDrawingCell> routeCells = [];
    private readonly List<int> routeNearbyLines = [], routeNearbyNodes = [];
    private readonly HashSet<int> routeNearbySet = [];
    private (int X, int Z)? routeNearbyCell;
    private int[] routeProjectionFrames = [];
    private int routeProjectionFrame;

    private static (int X, int Z) RouteCell(Vector3 p)
        => ((int)MathF.Floor(p.X / RouteCellSize), (int)MathF.Floor(p.Z / RouteCellSize));

    private void CacheRouteDrawing()
    {
        if (routeGraph == null) return;
        // Heading states at the same physical endpoints make the overview dense.
        // Use the shortest existing steering curve for each endpoint pair. A
        // focused node retains every heading candidate, as does the planner.
        var steering = new Dictionary<(Vector3, Vector3), (Vector3[] Path, float Length)>();
        if (routeFocus < 0)
            foreach (var path in routePaths)
            {
                if (!path.Steering || path.Recorded) continue;
                var key = (path.Points[0], path.Points[^1]);
                float length = 0;
                for (int i = 1; i < path.Points.Length; ++i)
                    length += Vector3.Distance(path.Points[i - 1], path.Points[i]);
                if (!steering.TryGetValue(key, out var previous) || length < previous.Length)
                    steering[key] = (path.Points, length);
            }
        var points = routeGraph.Nodes.Select(n => n.Feet).ToList();
        var indices = new Dictionary<Vector3, int>();
        // Node IDs remain valid even when different heading states coincide.
        for (int i = 0; i < points.Count; ++i) indices.TryAdd(points[i], i);
        var lines = new List<RouteDrawing>();
        var seen = new HashSet<RouteDrawing>();
        foreach (var path in routePaths)
        {
            if (!path.Recorded)
            {
                if (routeFocus >= 0 && path.From != routeFocus && path.To != routeFocus) continue;
                if (routeFocus < 0 && path.Steering &&
                    !ReferenceEquals(path.Points, steering[(path.Points[0], path.Points[^1])].Path)) continue;
            }
            var simplified = SimplifyRouteDrawing(path.Points);
            for (int i = 1; i < simplified.Count; ++i)
            {
                int a = Point(simplified[i - 1]), b = Point(simplified[i]);
                if (a == b) continue;
                // Walk/script lines have no arrow; drawing both directions is
                // redundant. Preserve direction on jump and recorded arrows.
                if (!path.Jump && !path.Recorded && a > b) (a, b) = (b, a);
                var line = new RouteDrawing(a, b, path.Jump, path.Recorded, path.Script);
                if (seen.Add(line)) lines.Add(line);
            }
        }
        int Point(Vector3 p)
        {
            if (indices.TryGetValue(p, out int index)) return index;
            index = points.Count; indices.Add(p, index); points.Add(p); return index;
        }
        var cells = new Dictionary<(int X, int Z), RouteDrawingCell>();
        RouteDrawingCell Cell((int X, int Z) key)
        {
            if (!cells.TryGetValue(key, out var cell)) cells.Add(key, cell = new());
            return cell;
        }
        for (int i = 0; i < routeGraph.Nodes.Length; ++i) Cell(RouteCell(points[i])).Nodes.Add(i);
        for (int i = 0; i < lines.Count; ++i)
        {
            var a = RouteCell(points[lines[i].A]); var b = RouteCell(points[lines[i].B]);
            for (int x = Math.Min(a.X, b.X); x <= Math.Max(a.X, b.X); ++x)
                for (int z = Math.Min(a.Z, b.Z); z <= Math.Max(a.Z, b.Z); ++z)
                    Cell((x, z)).Lines.Add(i);
        }
        routeWorld = points.Select(p => p + new Vector3(0, .12f, 0)).ToArray();
        routeScreen = new Vector2[points.Count]; routeVisible = new bool[points.Count];
        routeProjectionFrames = new int[points.Count]; routeProjectionFrame = 0;
        routeLines = lines.ToArray(); routeCells = cells; routeNearbyCell = null;
    }

    // Iterative Ramer-Douglas-Peucker, in three dimensions so jumps and slopes
    // retain their height. This changes drawing copies only, never motion paths.
    private static IReadOnlyList<Vector3> SimplifyRouteDrawing(Vector3[] path)
    {
        if (path.Length <= 2) return path;
        var keep = new bool[path.Length]; keep[0] = keep[^1] = true;
        var pending = new Stack<(int A, int B)>(); pending.Push((0, path.Length - 1));
        while (pending.TryPop(out var span))
        {
            var start = path[span.A]; var delta = path[span.B] - start;
            float length = delta.LengthSquared(), error = RouteDrawingError * RouteDrawingError;
            int split = -1;
            for (int i = span.A + 1; i < span.B; ++i)
            {
                float t = length > 0 ? Math.Clamp(Vector3.Dot(path[i] - start, delta) / length, 0, 1) : 0;
                float distance = Vector3.DistanceSquared(path[i], start + delta * t);
                if (distance > error) { error = distance; split = i; }
            }
            if (split < 0) continue;
            keep[split] = true; pending.Push((span.A, split)); pending.Push((split, span.B));
        }
        var result = new List<Vector3>();
        for (int i = 0; i < path.Length; ++i) if (keep[i]) result.Add(path[i]);
        return result;
    }

    private void CollectRouteDrawing(Vector3 feet)
    {
        if (routeProjectionFrame == int.MaxValue)
        { Array.Clear(routeProjectionFrames); routeProjectionFrame = 0; }
        ++routeProjectionFrame;
        var center = RouteCell(feet);
        if (routeNearbyCell == center) return;
        routeNearbyCell = center;
        routeNearbyLines.Clear(); routeNearbyNodes.Clear(); routeNearbySet.Clear();
        int radius = (int)MathF.Ceiling(RouteDistance / RouteCellSize) + 1;
        for (int x = center.X - radius; x <= center.X + radius; ++x)
            for (int z = center.Z - radius; z <= center.Z + radius; ++z)
            {
                if (!routeCells.TryGetValue((x, z), out var cell)) continue;
                routeNearbyNodes.AddRange(cell.Nodes);
                foreach (int id in cell.Lines)
                    if (routeNearbySet.Add(id)) routeNearbyLines.Add(id);
            }
        // Paths from a loaded recording are appended after network paths.
        routeNearbyLines.Sort();
    }

    private bool ProjectRoutePoint(int index, Vector3 feet)
    {
        if (routeProjectionFrames[index] == routeProjectionFrame) return routeVisible[index];
        routeProjectionFrames[index] = routeProjectionFrame;
        return routeVisible[index] = Vector3.DistanceSquared(feet, routeWorld[index]) <= RouteDistance * RouteDistance &&
            Service.GameGui.WorldToScreen(routeWorld[index], out routeScreen[index], out _);
    }
}
