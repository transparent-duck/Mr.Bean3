using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot.Courses;

// The baked graph keeps different headings at the same position as different
// states. Entry resets direction; later rows preserve it through joined curves.
internal sealed class HitSteeringRoutes(HitTimedRoutes graph, float[] headings)
{
    internal const float WaitingZ = -269.4f, EntryZ = -270.6f, ExitZ = -332.6f;
    internal const float EntryHeading = MathF.PI;
    private const int Format = 1;
    internal HitTimedRoutes Graph { get; } = graph;
    internal float[] Headings { get; } = headings;

    internal static HitSteeringRoutes Read(Stream stream)
    {
        using var gzip = new GZipStream(stream, CompressionMode.Decompress, true);
        using var reader = new BinaryReader(gzip);
        if (reader.ReadInt32() != Format) throw new InvalidDataException("Unknown Hit steering map format.");
        var nodes = new HitTimedRoutes.Node[reader.ReadInt32()];
        var headings = new float[nodes.Length];
        for (int i = 0; i < nodes.Length; ++i)
        {
            var position = new Vector2(reader.ReadSingle(), reader.ReadSingle());
            headings[i] = reader.ReadSingle();
            nodes[i] = new(position, position.Y == WaitingZ, position.Y == ExitZ);
        }
        int edgeCount = reader.ReadInt32();
        var edges = new List<HitTimedRoutes.Edge>(edgeCount);
        for (int i = 0; i < edgeCount; ++i)
        {
            int from = reader.ReadInt32(), to = reader.ReadInt32();
            // Terrain and curvature already selected these edges offline.
            // Recreate their samples once on load instead of storing millions
            // of repeated coordinates in the plugin resource.
            var path = Connection(nodes[from].Position, nodes[to].Position, headings[from], headings[to]);
            if (path.Length > 1) edges.Add(new(from, to, path));
        }
        return new(new(nodes, edges.ToArray(), HitGround.Loaded.Value), headings);
    }

    internal void Write(Stream stream)
    {
        using var gzip = new GZipStream(stream, CompressionLevel.SmallestSize, true);
        using var writer = new BinaryWriter(gzip);
        writer.Write(Format); writer.Write(Graph.Nodes.Count);
        for (int i = 0; i < Graph.Nodes.Count; ++i)
        {
            writer.Write(Graph.Nodes[i].Position.X); writer.Write(Graph.Nodes[i].Position.Y);
            writer.Write(Headings[i]);
        }
        int count = 0;
        foreach (var outgoing in Graph.Outgoing) count += outgoing.Count;
        writer.Write(count);
        foreach (var outgoing in Graph.Outgoing)
            foreach (var edge in outgoing)
            {
                writer.Write(edge.From); writer.Write(edge.To);
            }
    }

    internal static HitTimedRoutes.Point[] Connection(Vector2 start, Vector2 end, float fromHeading, float toHeading)
    {
        if (end.Y >= WaitingZ) return HitTimedRoutes.Walk(start, end);
        if (start.Y < WaitingZ) return Curve(start, end, fromHeading, toHeading);
        // Status 3702 resets the native direction toward the course exit.
        // Choose the lane before entry; an oblique approach cannot supply a
        // different initial heading to the native curve.
        if (Math.Abs(Geo.WrapAngle(toHeading - EntryHeading)) > 1e-5f) return [];
        var direction = Geo.FromHeading(toHeading);
        if (direction.Y >= 0) return [];
        var align = end - direction * ((end.Y - WaitingZ) / direction.Y);
        double walk = Vector2.Distance(start, align) / Actuator.WalkSpeed;
        return [new(start, 0), new(align, walk), new(end, walk + Vector2.Distance(align, end) / Actuator.WalkSpeed)];
    }

    internal IEnumerable<HitTimedRoutes.Edge> Connectors(Vector2 start, float heading, bool steering)
    {
        var ground = HitGround.Loaded.Value;
        for (int to = 0; to < Graph.Nodes.Count; ++to)
        {
            var node = Graph.Nodes[to];
            HitTimedRoutes.Point[] path;
            if (!steering && start.Y > EntryZ)
            {
                // Choose a lane while ordinary movement can still stop.
                // Status gain then establishes the forward entry heading.
                if (!node.CanWait) continue;
                path = HitTimedRoutes.Walk(start, node.Position);
            }
            else
            {
                if (node.CanWait || node.Position.Y >= start.Y) continue;
                path = Curve(start, node.Position, heading, Headings[to]);
            }
            if (path.Length < 2) continue;
            bool clear = true;
            for (int i = 1; i < path.Length; ++i)
                if (!ground.Clear(path[i - 1].Position, path[i].Position)) { clear = false; break; }
            if (clear) yield return new(Graph.Nodes.Count, to, path);
        }
    }

    // Cubic x(progress) preserves the direction at BOTH ends. Check its exact
    // curvature extrema before allocating samples; native speed is constant.
    internal static HitTimedRoutes.Point[] Curve(Vector2 start, Vector2 end, float fromHeading, float toHeading)
    {
        double distance = start.Y - end.Y;
        var from = Geo.FromHeading(fromHeading); var to = Geo.FromHeading(toHeading);
        if (distance <= 0 || from.Y >= 0 || to.Y >= 0) return [];
        double first = from.X / -from.Y * distance, last = to.X / -to.Y * distance;
        double dx = end.X - start.X;
        double a = first + last - 2 * dx, b = 3 * dx - 2 * first - last, c = first;
        if (PeakTurnRate(a, b, c, distance) > SteeringPassage.TurnRate) return [];
        int count = Math.Max(1, (int)Math.Ceiling(distance / .2));
        var result = new HitTimedRoutes.Point[count + 1]; result[0] = new(start, 0);
        double time = 0;
        for (int i = 1; i <= count; ++i)
        {
            double u = (double)i / count;
            var position = i == count ? end : new Vector2(
                (float)(start.X + ((a * u + b) * u + c) * u), (float)(start.Y - distance * u));
            time += Vector2.Distance(result[i - 1].Position, position) / SteeringPassage.Speed;
            result[i] = new(position, time);
        }
        return result;
    }

    internal static double PeakTurnRate(double a, double b, double c, double distance)
    {
        double peak = Math.Max(At(0), At(1));
        if (Math.Abs(a) < 1e-12)
        {
            if (Math.Abs(b) > 1e-12) Include(-c / (2 * b));
        }
        else
        {
            // For p = 3*a*u^2 + 2*b*u + c, the interior curvature extrema
            // satisfy 5*a*p^2 + 2*(b^2-3*a*c)*p - a*distance^2 = 0.
            double q = b * b - 3 * a * c;
            double root = Math.Sqrt(q * q + 5 * a * a * distance * distance);
            Roots((-q - root) / (5 * a)); Roots((-q + root) / (5 * a));
        }
        return peak;
        double At(double u)
        {
            double slope = (3 * a * u * u + 2 * b * u + c) / distance;
            return Math.Abs((6 * a * u + 2 * b) / (distance * distance)) /
                Math.Pow(1 + slope * slope, 1.5) * SteeringPassage.Speed;
        }
        void Include(double u) { if (u > 0 && u < 1) peak = Math.Max(peak, At(u)); }
        void Roots(double p)
        {
            double discriminant = b * b - 3 * a * (c - p);
            if (discriminant < 0) return;
            double root = Math.Sqrt(discriminant);
            Include((-b - root) / (3 * a)); Include((-b + root) / (3 * a));
        }
    }
}
