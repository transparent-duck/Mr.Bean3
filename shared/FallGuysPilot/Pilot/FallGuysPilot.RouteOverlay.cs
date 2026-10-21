using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text.Json;
using MrBean.Modules.Blunderville;
using MrBean.Modules.FallGuysPilot.Courses;
using MrBean.Host;
using Dalamud.Bindings.ImGui;

namespace MrBean.Modules.FallGuysPilot;

public sealed partial class FallGuysPilot
{
    private readonly record struct RouteDrawing(int A, int B, bool Jump, bool Recorded, bool Script);
    private PilotCourseGeometry? routeGraph;
    private Vector3[] routeWorld = [];
    private Vector2[] routeScreen = [];
    private bool[] routeVisible = [], routeMarkedNodes = [];
    private byte[] routeBoundaryNodes = [];
    private string[] routeNodeLabels = [];
    private RouteDrawing[] routeLines = [];
    private bool routeOverlay;
    private int routeFocus = -1, routeProjected;
    private string routeDescription = "";
    private static readonly string[] RouteMaps = ["hit", "knight", "ring", "crystal", "crown"];
    private static readonly string[] HitSections = ["all", "opening", "water", "column", "steering", "wet"];
    private int routeMap = -1;
    private const float RouteDistance = 45;

    private PilotCourseGeometry HitInspectionGraph(string section, out Dictionary<int, string> labels)
    {
        var full = HitCourseGeometry.Load();
        labels = new();
        if (section == "all") return full;
        var ids = full.Sections.Single(s => s.Name == section).Nodes.Distinct().ToArray();
        var local = ids.Select((id, index) => (id, index)).ToDictionary(p => p.id, p => p.index);
        labels = ids.Select((id, index) => (id, index)).ToDictionary(p => p.index, p => p.id.ToString());
        return new()
        {
            Nodes = ids.Select(id => full.Nodes[id]).ToArray(),
            Edges = full.Edges.Where(e => local.ContainsKey(e.From) && local.ContainsKey(e.To))
                .Select(e => new PilotCourseGeometry.Route
                    { From = local[e.From], To = local[e.To], Moves = e.Moves, EmptyOnly = e.EmptyOnly }).ToArray()
        };
    }
    // Inspection only. Loading a graph/recording never starts a Pilot session,
    // changes the scene, supplies observations, or creates a movement owner.
    private void LoadRouteGraph(string map, string? recording = null, int planNumber = 1, string section = "all")
    {
        Dictionary<int, string>? labels = null;
        var data = map == "hit" ? HitInspectionGraph(section, out labels) : PilotCourseGeometry.Load(map,
            course == MrBean.FallGuys.CourseId.Crown ? activeCrownStairBoost : configuration.FallGuysCrownStairBoost,
            engine?.Current is CourseRouteStep route ? route.CrownFrameRate : 30);
        var paths = new List<RouteDrawingPath>();
        var marked = new bool[data.Nodes.Length];
        void Segment(Vector3 a, Vector3 b, int from, int to, bool jump, bool recorded, bool script = false)
        {
            if (a != b) paths.Add(new([a, b], from, to, jump, recorded, script, false));
        }
        void Edge(PilotCourseGeometry.Route edge, bool recorded)
        {
            foreach (var move in edge.Moves)
                if (move.Path.Length > 1)
                    paths.Add(new(move.Path, edge.From, edge.To, move.Kind == "jump", recorded,
                        move.Kind == "script", move.Kind == "steering"));
        }
        foreach (var edge in data.Edges) Edge(edge, false);
        string description = $"{map}/{(map == "hit" ? section : "all")}：{data.Nodes.Length} 節點，{data.Edges.Length} 有向連接。";
        if (recording != null)
        {
            if (map == "hit" || Path.GetFileName(recording) != recording || !recording.StartsWith(map + "-", StringComparison.OrdinalIgnoreCase) ||
                !recording.EndsWith(".jsonl", StringComparison.OrdinalIgnoreCase) || planNumber < 1)
                throw new ArgumentException("Use a matching course recording filename and a plan number starting at 1.");
            int found = 0;
            bool loaded = false;
            foreach (string line in File.ReadLines(Path.Combine(directory, recording)))
            {
                using var document = JsonDocument.Parse(line);
                var row = document.RootElement;
                if (row.GetProperty("kind").GetString() != "course-ctc-plan" || ++found != planNumber) continue;
                var value = row.GetProperty("value");
                var start = value.GetProperty("start");
                var feet = new Vector3(start.GetProperty("X").GetSingle(), start.GetProperty("Y").GetSingle(), start.GetProperty("Z").GetSingle());
                int jumps = 0;
                bool first = true;
                foreach (var leg in value.GetProperty("route").EnumerateArray())
                {
                    int from = leg.GetProperty("From").GetInt32(), to = leg.GetProperty("To").GetInt32();
                    if (to < 0 || to >= data.Nodes.Length) throw new InvalidDataException("Recording uses a different graph.");
                    marked[to] = true;
                    // The first leg is the actual-position connector. Its
                    // historical source ID changes when the graph gains nodes.
                    if (first) Segment(feet, data.Nodes[to].Feet, -1, to, false, true);
                    else
                    {
                        var edge = data.Edges.FirstOrDefault(e => e.From == from && e.To == to)
                            ?? throw new InvalidDataException("Recording connection is absent from the current graph.");
                        marked[from] = true; Edge(edge, true);
                        jumps += edge.Moves.Count(m => m.Kind == "jump");
                    }
                    first = false;
                }
                double duration = value.GetProperty("Arrival").GetDouble() - value.GetProperty("at").GetDouble();
                description += $" {recording} 第 {planNumber} 次計畫：{duration:F2} 秒、{jumps} 跳。";
                loaded = true; break;
            }
            if (!loaded) throw new ArgumentException("The requested plan is not in this recording.");
        }
        // Publish only after the complete inspection data loaded successfully.
        routeGraph = data; routePaths = paths.ToArray();
        routeNodeLabels = Enumerable.Range(0, data.Nodes.Length).Select(i => labels?.GetValueOrDefault(i) ?? i.ToString()).ToArray();
        routeMarkedNodes = marked; routeDescription = description;
        routeBoundaryNodes = new byte[data.Nodes.Length];
        if (map == "hit")
        {
            var full = HitCourseGeometry.Load();
            var boundaries = new Dictionary<Vector3, byte>();
            foreach (var area in full.Sections.Where(s => section == "all" || s.Name == section))
            {
                foreach (int id in area.Entries) boundaries[full.Nodes[id].Feet] = (byte)(boundaries.GetValueOrDefault(full.Nodes[id].Feet) | 1);
                foreach (int id in area.Exits) boundaries[full.Nodes[id].Feet] = (byte)(boundaries.GetValueOrDefault(full.Nodes[id].Feet) | 2);
            }
            for (int i = 0; i < data.Nodes.Length; ++i) routeBoundaryNodes[i] = boundaries.GetValueOrDefault(data.Nodes[i].Feet);
        }
        routeFocus = -1; routeOverlay = true;
        CacheRouteDrawing();
        routeMap = Array.IndexOf(RouteMaps, map);
    }

    private void HandleGraphCommand(string[] args)
    {
        string option = args.FirstOrDefault() ?? "on";
        if (option == "active")
        {
            string state = args.ElementAtOrDefault(1) ?? "on";
            if (state is not ("on" or "off")) throw new ArgumentException("Use fallguys graph active on/off.");
            configuration.FallGuysShowActiveRoute = state == "on"; configuration.Save(); return;
        }
        if (option == "off") { routeOverlay = false; return; }
        if (option == "on") { routeOverlay = true; routeMap = -1; }
        else if (option == "hit")
        {
            string section = args.ElementAtOrDefault(1) ?? "all";
            if (!HitSections.Contains(section)) throw new ArgumentException("Hit sections: all, opening, water, column, steering, wet.");
            LoadRouteGraph(option, section: section);
        }
        else if (RouteMaps.Contains(option))
            LoadRouteGraph(option, args.ElementAtOrDefault(1), args.Length > 2 ? int.Parse(args[2]) : 1);
        else if (option == "node")
        {
            if (routeGraph == null) throw new ArgumentException("Enable the route overlay in a course first.");
            routeFocus = Math.Clamp(int.Parse(args.ElementAtOrDefault(1) ?? "-1"), -1, routeGraph!.Nodes.Length - 1);
            CacheRouteDrawing();
            routeOverlay = true;
        }
        else if (option != "status") throw new ArgumentException("Use fallguys graph hit [section], knight/ring/crystal/crown [recording.jsonl] [plan], node <id>, off or status.");
        Service.Log.Information("[FallGuysPilot.Graph] " + JsonSerializer.Serialize(new
        { enabled = routeOverlay, description = routeDescription, focus = routeFocus, vertices = routeWorld.Length,
            segments = routeLines.Length, projected = routeProjected }, Json));
    }

    private void DrawRouteSettings()
    {
        bool active = configuration.FallGuysShowActiveRoute;
        if (ImGui.Checkbox("顯示目前路線與等待時間", ref active))
        { configuration.FallGuysShowActiveRoute = active; configuration.Save(); }
        if (active) ImGui.TextWrapped("綠：目前路線；橙：跳躍；黃圈：等待點。圈旁顯示預計停留秒數，抵達後顯示剩餘秒數。計算次數包含預先計算與調時；路線編號在產生新路線或調整時間時增加。");
        if (ImGui.Checkbox("顯示路網", ref routeOverlay) && routeOverlay) routeMap = -1;
        if (routeOverlay) ImGui.TextWrapped("自動顯示目前地圖。灰：步行；橙：跳躍；綠：載入的紀錄路線。");
        if (routeOverlay && routeMap == 0) ImGui.TextWrapped("角速度區概覽合併同起終點的朝向候選；用 graph node <編號> 檢查該節點全部候選。");
        if (routeOverlay && routeMap == 0) ImGui.TextWrapped("青圈：入口候選；紫圈：出口候選；雙圈：兩者重合。");
    }

    private void DrawRouteOverlay()
    {
        routeProjected = 0;
        if (!routeOverlay || Service.ClientState.TerritoryType != TerritoryDuty ||
            Service.LocalPlayer is not { } player) return;
        var feet = player.Position;
        if (CourseScene.Detect(feet) is not { } current) return;
        if (routeMap != (int)current) LoadRouteGraph(RouteMaps[(int)current]);
        if (routeGraph == null) return;
        CollectRouteDrawing(feet);
        var viewport = ImGui.GetMainViewport();
        var screenMin = viewport.Pos - new Vector2(10);
        var screenMax = viewport.Pos + viewport.Size + new Vector2(10);
        var draw = ImGui.GetBackgroundDrawList();
        // The nearby list is cached until the player's grid cell changes.
        // Recorded lines follow the overview even when they occupy other cells.
        foreach (int id in routeNearbyLines)
        {
            var line = routeLines[id];
            if (!ProjectRoutePoint(line.A, feet) || !ProjectRoutePoint(line.B, feet)) continue;
            uint color = line.Recorded ? 0xff80ef80u : line.Jump ? 0xff40bfffu : line.Script ? 0xffffb050u : 0x609c9c9cu;
            var a = routeScreen[line.A]; var b = routeScreen[line.B];
            // Keep segments crossing the viewport even if both ends are outside.
            if (MathF.Max(a.X, b.X) < screenMin.X || MathF.Min(a.X, b.X) > screenMax.X ||
                MathF.Max(a.Y, b.Y) < screenMin.Y || MathF.Min(a.Y, b.Y) > screenMax.Y) continue;
            draw.AddLine(a, b, color, line.Recorded ? 4 : line.Jump || line.Script ? 2.5f : 1);
            ++routeProjected;
            if (!(line.Jump || line.Recorded)) continue;
            var direction = b - a;
            if (direction.LengthSquared() < 100) continue;
            direction = Vector2.Normalize(direction); var side = new Vector2(-direction.Y, direction.X);
            var middle = Vector2.Lerp(a, b, .65f);
            draw.AddTriangleFilled(middle + direction * 6, middle - direction * 4 + side * 4,
                middle - direction * 4 - side * 4, color);
        }
        foreach (int i in routeNearbyNodes)
        {
            bool selected = i == routeFocus, recorded = routeMarkedNodes[i];
            byte boundary = routeBoundaryNodes[i];
            if (routeFocus >= 0 && !selected && !recorded && boundary == 0) continue;
            if (routeNodeLabels[i].Length == 0 || !ProjectRoutePoint(i, feet)) continue;
            var screen = routeScreen[i];
            if (screen.X < screenMin.X || screen.X > screenMax.X || screen.Y < screenMin.Y || screen.Y > screenMax.Y) continue;
            uint color = selected ? 0xffffffffu : recorded ? 0xff80ef80u : 0xffb0b0b0u;
            draw.AddCircleFilled(routeScreen[i], selected ? 5 : recorded ? 4 : 2, color);
            if ((boundary & 1) != 0) draw.AddCircle(routeScreen[i], 6, 0xffffff40u, 0, 2);
            if ((boundary & 2) != 0) draw.AddCircle(routeScreen[i], boundary == 3 ? 9 : 6, 0xffff60dfu, 0, 2);
            if (!selected && !recorded && Vector3.DistanceSquared(feet, routeWorld[i]) > 8 * 8) continue;
            var at = routeScreen[i] + new Vector2(5, -7); var size = ImGui.CalcTextSize(routeNodeLabels[i]);
            draw.AddRectFilled(at - new Vector2(2, 1), at + size + new Vector2(2, 1), 0xc0202020);
            draw.AddText(at, color, routeNodeLabels[i]);
        }
    }
}
