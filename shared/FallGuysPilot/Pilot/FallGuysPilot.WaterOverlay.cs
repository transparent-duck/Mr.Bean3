using System;
using System.Linq;
using System.Numerics;
using MrBean.Modules.FallGuysPilot.Water;
using MrBean.Host;
using Dalamud.Bindings.ImGui;

namespace MrBean.Modules.FallGuysPilot;

public sealed partial class FallGuysPilot
{
    private bool waterOverlay = true;
    private double nextWaterOverlayText;
    private string[] waterOverlayText = [];
    private static readonly Vector2[] WaterCircle = Enumerable.Range(0, 33)
        .Select(i => new Vector2(MathF.Sin(i * MathF.Tau / 32), MathF.Cos(i * MathF.Tau / 32))).ToArray();
    private const float WaterOverlayHeight = 10.08f;

    // Adapted from the archived FallGuysNavigationModule.MechanicOverlay.cs:
    // ground-plane circles, triangle-fan fill, off-screen projection and labels.
    // Only rendering is reused. All geometry/timing comes from the active session.
    public void DrawOverlay()
    {
        DrawHazardOverlay();
        DrawRouteOverlay();
        DrawActiveRouteOverlay();
        if (!waterOverlay || !water.HasPhase || Service.LocalPlayer == null) return;
        if (!water.Manual && course != MrBean.FallGuys.CourseId.Hit) return;
        var cache = water.Cache;
        double now = PilotPose.Now, at = now - water.Epoch;
        var draw = ImGui.GetBackgroundDrawList();
        if (waterOverlayText.Length != cache.Model.Stations.Length) waterOverlayText = new string[cache.Model.Stations.Length];
        bool updateText = now >= nextWaterOverlayText;
        if (updateText) nextWaterOverlayText = now + .1;
        for (int i = 0; i < cache.Model.Stations.Length; ++i)
        {
            var station = cache.Model.Stations[i];
            var state = cache.StationState(water.Pattern, i, at);
            uint color = state.Active ? 0xff4080ffu : 0xffa0a0a0u;
            if (updateText) waterOverlayText[i] = $"#{i + 1} {(state.Active ? "剩餘" : "下次")} {state.Seconds:F1}s";
            Circle(draw, station.Vector, (float)cache.Model.Radius, color, state.Active);
            Label(draw, station.Vector, waterOverlayText[i] ?? "", color);
        }
        if (water.Plan is not { } plan) return;
        var from = water.PlanStart.Vector;
        double arrival = water.PlannedAt;
        for (int e = 0; e < plan.Edges.Length; ++e)
        {
            var edge = plan.Edges[e];
            double departure = plan.Departures[e] + water.PlanEpoch;
            double wait = departure - Math.Max(arrival, now);
            if (wait > 0) Label(draw, from, $"等 {wait:F1}s", 0xff50cfff);
            for (int p = 0; p < edge.Points.Length; ++p)
            {
                var to = edge.Points[p].Vector;
                uint color = edge.Jump == p - 1 && edge.Jump >= 0 ? 0xff50a0ffu : 0xff80dd80u;
                if (Project(from, out var a) && Project(to, out var b))
                {
                    draw.AddLine(a, b, color, 3);
                    if (Vector2.DistanceSquared(a, b) > 100)
                    {
                        var d = Vector2.Normalize(b - a); var side = new Vector2(-d.Y, d.X); var mid = (a + b) / 2;
                        draw.AddTriangleFilled(mid + d * 6, mid - d * 4 + side * 4, mid - d * 4 - side * 4, color);
                    }
                }
                from = to;
            }
            arrival = departure + edge.Duration;
        }
        Label(draw, from, "水區終點", 0xff80dd80);
    }

    private static bool Project(Vector2 p, out Vector2 screen)
        => Service.GameGui.WorldToScreen(new Vector3(p.X, WaterOverlayHeight, p.Y), out screen, out _);

    private static void Circle(ImDrawListPtr draw, Vector2 center, float radius, uint color, bool active)
    {
        bool centerVisible = Project(center, out var screenCenter);
        bool previousVisible = Project(center + WaterCircle[0] * radius, out var previous);
        for (int i = 1; i < WaterCircle.Length; ++i)
        {
            bool visible = Project(center + WaterCircle[i] * radius, out var point);
            if (visible && previousVisible)
            {
                if (active && centerVisible) draw.AddTriangleFilled(screenCenter, previous, point, (color & 0x00ffffffu) | 0x40000000u);
                draw.AddLine(previous, point, color, 1.5f);
            }
            previousVisible = visible; previous = point;
        }
    }

    private static void Label(ImDrawListPtr draw, Vector2 world, string text, uint color)
    {
        if (!Project(world, out var at)) return;
        var size = ImGui.CalcTextSize(text);
        draw.AddRectFilled(at - new Vector2(4, 3), at + size + new Vector2(4, 3), 0xc0202020, 3);
        draw.AddText(at, color, text);
    }
}
