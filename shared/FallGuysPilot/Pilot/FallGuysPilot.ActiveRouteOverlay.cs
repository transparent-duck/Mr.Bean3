using System;
using System.Collections.Generic;
using System.Numerics;
using MrBean.Modules.FallGuysPilot.Courses;
using MrBean.Host;
using Dalamud.Bindings.ImGui;

namespace MrBean.Modules.FallGuysPilot;

public sealed partial class FallGuysPilot
{
    private readonly record struct ActiveRouteLine(Vector3 From, Vector3 To, int Leg, int Move, bool Jump);
    private readonly record struct ActiveRouteWait(Vector3 At, int Leg, double Departure);
    private HitTimedRoutes.Plan? drawnActivePlan;
    private ActiveRouteLine[] activeRouteLines = [];
    private ActiveRouteWait[] activeRouteWaits = [];
    private double nextActiveRouteText;
    private string activeRouteText = "";

    private void CacheActiveRoute(HitTimedRoutes.Plan? plan)
    {
        if (ReferenceEquals(drawnActivePlan, plan)) return;
        drawnActivePlan = plan;
        nextActiveRouteText = 0;
        var lines = new List<ActiveRouteLine>();
        var waits = new List<ActiveRouteWait>();
        if (plan is { Found: true })
            for (int leg = 0; leg < plan.Legs.Length; ++leg)
            {
                var item = plan.Legs[leg];
                if (item.Edge.Motion is not PilotCourseGeometry.Move[] moves) continue;
                for (int move = 0; move < moves.Length; ++move)
                {
                    var path = moves[move].Path;
                    for (int i = 1; i < path.Length; ++i)
                        if (path[i - 1] != path[i]) lines.Add(new(path[i - 1], path[i], leg, move, moves[move].Kind == "jump"));
                }
                if (moves.Length > 0 && moves[0].Path.Length > 0 &&
                    (leg == 0 || item.Departure > plan.Legs[leg - 1].Departure + plan.Legs[leg - 1].Edge.Duration + .1))
                    waits.Add(new(moves[0].Path[0], leg, item.Departure));
            }
        activeRouteLines = lines.ToArray(); activeRouteWaits = waits.ToArray();
    }

    private void DrawActiveRouteOverlay()
    {
        if (!configuration.FallGuysShowActiveRoute || Service.ClientState.TerritoryType != TerritoryDuty ||
            Service.LocalPlayer is not { } player || context == null) return;
        var step = engine?.Current as CourseRouteStep;
        CacheActiveRoute(step?.ActivePlan);
        double now = PilotPose.Now;
        if (now >= nextActiveRouteText)
        {
            nextActiveRouteText = now + .1;
            if (step == null) activeRouteText = "目前沒有執行路線";
            else
            {
                string state = step.WaitUntil > now ? $"等待 {step.WaitUntil - now:F1} 秒" :
                    step.ActivePlan is { Found: true } ? "執行路線" : "等待路線";
                string search = step.Searching ? step.Forecasting ? "｜計算下一區" : "｜計算路線中" :
                    step.AwaitingHandoff ? "｜等待切換路線" : "";
                activeRouteText = $"{step.SectionName}｜計算 #{step.SearchNumber}／路線 #{step.RouteNumber}｜{state}{search}";
            }
        }
        ImGui.SetNextWindowPos(new Vector2(20, 180), ImGuiCond.Always);
        ImGui.SetNextWindowBgAlpha(.8f);
        if (ImGui.Begin("Fall Guys 目前路線##pilot-active-route", ImGuiWindowFlags.NoDecoration |
            ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.NoSavedSettings))
            ImGui.TextUnformatted(activeRouteText);
        ImGui.End();
        if (step == null) return;
        var draw = ImGui.GetBackgroundDrawList();
        var lift = new Vector3(0, .15f, 0);
        foreach (var line in activeRouteLines)
        {
            if (line.Leg < step.ActiveLeg || line.Leg == step.ActiveLeg && line.Move < step.ActiveMove) continue;
            if (Vector3.DistanceSquared(player.Position, line.From) > RouteDistance * RouteDistance &&
                Vector3.DistanceSquared(player.Position, line.To) > RouteDistance * RouteDistance) continue;
            if (Service.GameGui.WorldToScreen(line.From + lift, out var a, out _) &&
                Service.GameGui.WorldToScreen(line.To + lift, out var b, out _))
                draw.AddLine(a, b, line.Jump ? 0xff40bfffu : 0xff80ef80u, 4);
        }
        foreach (var wait in activeRouteWaits)
            if (wait.Leg >= step.ActiveLeg && wait.Departure > now &&
                Vector3.DistanceSquared(player.Position, wait.At) <= RouteDistance * RouteDistance &&
                Service.GameGui.WorldToScreen(wait.At + lift, out var point, out _))
                draw.AddCircle(point, 8, 0xff40ffffu, 0, 3);
    }
}
