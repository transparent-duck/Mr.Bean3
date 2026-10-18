using System;
using System.IO;
using System.Linq;
using MrBean.FallGuys;
using MrBean.Modules.Blunderville;
using MrBean.Modules.FallGuysPilot.Courses;
using MrBean.Modules.FallGuysPilot.Water;
using MrBean.Host;
using Dalamud.Bindings.ImGui;
using Dalamud.Game.ClientState.Conditions;

namespace MrBean.Modules.FallGuysPilot;

public sealed partial class FallGuysPilot
{
    private readonly WaterSession water = new();
    private int selectedWaterPattern;
    private float selectedWaterPhase;
    private bool waterDebugRunning;
    private string waterDebugStatus = "載入相位後，可從角色目前位置計算水區路線。";
    private ulong waterDebugActor;
    private uint waterDebugTerritory;

    private sealed class WaterDebugScript : CourseScript
    {
        public override CourseId Course => CourseId.Hit;
        public override HazardSpec[] Hazards => [];
        public override Step[] Build(PilotPose start) =>
            [new HitWaterRouteStep { Name = "water-debug", Zone = Zone.ZBand((float)WaterRoutes.ExitZ, -194.9f, 9, 12), UsePreparedPlan = true }];
    }

    private void LoadWaterPattern(int pattern)
    {
        StopWaterDebug("相位已載入");
        if (course != null) { controlling = false; EndScene(); } // changing input mode is not a race elimination
        water.LoadPattern(pattern, PilotPose.Now - selectedWaterPhase, true);
        selectedWaterPattern = pattern;
        waterDebugStatus = $"模式 {pattern + 1} 已載入，AoE 時鐘持續運行。";
    }

    private void CalculateWater(bool run)
    {
        StopWaterDebug("計算路線");
        if (!water.Manual) LoadWaterPattern(selectedWaterPattern);
        if (Service.LocalPlayer is not { } player) { waterDebugStatus = "目前沒有角色位置。"; return; }
        PilotPose pose = default;
        if (run)
        {
            actuator = new Actuator();
            if (!PilotPose.TryRead(actuator.Movement, out pose))
            { StopWaterDebug("無法讀取角色動作狀態。"); return; }
        }
        double now = PilotPose.Now;
        water.Calculate(new(player.Position.X, player.Position.Z), now);
        waterDebugStatus = $"計算完成：{water.QueryMs:F3} ms；預計 {(water.Plan?.Arrival + water.PlanEpoch - now) ?? 0:F2} 秒。";
        Service.Log.Information("[FallGuysPilot.Water] " + System.Text.Json.JsonSerializer.Serialize(water.Snapshot(now), Json));
        if (!run) return;
        Directory.CreateDirectory(directory);
        recordPath = Path.Combine(directory, $"HitWaterDebug-{DateTime.Now:yyyyMMdd-HHmmss}.jsonl");
        recorder = new SessionRecorder(recordPath, Json);
        context = new PilotContext { Act = actuator!, Hazards = new HazardField([]), Log = Write,
            Water = water, Now = now, Pose = pose, ReleasedAt = water.Epoch };
        engine = new PilotEngine(new WaterDebugScript(), pose, context);
        waterDebugActor = player.GameObjectId; waterDebugTerritory = Service.ClientState.TerritoryType;
        waterDebugRunning = controlling = true;
        waterDebugStatus = "執行中：到水區終點後停止。";
        Write("water-debug-start", water.Snapshot(now));
    }

    private void UpdateWaterDebug(double now)
    {
        if (!waterDebugRunning) return;
        if (Service.LocalPlayer is not { IsDead: false } player || player.GameObjectId != waterDebugActor ||
            Service.ClientState.TerritoryType != waterDebugTerritory || Service.Condition[ConditionFlag.BetweenAreas] ||
            Service.Condition[ConditionFlag.BetweenAreas51])
        { StopWaterDebug("角色或地圖已切換。"); return; }
        if (actuator!.CancelledByUser) { StopWaterDebug("手動操作，已交還控制。"); return; }
        if (!PilotPose.TryRead(actuator.Movement, out var pose)) return;
        context!.Now = now; context.Pose = pose; actuator.Update(now);
        engine!.Tick(context);
        if (engine.Current == null) { StopWaterDebug("已到達水區終點。"); return; }
        if (now >= nextFrame)
        {
            nextFrame = now + .1;
            Write("frame", new { at = now, feet = pose.Feet, pose.Motion, pose.Grounded, pose.Speed,
                command = actuator.LastCommand, water.Pattern, phase = WaterRoutes.Mod(now - water.Epoch, water.Cache.Model.Period) });
        }
    }

    private void StopWaterDebug(string status)
    {
        waterDebugStatus = status;
        if (!water.Manual) return;
        Write("water-debug-stop", new { at = PilotPose.Now, status });
        waterDebugRunning = controlling = false;
        actuator?.Dispose(); actuator = null;
        engine = null; context = null;
        if (recorder is { } finished) { finished.Complete(); System.Threading.Tasks.Task.Run(finished.Dispose); }
        recorder = null;
    }

    private void DrawWaterDebugSettings()
    {
        ImGui.Separator();
        if (!ImGui.CollapsingHeader("起始水區 Debug")) return;
        ImGui.Checkbox("顯示 AoE 與路線", ref waterOverlay);
        var patterns = water.Cache.Model.Patterns;
        if (ImGui.BeginCombo("AoE 相位模式", $"模式 {selectedWaterPattern + 1}"))
        {
            for (int i = 0; i < patterns.Length; ++i)
                if (ImGui.Selectable($"模式 {i + 1}（錄製 {string.Join(", ", patterns[i].Recordings)}）", i == selectedWaterPattern)) selectedWaterPattern = i;
            ImGui.EndCombo();
        }
        ImGui.InputFloat("載入時相位（秒）", ref selectedWaterPhase, .1f, 1f, "%.3f");
        if (ImGui.Button("載入所選模式")) LoadWaterPattern(selectedWaterPattern);
        ImGui.SameLine();
        if (ImGui.Button("隨機選取並載入")) LoadWaterPattern(Random.Shared.Next(patterns.Length));
        if (ImGui.Button("計算路徑")) CalculateWater(false);
        ImGui.SameLine();
        if (ImGui.Button("計算路徑並開始跑")) CalculateWater(true);
        ImGui.SameLine();
        if (ImGui.Button("停止##water")) StopWaterDebug("已停止。");
        if (ImGui.Button("返回正常觀測")) { StopWaterDebug("返回正常觀測。"); water.UseObservation(); }
        if (water.HasPhase) ImGui.TextUnformatted($"已載入：模式 {water.Pattern + 1} / {water.Source} / 相位 {WaterRoutes.Mod(PilotPose.Now - water.Epoch, water.Cache.Model.Period):F3} s");
        ImGui.TextWrapped(waterDebugStatus);
        ImGui.TextUnformatted("紅色填滿：生效中；灰色輪廓：等待；綠線：行走；橙線：跳躍。");
        if (water.HasPhase && ImGui.TreeNode("AoE list（13 個噴點）"))
        {
            double phaseTime = PilotPose.Now - water.Epoch;
            for (int i = 0; i < water.Cache.Model.Stations.Length; ++i)
            {
                var p = water.Cache.Model.Stations[i];
                var state = water.Cache.StationState(water.Pattern, i, phaseTime);
                ImGui.TextUnformatted($"#{i + 1} ({p.X:F1}, {p.Z:F1})  {(state.Active ? "生效，剩餘" : "下次生效")} {state.Seconds:F2} s");
            }
            ImGui.TreePop();
        }
        ImGui.TextWrapped("從角色目前位置執行至水區與立柱區之間的交接點；載入與計算本身不會移動角色。");
    }

    private void HandleWaterCommand(string[] args)
    {
        switch (args.FirstOrDefault() ?? "status")
        {
            case "load": LoadWaterPattern(int.Parse(args[1]) - 1); break;
            case "random": LoadWaterPattern(Random.Shared.Next(water.Cache.Model.Patterns.Length)); break;
            case "plan": CalculateWater(false); break;
            case "run": CalculateWater(true); break;
            case "stop": StopWaterDebug("已停止。"); break;
            case "observe": StopWaterDebug("返回正常觀測。"); water.UseObservation(); break;
            case "status": break;
            default: throw new ArgumentException("Use fallguys water load 1..4, random, plan, run, stop, observe or status.");
        }
        Service.Log.Information("[FallGuysPilot.Water] " + System.Text.Json.JsonSerializer.Serialize(new
            { waterDebugRunning, waterDebugStatus, state = water.Snapshot(PilotPose.Now) }, Json));
    }
}
