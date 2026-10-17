using System.Numerics;
using MrBean.Host;
using Dalamud.Game.ClientState.Conditions;
using Dalamud.Game.ClientState.Objects.Enums;
using DeepDungeon.Fsd.Dalamud.Runtime.Helpers;
using FFXIVClientStructs.FFXIV.Client.Game.Control;
using FFXIVClientStructs.FFXIV.Client.Game.Event;
using FFXIVClientStructs.FFXIV.Component.GUI;
using OmenTools.Extensions;

namespace Mr.Bean3;

// Entry/exit operations follow the Blunderville travel flow.
internal sealed unsafe class PilotTravel : IDisposable
{
    private string state = "idle";
    private DateTime deadline, next;
    private bool ownsNavigation, cameraAligned;
    private (uint Limit, uint Background, uint Idle)? savedFrameSettings;

    public void Execute(string action)
    {
        if (action == "cancel") { Cancel(); return; }
        if (action == "restore-fps") { RestoreFrameSettings(); return; }
        if (action == "full-fps")
        {
            var settings = OmenTools.DService.Instance().GameConfig.System;
            if (!settings.TryGetUInt("Fps", out var limit) || !settings.TryGetUInt("FPSInActive", out var background) ||
                !settings.TryGetUInt("FPSDownAFK", out var idle)) throw new InvalidOperationException("無法讀取幀率設定。");
            savedFrameSettings ??= (limit, background, idle);
            settings.Set("Fps", 0u); settings.Set("FPSInActive", 0u); settings.Set("FPSDownAFK", 0u);
            return;
        }
        if (state != "idle") return;
        if (action == "enter" && Service.ClientState.TerritoryType == 1197) state = "approach";
        else if (action == "leave" && Service.ClientState.TerritoryType == 1165)
        {
            if (!LeaveDutyHelper.TryRequestLeaveDuty()) throw new InvalidOperationException("無法送出退本請求。");
            state = "leave";
        }
        else throw new InvalidOperationException("請先進入糖豆人大廳。");
        deadline = DateTime.UtcNow.AddMinutes(action == "enter" ? 4 : 1);
        next = DateTime.MinValue;
        cameraAligned = false;
    }

    public void Update()
    {
        if (state == "idle" || Service.Condition[ConditionFlag.BetweenAreas] || Service.Condition[ConditionFlag.BetweenAreas51]) return;
        var now = DateTime.UtcNow;
        if (now >= deadline) { Cancel(); throw new InvalidOperationException("報名或退本逾時。"); }
        if (now < next) return;
        next = now.AddMilliseconds(250);
        uint zone = Service.ClientState.TerritoryType;
        if (state == "leave")
        {
            if (zone != 1165) { Cancel(); return; }
            if (Click("FGSExitDialog", "退出") || Click("FGSResult", "退出")) next = now.AddSeconds(2);
            return;
        }
        if (zone == 1165) { Cancel(); return; }
        if (zone != 1197 || Service.LocalPlayer is not { IsDead: false } player) return;
        if (state == "approach")
        {
            if (Ready("FGSEnterDialog") != null) { StopNavigation(); state = "dialog"; return; }
            var npc = Service.GameObjects.FirstOrDefault(o => o.ObjectKind == ObjectKind.EventNpc && o.BaseId == 1046440 && o.IsTargetable);
            if (npc == null) return;
            var native = (FFXIVClientStructs.FFXIV.Client.Game.Object.GameObject*)npc.Address;
            if (Vector3.Distance(player.Position, npc.Position) > 3.5f ||
                !EventFramework.Instance()->CheckInteractRange((FFXIVClientStructs.FFXIV.Client.Game.Object.GameObject*)player.Address, native, 1, false))
            {
                if (!ownsNavigation)
                {
                    bool accepted = Service.PluginInterface.GetIpcSubscriber<Vector3, bool, bool>("vnavmesh.SimpleMove.PathfindAndMoveTo")
                        .InvokeFunc(npc.Position, false);
                    if (!accepted) throw new InvalidOperationException("無法接近報名 NPC；請確認 vnavmesh 可用，或先走到 NPC 旁。");
                    ownsNavigation = true;
                }
                return;
            }
            StopNavigation();
            if (!cameraAligned)
            {
                var camera = CameraManager.Instance()->GetActiveCamera();
                if (camera == null) return;
                var toward = npc.Position - player.Position;
                camera->DirH = MathF.IEEERemainder(MathF.Atan2(toward.X, toward.Z) + MathF.PI, MathF.Tau);
                camera->DirV = Math.Clamp(-MathF.PI / 6, camera->DirVMin, camera->DirVMax);
                cameraAligned = true;
                return;
            }
            if (!Service.GameGui.WorldToScreen(npc.Position + Vector3.UnitY, out _)) return;
            var targets = TargetSystem.Instance();
            targets->Target = native;
            targets->InteractWithObject(native);
            state = "dialog";
            return;
        }
        if (state == "dialog" && Click("FGSEnterDialog", "开始")) { state = "queue"; next = now.AddSeconds(2); }
        else if (state == "queue" && Click("ContentsFinderConfirm", "进入")) next = now.AddSeconds(2);
    }

    private static AtkUnitBase* Ready(string name)
    {
        var addon = (AtkUnitBase*)Service.GameGui.GetAddonByName(name).Address;
        return addon != null && addon->IsVisible && addon->IsReady ? addon : null;
    }

    private static bool Click(string name, string label)
    {
        var addon = Ready(name);
        if (addon == null) return false;
        for (int i = 0; i < addon->UldManager.NodeListCount; ++i)
        {
            var node = addon->UldManager.NodeList[i];
            if (node == null || (int)node->Type < 1000 || !node->IsVisible()) continue;
            var component = ((AtkComponentNode*)node)->Component;
            if (component == null || component->GetComponentType() != ComponentType.Button) continue;
            var button = (AtkComponentButton*)component;
            if (!button->IsEnabled || button->ButtonTextNode == null || button->ButtonTextNode->NodeText.ToString() != label) continue;
            var evt = node->AtkEventManager.Event;
            for (int j = 0; evt != null && j < 32; ++j, evt = evt->NextEvent)
                if (evt->State.EventType == AtkEventType.ButtonClick) { button->Click(evt); return true; }
        }
        return false;
    }

    private void StopNavigation()
    {
        if (!ownsNavigation) return;
        ownsNavigation = false;
        Service.PluginInterface.GetIpcSubscriber<object>("vnavmesh.Path.Stop").InvokeAction();
    }
    private void Cancel() { state = "idle"; StopNavigation(); }
    private void RestoreFrameSettings()
    {
        if (savedFrameSettings is not { } saved) return;
        var settings = OmenTools.DService.Instance().GameConfig.System;
        settings.Set("Fps", saved.Limit); settings.Set("FPSInActive", saved.Background); settings.Set("FPSDownAFK", saved.Idle);
        savedFrameSettings = null;
    }
    public void Dispose() { try { Cancel(); } finally { RestoreFrameSettings(); } }
}
