using System;
using System.IO;
using MrBean.FallGuys;
using MrBean.Modules.Blunderville;
using MrBean.Modules.FallGuysPilot.Courses;
using MrBean.Host;
using Dalamud.Game.ClientState.Conditions;
using FFXIVClientStructs.FFXIV.Component.GUI;

namespace MrBean.Modules.FallGuysPilot;

public sealed partial class FallGuysPilot
{
    private readonly Action<string> travel;
    private SessionRecorder? cycleRecorder;
    private string? cycleRecordPath;
    private bool cycleActive, cycleWon, cycleEntryRequested, cycleFullEntry, terminal;
    private int completedCycles, wonCycles;
    private double nextTerminalProbe, nextCycleAction;
    private uint cycleZone;

    private unsafe void ObserveTerminal(double now)
    {
        if (now < nextTerminalProbe) return;
        nextTerminalProbe = now + .2;
        static bool Visible(string name)
        {
            var addon = (AtkUnitBase*)Service.GameGui.GetAddonByName(name).Address;
            return addon != null && addon->IsVisible && addon->IsReady;
        }
        bool result = Visible("FGSResult");
        bool spectator = !qualified && released && Visible("FGSSpectatorMenu");
        terminal |= result || spectator || course == CourseId.Crown && qualified;
        if ((result || spectator) && !qualified && !eligibilityLost)
        {
            eligibilityLost = true;
            Write("eliminated", new { at = now, course, evidence = result ? "FGSResult" : "FGSSpectatorMenu" });
        }
    }

    private void SetLoop(bool enabled, string reason)
    {
        configuration.BlundervilleAutoLoop = enabled;
        if (enabled) { configuration.BlundervilleEnabled = true; yielded = false; }
        configuration.Save();
        if (enabled) StartCycle();
        else
        {
            CycleWrite("stopped", new { at = PilotPose.Now, reason });
            CloseCycle();
            actuator?.Release(); controlling = false;
        }
    }

    private void StartCycle()
    {
        if (cycleRecorder != null) return;
        Directory.CreateDirectory(directory);
        cycleRecordPath = Path.Combine(directory, $"Loop-{DateTime.Now:yyyyMMdd-HHmmss}.jsonl");
        cycleRecorder = new SessionRecorder(cycleRecordPath, Json);
        cycleZone = Service.ClientState.TerritoryType;
        cycleActive = cycleZone == TerritoryDuty;
        cycleFullEntry = cycleEntryRequested = false;
        cycleWon = course == CourseId.Crown && qualified;
        terminal = eligibilityLost || cycleWon;
        nextCycleAction = 0;
        travel("full-fps");
        CycleWrite("started", new { at = PilotPose.Now, zone = cycleZone, partialDuty = cycleActive,
            assembly = typeof(FallGuysPilot).Assembly.ManifestModule.ModuleVersionId });
    }

    private void UpdateCycle()
    {
        if (!configuration.BlundervilleAutoLoop || !configuration.BlundervilleEnabled)
        { if (cycleRecorder != null) CloseCycle(); return; }
        if (Service.LocalPlayer == null || Service.Condition[ConditionFlag.BetweenAreas] ||
            Service.Condition[ConditionFlag.BetweenAreas51]) return;
        if (cycleRecorder == null) StartCycle();
        double now = PilotPose.Now;
        uint zone = Service.ClientState.TerritoryType;
        if (zone != cycleZone)
        {
            if (zone == TerritoryDuty)
            {
                cycleActive = true; cycleWon = false; terminal = false;
                cycleFullEntry = cycleZone == 1197 && cycleEntryRequested;
                cycleEntryRequested = false;
                CycleWrite("entered", new { at = now, fullEntry = cycleFullEntry });
            }
            else if (cycleActive)
            {
                if (zone == 1197 && cycleFullEntry)
                { ++completedCycles; if (cycleWon) ++wonCycles; }
                CycleWrite("left", new { at = now, zone, won = cycleWon, fullEntry = cycleFullEntry,
                    completedCycles, wonCycles });
                cycleActive = false;
            }
            cycleZone = zone; nextCycleAction = now + 1;
        }
        if (now < nextCycleAction) return;
        string? action = zone == 1197 ? "enter" : zone == TerritoryDuty && (terminal || eligibilityLost) ? "leave" : null;
        if (action == null) return;
        nextCycleAction = now + 5;
        try
        {
            travel(action);
            if (action == "enter") cycleEntryRequested = true;
            CycleWrite("travel", new { at = now, action, zone });
        }
        catch (Exception error)
        { CycleWrite("travel-error", new { at = now, action, error = error.Message }); }
    }

    private void CycleWrite(string kind, object value) => cycleRecorder?.Write(kind, value);

    private void CloseCycle()
    {
        if (cycleRecorder == null) return;
        try { travel("cancel"); }
        finally
        {
            try { travel("restore-fps"); }
            finally { cycleRecorder.Dispose(); cycleRecorder = null; }
        }
    }
}
