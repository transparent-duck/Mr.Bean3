using MrBean.Modules.FallGuysPilot;
using MrBean.Host;
using MrBean.Host.Modules;
using Dalamud.Bindings.ImGui;
using Dalamud.Game.Command;
using Dalamud.Game.ClientState.Conditions;
using Dalamud.Plugin;
using Dalamud.Plugin.Services;
using OmenTools;
using OmenTools.OmenService;

namespace Mr.Bean3;

public sealed class Plugin : IDalamudPlugin
{
    private readonly IDalamudPluginInterface pluginInterface;
    private readonly ICommandManager commands;
    private readonly IFramework framework;
    private readonly IChatGui chat;
    private readonly IMrBean3AccessGate accessGate;
    private readonly Configuration configuration = new();
    private readonly LocalStatistics statistics;
    private FallGuysPilot? pilot;
    private PilotTravel? travel;
    private ActionTracker? tracker;
    private bool open, disposed;
    private ulong runningCid;
    public string Name => ProductIdentity.DisplayName;

    public Plugin(IDalamudPluginInterface pluginInterface, ICommandManager commandManager,
        IFramework framework, IChatGui chatGui, IPluginLog pluginLog)
    {
        this.pluginInterface = pluginInterface;
        commands = commandManager;
        this.framework = framework;
        chat = chatGui;
        statistics = new LocalStatistics(pluginInterface.GetPluginConfigDirectory(),
            error => pluginLog.Error(error, "[Mr.Bean3] Statistics persistence failed."));
        bool omenInitialized = false;
        try
        {
            pluginInterface.Create<Service>();
            DService.Init(pluginInterface, static () => new DServiceInitOptions()
                .Disable<ItemSourceManager>().Disable<LogMessageManager>().Disable<TargetManager>().Disable<UseActionManager>());
            omenInitialized = true;
            accessGate = MrBean3BuildProfile.CreateAccessGate(pluginInterface, pluginLog);
            commands.AddHandler(ProductIdentity.Command, new CommandInfo(OnCommand) { ShowInHelp = false });
            pluginInterface.UiBuilder.Draw += Draw;
            pluginInterface.UiBuilder.OpenConfigUi += Open;
            pluginInterface.UiBuilder.OpenMainUi += Open;
            framework.Update += Update;
        }
        catch
        {
            framework.Update -= Update;
            pluginInterface.UiBuilder.Draw -= Draw;
            pluginInterface.UiBuilder.OpenConfigUi -= Open;
            pluginInterface.UiBuilder.OpenMainUi -= Open;
            commands.RemoveHandler(ProductIdentity.Command);
            if (omenInitialized) DService.Uninit();
            throw;
        }
    }

    private void Open() => open = true;
    private void OnCommand(string _, string args)
    {
        if (args.Trim().Equals("stop", StringComparison.OrdinalIgnoreCase)) Stop();
        else if (args.Trim().Equals("start", StringComparison.OrdinalIgnoreCase)) Start();
        else Open();
    }

    private void Start()
    {
        if (pilot != null) return;
        if (!accessGate.TryAuthorize(out _)) return;
        if (Service.LocalPlayer is not { IsDead: false } || Service.ClientState.TerritoryType is not (1197 or 1165))
        { chat.PrintError("請先進入糖豆人大廳。"); return; }
        try
        {
            runningCid = Service.PlayerState.ContentId;
            travel = new PilotTravel();
            tracker = new ActionTracker();
            pilot = new FallGuysPilot(configuration, tracker, travel.Execute);
            statistics.BeginSession(Service.ClientState.TerritoryType == 1165);
            pilot.Observed += statistics.Observe;
            pilot.HandleCommand(["loop", "on"]);
        }
        catch (Exception failure) { Stop(); chat.PrintError("Mr.Bean3 無法開始：" + failure.Message); }
    }

    private void Update(IFramework _)
    {
        statistics.FlushPending();
        if (pilot == null) return;
        bool zoning = Service.Condition[ConditionFlag.BetweenAreas] || Service.Condition[ConditionFlag.BetweenAreas51];
        if (!zoning && (!Service.ClientState.IsLoggedIn || Service.PlayerState.ContentId != runningCid))
        { Stop(); return; }
        try
        {
            statistics.UpdateTerritory(Service.ClientState.TerritoryType, zoning);
            pilot.Update();
            travel?.Update();
            if (!configuration.BlundervilleAutoLoop) Stop();
        }
        catch (Exception failure) { Stop(); Service.Log.Error(failure, "[Mr.Bean3] Pilot host stopped."); }
    }

    private void Draw()
    {
        if (!open) return;
        ImGui.SetNextWindowSize(new(480, 100), ImGuiCond.FirstUseEver);
        if (ImGui.Begin("你是壞孩子###Mr.Bean3", ref open, ImGuiWindowFlags.AlwaysAutoResize))
        {
            ImGui.BeginDisabled(pilot != null || !accessGate.Current.IsAllowed);
            if (ImGui.Button("開始")) Start();
            ImGui.EndDisabled();
            ImGui.SameLine();
            if (ImGui.Button("停止")) Stop();
            if (!accessGate.Current.IsAllowed)
                ImGui.TextUnformatted(string.IsNullOrEmpty(accessGate.StartDenialNotice) ? "目前無法使用。" : accessGate.StartDenialNotice);
            ImGui.TextUnformatted($"關卡總數: {statistics.TotalRuns}");
            ImGui.TextUnformatted($"奪冠: {statistics.Wins}");
            ImGui.TextUnformatted($"失敗: {statistics.Failures}");
            ImGui.TextUnformatted($"手動接管或中途停止: {statistics.Interrupted}");
            ImGui.TextUnformatted($"勝率: {statistics.WinRate * 100:0.0}%");
            ImGui.TextUnformatted("要不要試試一鍵登頂妖宮");
            ImGui.SameLine();
            if (ImGui.SmallButton("打開"))
                Dalamud.Utility.Util.OpenLink("https://github.com/transparent-duck/NewFolder3");
        }
        ImGui.End();
        if (!open) Stop();
    }

    private void Stop()
    {
        statistics.Stop();
        configuration.BlundervilleEnabled = configuration.BlundervilleAutoLoop = false;
        var oldPilot = pilot; pilot = null;
        var oldTravel = travel; travel = null;
        var oldTracker = tracker; tracker = null;
        DisposePart(oldPilot);
        DisposePart(oldTravel);
        DisposePart(oldTracker);
    }

    private static void DisposePart(IDisposable? part)
    {
        try { part?.Dispose(); }
        catch (Exception error) { Service.Log.Error(error, "[Mr.Bean3] Cleanup failed."); }
    }

    public void Dispose()
    {
        if (disposed) return;
        disposed = true;
        framework.Update -= Update;
        pluginInterface.UiBuilder.Draw -= Draw;
        pluginInterface.UiBuilder.OpenConfigUi -= Open;
        pluginInterface.UiBuilder.OpenMainUi -= Open;
        commands.RemoveHandler(ProductIdentity.Command);
        try { Stop(); }
        finally { try { statistics.FlushPending(true); } finally { DService.Uninit(); } }
    }
}
