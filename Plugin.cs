using Dalamud.Game.Command;
using Dalamud.Plugin;
using Dalamud.Plugin.Services;

namespace Mr.Bean3;

public sealed class Plugin : IDalamudPlugin
{
    private readonly ICommandManager _commandManager;
    private readonly IChatGui _chatGui;
    private readonly IMrBean3AccessGate _accessGate;
    private bool _disposed;

    public string Name => ProductIdentity.DisplayName;

    public Plugin(
        IDalamudPluginInterface pluginInterface,
        ICommandManager commandManager,
        IChatGui chatGui,
        IPluginLog pluginLog)
    {
        _commandManager = commandManager ?? throw new ArgumentNullException(nameof(commandManager));
        _chatGui = chatGui ?? throw new ArgumentNullException(nameof(chatGui));
        _ = pluginInterface ?? throw new ArgumentNullException(nameof(pluginInterface));

        _accessGate = MrBean3BuildProfile.CreateAccessGate(pluginInterface, pluginLog);
        _commandManager.AddHandler(ProductIdentity.Command, new CommandInfo(OnCommand)
        {
            HelpMessage = $"Show {ProductIdentity.DisplayName} status."
        });
    }

    private void OnCommand(string command, string arguments)
    {
        if (_accessGate.TryAuthorize(out var error))
        {
            _chatGui.Print($"{ProductIdentity.DisplayName}: authorized.");
        }
        else
        {
            var notice = _accessGate.StartDenialNotice;
            _chatGui.PrintError(string.IsNullOrEmpty(notice)
                ? $"{ProductIdentity.DisplayName}: {error}"
                : $"{ProductIdentity.DisplayName}: {notice}");
        }
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        _commandManager.RemoveHandler(ProductIdentity.Command);
    }
}
