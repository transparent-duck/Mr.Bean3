using Dalamud.Game.ClientState.Objects.SubKinds;
using Dalamud.IoC;
using Dalamud.Plugin;
using Dalamud.Plugin.Services;

namespace MrBean.Host
{
    // Standalone bindings for the unchanged exported Pilot source.
    internal class Service
    {
        [PluginService] internal static IDalamudPluginInterface PluginInterface { get; private set; } = null!;
        [PluginService] internal static IClientState ClientState { get; private set; } = null!;
        [PluginService] internal static IPlayerState PlayerState { get; private set; } = null!;
        [PluginService] internal static IObjectTable GameObjects { get; private set; } = null!;
        [PluginService] internal static IGameGui GameGui { get; private set; } = null!;
        [PluginService] internal static ICondition Condition { get; private set; } = null!;
        [PluginService] internal static IPluginLog Log { get; private set; } = null!;
        [PluginService] internal static IGameInteropProvider GameInteropProvider { get; private set; } = null!;
        [PluginService] internal static ISigScanner Scanner { get; private set; } = null!;
        internal static IPlayerCharacter? LocalPlayer => GameObjects.LocalPlayer;
    }

    // Running state is intentionally never restored after a plugin reload.
    public sealed class Configuration
    {
        public bool BlundervilleEnabled;
        public bool BlundervilleAutoLoop;
        public bool FallGuysShowActiveRoute { get; set; }
        public void Save() { }
    }
}

namespace DeepDungeon.Fsd.Dalamud
{
    internal sealed class Service : MrBean.Host.Service { }
}
