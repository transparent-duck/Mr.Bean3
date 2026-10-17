using Dalamud.Plugin;

namespace Mr.Bean3;

public sealed class Plugin : IDalamudPlugin
{
    public string Name => ProductIdentity.DisplayName;

    public Plugin(IDalamudPluginInterface pluginInterface)
    {
        _ = pluginInterface;
    }

    public void Dispose()
    {
    }
}
