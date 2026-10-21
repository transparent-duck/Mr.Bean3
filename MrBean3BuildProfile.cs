using Dalamud.Plugin;
using Dalamud.Plugin.Services;

namespace Mr.Bean3;

/// <summary>
/// Explicit capabilities for this host build.
/// Public checkouts resolve to <see cref="PublicNoService"/>. Official private
/// builds may stage <c>OfficialBuildProfile.g.cs</c> to replace these values via
/// <see cref="MrBean3BuildProfile.ProvideOfficialCapabilities"/>.
/// </summary>
internal sealed class MrBean3BuildCapabilities
{
    public MrBean3AccessGateFactory? CreateAccessGate { get; init; }

    public static MrBean3BuildCapabilities PublicNoService { get; } = new()
    {
        CreateAccessGate = null
    };
}

/// <summary>
/// Build-profile extension boundary. Without an official overlay this is the
/// public no-service profile: allow-all gate.
/// </summary>
internal static partial class MrBean3BuildProfile
{
    internal static MrBean3BuildCapabilities Capabilities { get; } = ResolveCapabilities();

    internal static IMrBean3AccessGate CreateAccessGate(
        IDalamudPluginInterface pluginInterface,
        IPluginLog pluginLog)
    {
        ArgumentNullException.ThrowIfNull(pluginInterface);
        ArgumentNullException.ThrowIfNull(pluginLog);

        MrBean3AccessGateFactory? factory = Capabilities.CreateAccessGate;
        IMrBean3AccessGate gate = factory != null
            ? factory(pluginInterface, pluginLog)
            : new MrBean3AllowAllAccessGate();
        return new MrBean3CidExemptGate(gate);
    }

    /// <summary>
    /// Optional official overlay hook. Implemented only by staged
    /// <c>OfficialBuildProfile.g.cs</c>; public builds leave this unimplemented.
    /// </summary>
    static partial void ProvideOfficialCapabilities(
        ref MrBean3BuildCapabilities? capabilities);

    private static MrBean3BuildCapabilities ResolveCapabilities()
    {
        MrBean3BuildCapabilities? official = null;
        ProvideOfficialCapabilities(ref official);
        return official ?? MrBean3BuildCapabilities.PublicNoService;
    }
}
