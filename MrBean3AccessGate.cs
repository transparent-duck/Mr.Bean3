using Dalamud.Plugin;
using Dalamud.Plugin.Services;

namespace Mr.Bean3;

/// <summary>
/// Public access decision used by start authorization.
/// Official builds may supply a stricter gate via the build-profile overlay; the
/// public default is explicit allow-all.
/// </summary>
internal sealed record MrBean3AccessDecision(bool IsAllowed, string Reason)
{
    internal static MrBean3AccessDecision Allowed() =>
        new(true, "Public build profile allows all access.");

    internal static MrBean3AccessDecision Denied(string reason) =>
        new(false, reason);
}

/// <summary>
/// Host access gate abstraction. Public builds use <see cref="MrBean3AllowAllAccessGate"/>.
/// Official overlays may inject a private implementation without exposing it publicly.
/// </summary>
internal interface IMrBean3AccessGate
{
    MrBean3AccessDecision Current { get; }

    /// <summary>
    /// Read-only notice shown when a start is denied. Empty when unused.
    /// </summary>
    string StartDenialNotice { get; }

    /// <summary>
    /// True when this gate exempts the current context from the build-profile
    /// region restriction. Public builds always report false.
    /// </summary>
    bool IsRegionRestrictionExempt { get; }

    bool TryAuthorize(out string error);
}

internal sealed class MrBean3AllowAllAccessGate : IMrBean3AccessGate
{
    public MrBean3AccessDecision Current => MrBean3AccessDecision.Allowed();

    public string StartDenialNotice => string.Empty;

    public bool IsRegionRestrictionExempt => false;

    public bool TryAuthorize(out string error)
    {
        error = string.Empty;
        return true;
    }
}

internal delegate IMrBean3AccessGate MrBean3AccessGateFactory(
    IDalamudPluginInterface pluginInterface,
    IPluginLog pluginLog);
