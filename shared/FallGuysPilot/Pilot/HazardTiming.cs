namespace MrBean.Modules.FallGuysPilot;

internal static class HazardTiming
{
    // Apply once outside the modeled danger interval, after mechanism duration
    // and clock uncertainty. This is independent of movement/contact margins.
    internal const double Padding = .150;
}
