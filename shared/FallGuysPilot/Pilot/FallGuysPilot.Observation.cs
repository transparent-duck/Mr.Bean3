using System;
using MrBean.Host;

namespace MrBean.Modules.FallGuysPilot;

public sealed partial class FallGuysPilot
{
    private long sceneSequence;

    // No snapshot allocation or
    // serialization takes place unless a standalone host subscribes.
    public event Action<PilotObservation>? Observed;

    private void PublishObservation(string kind)
    {
        var observer = Observed;
        if (observer == null || kind is not ("start" or "release" or "qualified" or
            "winner" or "eliminated" or "yield" or "scene-end")) return;
        double elapsed = released && context is { ReleasedAt: > 0 } current
            ? Math.Max(0, PilotPose.Now - current.ReleasedAt) : 0;
        var observation = new PilotObservation(kind, sceneSequence, course, DateTimeOffset.UtcNow,
            elapsed, released, qualified, eligibilityLost, yielded, rank,
            localHits, actuator?.Jumps ?? 0, actuator?.Nudges ?? 0);
        try { observer(observation); }
        catch (Exception error) { Service.Log.Error(error, "[FallGuysPilot] Host observation failed."); }
    }
}
