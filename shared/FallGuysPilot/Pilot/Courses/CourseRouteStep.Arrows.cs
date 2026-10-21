using System;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed partial class CourseRouteStep
{
    private KnightArrowPolicy? arrowPolicy;
    private bool arrowRegionActive;
    private long arrowChanges;
    private Vector2? arrowRecoveryPoint;

    private bool TickArrowRegions(PilotContext c)
    {
        if (course != CourseId.Knight) return false;
        if (arrowRecoveryPoint is { } recovery && (!c.Pose.Has(3694) || seenHit != c.LastHitAt ||
            Vector2.DistanceSquared(recovery, c.Pose.XZ) > PilotEngine.BodyRadius * PilotEngine.BodyRadius))
            arrowRecoveryPoint = null;
        if (arrowRegionActive && arrowPolicy!.NeedsRecovery)
        {
            arrowRecoveryPoint = c.Pose.XZ;
            c.Log("knight-arrow-region-recover", new { at = c.Now, feet = c.Pose.Feet });
        }
        bool inside = c.Pose.Has(3694) && c.Pose.Grounded &&
            c.Pose.Feet.Z is > -295 and < -246 &&
            data.Ground.At(c.Pose.XZ) is { } height && Math.Abs(c.Pose.Feet.Y - height) <= .8f &&
            arrowRecoveryPoint == null;
        if (!inside)
        {
            if (arrowRegionActive)
            {
                c.Log("knight-arrow-region-exit", new { at = c.Now, feet = c.Pose.Feet,
                    arrowPolicy!.Decisions, arrowPolicy.Changes });
                c.Act.Hold(); arrowRegionActive = false;
                plan = null; moves = []; moveStarted = false; execution.ClearMove();
                nextRetry = 0;
            }
            return false;
        }
        arrowPolicy ??= new(data.Ground);
        if (!arrowRegionActive || seenHit != c.LastHitAt)
        {
            CancelBackground(); sections?.Cancel(); preparedSection = null;
            replanWanted = false; requestAfter = 0;
            plan = null; moves = []; execution.ClearMove(); moveStarted = false;
            arrowPolicy.Reset(); arrowRegionActive = true; seenHit = c.LastHitAt;
            sectionIndex = sections?.Locate(c.Pose.Feet) ?? 0;
            c.Log("knight-arrow-region-enter", new { at = c.Now, feet = c.Pose.Feet,
                heading = c.Pose.Raw.ArrowDirection, frameBudget = KnightArrowPolicy.FrameBudget });
        }
        arrowPolicy.Prepare(c.Hazards, c.Now, c.Act.ArrowRate);
        c.Act.ArrowRegions(arrowPolicy);
        progressAt = c.Now; progressPoint = c.Pose.XZ;
        if (arrowChanges != arrowPolicy.Changes)
        {
            arrowChanges = arrowPolicy.Changes;
            c.Log("knight-arrow-region-input", new { at = c.Now, nativeAt = arrowPolicy.ChangedAt,
                feet = arrowPolicy.ChangedFeet, heading = arrowPolicy.ChangedHeading, arrowPolicy.Target, arrowPolicy.Moving,
                arrowPolicy.Forced, arrowPolicy.Decisions });
        }
        return true;
    }
}
