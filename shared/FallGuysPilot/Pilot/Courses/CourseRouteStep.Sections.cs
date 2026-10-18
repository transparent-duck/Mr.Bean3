using System;
using System.Linq;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed partial class CourseRouteStep
{
    private SectionRoutePlanner? sections;
    private SectionRoutePlanner.Result? preparedSection;
    private int sectionIndex;
    private double requestedHit, requestedAt;
    private Vector2 requestedFeet;
    private bool requestedForecast;
    private bool requestedForced;
    private double prefetchAfter;

    public void Dispose()
    {
        sections?.Dispose(); CancelBackground(); background?.Dispose(); background = null;
    }

    private bool TrackRecovery(PilotContext c)
    {
        if (plan is not { Found: true }) return false;
        for (int i = legIndex; i < plan.Legs.Length; ++i)
        {
            var motion = (PilotCourseGeometry.Move[])plan.Legs[i].Edge.Motion!;
            for (int j = i == legIndex ? moveIndex : 0; j < motion.Length; ++j)
            {
                var move = motion[j];
                if (move.Kind != "recovery" || Vector2.DistanceSquared(move.Path[0].XZ(), c.Pose.XZ) > .3f * .3f) continue;
                legIndex = i; moves = motion; moveIndex = j;
                moveDuration = PilotCourseGeometry.Trajectory([move], speed)[^1].Time;
                moveStarted = true; passage = null;
                return true;
            }
        }
        return false;
    }

    private void SelectSection(PilotContext c, double? readyAt, int? nextSection = null)
    {
        if (sections!.Pending && !requestedForecast && requestedHit == c.LastHitAt &&
            (requestedForced && march.Native || Vector2.DistanceSquared(requestedFeet, c.Pose.XZ) < .01f) &&
            (readyAt == null || Math.Abs(readyAt.Value - requestedAt) < .12)) return;
        preparedSection = null;
        sectionIndex = nextSection ?? sections.Locate(c.Pose.Feet);
        bool forced = march.Pending && (c.Pose.Grounded || march.Native);
        var connectors = EntryConnectors(c, data.Sections[sectionIndex].Entries);
        if (!connectors.Any(e => data.Sections[sectionIndex].Nodes.Contains(e.To)))
            for (int i = sectionIndex + 1; i < data.Sections.Length; ++i)
                if (connectors.Any(e => data.Sections[i].Nodes.Contains(e.To))) { sectionIndex = i; break; }
        bool canWait = !forced && HitCourseGeometry.CanWait(c.Pose);
        // Leave time for the worker. Otherwise every result starts in the past
        // and retiming can miss a narrow opening before it is even executed.
        requestedAt = readyAt ?? c.Now + (canWait ? sections.ReadySeconds : 0);
        requestedFeet = c.Pose.XZ;
        requestedHit = c.LastHitAt; requestedForecast = false;
        requestedForced = forced;
        ++SearchNumber;
        sections.Start(sectionIndex, c.Hazards, requestedFeet, requestedAt,
            canWait, connectors, c.LastHitAt, false,
            forced ? march.Snapshot() : null, forced ? c.Pose : null);
        plan = null; moves = []; passage = null; moveStarted = false;
        seenHit = c.LastHitAt; plannedRevision = c.Hazards.RouteRevision;
    }

    private void PollSections(PilotContext c)
    {
        if (sections == null || !sections.TryTake(out var result) || result == null) return;
        c.Log("course-section-search", new { at = c.Now, section = data.Sections[result.Section].Name,
            result.Forecast, startAt = result.At, result.Start, result.Milliseconds, result.Plan.Found, result.Plan.Expanded, result.Error });
        if (result.Forecast)
        {
            if (plan != null && result.HitAt == c.LastHitAt && result.Section == sectionIndex + 1)
                preparedSection = result;
            return;
        }
        if (!AcceptSection(c, result)) { plan = null; nextRetry = c.Now + .25; }
    }

    private bool AcceptSection(PilotContext c, SectionRoutePlanner.Result result)
    {
        bool forced = march.Pending && (c.Pose.Grounded || march.Native);
        if (!result.Plan.Found || result.HitAt != c.LastHitAt) return false;
        double at = nativePaused && double.IsFinite(nativeReadyAt) ? Math.Max(c.Now, nativeReadyAt) : c.Now;
        if (!forced && !result.Forecast) at = Math.Max(at, result.At);
        var candidate = result.Plan;
        bool displaced = Vector2.DistanceSquared(result.Start, c.Pose.XZ) > .01f * .01f;
        bool retimed = Math.Abs(at - result.At) > .001 || displaced;
        if (retimed && forced) candidate = march.Retime(data, candidate, c.Pose, at);
        if (retimed && !forced)
        {
            long began = System.Diagnostics.Stopwatch.GetTimestamp();
            ++SearchNumber;
            candidate = graph.Reschedule(c.Hazards.Snapshot(at), candidate, 0, c.Pose.XZ, at,
                HitCourseGeometry.CanWait(c.Pose), reconnect: displaced ? SectionConnectors(c, candidate) : null);
            c.Now += System.Diagnostics.Stopwatch.GetElapsedTime(began).TotalSeconds;
        }
        // A prefetched spatial path may lose an entire opening after a small
        // handoff error. Search other lanes from the actual feet in that case.
        if (!forced && result.Forecast && candidate.Found &&
            candidate.Arrival - result.Plan.Arrival > Math.Max(0, at - result.At) + .25)
        {
            c.Log("course-section-reselect", new { at = c.Now, section = data.Sections[result.Section].Name,
                originalArrival = result.Plan.Arrival, retimedArrival = candidate.Arrival,
                late = at - result.At, feet = c.Pose.Feet });
            return false;
        }
        if (!candidate.Found || march.Native && !march.Matches(candidate) ||
            graph.HasConflict(c.Hazards, candidate, 0, c.Pose.XZ, Math.Min(c.Now, at))) return false;
        sectionIndex = result.Section; plan = candidate;
        legIndex = moveIndex = 0; moves = []; passage = null; moveStarted = false;
        seenHit = c.LastHitAt; plannedRevision = c.Hazards.RouteRevision;
        nextRetry = 0; prefetchAfter = 0;
        c.Log("course-section-ready", new { at = c.Now, section = data.Sections[sectionIndex].Name,
            result.Forecast, retimed, candidate.Arrival, candidate.Expanded, feet = c.Pose.Feet, RouteNumber,
            route = candidate.Legs.Select(l => new { l.Edge.From, l.Edge.To, l.Departure,
                duration = l.Edge.Duration, motion = l.Edge.Motion }).ToArray() });
        return true;
    }

    private HitTimedRoutes.Edge[] SectionConnectors(PilotContext c, HitTimedRoutes.Plan candidate)
        => EntryConnectors(c, candidate.Legs.Length > 0 ? [candidate.Legs[0].Edge.To] : []);

    private HitTimedRoutes.Edge[] EntryConnectors(PilotContext c, int[] targets)
    {
        var connectors = data.Connectors(c.Pose, speed).ToList();
        if (HitCourseGeometry.CanWait(c.Pose))
        foreach (int target in targets)
        {
            var feet = data.Nodes[target].Feet;
            if (Vector2.DistanceSquared(feet.XZ(), c.Pose.XZ) <= .29f * .29f && data.Clear(c.Pose.Feet, feet.XZ()))
                connectors.Add(data.Edge(data.Nodes.Length, target, [PilotCourseGeometry.Move.Walk(c.Pose.Feet, feet)], speed));
        }
        return connectors.ToArray();
    }

    private void PrepareNextSection(PilotContext c)
    {
        if (preparedSection is { } prepared && prepared.Revision != c.Hazards.RouteRevision)
        {
            if (!prepared.Plan.Found || graph.HasConflict(c.Hazards, prepared.Plan, 0, prepared.Start, prepared.At, prepared.Revision))
                preparedSection = null;
            else preparedSection = prepared with { Revision = c.Hazards.RouteRevision };
        }
        if (sections == null || sections.Pending || preparedSection != null || plan is not { Found: true, Legs.Length: > 0 } ||
            sectionIndex + 1 >= data.Sections.Length || c.Now < prefetchAfter ||
            plan.Arrival - c.Now > sections.LeadSeconds || march.Pending) return;
        var end = plan.Legs[^1].Edge;
        int target = end.To;
        var feet = data.Nodes[target].Feet;
        var join = new PilotCourseGeometry.Move { Kind = "join", Points = [[feet.X, feet.Y, feet.Z], [feet.X, feet.Y, feet.Z]] };
        var connector = data.Edge(data.Nodes.Length, target, [join], speed);
        requestedForecast = true; requestedHit = c.LastHitAt; requestedFeet = feet.XZ(); requestedAt = plan.Arrival;
        ++SearchNumber;
        sections.Start(sectionIndex + 1, c.Hazards, requestedFeet, requestedAt, data.Nodes[target].CanWait,
            [connector], c.LastHitAt, true);
        c.Log("course-section-prefetch", new { at = c.Now, section = data.Sections[sectionIndex + 1].Name,
            readyAt = requestedAt, remaining = requestedAt - c.Now });
        prefetchAfter = c.Now + .25;
    }

    private bool AdvanceSection(PilotContext c)
    {
        if (preparedSection is { } next)
        {
            preparedSection = null;
            if (AcceptSection(c, next)) return true;
            sections!.Cancel();
            SelectSection(c, null, next.Section);
            c.Act.Hold(); progressAt = c.Now;
            return false;
        }
        if (sections!.Pending) { c.Act.Hold(); progressAt = c.Now; return false; }
        plan = null; SelectSection(c, null);
        c.Act.Hold(); progressAt = c.Now;
        return false;
    }
}
