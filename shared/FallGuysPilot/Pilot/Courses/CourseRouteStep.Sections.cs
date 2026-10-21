using System;
using System.Linq;
using System.Numerics;
using MrBean.FallGuys;

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
    private long prefetchRevision, prefetchCheckedRevision;

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
                execution.ClearMove();
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
        var connectors = course == CourseId.Knight && sectionIndex == 1 ? [] : EntryConnectors(c, data.Sections[sectionIndex].Entries);
        if (!connectors.Any(e => data.Sections[sectionIndex].Nodes.Contains(e.To)))
            for (int i = sectionIndex + 1; i < data.Sections.Length; ++i)
                if (connectors.Any(e => data.Sections[i].Nodes.Contains(e.To))) { sectionIndex = i; break; }
        bool canWait = !forced && (course != CourseId.Hit || HitCourseGeometry.CanWait(c.Pose));
        // Leave time for the worker. Otherwise every result starts in the past
        // and retiming can miss a narrow opening before it is even executed.
        requestedAt = readyAt ?? c.Now + (canWait ? sections.ReadySeconds : 0);
        requestedFeet = c.Pose.XZ;
        requestedHit = c.LastHitAt; requestedForecast = false;
        requestedForced = forced;
        ++SearchNumber;
        sections.Start(sectionIndex, c.Hazards, requestedFeet, requestedAt,
            canWait, connectors, c.LastHitAt, false,
            forced ? march.Snapshot() : null, forced || course == CourseId.Knight ? c.Pose : null,
            immobile: hadBomb ? new(bombExpires - BombLead, bombExpires + BombTail) : null,
            arrowHeading: c.Pose.Has(3694) && c.Pose.Raw.ArrowDirection is { } arrow
                ? Geo.WrapAngle(arrow + c.Act.ArrowRate * (float)(requestedAt - c.Now)) : null,
            arrowRate: c.Act.ArrowRate, entryArrowHeading: c.Act.ArrowEntryHeading);
        plan = null; moves = []; execution.ClearMove(); moveStarted = false;
        seenHit = c.LastHitAt; plannedRevision = c.Hazards.RouteRevision;
    }

    private void PollSections(PilotContext c)
    {
        if (sections == null || !sections.TryTake(out var result) || result == null) return;
        c.Log("course-section-search", new { at = c.Now, section = data.Sections[result.Section].Name,
            result.Forecast, startAt = result.At, result.Start, result.Milliseconds, result.Plan.Found, result.Plan.Expanded, result.Error, result.Columns });
        if (result.Forecast)
        {
            if (plan != null && result.HitAt == c.LastHitAt && result.Section == sectionIndex + 1 &&
                (result.Plan.Found || preparedSection == null))
                preparedSection = result;
            return;
        }
        if (!AcceptSection(c, result))
        {
            plan = null; nextRetry = c.Now + .25;
            if (!result.Plan.Found && result.HitAt == c.LastHitAt) FailedRoute(c);
        }
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
        if (course == CourseId.Knight && result.Section == 1 && c.Pose.Has(3693) && retimed)
        {
            // A pending timing gate still permits adoption of the unchanged
            // schedule. Otherwise retry on the worker, never search on-frame.
            var first = candidate.Legs.FirstOrDefault(l => l.Edge.Duration > 1e-6);
            if (displaced || first == null || first.Departure < c.Now - .001) return false;
            retimed = false;
        }
        if (retimed && forced) candidate = march.Retime(data, candidate, c.Pose, at);
        if (retimed && !forced)
        {
            long began = System.Diagnostics.Stopwatch.GetTimestamp();
            ++SearchNumber;
            candidate = graph.Reschedule(c.Hazards.Snapshot(at), candidate, 0, c.Pose.XZ, at,
                course != CourseId.Hit || HitCourseGeometry.CanWait(c.Pose),
                immobile: hadBomb ? new(bombExpires - BombLead, bombExpires + BombTail) : null,
                arrowHeading: c.Pose.Has(3694) && c.Pose.Raw.ArrowDirection is { } arrow
                    ? Geo.WrapAngle(arrow + c.Act.ArrowRate * (float)(at - c.Now)) : null,
                arrowRate: c.Act.ArrowRate, reconnect: displaced ? SectionConnectors(c, candidate) : null,
                entryArrowHeading: c.Act.ArrowEntryHeading);
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
        noRouteSince = double.NaN;
        // A refresh for this handoff can still be running when the retained
        // route is used. It is no longer the next section's search.
        sections?.Cancel();
        legIndex = moveIndex = 0; moves = []; execution.ClearMove(); moveStarted = false;
        seenHit = c.LastHitAt; plannedRevision = c.Hazards.RouteRevision;
        nextRetry = 0; prefetchAfter = 0;
        c.Log("course-section-ready", new { at = c.Now, section = data.Sections[sectionIndex].Name,
            result.Forecast, retimed, candidate.Arrival, candidate.Expanded, feet = c.Pose.Feet, RouteNumber,
            route = candidate.Legs.Select(l => new { l.Edge.From, l.Edge.To, l.Departure,
                duration = l.Edge.Duration, motion = l.Edge.Motion }).ToArray() });
        return true;
    }

    private HitTimedRoutes.Edge[] SectionConnectors(PilotContext c, HitTimedRoutes.Plan candidate)
    {
        if (course == CourseId.Knight && candidate.Legs.Length > 0 &&
            candidate.Legs[0].Edge is { Destination: not null } first && first.To >= data.Nodes.Length)
        {
            var target = ((PilotCourseGeometry.Move[])first.Motion!)[^1].Path[^1];
            if (c.Pose.Has(3693) || Vector2.DistanceSquared(c.Pose.XZ, target.XZ()) > 400 ||
                PilotCourseGeometry.CrossesIce(c.Pose.Feet, target) || !data.Ground.ClearFromGrounded(c.Pose.Feet, target.XZ())) return [];
            var edge = data.Edge(data.Nodes.Length, first.To, [PilotCourseGeometry.Move.Walk(c.Pose.Feet, target)], speed);
            edge.Destination = first.Destination;
            return [edge];
        }
        return EntryConnectors(c, candidate.Legs.Length > 0 ? [candidate.Legs[0].Edge.To] : []);
    }

    private HitTimedRoutes.Edge[] EntryConnectors(PilotContext c, int[] targets)
        => data.EntryConnectors(c.Pose, speed, targets);

    private void PrepareNextSection(PilotContext c)
    {
        if (preparedSection is { } prepared && prepared.Revision != c.Hazards.RouteRevision)
        {
            if (!prepared.Plan.Found || graph.HasConflict(c.Hazards, prepared.Plan, 0, prepared.Start, prepared.At, prepared.Revision))
                preparedSection = null;
            else preparedSection = prepared with { Revision = c.Hazards.RouteRevision };
        }
        if (sections == null || sections.Pending || plan is not { Found: true, Legs.Length: > 0 } ||
            sectionIndex + 1 >= data.Sections.Length ||
            plan.Arrival - c.Now > sections.LeadSeconds || march.Pending) return;
        // A safe steering route can still contain an avoidable entrance wait
        // or lateral move after a clock update. Refresh its selection in the
        // worker, retaining the usable route until a replacement is ready.
        if (preparedSection != null)
        {
            if (data.Sections[sectionIndex + 1].Name != "steering" ||
                prefetchCheckedRevision == c.Hazards.Revision) return;
            prefetchCheckedRevision = c.Hazards.Revision;
            if (!HitSteeringStep.TimingChanged(c.Hazards, prefetchRevision)) return;
        }
        else if (c.Now < prefetchAfter) return;
        var end = plan.Legs[^1].Edge;
        int target = end.To;
        var feet = data.Nodes[target].Feet;
        var join = new PilotCourseGeometry.Move { Kind = "join", Points = [[feet.X, feet.Y, feet.Z], [feet.X, feet.Y, feet.Z]] };
        var connector = data.Edge(data.Nodes.Length, target, [join], speed);
        requestedForecast = true; requestedHit = c.LastHitAt; requestedFeet = feet.XZ(); requestedAt = plan.Arrival;
        prefetchRevision = prefetchCheckedRevision = c.Hazards.Revision;
        ++SearchNumber;
        sections.Start(sectionIndex + 1, c.Hazards, requestedFeet, requestedAt, data.Nodes[target].CanWait,
            [connector], c.LastHitAt, true,
            immobile: hadBomb ? new(bombExpires - BombLead, bombExpires + BombTail) : null,
            arrowRate: c.Act.ArrowRate, entryArrowHeading: c.Act.ArrowEntryHeading);
        c.Log("course-section-prefetch", new { at = c.Now, section = data.Sections[sectionIndex + 1].Name,
            readyAt = requestedAt, remaining = requestedAt - c.Now, refresh = preparedSection != null });
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
