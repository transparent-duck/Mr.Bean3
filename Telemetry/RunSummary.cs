using MrBean.FallGuys;
using MrBean.Modules.FallGuysPilot;

namespace Mr.Bean3;

internal sealed record WorldIdentity(uint DataCenterId, string DataCenter, uint WorldId, string World);
internal sealed record CharacterIdentity(string Cid, string Name, WorldIdentity Home, WorldIdentity Current);
internal sealed record StageSummary(int Stage, string Course, string Outcome, long DurationMs, uint? Rank,
    int Hits, long Jumps, int Nudges, bool ManualIntervention);
internal sealed record RunSummary(int SchemaVersion, string RunId, int Revision, string ClientVersion,
    string MapRevision, CharacterIdentity Character, DateTimeOffset StartedAt, DateTimeOffset? EndedAt,
    string Outcome, long DurationMs, bool PartialDuty, bool ManualIntervention, StageSummary[] Stages);

// One row per duty. Scene changes caused by checkpoints accumulate into the same stage.
internal sealed class RunSummaryBuilder
{
    private readonly RunSummary initial;
    private readonly Dictionary<int, StageState> stages = [];
    private bool manual;
    public bool Finished { get; private set; }
    public RunSummary Initial => initial;

    public RunSummaryBuilder(CharacterIdentity character, string clientVersion, string mapRevision,
        bool partialDuty, DateTimeOffset now)
    {
        initial = new(1, $"{now:yyyyMMdd}-{Guid.NewGuid():N}", 0, clientVersion, mapRevision,
            character, now, null, "running", 0, partialDuty, false, []);
    }

    public string? Observe(PilotObservation observation)
    {
        if (Finished || observation.Course is not { } course) return null;
        int stage = course switch { CourseId.Hit or CourseId.Knight => 1, CourseId.Ring or CourseId.Crystal => 2, CourseId.Crown => 3, _ => 0 };
        if (stage == 0) return null;
        if (!stages.TryGetValue(stage, out var state)) stages[stage] = state = new(stage, course.ToString());
        state.Observe(observation);
        manual |= observation.ManualIntervention || observation.Kind == "yield";
        return observation.Kind switch { "winner" => "won", "eliminated" => "eliminated", "yield" => "cancelled", _ => null };
    }

    public RunSummary Finish(string outcome, DateTimeOffset now)
    {
        if (Finished) throw new InvalidOperationException("Run summary was already finalized.");
        Finished = true;
        return initial with
        {
            Revision = 1,
            EndedAt = now < initial.StartedAt ? initial.StartedAt : now,
            Outcome = outcome,
            DurationMs = Math.Max(0, (long)(now - initial.StartedAt).TotalMilliseconds),
            ManualIntervention = manual,
            Stages = stages.OrderBy(s => s.Key).Select(s => s.Value.Snapshot()).ToArray(),
        };
    }

    private sealed class StageState(int stage, string course)
    {
        private long scene;
        private int baseHits, baseNudges;
        private long baseJumps, baseDuration;
        private StageSummary current = new(stage, course, "running", 0, null, 0, 0, 0, false);
        public void Observe(PilotObservation value)
        {
            if (current.Outcome is "qualified" or "won" or "eliminated") return;
            if (scene != 0 && scene != value.SceneSequence)
            {
                baseHits = current.Hits; baseJumps = current.Jumps;
                baseNudges = current.Nudges; baseDuration = current.DurationMs;
            }
            scene = value.SceneSequence;
            string outcome = value.Kind switch
            {
                "winner" => "won", "qualified" => "qualified", "eliminated" => "eliminated",
                "yield" => "cancelled", _ => current.Outcome,
            };
            current = current with
            {
                Outcome = outcome,
                DurationMs = baseDuration + Math.Max(0, (long)(value.ElapsedSeconds * 1000)),
                Rank = value.Rank,
                Hits = baseHits + value.Hits, Jumps = baseJumps + value.Jumps, Nudges = baseNudges + value.Nudges,
                ManualIntervention = current.ManualIntervention || value.ManualIntervention || value.Kind == "yield",
            };
        }
        public StageSummary Snapshot() => current.Outcome == "running" ? current with { Outcome = "unknown" } : current;
    }
}
