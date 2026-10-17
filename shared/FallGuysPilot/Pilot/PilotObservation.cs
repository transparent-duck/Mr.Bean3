using System;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot;

/// <summary>Local lifecycle values only. A host owns any persistence or transport.</summary>
public readonly record struct PilotObservation(
    string Kind, long SceneSequence, CourseId? Course, DateTimeOffset OccurredAt,
    double ElapsedSeconds, bool Released, bool Qualified, bool Eliminated,
    bool ManualIntervention, uint? Rank, int Hits, long Jumps, int Nudges);
