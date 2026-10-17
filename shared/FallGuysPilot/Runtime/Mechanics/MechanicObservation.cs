using System.Collections.Immutable;
using System.Numerics;

namespace MrBean.FallGuys.Mechanics;

// Owned receipt data. Native callbacks never publish pointers or actor wrappers.
// Seconds is local monotonic receipt time, not a server hit timestamp.
internal abstract record MechanicObservation(long Session, double Seconds);

internal sealed record MechanicActorObservation(long Session, double Seconds,
    uint EntityId, uint? BaseId, Vector3? Position, float? Rotation,
    string ActorEvent, long? SpawnOffset, float? Radius = null)
    : MechanicObservation(Session, Seconds);

internal readonly record struct MechanicEffectRow(byte Type, byte Param0, byte Param1,
    byte Param2, byte Param3, byte Param4, ushort Value);
internal readonly record struct MechanicEffectTarget(ulong ObjectId,
    ImmutableArray<MechanicEffectRow> Effects);

internal sealed record MechanicEffectObservation(long Session, double Seconds,
    uint EntityId, uint ActionId, uint Sequence, ulong AnimationTarget,
    uint? BaseId, Vector3? Origin, float Rotation, float? SourceRadius,
    Vector3? PacketTargetPosition, float AnimationLock,
    ImmutableArray<MechanicEffectTarget> Targets) : MechanicObservation(Session, Seconds);

// Position is the quantized cast packet location, not a substitute for caster origin.
internal sealed record MechanicCastObservation(long Session, double Seconds,
    uint EntityId, uint ActionId, ushort SpellId, byte ActionType, float Duration,
    uint TargetEntityId, Vector3 Position, float Rotation, byte OmenDelay,
    uint BallistaEntityId) : MechanicObservation(Session, Seconds);

internal sealed record MechanicStatusObservation(long Session, double Seconds,
    ulong ObjectId, ushort StatusId, ushort Param, ushort StackCount,
    double? Remaining, ulong Source, bool Gained, bool MessageCandidate = false)
    : MechanicObservation(Session, Seconds);

// This observes the native dispatcher, which can also be called by explicit
// offline replay. It is not automatically evidence of a server packet.
internal sealed record MechanicControlObservation(long Session, double Seconds,
    uint EntityId, uint Category, uint Arg1, uint Arg2, uint Arg3, uint Arg4,
    uint Arg5, uint Arg6, uint Arg7, uint Arg8, ulong TargetId, bool IsRecorded,
    byte PresentArguments = 255,Vector3? LocalPosition=null)
    : MechanicObservation(Session, Seconds);

internal sealed record MechanicMapObservation(long Session, double Seconds,
    uint EventId, ushort State, ushort TimelineIndex, byte Index)
    : MechanicObservation(Session, Seconds);

// Unmapped packet candidates remain raw evidence. No director/MapEffect
// semantics or missing native fields are synthesized for them.
internal sealed record MechanicOpaqueObservation(long Session, double Seconds,
    int Opcode, string ControlType, string RawJson)
    : MechanicObservation(Session, Seconds);
