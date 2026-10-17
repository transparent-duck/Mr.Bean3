using System;
using System.Collections.Immutable;
using System.Numerics;
using MrBean.Host;
using DeepDungeon.Fsd.Dalamud.Utilities;

using OmenTools.Interop.Game.Models.Native;
using NativeCharacter = FFXIVClientStructs.FFXIV.Client.Game.Character.Character;

namespace MrBean.Modules.Blunderville;

// Presence only: several bits can coexist. These flags do not select the
// movement channel or infer which status wins in the native update order.
[Flags]
internal enum NativeMechanicStatusFlags : ushort
{
    None = 0,
    Status149 = 1 << 0,
    Status529 = 1 << 1,
    Status3693 = 1 << 2,
    Status3694 = 1 << 3,
    Status3699 = 1 << 4,
    Status3700 = 1 << 5,
    Status3701 = 1 << 6,
    Status3702 = 1 << 7,
    Status3703 = 1 << 8,
    Status3704 = 1 << 9,
    Status3737 = 1 << 10,
    Status3782 = 1 << 11,
    Status3802 = 1 << 12,
    Status1548 = 1 << 13,
    Status2056 = 1 << 14,
}

internal readonly record struct NativeMechanicStatus(
    ushort Id, ushort Param, float Remaining, ulong Source);

[Flags]
internal enum NativeMechanicReadIssues : ushort
{
    None = 0,
    StatusManagerMissing = 1 << 0,
    WalkControllerMissing = 1 << 1,
    WalkParentMissing = 1 << 2,
    WalkApiFailed = 1 << 3,
    TargetSpeedApiFailed = 1 << 4,
    ArrowReaderMissing = 1 << 5,
    ArrowApiFailed = 1 << 6,
    ArrowValueUnavailable = 1 << 7,
    FixedDirectionNonFinite = 1 << 8,
    TargetSpeedNonFinite = 1 << 9,
    TimelineOwnerMissing = 1 << 10,
    TimelineValuesNonFinite = 1 << 11,
    IceClockApiFailed = 1 << 12,
    IceSlotSpeedNonFinite = 1 << 13,
    IceSchedulerMissing = 1 << 14,
    IceSchedulerValuesNonFinite = 1 << 15,
}

// Native input/target fields, not proof of the final collision-resolved velocity.
// CachedInput is Walk+114/+118, written inside Original before OnWalk overrides
// its output pointers. It must not be treated as LastAppliedInput. RawMode3E is
// also used outside status-driven movement; it does not prove a forced channel.
internal readonly record struct NativeMechanicWalkMotion(
    Vector3 MovementDirection,
    Vector3 InputWorldDirection,
    Vector2 CachedInput,
    uint InputState,
    float Speed,
    float? TargetSpeed,
    float? AvailableTargetSpeed,
    bool HasInput,
    bool AutoForward,
    bool AutoStrafe,
    bool RawMode3E,
    byte MovementSubtype);

// Actual TimelineMove activity, separate from status or looping animation
// presence. RawPositionScale scales native displacement; it is not seconds,
// speed or a proven complete trajectory/time law. Pending can precede Active.
internal readonly record struct NativeTimelineMove(
    bool Active,
    bool Pending,
    ushort SourceTimelineId,
    Vector3 Origin,
    Vector3 Target,
    float RawPositionScale,
    bool PositionUpdate,
    bool RotationUpdate);

// Row3788 belongs to slot0. ProspectiveSlotSpeed is the current slot speed that
// native action creation copies as its initial multiplier. Matched scheduler
// time/speed are current observations, not proof that its clock is advancing:
// native pause conditions can include ancestors. No pointers leave this DTO.
internal readonly record struct NativeIceClock(
    float? ProspectiveSlotSpeed,
    ushort? CurrentSlotTimelineId,
    float? CurrentTimestamp,
    float? ActualSchedulerSpeed);

// Slot0 8514 C125 locks movement for ticks [0,65). C131 permits movement
// cancellation at tick65; the full animation duration143 is not the lock.
internal readonly record struct NativeSteeringClock(float? ProspectiveSlotSpeed,
    ushort CurrentTimelineId,float? CurrentTimestamp,float? ActualSchedulerSpeed);

/// <summary>
/// Owned scalar snapshot. Read on the Framework thread while the local actor is
/// valid; the snapshot can then cross a frame or worker boundary without pointers.
/// Null angles/Walk and StatusesAvailable=false are explicit missing observations.
/// </summary>
internal readonly record struct NativeMechanicMotion(
    ulong ActorId,
    Vector3 Position,
    float Facing,
    ImmutableArray<NativeMechanicStatus> Statuses,
    NativeMechanicStatusFlags StatusFlags,
    bool StatusesAvailable,
    NativeMechanicReadIssues ReadIssues,
    NativeMechanicWalkMotion? Walk,
    float? ArrowDirection,
    float? FixedDirection,
    float? StatusFrameSeconds = null,
    NativeTimelineMove? TimelineMove = null,
    NativeIceClock? IceClock = null, Vector3? ExternalVelocity = null,NativeSteeringClock? SteeringClock=null,
    uint? NativeFrame=null,double? NativeFrameStartedAt=null,float? RealFrameSeconds=null)
{
    // ActorControl244 stores XYZ at+420 and enables bit0 at+430. The current
    // disk client (checked 2026-10-14) adds force displacement at 14181AE4C,
    // after the ordinary CurrentSpeed-based displacement. Earlier captures
    // used 14181998C for that same operation; addresses are build-specific.
    // This is a copied native force, not a residual inferred from replay poses.
    internal static unsafe Vector3? ReadExternalVelocity(NativeCharacter* character)
    {
        byte* controller=(byte*)&character->MoveController;
        if((controller[0x430]&1)==0)return Vector3.Zero;
        var velocity=*(Vector3*)(controller+0x420);
        return float.IsFinite(velocity.X)&&float.IsFinite(velocity.Y)&&float.IsFinite(velocity.Z)?velocity:null;
    }

    public bool HasStatus(ushort id) => (StatusFlags & FlagFor(id)) != 0;

    public static unsafe bool TryRead(DirectMovementController? movement, out NativeMechanicMotion motion)
    {
        motion = default;
        if (Service.LocalPlayer is not { } player || player.Address == 0) return false;

        var character = (NativeCharacter*)player.Address;
        var statusManager = character->GetStatusManager();
        NativeMechanicStatusFlags flags = NativeMechanicStatusFlags.None;
        NativeMechanicReadIssues issues = NativeMechanicReadIssues.None;
        Span<NativeMechanicStatus> copied = stackalloc NativeMechanicStatus[60];
        int count = 0;
        float? fixedDirection = null;
        if (statusManager != null)
        {
            // Scan the fixed native slots; NumValidStatuses is not a slot bound.
            // Keep duplicate sources and zero/negative remaining time as observed.
            var slots = statusManager->Status;
            for (int i = 0; i < slots.Length; ++i)
            {
                var status = slots[i];
                var flag = FlagFor(status.StatusId);
                if (flag == NativeMechanicStatusFlags.None) continue;
                flags |= flag;
                copied[count++] = new(status.StatusId, status.Param, status.RemainingTime, status.SourceObject);
            }
            if ((flags & NativeMechanicStatusFlags.Status3737) != 0)
            {
                float angle = statusManager->SpecialStatusTimerOrDirection;
                if (float.IsFinite(angle)) fixedDirection = angle;
                else issues |= NativeMechanicReadIssues.FixedDirectionNonFinite;
            }
        }
        else issues |= NativeMechanicReadIssues.StatusManagerMissing;

        // Current native14181E9B0/141806810 sets up this owned subcontroller;
        // 141806EA0 activates it and141806C00 resets it. Read unconditionally:
        // a3693 loss can precede the actual motion ending. Native pointers stay
        // inside the reader, including the subcontroller's owner(+10).
        NativeTimelineMove? timelineMove = null;
        byte* timeline = (byte*)&character->MoveController + 0x250;
        if (*(void**)(timeline + 0x10) == null)
            issues |= NativeMechanicReadIssues.TimelineOwnerMissing;
        else
        {
            var origin = *(Vector3*)(timeline + 0x30);
            var target = *(Vector3*)(timeline + 0x20);
            float positionScale = *(float*)(timeline + 0x60);
            timelineMove = new(timeline[0x76] != 0, timeline[0x7B] != 0,
                *(ushort*)(timeline + 0x70), origin, target, positionScale,
                timeline[0x74] != 0, timeline[0x75] != 0);
            if (!float.IsFinite(origin.X) || !float.IsFinite(origin.Y) || !float.IsFinite(origin.Z)
                || !float.IsFinite(target.X) || !float.IsFinite(target.Y) || !float.IsFinite(target.Z)
                || !float.IsFinite(positionScale))
                issues |= NativeMechanicReadIssues.TimelineValuesNonFinite;
        }

        NativeIceClock? iceClock = null;
        if ((flags & NativeMechanicStatusFlags.Status3693) != 0
            || timelineMove is { Active: true } or { Pending: true })
        {
            float? prospectiveSlotSpeed = null;
            ushort? currentSlotTimelineId = null;
            float? currentTimestamp = null;
            float? actualSchedulerSpeed = null;
            try
            {
                float speed = character->Timeline.TimelineSequencer.GetSlotSpeed(0);
                if (float.IsFinite(speed)) prospectiveSlotSpeed = speed;
                else issues |= NativeMechanicReadIssues.IceSlotSpeedNonFinite;
            }
            catch { issues |= NativeMechanicReadIssues.IceClockApiFailed; }
            try
            {
                currentSlotTimelineId = character->Timeline.TimelineSequencer.GetSlotTimeline(0);
                if (timelineMove is { SourceTimelineId: 3788 } && currentSlotTimelineId == 3788)
                {
                    var scheduler = character->Timeline.TimelineSequencer.GetSchedulerTimeline(0);
                    if (scheduler == null) issues |= NativeMechanicReadIssues.IceSchedulerMissing;
                    else
                    {
                        float timestamp = scheduler->TimelineController.CurrentTimestamp;
                        // Native141BFBCC4 consumes actual SchedulerTimeline+240.
                        //141990910 resolves the same public slot double pointer;
                        //141BFCDE4 writes that scalar. Slot speed is used as-is:
                        // OverallSpeed has already been applied by native code.
                        float speed = *(float*)((byte*)scheduler + 0x240);
                        if (float.IsFinite(timestamp)) currentTimestamp = timestamp;
                        else issues |= NativeMechanicReadIssues.IceSchedulerValuesNonFinite;
                        if (float.IsFinite(speed)) actualSchedulerSpeed = speed;
                        else issues |= NativeMechanicReadIssues.IceSchedulerValuesNonFinite;
                    }
                }
            }
            catch { issues |= NativeMechanicReadIssues.IceClockApiFailed; }
            iceClock = new(prospectiveSlotSpeed, currentSlotTimelineId, currentTimestamp, actualSchedulerSpeed);
        }

        NativeSteeringClock? steeringClock=null;
        try
        {
            ushort id=character->Timeline.TimelineSequencer.GetSlotTimeline(0);
            if(character!=null)
            {
                float prospective=character->Timeline.TimelineSequencer.GetSlotSpeed(0);
                float? timestamp=null,actual=null;
                if(id==8514&&character->Timeline.TimelineSequencer.GetSchedulerTimeline(0) is var scheduler&&scheduler!=null)
                {
                    float ticks=scheduler->TimelineController.CurrentTimestamp;
                    float rate=*(float*)((byte*)scheduler+0x240);
                    if(float.IsFinite(ticks))timestamp=ticks;
                    if(float.IsFinite(rate))actual=rate;
                }
                steeringClock=new(float.IsFinite(prospective)?prospective:null,id,timestamp,actual);
            }
        }
        catch { /* Missing clock remains explicit; the movement model cannot resolve it. */ }

        NativeMechanicWalkMotion? walkMotion = null;
        try
        {
            var walk = PlayerMoveControllerWalk.Instance();
            if (walk == null) issues |= NativeMechanicReadIssues.WalkControllerMissing;
            else if (walk->Parent == null) issues |= NativeMechanicReadIssues.WalkParentMissing;
            else
            {
                float? targetSpeed = null,availableTargetSpeed=null;
                try
                {
                    targetSpeed = walk->GetTargetSpeed();
                    // Evaluate the requested ordinary run on stack copies. The
                    // live getter includes idle input and speed-blend state,
                    // so setting only Walk+3C still returns idle speed zero.
                    // Current getter: +1E4/+1D8 use -1 sentinels; +1F4 modes
                    // 1..4 select ordinary base speed and the native status
                    // multiplier. Keep actual lock, walk/run and status inputs.
                    // No live controller field is changed by this probe.
                    var parentProbe=*walk->Parent;
                    parentProbe.WalkSpeedBlendDuration=-1;
                    parentProbe.CachedWalkSpeed=-1;
                    parentProbe.MovementModeFlags=1;
                    var inputProbe=*walk;inputProbe.Parent=&parentProbe;inputProbe.IsAutoRunForward=true;
                    availableTargetSpeed=inputProbe.GetTargetSpeed();
                    if (!float.IsFinite(targetSpeed.Value)) issues |= NativeMechanicReadIssues.TargetSpeedNonFinite;
                    if(!float.IsFinite(availableTargetSpeed.Value))availableTargetSpeed=null;
                }
                catch { issues |= NativeMechanicReadIssues.TargetSpeedApiFailed; }
                // Current CN 2026.09.15: 14182B959 reads BYTE params+30;
                // 14182B95D writes BYTE Parent+1FC; 14182C78B compares BYTE with 9.
                byte subtype = *((byte*)walk->Parent + 0x1FC);
                walkMotion = new(walk->MovementDirection, walk->MovementDirectionWorld,
                    new(walk->MovementLeft, walk->MovementForward), walk->MovementState,
                    walk->CurrentSpeed, targetSpeed,availableTargetSpeed, walk->IsMovementInput,
                    walk->IsAutoRunForward, walk->IsAutoRunStrafe,
                    walk->IsCameraRelativeMode, subtype);
            }
        }
        catch { issues |= NativeMechanicReadIssues.WalkApiFailed; }

        float? arrowDirection = null;
        if ((flags & (NativeMechanicStatusFlags.Status3694 | NativeMechanicStatusFlags.Status3702)) != 0)
        {
            if (movement == null) issues |= NativeMechanicReadIssues.ArrowReaderMissing;
            else try
            {
                if (movement.TryReadForcedDirection(out float angle)) arrowDirection = angle;
                else issues |= NativeMechanicReadIssues.ArrowValueUnavailable;
            }
            catch { issues |= NativeMechanicReadIssues.ArrowApiFailed; }
        }

        // No heap allocation for the ordinary no-status frame; otherwise one
        // exact-length immutable array owns the copied matching status entries.
        var statuses = count == 0 ? ImmutableArray<NativeMechanicStatus>.Empty
            : ImmutableArray.Create(copied[..count]);
        float? statusFrameSeconds=null;
        var framework=FFXIVClientStructs.FFXIV.Client.System.Framework.Framework.Instance();
        if(statusManager!=null&&framework!=null)
        {
            // Native14089BA70: Owner indices200..448 use unscaled frame time.
            // Other indices and null Owner multiply by GameSpeedMultiplier.
            var owner=statusManager->Owner;
            bool unscaled=owner!=null&&owner->ObjectIndex is >=200 and <=448;
            float delta=framework->FrameDeltaTime*(unscaled?1:framework->GameSpeedMultiplier);
            if(float.IsFinite(delta)&&delta>=0)statusFrameSeconds=delta;
        }
        motion = new(player.GameObjectId, player.Position, player.Rotation, statuses,
            flags, statusManager != null, issues, walkMotion, arrowDirection, fixedDirection,statusFrameSeconds,timelineMove,iceClock,
              ReadExternalVelocity(character),steeringClock,framework!=null?framework->FrameCounter:null,
              framework!=null&&framework->PerformanceCounterFrequency==System.Diagnostics.Stopwatch.Frequency?
                  (double)framework->PerformanceCounterValue/framework->PerformanceCounterFrequency:null,
              framework!=null?framework->RealFrameDeltaTime:null);
        return true;
    }

    private static NativeMechanicStatusFlags FlagFor(ushort id) => id switch
    {
        149 => NativeMechanicStatusFlags.Status149,
        529 => NativeMechanicStatusFlags.Status529,
        1548 => NativeMechanicStatusFlags.Status1548,
        2056 => NativeMechanicStatusFlags.Status2056,
        3693 => NativeMechanicStatusFlags.Status3693,
        3694 => NativeMechanicStatusFlags.Status3694,
        3699 => NativeMechanicStatusFlags.Status3699,
        3700 => NativeMechanicStatusFlags.Status3700,
        3701 => NativeMechanicStatusFlags.Status3701,
        3702 => NativeMechanicStatusFlags.Status3702,
        3703 => NativeMechanicStatusFlags.Status3703,
        3704 => NativeMechanicStatusFlags.Status3704,
        3737 => NativeMechanicStatusFlags.Status3737,
        3782 => NativeMechanicStatusFlags.Status3782,
        3802 => NativeMechanicStatusFlags.Status3802,
        _ => NativeMechanicStatusFlags.None,
    };
}
