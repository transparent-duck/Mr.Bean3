using MrBean.FallGuys.Mechanics;
using System;
using System.Collections.Concurrent;
using System.Collections.Immutable;
using System.Diagnostics;
using System.Numerics;
using System.Threading;
using MrBean.Host;
using MrBean.Host.Modules;
using Dalamud.Hooking;
using FFXIVClientStructs.FFXIV.Client.Game.Character;
using FFXIVClientStructs.FFXIV.Client.Game.Network;
using FFXIVClientStructs.FFXIV.Client.Game.Object;
using FFXIVClientStructs.FFXIV.Client.Network;
using OmenTools.OmenService;
using OmenTools.Interop.Game.Models;
using NativeCharacter = FFXIVClientStructs.FFXIV.Client.Game.Character.Character;
using StatusActor = OmenTools.Dalamud.Services.Game.Object.Abstractions.ObjectKinds.IBattleChara;

namespace MrBean.Modules.Blunderville;

internal enum MechanicObserverState { Inactive, Observing, Unavailable, Disposed }

// Framework owns lifecycle/draining. Native receive callbacks only copy values.
// Uses the existing global ActionTracker publisher and status service. The three
// Cast/control use FFCS dispatchers. The installed CN MapEffect entry aliases
// the replay dispatcher; the actual director packet handler is not exposed.
internal sealed unsafe class PacketObservations(ActionTracker tracker) : IDisposable
{
    private readonly ConcurrentQueue<MechanicObservation> pending = new();
    private Hook<PacketDispatcher.Delegates.HandleActorCastPacket>? cast;
    private Hook<PacketDispatcher.Delegates.HandleActorControlPacket>? control;
    private delegate void ProcessMapEffect(void* framework, MapEffectPacket* packet);
    private Hook<ProcessMapEffect>? map;
    private CharacterStatusManager? statuses;
    private long nextSession, captureSession, faults, dropped;
    private int pendingCount;
    private MechanicControlObservation? localTransport;
    public MechanicControlObservation? LocalTransport=>Volatile.Read(ref localTransport);
    public MechanicObserverState State { get; private set; }
    public string? LastError { get; private set; }
    public long Session => Interlocked.Read(ref captureSession);
    public long Faults => Interlocked.Read(ref faults);
    public long Dropped => Interlocked.Read(ref dropped);
    public bool HasPending => Volatile.Read(ref pendingCount) > 0;

    public static bool IsMechanicAction(uint id) => id is 29795 or 34716 or 34717 or 34718
        or 34773 or 34774 or 34775 or 34776 or 34796 or 34797 or 34798 or 34799
        or 34800 or 34801 or 34802 or 34803 or 34804 or 34805 or 34808 or 34812 or 35409;
    public static bool IsMechanicStatus(ushort id) => id is 149 or 529 or 1548 or 2056 or 3693 or 3694
        or 3699 or 3700 or 3701 or 3702 or 3703 or 3704 or 3737 or 3782 or 3802;
    private static double Now => (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
    private static float Rotation(ushort value) => value * (MathF.Tau / 65535) - MathF.PI;
    private static float Coordinate(ushort value) => value * (2000f / 65535) - 1000;

    public void Start()
    {
        ObjectDisposedException.ThrowIf(State == MechanicObserverState.Disposed, this);
        if (State == MechanicObserverState.Observing) return;
        Stop();
        try
        {
            cast ??= Service.GameInteropProvider.HookFromAddress<PacketDispatcher.Delegates.HandleActorCastPacket>(
                (nint)PacketDispatcher.MemberFunctionPointers.HandleActorCastPacket, OnCast);
            control ??= Service.GameInteropProvider.HookFromAddress<PacketDispatcher.Delegates.HandleActorControlPacket>(
                (nint)PacketDispatcher.MemberFunctionPointers.HandleActorControlPacket, OnControl);
            // CN build 14101351: opcode422 -> B4D450 -> B82D60. This function
            // reads EventId at0, index at8, state at4 and timeline at6. Both
            // live and replay packet dispatch reach it. Unique in this build.
            map ??= new CompSig("48 89 5C 24 ?? 57 48 83 EC ?? 48 8B F9 48 8B DA 48 8B 89 58 01 00 00 48 85 C9 74 ?? 48 8B 01 FF 50 18 84 C0")
                .GetHook<ProcessMapEffect>(OnMap);
            statuses = CharacterStatusManager.Instance();
            tracker.RawActionEffectReceiving += OnEffect;
            statuses.RegGain(OnStatusGain);
            statuses.RegLose(OnStatusLoss);
            Interlocked.Exchange(ref captureSession, ++nextSession);
            cast.Enable(); control.Enable(); map.Enable();
            LastError = null; State = MechanicObserverState.Observing;
        }
        catch (Exception ex)
        {
            Stop(); LastError = ex.Message; State = MechanicObserverState.Unavailable;
            throw new InvalidOperationException("Fall Guys mechanism observation could not start.", ex);
        }
    }

    public void Stop()
    {
        if (State == MechanicObserverState.Disposed) return;
        Interlocked.Exchange(ref captureSession, 0);
        Volatile.Write(ref localTransport,null);
        tracker.RawActionEffectReceiving -= OnEffect;
        statuses?.Unreg((CharacterStatusManager.GainStatusDelegate)OnStatusGain);
        statuses?.Unreg((CharacterStatusManager.LoseStatusDelegate)OnStatusLoss);
        cast?.Disable(); control?.Disable(); map?.Disable();
        while (pending.TryDequeue(out _)) Interlocked.Decrement(ref pendingCount);
        State = MechanicObserverState.Inactive;
    }

    public bool TryRead(out MechanicObservation observation)
    {
        while (pending.TryDequeue(out var next))
        {
            Interlocked.Decrement(ref pendingCount);
            if (next.Session == Session) { observation = next; return true; }
        }
        observation = null!; return false;
    }

    private void Publish(MechanicObservation observation)
    {
        // Resource bound only. Loss is observable; the forecaster must invalidate
        // its event history when Dropped changes instead of assuming no hazards.
        if (Interlocked.Increment(ref pendingCount) > 8192)
        { Interlocked.Decrement(ref pendingCount); Interlocked.Increment(ref dropped); return; }
        pending.Enqueue(observation);
    }

    private void OnEffect(uint entityId, NativeCharacter* actor, Vector3* targetPosition,
        ActionEffectHandler.Header* header, ActionEffectHandler.TargetEffects* effects, GameObjectId* targetIds)
    {
        long session = Session;
        if (session == 0 || header == null) return;
        try
        {
            int count = header->NumTargets;
            // Keep local consequences even when their action is not yet in
            // the compiled mechanic catalog. Known-source filtering alone
            // cannot explain a newly observed stun from an unknown action.
            if(!IsMechanicAction(header->ActionId))
            {
                ulong local=Service.LocalPlayer?.GameObjectId??0;
                bool targetsLocal=false;
                for(int i=0;i<count&&!targetsLocal;++i)targetsLocal=local!=0&&targetIds[i].Id==local;
                if(!targetsLocal)return;
            }
            var targets = ImmutableArray.CreateBuilder<MechanicEffectTarget>(count);
            for (int i = 0; i < count; ++i)
            {
                var rows = ImmutableArray.CreateBuilder<MechanicEffectRow>(8);
                for (int j = 0; j < 8; ++j)
                {
                    var e = effects[i].Effects[j];
                    rows.Add(new(e.Type, e.Param0, e.Param1, e.Param2, e.Param3, e.Param4, e.Value));
                }
                targets.Add(new(targetIds[i].Id, rows.MoveToImmutable()));
            }
            Publish(new MechanicEffectObservation(session, Now, entityId, header->ActionId,
                header->GlobalSequence, header->AnimationTargetId.Id, actor == null ? null : actor->BaseId,
                actor == null ? null : actor->Position, Rotation(header->RotationInt),
                actor == null ? null : actor->GetRadius(),
                targetPosition == null ? null : *targetPosition, header->AnimationLock, targets.MoveToImmutable()));
        }
        catch { Interlocked.Increment(ref faults); }
    }

    private void OnCast(uint entityId, ActorCastPacket* packet)
    {
        try
        {
            long session = Session;
            if (session != 0 && packet != null && IsMechanicAction(packet->ActionId))
            {
                var p = packet->PositionInt;
                Publish(new MechanicCastObservation(session, Now, entityId, packet->ActionId, packet->SpellId,
                    packet->ActionType, packet->CastTime, packet->TargetEntityId,
                    new(Coordinate(p[0]), Coordinate(p[1]), Coordinate(p[2])), Rotation(packet->RotationInt),
                    packet->OmenDelay, packet->BallistaEntityId));
            }
        }
        catch { Interlocked.Increment(ref faults); }
        finally { cast!.Original(entityId, packet); }
    }

    private void OnControl(uint entityId, uint category, uint a1, uint a2, uint a3, uint a4,
        uint a5, uint a6, uint a7, uint a8, GameObjectId target, bool isRecorded)
    {
        try
        {
            long session = Session;
            if (session != 0)
            {
                // Copy the local origin before Original installs profile3.
                // Framework consumption can occur after the first native step.
                Vector3? origin=category==220&&Service.LocalPlayer is { } actor&&actor.EntityId==entityId?actor.Position:null;
                var observation=new MechanicControlObservation(session, Now, entityId, category,
                    a1,a2,a3,a4,a5,a6,a7,a8,target.Id,isRecorded,LocalPosition:origin);
                if(origin!=null)Volatile.Write(ref localTransport,observation);
                Publish(observation);
            }
        }
        catch { Interlocked.Increment(ref faults); }
        finally { control!.Original(entityId, category, a1, a2, a3, a4, a5, a6, a7, a8, target, isRecorded); }
    }

    private void OnMap(void* framework, MapEffectPacket* packet)
    {
        try
        {
            long session = Session;
            if (session != 0 && packet != null) Publish(new MechanicMapObservation(session, Now,
                packet->EventId, packet->State, packet->TimelineIndex, packet->Index));
        }
        catch { Interlocked.Increment(ref faults); }
        finally { map!.Original(framework, packet); }
    }

    private void OnStatusGain(StatusActor actor, ushort id, ushort param, ushort stack, TimeSpan remaining, ulong source)
        => OnStatus(actor, id, param, stack, remaining.TotalSeconds, source, true);
    private void OnStatusLoss(StatusActor actor, ushort id, ushort param, ushort stack, ulong source)
        => OnStatus(actor, id, param, stack, null, source, false);
    private void OnStatus(StatusActor actor, ushort id, ushort param, ushort stack, double? remaining, ulong source, bool gained)
    {
        long session = Session;
        if (session == 0 || !IsMechanicStatus(id)) return;
        try { Publish(new MechanicStatusObservation(session, Now, actor.GameObjectID, id, param, stack, remaining, source, gained)); }
        catch { Interlocked.Increment(ref faults); }
    }

    public void Dispose()
    {
        if (State == MechanicObserverState.Disposed) return;
        Stop(); cast?.Dispose(); control?.Dispose(); map?.Dispose();
        State = MechanicObserverState.Disposed;
    }
}
