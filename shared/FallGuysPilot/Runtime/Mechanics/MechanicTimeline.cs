using System;
using System.Collections.Generic;
using System.Numerics;

namespace MrBean.FallGuys.Mechanics;

internal readonly record struct MechanicSourceKey(string Course,uint EntityId);
internal readonly record struct MechanicActionKey(MechanicSourceKey Source,uint ActionId);
internal sealed class MechanicSourceState
{
    public uint? BaseId;
    public long? SpawnOffset;
    public Vector3? Position;
    public float? Rotation,Radius;
    public double PositionAt;
    public Vector3? PreviousPosition;
    public double PreviousPositionAt;
    public Vector3? Velocity=>Position is { } current&&PreviousPosition is { } previous&&PositionAt>PreviousPositionAt
        ?(current-previous)/(float)(PositionAt-PreviousPositionAt):null;
}
internal sealed class MechanicActionState(MechanicDefinition definition)
{
    public readonly MechanicDefinition Definition=definition;
    public MechanicCastObservation? PendingCast,LastCast;
    public Vector3? CastSourcePosition;
    public MechanicEffectObservation? LastEffect;
    public long Effects,Casts,PairedCasts;
    public double LastEffectAt;
    // A receipt interval is not an activation window or immunity between pulses.
    public readonly Queue<double> ReceiptIntervals=new();
    public readonly Queue<double> CastReceiptExcess=new();
    public float? Facing=>LastCast is { } cast&&Definition.TimingKind=='C'
        ?cast.Rotation:LastEffect?.Rotation;
    public Vector3? ChargeDestination=>PendingCast?.Position??LastEffect?.PacketTargetPosition??LastCast?.Position;
}

// A single owned history for live data and recorded packets. No native API here.
// Pose ages/packet-compatible authority are supplied by the caller, rather than
// treating a recorded message as a current native status snapshot.
internal sealed class MechanicTimeline(MechanicCatalog catalog)
{
    private long session;
    private readonly Dictionary<MechanicSourceKey,MechanicSourceState> sources=[];
    private readonly Dictionary<MechanicActionKey,MechanicActionState> actions=[];
    public IReadOnlyDictionary<MechanicSourceKey,MechanicSourceState> Sources=>sources;
    public IReadOnlyDictionary<MechanicActionKey,MechanicActionState> Actions=>actions;
    public long SourceRegistryRevision { get; private set; }
    public long RecognizedEffects { get; private set; }
    public long RecognizedCasts { get; private set; }
    public long PairedCasts { get; private set; }
    public long ReplacedPendingCasts { get; private set; }
    public long RetiredPendingCasts { get; private set; }
    public int PendingCastCount
    {
        get {int count=0;foreach(var action in actions.Values)if(action.PendingCast!=null)++count;return count;}
    }
    public void Clear()
    {sources.Clear();actions.Clear();session=0;++SourceRegistryRevision;}
    private void ChangeSession(long incoming)
    {
        if(session==incoming)return;
        RetiredPendingCasts+=PendingCastCount;
        sources.Clear();actions.Clear();session=incoming;++SourceRegistryRevision;
    }
    private MechanicSourceState Source(MechanicSourceKey key)
    {
        if(!sources.TryGetValue(key,out var result)){sources.Add(key,result=new());++SourceRegistryRevision;}
        return result;
    }
    private static void Pose(MechanicSourceState source,Vector3? position,float? rotation,double at)
    {
        if(position is { } point&&(source.Position==null||at>=source.PositionAt))
        {
            if(at>source.PositionAt&&source.Position!=null)
            {source.PreviousPosition=source.Position;source.PreviousPositionAt=source.PositionAt;}
            source.Position=point;source.PositionAt=at;
        }
        if(rotation is { } facing)source.Rotation=facing;
    }
    private MechanicActionState? Action(string course,uint entity,uint id)
    {
        if(!catalog.TryGet(course,id,out var definition))return null;
        var key=new MechanicActionKey(new(course,entity),id);
        if(!actions.TryGetValue(key,out var result))actions.Add(key,result=new(definition));
        return result;
    }
    public MechanicActionState Register(MechanicActionKey key)=>Action(key.Source.Course,key.Source.EntityId,key.ActionId)
        ??throw new InvalidOperationException("Cannot register an uncatalogued mechanic action.");
    public bool Apply(string course,MechanicObservation observation,Vector3? observedSourcePosition=null,
        float? observedSourceRotation=null,double? sourcePoseAt=null)
    {
        ChangeSession(observation.Session);
        switch(observation)
        {
            case MechanicActorObservation actor:
            {
                var key=new MechanicSourceKey(course,actor.EntityId);var source=Source(key);
                if(actor.SpawnOffset is { } spawn&&source.SpawnOffset is { } old&&old!=spawn)
                {
                    var remove=new List<MechanicActionKey>();
                    foreach(var action in actions)
                        if(action.Key.Source==key){if(action.Value.PendingCast!=null)++RetiredPendingCasts;remove.Add(action.Key);}
                    foreach(var id in remove)actions.Remove(id);
                    sources[key]=source=new();
                }
                source.SpawnOffset=actor.SpawnOffset??source.SpawnOffset;source.BaseId=actor.BaseId??source.BaseId;
                source.Radius=actor.Radius??source.Radius;Pose(source,actor.Position,actor.Rotation,actor.Seconds);return true;
            }
            case MechanicCastObservation cast:
            {
                var state=Action(course,cast.EntityId,cast.ActionId);if(state==null)return false;
                var source=Source(new(course,cast.EntityId));
                Pose(source,observedSourcePosition,observedSourceRotation,sourcePoseAt??cast.Seconds);
                if(state.PendingCast!=null)++ReplacedPendingCasts;
                state.CastSourcePosition=source.Position;
                state.PendingCast=state.LastCast=cast;++state.Casts;++RecognizedCasts;return true;
            }
            case MechanicEffectObservation effect:
            {
                var state=Action(course,effect.EntityId,effect.ActionId);if(state==null)return false;
                var source=Source(new(course,effect.EntityId));
                source.BaseId=effect.BaseId??source.BaseId;source.Radius=effect.SourceRadius??source.Radius;
                Pose(source,observedSourcePosition??effect.Origin,null,sourcePoseAt??effect.Seconds);
                if(state.LastEffect!=null)
                {
                    double interval=effect.Seconds-state.LastEffectAt;
                    if(interval>=0){state.ReceiptIntervals.Enqueue(interval);if(state.ReceiptIntervals.Count>32)state.ReceiptIntervals.Dequeue();}
                }
                if(state.PendingCast is { } pending&&effect.Seconds>=pending.Seconds)
                {
                    state.CastReceiptExcess.Enqueue(effect.Seconds-pending.Seconds-pending.Duration);
                    if(state.CastReceiptExcess.Count>16)state.CastReceiptExcess.Dequeue();
                    state.PendingCast=null;++state.PairedCasts;++PairedCasts;
                }
                state.LastEffect=effect;state.LastEffectAt=effect.Seconds;++state.Effects;++RecognizedEffects;
                // Do not replace C's cast-facing with the effect's helper baseline.
                if(state.Definition.TimingKind!='C')source.Rotation=effect.Rotation;
                return true;
            }
            default:return false;
        }
    }
}
