using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Numerics;

namespace MrBean.FallGuys.Mechanics;

internal enum MechanicForecastCertainty { Candidate,Unknown }
internal enum MechanicDangerPolicy
{
    PendingCast,ReceiptInstant,CurrentMovingSource,ConservativePulseEnvelope,
    ContinuousWind,ParentCastSequence,UnknownContinuation,FixedOriginRotationEnvelope,SourceLocalBurstWindows,
    UnresolvedPendulum,HitPhaseWindow,HitPhaseUninitialized,
    CastCycleWindow,CastCycleUninitialized,UninitializedSourceEnvelope,ReceiptCycleWindow,CrownCycleWindow,RecordedCycleWindow
}
internal enum MechanicDangerEnd
{
    EffectOrExplicitCancellation,ReceiptInstant,NextObservationOrExplicitEnd,
    EndpointOrNextAnnouncement,FourChildReceiptsOrExplicitEnd,CallerHorizon
}
internal enum MechanicForecastIssue
{
    InvalidHorizon,MissingSource,MissingDirection,MissingDimensions,MissingEndpoint,
    MissingTiming,MissingMotion,MissingAngularVelocity,MissingParentCast,
    PendingCastPastDeclaredEnd,UncalibratedActivationWindow,FutureLegUnknown,
    ChildSequenceIncomplete,ChildSequenceOverrun,CastFacingUnavailable,
    OutOfOrderObservation,InvalidObservation,UnknownTimingKind,NoSpatialShape
}

// Receipt predictions are descriptive, source-local fits. Cycle families hold
// intervals; cast families hold excess after declared cast duration; child
// families hold four delays from the ACTUAL parent cast. Neither bound is a
// server hit instant, and no interval between these bounds is certified safe.
internal sealed record MechanicReceiptPrediction(string Family,int IntervalsPerCycle,
    int? NextPhase,ImmutableArray<double> PhaseMedians,ImmutableArray<double> PhaseMinimums,
    ImmutableArray<double> PhaseMaximums,double? NextReceiptMinimum,double? NextReceiptMaximum,
    int Samples,int? BurstPulseCount=null,int? ObservedPulseOrdinal=null,
    double? ObservedOffGap=null);

internal sealed record MechanicForecastDiagnostic(MechanicActionKey? Action,
    MechanicForecastIssue Issue,string Detail);
internal sealed record MechanicLifecycleDiagnostic(double? Seconds,MechanicSourceKey? Source,
    uint? ActionId,string Reason,int UnresolvedCasts,int UnresolvedChildSequences);

// Overall uncertainty can coexist with a known conservative spatial envelope.
// Timing/speed diagnostics do not erase a pulse shape or complete charge leg;
// Hazard.Footprint is null when a bounded spatial query itself is unresolved.
internal sealed record MechanicDanger(TemporalHazard Hazard,MechanicForecastCertainty Certainty,
    MechanicDangerPolicy Policy,MechanicDangerEnd End,MechanicFootprint? CandidateFootprint,
    double? PoseObservedAt,double? FacingObservedAt,MechanicReceiptPrediction? Receipts,
    double? ParentCastAt=null,int? ObservedChildPulses=null);
internal sealed record MechanicForecastSignal(MechanicActionKey Action,string ReceiptRole,char TimingKind,
    long Effects,long Casts,double? LastEffectAt,double? PendingCastAt,
    MechanicReceiptPrediction? Receipts);
internal sealed record MechanicDangerSnapshot(long Session,double Seconds,double HorizonEnd,
    ImmutableArray<TemporalHazard> Hazards,ImmutableArray<MechanicDanger> Dangers,ImmutableArray<MechanicForecastSignal> Signals,
    ImmutableArray<MechanicForecastDiagnostic> Unresolved,
    ImmutableArray<MechanicLifecycleDiagnostic> Lifecycle)
{
    public double PlanningBuffer { get; init; }
    public ImmutableArray<TemporalSupport> Supports {get;init;}=[];

    // Apply once at search/repair entry. Live execution still checks the raw
    // windows, so the reserved time absorbs small observation/arrival changes.
    public MechanicDangerSnapshot ForPlanning()
    {
        if(PlanningBuffer>0)return this;
        var known=this;
        var padded=ImmutableArray.CreateBuilder<TemporalHazard>(known.Hazards.Length);
        foreach(var hazard in known.Hazards)
            padded.Add(hazard with{ActiveStart=hazard.ActiveStart-.1,ActiveEnd=hazard.ActiveEnd+.1});
        var supports=ImmutableArray.CreateBuilder<TemporalSupport>(known.Supports.Length);
        foreach(var support in known.Supports)supports.Add(support.Reserve(.1));
        return known with{Hazards=padded.MoveToImmutable(),Supports=supports.MoveToImmutable(),PlanningBuffer=.1};
    }

}

// Pure owned state. Callers provide current scalar pose and explicit lifecycle
// authority. No packet timeout, old representative profile, action-wide median,
// native pointer, movement write, collision padding or launch tolerance lives here.
internal sealed class MechanicForecaster
{
    private const int MaximumReceiptHistory=64,MaximumLifecycleHistory=64;
    private sealed class History
    {
        public MechanicCastObservation? PendingCast;
        public MechanicEffectObservation? Effect;
        public long Effects,Casts;
        public readonly List<double> Intervals=[];
        public long IntervalCount;
        public MechanicReceiptPrediction? Fit;
        public double? ParentCastAt;
        public int ChildPulses;
        public bool ChildComplete;
        public bool ChargeComplete;
        public double? FacingAt;
        public readonly List<double>[] ChildDelays=[[],[],[],[]];
        public readonly Direction ReceiptDirection=new();
        public MechanicPhaseCycle? PhaseCycle;
        public KnightRecordedCycle? RecordedCycle;
        public MechanicSourceBounds? SourceBounds;
        public MechanicFootprint? InitialEnvelope;
        public bool SourceBoundsExceeded;
    }
    private sealed class Direction
    {
        public float? Heading,Velocity;
        public double At;
        public void Observe(float heading,double at)
        {
            if(!float.IsFinite(heading)||!double.IsFinite(at)||Heading!=null&&at<At)return;
            if(Heading is { } previous&&at>At)
            {
                double delta=Math.IEEERemainder((double)heading-previous,2*Math.PI);
                double velocity=delta/(at-At);
                Velocity=double.IsFinite(velocity)&&Math.Abs(velocity)<=float.MaxValue?(float)velocity:null;
            }
            Heading=heading;At=at;
        }
    }
    private readonly MechanicCatalog catalog;
    private readonly Dictionary<MechanicActionKey,History> histories=[];
    private readonly Dictionary<MechanicSourceKey,List<(MechanicActionKey Key,History Value)>> sourceHistories=[];
    private readonly Dictionary<MechanicSourceKey,Direction> directions=[];
    private readonly HashSet<MechanicSourceKey> retiredSources=[];
    private readonly HashSet<MechanicActionKey> endedActions=[];
    private readonly Queue<MechanicLifecycleDiagnostic> lifecycle=[];
    private readonly Dictionary<MechanicActionKey,MechanicForecastDiagnostic> observationIssues=[];
    private long session;
    private long lifecycleRevision;
    private bool hasSession;
    private HitParadePhaseModel? hit;
    private CrownPhaseTracker? crown;
    private KnightBridgeSupport bridges=new();
    public MechanicTimeline Timeline { get; }
    // Discrete observations invalidate schedules without making every native
    // pose sample trigger another whole-route validation job.
    public long Revision=>lifecycleRevision+Timeline.SourceRegistryRevision+Timeline.RecognizedCasts+Timeline.RecognizedEffects+bridges.Revision+(crown?.Revision??0);
    public CrownPhaseTracker? Crown=>crown;
    public void BeginCrownRound(double release)
    {if((crown??=new()).Release==null)crown.Begin(release);}
    public void BeginRound(string course,double release)
    {
        if(!double.IsFinite(release))return;
        if(course=="CrownMountain")BeginCrownRound(release);
        else if(course=="HitParade")(hit??=new()).BeginRound(release);
        else if(course=="KnightFever")bridges.BeginRound(release);
    }
    public HitMechanicCoverage? HitCoverage=>hit?.Coverage;
    public bool AcceptsHitSource(uint? baseId,Vector3 position)=>(hit??=new()).Accepts(baseId,position);

    public MechanicForecaster(MechanicCatalog catalog)
    {this.catalog=catalog;Timeline=new(catalog);}

    public void Reset(string reason,double? at=null)
    {
        RequireReason(reason);Remember(at,null,null,reason);
        histories.Clear();sourceHistories.Clear();directions.Clear();retiredSources.Clear();endedActions.Clear();observationIssues.Clear();
        hit?.Reset();bridges=new();crown=null;
        Timeline.Clear();hasSession=false;
    }
    public void RetireSource(MechanicSourceKey source,string reason,double at)
    {
        RequireReason(reason);Remember(at,source,null,reason);retiredSources.Add(source);
        hit?.Retire(source);
        if(source.Course=="CrownMountain")crown?.Retire(source.EntityId);
        Forget(source);directions.Remove(source);
        foreach(var item in Timeline.Actions)if(item.Key.Source==source)RetireTimelineAction(item.Value);
        if(Timeline.Sources.TryGetValue(source,out var pose))
        {pose.Position=null;pose.PreviousPosition=null;pose.Rotation=null;pose.Radius=null;pose.BaseId=null;}
    }
    public void EndAction(MechanicActionKey action,string reason,double at)
    {
        RequireReason(reason);Remember(at,action.Source,action.ActionId,reason);endedActions.Add(action);
        hit?.End(action);
        histories.Remove(action);observationIssues.Remove(action);
        if(sourceHistories.TryGetValue(action.Source,out var owned))owned.RemoveAll(item=>item.Key==action);
        if(Timeline.Actions.TryGetValue(action,out var state))RetireTimelineAction(state);
    }
    private static void RetireTimelineAction(MechanicActionState state)
    {
        state.PendingCast=null;state.LastCast=null;state.LastEffect=null;state.CastSourcePosition=null;
        state.Casts=0;state.Effects=0;state.PairedCasts=0;state.LastEffectAt=0;
        state.ReceiptIntervals.Clear();state.CastReceiptExcess.Clear();
    }
    private static void RequireReason(string reason)
    {if(string.IsNullOrWhiteSpace(reason))throw new ArgumentException("Lifecycle change requires an explicit reason.",nameof(reason));}
    private void Remember(double? at,MechanicSourceKey? source,uint? action,string reason)
    {
        ++lifecycleRevision;
        int casts=0,children=0;
        foreach(var item in histories)
            if((source==null||item.Key.Source==source)&&(action==null||item.Key.ActionId==action))
            {if(item.Value.PendingCast!=null)++casts;if(item.Value.ParentCastAt!=null&&!item.Value.ChildComplete)++children;}
        lifecycle.Enqueue(new(at,source,action,reason,casts,children));
        if(lifecycle.Count>MaximumLifecycleHistory)lifecycle.Dequeue();
    }
    private void Forget(MechanicSourceKey source)
    {
        var remove=new List<MechanicActionKey>();
        foreach(var key in histories.Keys)if(key.Source==source)remove.Add(key);
        foreach(var key in remove){histories.Remove(key);observationIssues.Remove(key);endedActions.Remove(key);}
        sourceHistories.Remove(source);
    }
    private History Get(MechanicActionKey key)
    {
        if(!histories.TryGetValue(key,out var result))
        {
            histories.Add(key,result=new());
            if(!sourceHistories.TryGetValue(key.Source,out var owned))sourceHistories.Add(key.Source,owned=[]);
            owned.Add((key,result));
        }
        return result;
    }

    public bool Apply(string course,MechanicObservation observation,Vector3? observedSourcePosition=null,
        float? observedSourceRotation=null,double? sourcePoseAt=null,double? sourceRotationAt=null,float nativeFrameSeconds=1f/60)
    {
        if(!hasSession||session!=observation.Session)
        {
            if(hasSession)Remember(observation.Seconds,null,null,"Observation session changed; prior recording state retired.");
            histories.Clear();sourceHistories.Clear();directions.Clear();retiredSources.Clear();endedActions.Clear();observationIssues.Clear();
            hit?.Reset();bridges=new();crown=null;
            hasSession=true;session=observation.Session;
        }
        if(course=="KnightFever"&&observation is MechanicMapObservation map)return bridges.Apply(map);
        if(course=="CrownMountain")crown??=new();
        if(observation is MechanicActorObservation actor)
        {
            var source=new MechanicSourceKey(course,actor.EntityId);
            if(actor.SpawnOffset is { } spawn&&Timeline.Sources.TryGetValue(source,out var old)&&
                old.SpawnOffset is { } previous&&previous!=spawn)
            {
                Remember(actor.Seconds,source,null,"Observed spawn identity changed.");Forget(source);directions.Remove(source);
                hit?.Retire(source);
                retiredSources.Remove(source);
            }
            if(actor.Rotation is { } rotation)ObserveDirection(source,rotation,actor.Seconds);
            if(course=="HitParade") (hit??=new()).Observe(observation,observedSourcePosition,observedSourceRotation,sourcePoseAt);
            bool accepted=Timeline.Apply(course,observation,observedSourcePosition,observedSourceRotation,sourcePoseAt);
            if(Timeline.Sources.TryGetValue(source,out var poseState))
            {
                if(!sourceHistories.ContainsKey(source)&&actor.BaseId is { } baseId&&actor.Position is { } first&&
                    catalog.TryBind(course,baseId,first,out var boundDefinition,out var sourceBounds))
                {
                    var boundKey=new MechanicActionKey(source,boundDefinition.ActionId);
                    var state=Timeline.Register(boundKey);var boundHistory=Get(boundKey);
                    if(MechanicFootprints.Build(state,poseState,out var shape)==MechanicFootprintResult.Candidate)
                    {boundHistory.SourceBounds=sourceBounds;boundHistory.InitialEnvelope=sourceBounds.Envelope(shape,boundDefinition.FixedOriginRotationEnvelope);}
                }
                if(sourceHistories.TryGetValue(source,out var owned))foreach(var item in owned)
                {
                    if(item.Value.PhaseCycle is { } cycle&&Timeline.Actions.TryGetValue(item.Key,out var poseAction)&&
                        cycle.ObservePose(poseAction,poseState,actor.Seconds))++lifecycleRevision;
                    if(!item.Value.SourceBoundsExceeded&&item.Value.SourceBounds is { } bounds&&actor.Position is { } currentPose&&!bounds.Contains(currentPose))
                    {item.Value.SourceBoundsExceeded=true;++lifecycleRevision;}
                }
            }
            if(actor.Position is { } current)
                foreach(var item in histories)
                    if(item.Key.Source==source&&item.Key.ActionId==35409&&
                        Timeline.Actions.TryGetValue(item.Key,out var charge)&&charge.ChargeDestination is { } destination&&
                        current.X==destination.X&&current.Z==destination.Z&&
                        actor.Seconds>=(item.Value.PendingCast?.Seconds??item.Value.Effect?.Seconds??double.PositiveInfinity))
                        item.Value.ChargeComplete=true;
            return accepted;
        }
        uint entity,id;
        switch(observation)
        {
            case MechanicCastObservation cast:entity=cast.EntityId;id=cast.ActionId;break;
            case MechanicEffectObservation effect:entity=effect.EntityId;id=effect.ActionId;break;
            default:return Timeline.Apply(course,observation,observedSourcePosition,observedSourceRotation,sourcePoseAt);
        }
        if(!catalog.TryGet(course,id,out var definition))return false;
        var key=new MechanicActionKey(new(course,entity),id);
        if(retiredSources.Remove(key.Source))
        {Remember(observation.Seconds,key.Source,null,"New recognized action observation reactivates source.");Forget(key.Source);}
        endedActions.Remove(key);
        var history=Get(key);
        if(!double.IsFinite(observation.Seconds))
        {observationIssues[key]=new(key,MechanicForecastIssue.InvalidObservation,"Observation clock is not finite.");return false;}
        // Timeline retains all recognized receipts, including zero-target packets.
        bool recognized=Timeline.Apply(course,observation,observedSourcePosition,observedSourceRotation,sourcePoseAt);
        if(course=="CrownMountain"&&observation is MechanicEffectObservation crownEffect)
            crown!.Observe(crownEffect,observedSourcePosition,nativeFrameSeconds);
        if(course=="HitParade") (hit??=new()).Observe(observation,observedSourcePosition,observedSourceRotation,sourcePoseAt);
        observationIssues.Remove(key);
        if(course=="KnightFever"&&(observation is MechanicCastObservation {ActionId:34717}||
            observation is MechanicEffectObservation {ActionId:34716 or 34718})&&
            Timeline.Sources.TryGetValue(key.Source,out var knownSource)&&
            MechanicFootprints.Build(Timeline.Actions[key],knownSource,out var knownShape)==MechanicFootprintResult.Candidate)
        {
            if(history.RecordedCycle==null&&KnightCycleProfiles.Match(id,knownShape) is { } prior)
                history.RecordedCycle=new(prior,knownShape);
            history.RecordedCycle?.Observe(knownShape,observation.Seconds,
                observation is MechanicCastObservation knownCast?knownCast.Duration:0);
        }
        if(observedSourceRotation is { } facing)
            ObserveDirection(key.Source,facing,sourceRotationAt??sourcePoseAt??observation.Seconds);
        switch(observation)
        {
            case MechanicCastObservation cast:
                if(definition.CastOriginCycle>0&&Timeline.Sources.TryGetValue(key.Source,out var castSource))
                    (history.PhaseCycle??=new(definition.CastOriginCycle,definition.CastReceiptExcessBound,castEndActivation:definition.CastEndActivation,initialPeriod:definition.InitialCastPeriod,completionTail:definition.CastCompletionTail))
                        .Observe(Timeline.Actions[key],castSource);
                ++history.Casts;
                if(history.PendingCast!=null)
                    Remember(cast.Seconds,key.Source,id,"New cast replaced an unresolved same-action cast.");
                history.PendingCast=cast;history.FacingAt=cast.Seconds;history.ChargeComplete=false;
                if(definition.TimingKind=='C')history.Fit=FitCastReceipts(history,Timeline.Actions[key]);
                if(course=="CrownMountain"&&id==34812)
                {
                    var childKey=new MechanicActionKey(key.Source,34804);
                    var child=Get(childKey);
                    if(child.ParentCastAt!=null&&!child.ChildComplete)
                        Remember(cast.Seconds,key.Source,34804,"New parent cast replaced an incomplete child sequence.");
                    endedActions.Remove(childKey);child.ParentCastAt=cast.Seconds;child.ChildPulses=0;child.ChildComplete=false;
                    child.Fit=FitChildReceipts(child);
                }
                break;
            case MechanicEffectObservation effect:
                if(definition.ReceiptCycle>0&&Timeline.Sources.TryGetValue(key.Source,out var receiptSource))
                {
                    var cycle=history.PhaseCycle??=new(definition.ReceiptCycle,null,false,definition.ReceiptWindow,receiptClockError:definition.ReceiptClockError);
                    cycle.ObserveReceipt(Timeline.Actions[key],receiptSource,effect.Seconds);
                }
                else if(definition.PoseCycle>0&&Timeline.Sources.TryGetValue(key.Source,out var effectSource))
                {
                    var cycle=history.PhaseCycle??=new(definition.PoseCycle,definition.PoseReceiptBound,true,definition.ReceiptWindow);
                    cycle.ObservePose(Timeline.Actions[key],effectSource,sourceRotationAt??effect.Seconds);
                    cycle.Receipt(effect.Seconds);
                }
                ++history.Effects;
                if(history.Effect is { } previous&&effect.Seconds<previous.Seconds)
                {
                    observationIssues[key]=new(key,MechanicForecastIssue.OutOfOrderObservation,
                        "Earlier receipt was counted, but did not replace the current forecast phase.");break;
                }
                if(history.Effect is { } prior&&effect.Seconds>prior.Seconds)
                {
                    history.Intervals.Add(effect.Seconds-prior.Seconds);
                    ++history.IntervalCount;
                    if(history.Intervals.Count>MaximumReceiptHistory)history.Intervals.RemoveAt(0);
                }
                history.Effect=effect;
                if(history.PendingCast is { } pending&&effect.Seconds>=pending.Seconds)history.PendingCast=null;
                if(course=="CrownMountain"&&id==34804&&history.ParentCastAt is { } parent&&effect.Seconds>=parent)
                {
                    ++history.ChildPulses;history.ChildComplete=history.ChildPulses==4;
                    if(history.ChildPulses<=4)
                    {
                        var delays=history.ChildDelays[history.ChildPulses-1];delays.Add(effect.Seconds-parent);
                        if(delays.Count>16)delays.RemoveAt(0);
                    }
                }
                history.Fit=course=="CrownMountain"&&id==34804?FitChildReceipts(history):
                    definition.TimingKind=='C'?FitCastReceipts(history,Timeline.Actions[key]):
                    FitReceipts(history,id==34773&&course=="HitParade");
                if(id==34808&&course=="RingSpinner")
                {
                    // A native actor pose can arrive between wind receipts.
                    // Keep the measured heading channel separate from the
                    // angular slope inferred from this helper's effect channel.
                    history.ReceiptDirection.Observe(effect.Rotation,effect.Seconds);
                    ObserveDirection(key.Source,effect.Rotation,effect.Seconds);
                }
                else if(definition.TimingKind=='M'||course=="CrownMountain"&&id==34804)
                    ObserveDirection(key.Source,effect.Rotation,effect.Seconds);
                if(Timeline.Actions[key].Definition.TimingKind!='C')history.FacingAt=effect.Seconds;
                break;
        }
        return recognized;
    }
    private void ObserveDirection(MechanicSourceKey key,float facing,double at)
    {if(!directions.TryGetValue(key,out var direction))directions.Add(key,direction=new());direction.Observe(facing,at);}

    public MechanicDangerSnapshot Snapshot(double now,double horizonEnd,string? course=null,double? nativeFrameSeconds=null)
    {
        var hazards=ImmutableArray.CreateBuilder<MechanicDanger>();
        var signals=ImmutableArray.CreateBuilder<MechanicForecastSignal>();
        var issues=ImmutableArray.CreateBuilder<MechanicForecastDiagnostic>();
        if(!double.IsFinite(now)||!double.IsFinite(horizonEnd)||horizonEnd<now||!double.IsFinite(horizonEnd-now))
        {
            issues.Add(new(null,MechanicForecastIssue.InvalidHorizon,"Caller must supply a finite ordered prediction horizon."));
            return new(session,now,horizonEnd,[],[],[],issues.ToImmutable(),[..lifecycle]);
        }
        foreach(var item in histories)
        {
            var key=item.Key;var history=item.Value;
            if(course!=null&&key.Source.Course!=course||retiredSources.Contains(key.Source)||endedActions.Contains(key))continue;
            if(!catalog.TryGet(key.Source.Course,key.ActionId,out var definition))continue;
            signals.Add(new(key,definition.Role,definition.TimingKind,history.Effects,history.Casts,
                history.Effect?.Seconds,history.PendingCast?.Seconds,history.Fit));
            if(observationIssues.TryGetValue(key,out var observationIssue))issues.Add(observationIssue);
            if(key.Source.Course=="HitParade"&&hit?.Contains(key)==true)continue;
            if(key.Source.Course=="CrownMountain"&&crown!=null&&CrownPhaseTracker.Owns(key.ActionId))continue;
            if(!history.SourceBoundsExceeded&&history.RecordedCycle is { } recorded&&Timeline.Actions.TryGetValue(key,out var recordedAction))
            {recorded.Append(key,recordedAction,now,horizonEnd,nativeFrameSeconds??1d/60,hazards,issues);continue;}
            if(history.InitialEnvelope is { } envelope&&(history.PhaseCycle?.Initialized!=true||history.SourceBoundsExceeded))
            {
                hazards.Add(new(new(key.Source,key.ActionId,history.SourceBoundsExceeded?null:envelope,now,horizonEnd,now),
                    MechanicForecastCertainty.Unknown,MechanicDangerPolicy.UninitializedSourceEnvelope,
                    MechanicDangerEnd.CallerHorizon,envelope,null,null,null));
                if(history.SourceBoundsExceeded)issues.Add(new(key,MechanicForecastIssue.InvalidObservation,"Observed source escaped its recorded motion bound."));
                continue;
            }
            if(history.PhaseCycle is { } cycle&&Timeline.Actions.TryGetValue(key,out var cycling)&&
                cycle.Append(key,cycling,now,horizonEnd,nativeFrameSeconds,hazards,issues))continue;
            // A no-shape action remains a signal. Wind34808 explicitly has a
            // spatial rectangle despite the catalog's zero-target signal role.
            if(definition.Geometry.GetProperty("shape").GetString()=="none")
            {issues.Add(new(key,MechanicForecastIssue.NoSpatialShape,"Observed signal has no candidate spatial shape."));continue;}
            Forecast(key,history,definition,now,horizonEnd,hazards,issues,nativeFrameSeconds);
        }
        if(course==null||course=="HitParade")hit?.Append(now,horizonEnd,nativeFrameSeconds,hazards,issues);
        if(course=="CrownMountain"||course==null&&crown!=null)
        {
            var snapshot=(crown??=new()).Snapshot(session,now,horizonEnd,(float)(nativeFrameSeconds??1d/60));
            foreach(var hazard in snapshot.Hazards)
                hazards.Add(new(hazard,MechanicForecastCertainty.Candidate,MechanicDangerPolicy.CrownCycleWindow,
                    MechanicDangerEnd.CallerHorizon,hazard.Footprint,null,null,null));
            issues.AddRange(snapshot.Unresolved);
        }
        var models=ImmutableArray.CreateBuilder<TemporalHazard>(hazards.Count);
        foreach(var danger in hazards)models.Add(danger.Hazard);
        return new(session,now,horizonEnd,models.MoveToImmutable(),hazards.ToImmutable(),signals.ToImmutable(),issues.ToImmutable(),[..lifecycle])
            {Supports=course=="KnightFever"?bridges.Snapshot(now,horizonEnd,nativeFrameSeconds??1.0/60):[]};
    }

    private void Forecast(MechanicActionKey key,History history,MechanicDefinition definition,double now,double end,
        ImmutableArray<MechanicDanger>.Builder hazards,ImmutableArray<MechanicForecastDiagnostic>.Builder issues,double? nativeFrameSeconds)
    {
        bool child=key.Source.Course=="CrownMountain"&&key.ActionId==34804;
        bool wind=key.Source.Course=="RingSpinner"&&key.ActionId==34808;
        // The helper origin is fixed, but the pendulum body swings off track
        // and back. Its transform and contact window are not the helper pose.
        bool pendulum=key.Source.Course=="HitParade"&&key.ActionId==34775;
        bool charge=definition.Geometry.GetProperty("shape").GetString()=="moving-charge-corridor";
        var certainty=MechanicForecastCertainty.Candidate;
        bool blocksSpatialQuery=false;
        double start=now;var policy=MechanicDangerPolicy.CurrentMovingSource;
        var ending=MechanicDangerEnd.NextObservationOrExplicitEnd;
        void Issue(MechanicForecastIssue issue,string detail,bool unknown=true,bool blocksQuery=true)
        {
            issues.Add(new(key,issue,detail));
            if(unknown){certainty=MechanicForecastCertainty.Unknown;blocksSpatialQuery|=blocksQuery;}
        }

        if(child)
        {
            policy=MechanicDangerPolicy.ParentCastSequence;ending=MechanicDangerEnd.FourChildReceiptsOrExplicitEnd;
            if(history.ChildComplete)
            {
                if(history.Effect is not { } last||last.Seconds<now||last.Seconds>end)return;
                policy=MechanicDangerPolicy.ReceiptInstant;ending=MechanicDangerEnd.ReceiptInstant;start=end=last.Seconds;
            }
            else if(history.ParentCastAt is { } parent)
            {
                start=Math.Max(now,parent);
                if(history.ChildPulses>4)Issue(MechanicForecastIssue.ChildSequenceOverrun,"More than four child receipts followed the same parent cast.");
                Issue(MechanicForecastIssue.ChildSequenceIncomplete,"Parent cast remains active until its four observed child receipts or explicit lifecycle end.",false);
                if(history.ChildPulses==0)
                    Issue(MechanicForecastIssue.MissingMotion,"Announced child has no observed launch motion yet; its future moving waveform is unresolved.");
            }
            else Issue(MechanicForecastIssue.MissingParentCast,"No same-source34812 cast initializes this moving child sequence.");
        }
        else if(wind)
        {
            policy=MechanicDangerPolicy.ContinuousWind;
            if(history.Effect==null)Issue(MechanicForecastIssue.MissingTiming,"Wind has no observed direction receipt.");
        }
        else switch(definition.TimingKind)
        {
            case 'C':
                if(history.PendingCast is { } cast)
                {
                    policy=MechanicDangerPolicy.PendingCast;
                    ending=charge?MechanicDangerEnd.EndpointOrNextAnnouncement:MechanicDangerEnd.EffectOrExplicitCancellation;
                    start=Math.Max(now,cast.Seconds);
                    if(now>cast.Seconds+cast.Duration)
                        Issue(MechanicForecastIssue.PendingCastPastDeclaredEnd,"Declared cast time passed without effect/cancellation authority; danger remains unresolved.",false);
                }
                else if(charge&&history.Effect!=null&&!history.ChargeComplete)
                {policy=MechanicDangerPolicy.CurrentMovingSource;ending=MechanicDangerEnd.EndpointOrNextAnnouncement;}
                else if(history.Effect is { } receipt&&receipt.Seconds>=now&&receipt.Seconds<=end)
                {policy=MechanicDangerPolicy.ReceiptInstant;ending=MechanicDangerEnd.ReceiptInstant;start=end=receipt.Seconds;}
                else return;
                break;
            case 'P':
                policy=MechanicDangerPolicy.ConservativePulseEnvelope;
                Issue(MechanicForecastIssue.UncalibratedActivationWindow,"Physical activation and post-receipt tail remain uncalibrated; the selected timing policy is a candidate.",false);
                if(history.Fit==null||history.Fit.NextPhase==null)
                    Issue(MechanicForecastIssue.MissingTiming,
                        "Source-local receipt phase is unresolved; the known shape remains conservatively active over the caller horizon.",blocksQuery:false);
                break;
            case 'M':break;
            default:Issue(MechanicForecastIssue.UnknownTimingKind,"Catalog timing is unknown; no activation clock is initialized.");break;
        }
        if(charge)ending=MechanicDangerEnd.EndpointOrNextAnnouncement;
        if(start>end)return;
        MechanicFootprint? footprint=null;Vector2? velocity=null;float? angular=null;
        double poseAt=now;double? facingAt=history.FacingAt;
        bool moving=definition.TimingKind=='M'||child||charge;
        if(!Timeline.Sources.TryGetValue(key.Source,out var source)||!Timeline.Actions.TryGetValue(key,out var action))
        {
            // A child exists at parent-cast receipt, before its first own action.
            if(child&&source!=null)
            {
                action=new(definition);
                if(Timeline.Actions.TryGetValue(new(key.Source,34812),out var parentAction))action.LastCast=parentAction.LastCast;
            }
            else {Issue(MechanicForecastIssue.MissingSource,"Source/action scalar state is absent.");source=new();action=new(definition);}
        }
        var build=MechanicFootprints.Build(action!,source!,out var candidate);
        if(build==MechanicFootprintResult.Candidate)footprint=candidate;
        else Issue(build switch
        {
            MechanicFootprintResult.MissingSource=>MechanicForecastIssue.MissingSource,
            MechanicFootprintResult.MissingDirection=>MechanicForecastIssue.MissingDirection,
            MechanicFootprintResult.MissingEndpoint=>MechanicForecastIssue.MissingEndpoint,
            _=>MechanicForecastIssue.MissingDimensions
        },"Candidate footprint could not be constructed: "+build+".");
        if(source!.Position!=null)poseAt=source.PositionAt;
        if(source.Position is { } sourcePosition&&!Finite(sourcePosition))
            Issue(MechanicForecastIssue.InvalidObservation,"Source scalar position is not finite.");
        if(definition.FixedOriginRotationEnvelope&&footprint is { } bounded&&!blocksSpatialQuery)
        {
            var envelope=MechanicFootprints.RotationEnvelope(bounded);
            hazards.Add(new(new(key.Source,key.ActionId,envelope,now,end,now),certainty,
                MechanicDangerPolicy.FixedOriginRotationEnvelope,MechanicDangerEnd.CallerHorizon,
                bounded,source.PositionAt,history.FacingAt,history.Fit));
            return;
        }
        if(moving&&policy!=MechanicDangerPolicy.ReceiptInstant)
        {
            if(charge)
            {
                // The complete CURRENT P→Q corridor contains this announced
                // straight charge leg as P advances toward world-fixed Q.
                // Retain the entire corridor instead of needing speed to
                // shrink it, guessing arrival time, or inventing a next leg.
                if(source.Velocity is not { } chargeMotion||!Finite(chargeMotion))
                    Issue(MechanicForecastIssue.MissingMotion,
                        "Charge speed is unresolved; the entire known current P→Q corridor is retained.",blocksQuery:false);
            }
            else if(pendulum)
            {
                policy=MechanicDangerPolicy.UnresolvedPendulum;
                Issue(MechanicForecastIssue.MissingMotion,
                    "34775 helper pose and alternating receipts do not specify the pendulum body transform or track-contact interval.");
            }
            else if(source.Velocity is { } motion&&Finite(motion))velocity=new(motion.X,motion.Z);
            else Issue(MechanicForecastIssue.MissingMotion,"Moving source has no two ordered scalar pose samples; fixed-snapshot geometry is insufficient for future motion.");
        }
        if(definition.TimingKind=='C'&&!child&&!charge&&action!.LastCast==null&&
            footprint is { Kind:MechanicFootprintKind.Cone or MechanicFootprintKind.Rectangle })
            Issue(MechanicForecastIssue.CastFacingUnavailable,"Effect helper baseline cannot replace this action's missing cast-facing.");
        if(wind)
        {
            if(directions.TryGetValue(key.Source,out var direction)&&direction.Heading is { } heading)
            {
                if(footprint is { } fan)footprint=fan with {Rotation=heading};
                facingAt=direction.At;angular=history.ReceiptDirection.Velocity;
                if(angular is { } speed&&footprint is { } shape)
                    footprint=shape with {Rotation=shape.Rotation+speed*(float)(poseAt-direction.At)};
                else Issue(MechanicForecastIssue.MissingAngularVelocity,"Wind requires an independently observed same-helper angular slope.");
            }
            else Issue(MechanicForecastIssue.MissingDirection,"Wind has no measured same-helper heading.");
        }
        else if(moving&&footprint is { Kind:MechanicFootprintKind.Rectangle or MechanicFootprintKind.Cone } shape&&
            directions.TryGetValue(key.Source,out var direction)&&direction.Heading is { } heading)
        {
            angular=direction.Velocity;facingAt=direction.At;
            // Source-moving shapes follow the latest copied pose, rather than
            // freezing their facing at the preceding sparse effect receipt.
            footprint=shape with {Rotation=heading+(angular??0)*(float)(poseAt-direction.At)};
        }
        // A known stationary pulse envelope or complete current charge corridor
        // can exclude contact outside that candidate despite unknown exact
        // timing/speed. Unknown spatial location/shape or unbounded noncharge
        // motion still returns query-visible Unresolved through a null shape.
        var queryFootprint=blocksSpatialQuery?null:footprint;
        void Add(double first,double last,MechanicDangerPolicy selectedPolicy)
            =>hazards.Add(new(new(key.Source,key.ActionId,queryFootprint,first,last,poseAt,velocity,angular,
                TemporalChargeEndpointMotion.WorldFixed),certainty,selectedPolicy,ending,footprint,
                source.Position!=null?source.PositionAt:null,facingAt,history.Fit,
                child?history.ParentCastAt:null,child?history.ChildPulses:null));
        if(key.Source.Course=="HitParade"&&key.ActionId==34773&&nativeFrameSeconds is { } observedFrame&&
            history.Effect is { } lastReceipt&&history.Fit is { BurstPulseCount:8 } burst)
        {
            var windows=MechanicBurstWindows.Candidate(lastReceipt.Seconds,burst.PhaseMinimums.AsSpan(),
                burst.PhaseMaximums.AsSpan(),burst.ObservedPulseOrdinal,now,end,observedFrame);
            if(windows.Initialized)
            {
                foreach(var window in windows.Windows)Add(window.Start,window.End,MechanicDangerPolicy.SourceLocalBurstWindows);
                return;
            }
            Issue(MechanicForecastIssue.MissingTiming,"Eight-pulse candidate is not initialized: "+windows.Issue+
                "; conservative caller-horizon envelope retained.",blocksQuery:false);
        }
        Add(start,end,policy);
    }
    private static bool Finite(Vector3 value)=>float.IsFinite(value.X)&&float.IsFinite(value.Y)&&float.IsFinite(value.Z);

    private static MechanicReceiptPrediction? FitCastReceipts(History history,MechanicActionState action)
    {
        if(action.CastReceiptExcess.Count==0)return null;
        var excess=action.CastReceiptExcess.ToArray();Array.Sort(excess);int n=excess.Length,middle=n/2;
        double median=n%2==0?(excess[middle-1]+excess[middle])/2:excess[middle];
        double? castEnd=history.PendingCast is { } cast?cast.Seconds+cast.Duration:null;
        return new("source-local-cast-receipt-excess",0,null,[median],[excess[0]],[excess[^1]],
            castEnd+excess[0],castEnd+excess[^1],n);
    }
    private static MechanicReceiptPrediction? FitChildReceipts(History history)
    {
        foreach(var phase in history.ChildDelays)if(phase.Count==0)return null;
        var medians=ImmutableArray.CreateBuilder<double>(4);var mins=ImmutableArray.CreateBuilder<double>(4);
        var maxs=ImmutableArray.CreateBuilder<double>(4);int count=0;
        foreach(var phase in history.ChildDelays)
        {
            var delays=phase.ToArray();Array.Sort(delays);int n=delays.Length;count+=n;
            int middle=n/2;medians.Add(n%2==0?(delays[middle-1]+delays[middle])/2:delays[middle]);
            mins.Add(delays[0]);maxs.Add(delays[^1]);
        }
        bool pending=history.ParentCastAt!=null&&history.ChildPulses<4;
        var minimums=mins.MoveToImmutable();var maximums=maxs.MoveToImmutable();int next=history.ChildPulses;
        return new("same-source-parent-cast-four-child-delays",0,pending?next:null,medians.MoveToImmutable(),minimums,maximums,
            pending&&double.IsFinite(minimums[next])?history.ParentCastAt+minimums[next]:null,
            pending&&double.IsFinite(maximums[next])?history.ParentCastAt+maximums[next]:null,
            count,4,history.ChildPulses);
    }

    // Fit only this source's recent ordered intervals. BIC selects the smallest
    // supported ordinal cycle; catalog representative timings are never loaded.
    // Ranges retain observed receipt jitter rather than inventing hit precision.
    private static MechanicReceiptPrediction? FitReceipts(History history,bool eightPulseBurst)
    {
        var values=history.Intervals;int n=values.Count;
        if(n<3||history.Effect==null)return null;
        int best=1;double score=double.PositiveInfinity;
        int limit=eightPulseBurst?1:Math.Min(8,n/3);
        for(int length=1;length<=limit;++length)
        {
            double error=0;
            for(int phase=0;phase<length;++phase)
            {
                double total=0;int count=0;
                for(int i=phase;i<n;i+=length){total+=values[i];++count;}
                double mean=total/count;
                for(int i=phase;i<n;i+=length){double delta=values[i]-mean;error+=delta*delta;}
            }
            // Double.Epsilon only handles exact zero residual, not a timing pad.
            double bic=n*Math.Log(Math.Max(error/n,double.Epsilon))+length*Math.Log(n);
            if(bic<score){score=bic;best=length;}
        }
        int? ordinal=null;double? offGap=null;string family="source-local-ordinal-cycle";
        if(eightPulseBurst)
        {
            family="eight-pulse-burst-phase-unresolved";
            if(n>=8)
            {
                var sorted=values.ToArray();Array.Sort(sorted);double ratio=1;double threshold=0;
                for(int i=1;i<sorted.Length;++i)
                {
                    double separation=sorted[i]/sorted[i-1];
                    if(separation>ratio){ratio=separation;threshold=(sorted[i]+sorted[i-1])/2;}
                }
                // This separation test identifies a receipt-family candidate.
                // It does not define any danger window or safe off-gap.
                if(ratio>2)
                {
                    int lastGap=-1;
                    for(int i=0;i<n;++i)if(values[i]>threshold){lastGap=i;offGap=values[i];}
                    if(lastGap>=0&&n-lastGap<=8)
                    {ordinal=n-lastGap;best=8;family="observed-eight-pulse-burst";}
                }
            }
        }
        var medians=ImmutableArray.CreateBuilder<double>(best);
        var mins=ImmutableArray.CreateBuilder<double>(best);var maxs=ImmutableArray.CreateBuilder<double>(best);
        int nextPhase=(int)(history.IntervalCount%best);
        // Once an off-gap initializes the next first pulse, rotate phase zero
        // to that observed burst. Initial partial bursts never imply ordinal0.
        int phaseOffset=ordinal is { } current?(n-(current-1))%8:-(int)((history.IntervalCount-n)%best);
        for(int phase=0;phase<best;++phase)
        {
            var samples=new List<double>();
            for(int i=0;i<n;++i)if((i-phaseOffset+best)%best==phase)samples.Add(values[i]);
            samples.Sort();
            if(samples.Count==0){medians.Add(double.NaN);mins.Add(double.NaN);maxs.Add(double.NaN);continue;}
            int middle=samples.Count/2;
            medians.Add(samples.Count%2==0?(samples[middle-1]+samples[middle])/2:samples[middle]);
            mins.Add(samples[0]);maxs.Add(samples[^1]);
        }
        if(ordinal is { } pulse)nextPhase=(pulse-1)%8;
        var minimums=mins.MoveToImmutable();var maximums=maxs.MoveToImmutable();
        bool initialized=!eightPulseBurst||ordinal!=null;
        return new(family,best,initialized?nextPhase:null,medians.MoveToImmutable(),minimums,maximums,
            initialized&&double.IsFinite(minimums[nextPhase])?history.Effect.Seconds+minimums[nextPhase]:null,
            initialized&&double.IsFinite(maximums[nextPhase])?history.Effect.Seconds+maximums[nextPhase]:null,
            n,eightPulseBurst?8:null,ordinal,offGap);
    }
}
