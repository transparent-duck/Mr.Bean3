using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text.Json;

namespace MrBean.FallGuys.Mechanics;

internal readonly record struct HitPhase(double MinimumDelay,double MaximumDelay,
    Vector2 Origin,Vector2 Endpoint,float Heading,bool Continuous,float PositionError,
    double? TurnMinimum,double? TurnMaximum,double[][] CycleOffsets);
internal sealed record HitSourceProfile(uint ActionId,float SlotX,float SlotZ,float Radius,
    float HalfWidth,float Length,double Tail,double? BurstGapSplit,HitPhase[] Phases,double CycleMinimum,double CycleMaximum,double CycleAnchorError);
internal readonly record struct HitMechanicCoverage(int ExpectedSources,int ObservedSources,
    int SourcesWithPhase,long Effects,long PhaseResets,long SequenceErrors);

// Receipt-clock candidates measured from eight recordings. The two held-out
// recordings are replayed before deployment. A source's phase and clock always
// come from its own current observations; no saved recording clock is reused.
internal sealed class HitParadeProfiles
{
    public const float PositionQuantum = 2000f / 65535;
    private static readonly Lazy<HitParadeProfiles> embedded = new(LoadEmbedded);
    public static HitParadeProfiles Embedded => embedded.Value;
    public HitSourceProfile[] Sources { get; }
    public double ObservationEnvelope { get; }

    private HitParadeProfiles(HitSourceProfile[] sources,double envelope)
    {Sources=sources;ObservationEnvelope=envelope;}

    // Geometry does not require an observed phase. Precompute spatial indices
    // at load time; these inactive shapes do not invent a source clock.
    public MechanicDangerSnapshot GeometrySnapshot()
    {
        var hazards=new List<TemporalHazard>();
        foreach(var source in Sources)for(int i=0;i<source.Phases.Length;++i)
        {
            var p=source.Phases[i];var next=source.Phases[(i+1)%source.Phases.Length];
            MechanicFootprint shape=source.ActionId switch
            {
                34776 when p.Continuous=>new(MechanicFootprintKind.Charge,p.Origin,0,0,0,0,
                    source.Radius+MathF.Max(p.PositionError,next.PositionError),0,next.Origin),
                34773 or 34776=>new(MechanicFootprintKind.Circle,p.Origin,0,source.Radius+p.PositionError,0,0,0,0,default),
                34775=>new(MechanicFootprintKind.Rectangle,p.Origin,MathF.PI/2,0,-source.Length/2,source.Length/2,source.HalfWidth,0,default),
                _=>new(MechanicFootprintKind.Charge,p.Origin,0,0,0,0,source.HalfWidth,0,p.Endpoint),
            };
            hazards.Add(new(new("HitParade",0),source.ActionId,shape,2,3,0));
        }
        return new(0,0,1,[..hazards],[],[],[],[]);
    }

    private static HitParadeProfiles LoadEmbedded()
    {
        using var stream=typeof(HitParadeProfiles).Assembly.GetManifestResourceStream("MrBean.FallGuys.Data.hit-parade-phases.json")
            ??throw new InvalidOperationException("Hit Parade phase profiles are missing.");
        return Read(stream);
    }
    public static HitParadeProfiles Read(Stream stream)
    {
        using var document=JsonDocument.Parse(stream);var root=document.RootElement;
        if(root.GetProperty("schemaVersion").GetInt32()!=1||root.GetProperty("course").GetString()!="HitParade")
            throw new InvalidDataException("Unsupported Hit Parade phase profile.");
        var sources=new List<HitSourceProfile>();
        foreach(var item in root.GetProperty("profiles").EnumerateArray())
        {
            var phases=new List<HitPhase>();
            foreach(var phase in item.GetProperty("phases").EnumerateArray())
            {
                var delay=phase.GetProperty("nextDelay");
                double min=delay[0].GetDouble(),max=delay[1].GetDouble();
                if(!(min>0)||!double.IsFinite(max)||max<min)throw new InvalidDataException("Invalid Hit phase delay.");
                double? turnMin=null,turnMax=null;
                if(phase.TryGetProperty("turnDelay",out var turn))
                {turnMin=turn[0].GetDouble();turnMax=turn[1].GetDouble();}
                phases.Add(new(min,max,new(phase.GetProperty("x").GetSingle(),phase.GetProperty("z").GetSingle()),
                    new(phase.GetProperty("endX").GetSingle(),phase.GetProperty("endZ").GetSingle()),
                    phase.GetProperty("heading").GetSingle(),phase.GetProperty("continuous").GetBoolean(),
                    phase.GetProperty("positionError").GetSingle(),turnMin,turnMax,
                    phase.GetProperty("cycleOffsets").EnumerateArray().Select(a=>a.EnumerateArray().Select(v=>v.GetDouble()).ToArray()).ToArray()));
            }
            var split=item.GetProperty("burstGapSplit");
            sources.Add(new(item.GetProperty("actionId").GetUInt32(),item.GetProperty("slotX").GetSingle(),
                item.GetProperty("slotZ").GetSingle(),item.GetProperty("radius").GetSingle(),
                item.GetProperty("halfWidth").GetSingle(),item.GetProperty("length").GetSingle(),
                item.GetProperty("tail").GetDouble(),split.ValueKind==JsonValueKind.Null?null:split.GetDouble(),[..phases],
                item.GetProperty("cycleSeconds")[0].GetDouble(),item.GetProperty("cycleSeconds")[1].GetDouble(),
                item.GetProperty("cycleAnchorError").GetDouble()));
        }
        if(sources.Count!=40||sources.Any(p=>p.Phases.Length is <2 or >14))
            throw new InvalidDataException("Incomplete Hit Parade source profiles.");
        foreach(var source in sources)
        {
            if(!(source.CycleMinimum>0)||!double.IsFinite(source.CycleMaximum)||source.CycleMaximum<source.CycleMinimum||
                !double.IsFinite(source.CycleAnchorError)||source.CycleAnchorError<0)
                throw new InvalidDataException("Invalid Hit source cycle clock.");
            foreach(var phase in source.Phases)
                if(phase.CycleOffsets.Length!=source.Phases.Length||phase.CycleOffsets.Any(a=>a.Length!=2||
                    !double.IsFinite(a[0])||!double.IsFinite(a[1])||a[0]<0||a[1]<a[0]))
                    throw new InvalidDataException("Invalid Hit source cycle offsets.");
        }
        return new([..sources],root.GetProperty("observationEnvelopeSeconds").GetDouble());
    }

    public HitSourceProfile? Match(uint? baseId,Vector3 position,uint? action=null)
    {
        float tolerance=2*PositionQuantum;
        foreach(var source in Sources)
        {
            if(action!=null&&source.ActionId!=action)continue;
            if(baseId!=null&&baseId!=(source.ActionId==34774?16350u:16338u))continue;
            if(source.ActionId==34774)
            {if(MathF.Abs(position.Z-source.SlotZ)<=tolerance)return source;}
            else if(source.ActionId==34776)
            {
                if(MathF.Abs(position.X-source.SlotX)<=tolerance&&
                    (MathF.Abs(position.Z-(-366.8f))<=tolerance||
                     position.Z>=source.Phases[0].Origin.Y-source.Radius&&
                     position.Z<=source.Phases[^1].Origin.Y+source.Radius))return source;
            }
            else if(Vector2.DistanceSquared(new(position.X,position.Z),new(source.SlotX,source.SlotZ))<=tolerance*tolerance)
                return source;
        }
        return null;
    }
}

// Four complete Hit AoE families. Each finite window is a prediction for manual
// acceptance, not a certificate of server timing or immunity between receipts.
// Unknown initialization retains a bounded local envelope. It never turns into
// a globally unresolved shape or a permanently active initialized pendulum disk.
internal sealed class HitParadePhaseModel
{
    private readonly record struct Opening(uint Action,float X,float Z,double Minimum,double Maximum,int Batches);
    private static readonly Lazy<Opening[]> openings=new(()=>
    {
        using var stream=typeof(HitParadePhaseModel).Assembly.GetManifestResourceStream("MrBean.FallGuys.Data.hit-openings.json")
            ??throw new InvalidDataException("Hit opening profiles are missing.");
        using var document=JsonDocument.Parse(stream);
        return document.RootElement.GetProperty("sources").EnumerateArray().Select(p=>new Opening(
            p.GetProperty("action").GetUInt32(),p.GetProperty("x").GetSingle(),p.GetProperty("z").GetSingle(),
            p.GetProperty("minimum").GetDouble(),p.GetProperty("maximum").GetDouble(),p.GetProperty("batches").GetInt32())).ToArray();
    });
    private sealed class Source(HitSourceProfile profile)
    {
        public readonly HitSourceProfile Profile=profile;
        public int? Phase,HeadingSign;
        public double? EffectAt,TurnAt;
        public int TurnSign;
        public long Effects;
        public Vector3 Position;
        public double PositionAt;
        public string? Issue;
        public double FirstObservedAt;
        public Opening? Opening;
        public readonly PeriodicReceiptClock?[] Clocks=new PeriodicReceiptClock?[profile.Phases.Length];
    }
    private readonly HitParadeProfiles profiles;
    private readonly HitPendulumClockGroups? groups;
    private readonly Dictionary<MechanicSourceKey,Source> sources=[];
    private readonly HashSet<MechanicActionKey> ended=[];
    private long effects,phaseResets,sequenceErrors;
    private double? release;
    public void BeginRound(double at){if(double.IsFinite(at)){release??=at;groups?.BeginRound(at);}}
    public HitParadePhaseModel(HitParadeProfiles? profiles=null,HitPendulumClockGroups? groups=null)
    {this.profiles=profiles??HitParadeProfiles.Embedded;this.groups=groups;}
    public HitParadePhaseModel Copy()
    {
        var result=new HitParadePhaseModel(profiles,groups?.Copy())
            {effects=effects,phaseResets=phaseResets,sequenceErrors=sequenceErrors,release=release};
        foreach(var item in sources)
        {
            var s=item.Value;var copied=new Source(s.Profile){Phase=s.Phase,HeadingSign=s.HeadingSign,
                EffectAt=s.EffectAt,TurnAt=s.TurnAt,TurnSign=s.TurnSign,Effects=s.Effects,Position=s.Position,
                PositionAt=s.PositionAt,Issue=s.Issue,FirstObservedAt=s.FirstObservedAt,Opening=s.Opening};
            for(int n=0;n<s.Clocks.Length;n++)copied.Clocks[n]=s.Clocks[n]?.Copy();
            result.sources.Add(item.Key,copied);
        }
        foreach(var item in ended)result.ended.Add(item);
        return result;
    }
    public bool Accepts(uint? baseId,Vector3 position)=>profiles.Match(baseId,position)!=null;
    public bool Contains(MechanicActionKey action)=>sources.TryGetValue(action.Source,out var source)&&source.Profile.ActionId==action.ActionId;
    public HitMechanicCoverage Coverage=>new(profiles.Sources.Length,sources.Count,
        sources.Values.Count(s=>s.Phase!=null||s.TurnAt!=null),effects,phaseResets,sequenceErrors);
    internal int DisabledLinkedRows=>groups?.DisabledRows??0;
    internal bool DisabledSharedBatchClock=>groups?.DisabledCommon??false;
    public void Reset(){sources.Clear();ended.Clear();groups?.Reset();effects=phaseResets=sequenceErrors=0;release=null;}
    public void Retire(MechanicSourceKey source)
    {if(sources.Remove(source,out var retired))groups?.Invalidate(retired.Profile);}
    public void End(MechanicActionKey action){Retire(action.Source);ended.Add(action);}

    public void Observe(MechanicObservation observation,Vector3? actualPosition=null,float? actualHeading=null,double? poseAt=null)
    {
        uint entity;uint? baseId,action=null;Vector3? position;
        if(observation is MechanicActorObservation actor)
        {entity=actor.EntityId;baseId=actor.BaseId;position=actor.Position;actualHeading=actor.Rotation;}
        else if(observation is MechanicEffectObservation effect)
        {entity=effect.EntityId;baseId=effect.BaseId;position=actualPosition??effect.Origin;action=effect.ActionId;}
        else return;
        if(position is not { } point||!double.IsFinite(observation.Seconds))return;
        var key=new MechanicSourceKey("HitParade",entity);
        if(!sources.TryGetValue(key,out var source))
        {
            if(profiles.Match(baseId,point,action) is not { } profile)return;
            var actionKey=new MechanicActionKey(key,profile.ActionId);
            if(ended.Contains(actionKey)&&observation is not MechanicEffectObservation)return;
            ended.Remove(actionKey);sources[key]=source=new(profile);
            source.FirstObservedAt=observation.Seconds;
            foreach(var opening in openings.Value)
                if(opening.Action==profile.ActionId&&opening.X==profile.SlotX&&opening.Z==profile.SlotZ)
                {source.Opening=opening;break;}
        }
        if(poseAt==null||poseAt>=source.PositionAt)
        {source.Position=point;source.PositionAt=poseAt??observation.Seconds;}
        if(source.Profile.ActionId==34775&&actualHeading is { } heading)
        {
            int sign=MathF.Sin(heading)>=0?1:-1;
            if(source.HeadingSign is { } old&&old!=sign)
            {source.TurnAt=observation.Seconds;source.TurnSign=sign;++phaseResets;}
            source.HeadingSign=sign;
        }
        if(observation is not MechanicEffectObservation receipt||receipt.ActionId!=source.Profile.ActionId)return;
        var profileData=source.Profile;int? phase=null;
        switch(receipt.ActionId)
        {
            case 34773:
                if(source.Effects==0&&release is { } released&&source.FirstObservedAt<=released&&source.Opening is { } opening)
                {
                    int? candidatePhase=null;bool ambiguous=false;
                    for(int batch=0;batch<opening.Batches;++batch)
                        for(int pulse=0;pulse<profileData.Phases.Length;++pulse)
                        {
                            var lag=profileData.Phases[0].CycleOffsets[pulse];
                            double pad=pulse>0?profileData.CycleAnchorError:0;
                            if(receipt.Seconds<released+opening.Minimum+3*batch+lag[0]-pad||
                                receipt.Seconds>released+opening.Maximum+3*batch+lag[1]+pad)continue;
                            if(candidatePhase is { } prior&&prior!=pulse)ambiguous=true;
                            candidatePhase=pulse;
                        }
                    // A late group's first hit can overlap an early group's
                    // last pulse if receipts were missed. Keep that ambiguity.
                    if(!ambiguous)phase=candidatePhase;
                }
                if(source.EffectAt is { } previous)
                {
                    if(receipt.Seconds-previous>profileData.BurstGapSplit){phase=0;++phaseResets;}
                    else if(source.Phase is { } old&&old<7)phase=old+1;
                }
                break;
            case 34774:
                if(receipt.PacketTargetPosition is { } endpoint)
                {
                    int sign=MathF.Sin(receipt.Rotation)>=0?1:-1;
                    float best=float.PositiveInfinity;
                    for(int i=0;i<profileData.Phases.Length;++i)
                    {
                        var candidate=profileData.Phases[i];
                        if((MathF.Sin(candidate.Heading)>=0?1:-1)!=sign)continue;
                        float error=Vector2.DistanceSquared(candidate.Endpoint,new(endpoint.X,endpoint.Z));
                        if(error<best){best=error;phase=i;}
                    }
                    if(best>4*HitParadeProfiles.PositionQuantum*HitParadeProfiles.PositionQuantum)phase=null;
                }
                break;
            case 34775:phase=MathF.Sin(receipt.Rotation)>=0?0:1;break;
            case 34776:
                float minimum=float.PositiveInfinity;
                for(int i=0;i<profileData.Phases.Length;++i)
                {
                    var candidate=profileData.Phases[i];float error=Vector2.Distance(candidate.Origin,new(point.X,point.Z));
                    if(error<minimum){minimum=error;phase=i;}
                }
                if(phase is { } selected&&minimum>profileData.Phases[selected].PositionError+2*HitParadeProfiles.PositionQuantum)
                    phase=null;
                break;
        }
        bool contiguous=false;
        if(source.Phase is { } previousPhase&&phase is { } nextPhase&&source.EffectAt is { } previousAt)
        {
            var expected=ReceiptBounds(profileData,previousPhase,1,previousAt);
            contiguous=nextPhase==(previousPhase+1)%profileData.Phases.Length&&
                receipt.Seconds>previousAt&&receipt.Seconds>=expected.First&&receipt.Seconds<=expected.Last;
            if(!contiguous)++sequenceErrors;
        }
        // Keep only an attributable consecutive phase chain. A missing pulse
        // or ambiguous phase starts a new clock; it cannot invent cycle count.
        if(!contiguous)Array.Clear(source.Clocks);
        if(phase is { } observedPhase)
        {
            // Reuse the feasible origin/period polygon. The profile's anchor
            // error already contains TWO per-receipt residuals. Retain every
            // clock compatible with this source's contiguous observations.
            var clock=source.Clocks[observedPhase]??=new(profileData.CycleMinimum,
                profileData.CycleMaximum,profileData.CycleAnchorError/2,1);
            clock.Observe(receipt.Seconds);
            if(profileData.ActionId==34775)
                groups?.Observe(profileData,entity,observedPhase,clock.Count-1,receipt.Seconds,
                    source.Effects==0,contiguous);
        }
        source.Phase=phase;source.EffectAt=receipt.Seconds;++source.Effects;++effects;
        source.Issue=phase==null?"Waiting for a source-local phase observation.":null;
    }

    private static (double First,double Last) ReceiptBounds(HitSourceProfile profile,int phase,int count,double at)
    {
        int cycles=count/profile.Phases.Length,offset=count%profile.Phases.Length;
        var lag=profile.Phases[phase].CycleOffsets[offset];
        double error=count>0?profile.CycleAnchorError:0;
        return(at+cycles*profile.CycleMinimum+lag[0]-error,at+cycles*profile.CycleMaximum+lag[1]+error);
    }

    public void Append(double now,double horizon,double? frame,
        ImmutableArray<MechanicDanger>.Builder dangers,ImmutableArray<MechanicForecastDiagnostic>.Builder issues,
        ISet<MechanicSourceKey>? coveredSources=null,double windowPadding=0,IReadOnlySet<MechanicSourceKey>? selectedSources=null)
    {
        foreach(var item in sources)
        {
            if(selectedSources!=null&&!selectedSources.Contains(item.Key))continue;
            var key=item.Key;var source=item.Value;var profile=source.Profile;
            // ActorSetPos/effect skew was measured for the moving balls only.
            // Other sources use their measured receipt ranges. The current
            // rendering frame is not uncertainty on every future activation:
            // changing that pad each frame made distant windows oscillate.
            double margin=profile.ActionId==34776?profiles.ObservationEnvelope:0;
            int? phase=source.Phase;double earliest=source.EffectAt??now,latest=earliest;
            // A helper's sign reversal announces this station before its hit.
            // Initial facing alone never initializes the time of that reversal.
            // A received hit is the tighter clock. A later turn includes an
            // additional turn-to-hit delay range; replacing a still-current
            // receipt with it widened future windows and invalidated routes.
            // Turns bootstrap/reacquire the clock only while receipts cannot.
            bool receiptCurrent=phase is { } receiptPhase&&source.EffectAt is { } receiptAt&&
                now<=receiptAt+profile.Phases[receiptPhase].MaximumDelay+margin;
            if(profile.ActionId==34775&&!receiptCurrent&&source.TurnAt is { } turn&&turn>(source.EffectAt??double.NegativeInfinity))
            {
                phase=source.TurnSign>0?0:1;var next=profile.Phases[phase.Value];
                earliest=turn+next.TurnMinimum!.Value-profile.CycleAnchorError;
                latest=turn+next.TurnMaximum!.Value+profile.CycleAnchorError;
            }
            if(phase==null)
            {
                if(source.Effects==0&&release is { } released&&source.FirstObservedAt<=released&&source.Opening is { } opening&&
                    now<=released+opening.Maximum+3*(opening.Batches-1))
                {
                    // Union every remaining assignment. No negative evidence
                    // or future receipt is used to select a batch.
                    coveredSources?.Add(key);
                    for(int batch=0;batch<opening.Batches;++batch)
                        AppendCycle(0,released+opening.Minimum+3*batch,released+opening.Maximum+3*batch);
                }
                else Unknown("Waiting for phase initialization.");
                continue;
            }
            // A missing expected receipt does not authorize indefinite cycling.
            // Keep a local unknown envelope until new authoritative input arrives.
            if(now>latest+profile.Phases[phase.Value].MaximumDelay+margin)
            {Unknown("Expected source receipt has not arrived; reacquire phase.");continue;}
            if(source.Clocks.Any(clock=>clock is {Valid:false}))
            {Unknown("Current source receipts have no period within the retained cycle and residual bounds.");continue;}
            coveredSources?.Add(key);
            AppendCycle(phase.Value,earliest,latest);
            void AppendCycle(int anchorPhase,double anchorEarliest,double anchorLatest)
            {
            int currentPhase=anchorPhase;
            bool receiptClock=anchorEarliest==anchorLatest&&source.EffectAt==anchorEarliest&&source.Phase==anchorPhase;
            (double first,double last) At(int count)
            {
                var bounds=ReceiptBounds(profile,anchorPhase,count,anchorEarliest);
                double first=bounds.First,last=bounds.Last+anchorLatest-anchorEarliest;
                if(receiptClock&&count>0&&source.Clocks[(anchorPhase+count)%profile.Phases.Length] is { } clock&&
                    clock.Predict(clock.Count+(count-1)/profile.Phases.Length) is { } predicted)
                {
                    first=Math.Max(first,predicted.Start);last=Math.Min(last,predicted.End);
                }
                if(receiptClock&&count>0&&profile.ActionId==34775)
                {
                    int nextPhase=(anchorPhase+count)%profile.Phases.Length;
                    int ordinal=(source.Clocks[nextPhase]?.Count??0)+(count-1)/profile.Phases.Length;
                    if(groups?.Predict(profile,nextPhase,ordinal) is { } linked)
                    {first=Math.Max(first,linked.Start);last=Math.Min(last,linked.End);}
                }
                return(first,last);
            }
            var(firstAt,lastAt)=At(0);
            for(int count=0;firstAt-margin<=horizon+windowPadding;++count)
            {
                var current=profile.Phases[currentPhase];int nextIndex=(currentPhase+1)%profile.Phases.Length;
                var next=profile.Phases[nextIndex];
                var (nextEarliest,nextLatest)=At(count+1);
                if(firstAt>lastAt||nextEarliest>nextLatest)
                {
                    coveredSources?.Remove(key);
                    Unknown("Consecutive source receipts contradict the retained phase-clock bounds.");
                    return;
                }
                double end=current.Continuous?nextLatest:lastAt+profile.Tail;
                var shape=Shape(current);
                if(profile.ActionId==34776&&current.Continuous)
                    shape=new(MechanicFootprintKind.Charge,current.Origin,0,0,0,0,
                        profile.Radius+MathF.Max(current.PositionError,next.PositionError),0,next.Origin);
                Add(shape,firstAt-margin,end+margin,MechanicForecastCertainty.Candidate,MechanicDangerPolicy.HitPhaseWindow);
                firstAt=nextEarliest;lastAt=nextLatest;currentPhase=nextIndex;
            }
            }

            MechanicFootprint Shape(HitPhase selected)=>profile.ActionId switch
            {
                34773 or 34776=>new(MechanicFootprintKind.Circle,selected.Origin,0,profile.Radius+selected.PositionError,0,0,0,0,default),
                // Both swings cover the same station. Heading selects the
                // knockback direction; it must not translate the footprint.
                34775=>new(MechanicFootprintKind.Rectangle,selected.Origin,MathF.PI/2,0,-profile.Length/2,profile.Length/2,profile.HalfWidth,0,default),
                _=>new(MechanicFootprintKind.Charge,selected.Origin,0,0,0,0,profile.HalfWidth,0,selected.Endpoint),
            };
            void Add(MechanicFootprint shape,double first,double last,MechanicForecastCertainty certainty,MechanicDangerPolicy policy)
            {
                first=Math.Max(first,now-windowPadding);last=Math.Min(last,horizon+windowPadding);if(first>last)return;
                // Join adjacent identical static windows to avoid drawing each
                // water pulse over the same circle and repeating raster work.
                if(dangers.Count>0&&dangers[^1] is { } prior&&prior.Hazard.Source==key&&
                    prior.Policy==policy&&prior.Hazard.Footprint==shape&&prior.Hazard.ActiveEnd>=first&&last>=prior.Hazard.ActiveStart)
                {dangers[^1]=prior with{Hazard=prior.Hazard with{ActiveStart=Math.Min(first,prior.Hazard.ActiveStart),ActiveEnd=Math.Max(last,prior.Hazard.ActiveEnd)}};return;}
                dangers.Add(new(new(key,profile.ActionId,shape,first,last,now),certainty,policy,
                    MechanicDangerEnd.CallerHorizon,shape,source.PositionAt,source.TurnAt,null));
            }
            void Unknown(string detail)
            {
                issues.Add(new(new(key,profile.ActionId),MechanicForecastIssue.MissingTiming,detail));
                MechanicFootprint shape;
                if(profile.ActionId==34775)
                    shape=new(MechanicFootprintKind.Rectangle,new(profile.SlotX,profile.SlotZ),MathF.PI/2,
                        0,-profile.Length/2,profile.Length/2,profile.HalfWidth,0,default);
                else if(profile.ActionId is 34774 or 34776)
                {
                    var first=profile.Phases[0].Origin;var last=first;
                    float positionError=0;
                    foreach(var candidate in profile.Phases)
                    {
                        if(profile.ActionId==34774){first.X=MathF.Min(first.X,candidate.Origin.X);last.X=MathF.Max(last.X,candidate.Origin.X);}
                        else{first.Y=MathF.Min(first.Y,candidate.Origin.Y);last.Y=MathF.Max(last.Y,candidate.Origin.Y);positionError=MathF.Max(positionError,candidate.PositionError);}
                    }
                    if(profile.ActionId==34776)
                    {first.Y=MathF.Min(first.Y,source.Position.Z);last.Y=MathF.Max(last.Y,source.Position.Z);}
                    shape=new(MechanicFootprintKind.Charge,first,0,0,0,0,profile.ActionId==34774?profile.HalfWidth:profile.Radius+positionError,0,last);
                }
                else shape=Shape(profile.Phases[0]);
                Add(shape,now,horizon,MechanicForecastCertainty.Unknown,MechanicDangerPolicy.HitPhaseUninitialized);
            }
        }
    }
}
