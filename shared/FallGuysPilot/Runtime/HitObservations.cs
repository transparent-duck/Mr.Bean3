using System.Numerics;
using System.Collections.Immutable;
using MrBean.FallGuys.Mechanics;

namespace MrBean.FallGuys;

// The Hit phase model owns all four compiled hazard families. Do not also fit
// generic receipt models whose output the Hit model replaces. No program
// trajectory is sampled here.
internal sealed class HitObservations : ICourseObservations
{
    private const double BoundaryPadding=.1;
    private readonly CompiledCourse course;
    private readonly HitParadePhaseModel phases;
    private readonly Dictionary<uint,int> sources;
    private readonly Dictionary<(uint,Vector2),int> sourceSlots;
    private readonly HashSet<uint> actions;
    private string? sourceIssue;
    private readonly int[][] conditionsBySource;
    private long revision;
    private readonly long session;
    public long Revision=>revision;
    internal int DisabledLinkedRows=>phases.DisabledLinkedRows;
    internal bool DisabledSharedBatchClock=>phases.DisabledSharedBatchClock;
    public HitObservations(CompiledCourse course,long session,HitPendulumClockGroups? pendulumGroups=null)
    {
        this.course=course;this.session=session;phases=new(groups:pendulumGroups);sources=[];sourceSlots=[];actions=[];
        foreach(var source in course.Asset.Sources)
        {sourceSlots.Add((source.Action,source.Slot),source.Id);actions.Add(source.Action);}
        conditionsBySource=new int[course.Asset.Sources.Length][];
        for(int s=0;s<conditionsBySource.Length;++s)
            conditionsBySource[s]=course.Asset.Conditions.Where(c=>c.Source==s).Select(c=>c.Id).ToArray();
    }
    private HitObservations(HitObservations source)
    {
        course=source.course;session=source.session;revision=source.revision;sourceIssue=source.sourceIssue;
        phases=source.phases.Copy();sources=new(source.sources);
        // These lookup tables and the compiled course are never mutated after construction.
        sourceSlots=source.sourceSlots;actions=source.actions;conditionsBySource=source.conditionsBySource;
    }
    public ICourseForecast Capture()=>new HitObservations(this);
    public void Apply(MechanicObservation observation,Vector3? position=null,float? heading=null)
    {
        if(observation.Session!=session)return;
        uint entity;uint? baseId,action=null;Vector3? pose;
        switch(observation)
        {
            case MechanicActorObservation actor:
                entity=actor.EntityId;baseId=actor.BaseId;pose=actor.Position;break;
            case MechanicEffectObservation effect when actions.Contains(effect.ActionId):
                entity=effect.EntityId;baseId=effect.BaseId;action=effect.ActionId;pose=position??effect.Origin;break;
            case MechanicCastObservation cast when actions.Contains(cast.ActionId):
                entity=cast.EntityId;baseId=null;action=cast.ActionId;pose=position;break;
            default:return;
        }
        // Unknown or displaced recognized sources must remain an explicit
        // failure. Removing the unused generic fit must not remove this check.
        if(!double.IsFinite(observation.Seconds)||pose is not { } at||
            HitParadeProfiles.Embedded.Match(baseId,at,action) is not { } matched||
            !sourceSlots.TryGetValue((matched.ActionId,new(matched.SlotX,matched.SlotZ)),out int found)||
            sources.TryGetValue(entity,out int previous)&&previous!=found)
        {sourceIssue="Observed Hit source is outside the compiled source table.";++revision;return;}
        sources[entity]=found;
        phases.Observe(observation,position,heading,observation.Seconds);
        ++revision;
    }
    public void Release(double at){phases.BeginRound(at);++revision;}
    public CourseObservation Snapshot(double now,double horizon,float frame,WindowScope? scope=null)
    {
        var dangers=ImmutableArray.CreateBuilder<MechanicDanger>();
        var diagnostics=ImmutableArray.CreateBuilder<MechanicForecastDiagnostic>();
        var covered=new HashSet<MechanicSourceKey>();
        // Request events whose existing boundary padding can overlap the
        // queried horizon, even if their unpadded edge is just outside it.
        HashSet<MechanicSourceKey>? selected=null;
        if(scope!=null)
        {
            selected=[];
            foreach(var source in sources)if(scope.Sources[source.Value])selected.Add(new("HitParade",source.Key));
        }
        phases.Append(now,horizon,frame,dangers,diagnostics,covered,BoundaryPadding,selected);
        var blocked=new List<TimeWindow>[course.Asset.Conditions.Length];
        var seen=new bool[conditionsBySource.Length];var known=new bool[seen.Length];
        for(int n=0;n<blocked.Length;++n)if(scope==null||scope.Conditions[n])blocked[n]=[];
        string? unsupported=sourceIssue;
        // A valid source clock can cover a wholly quiet horizon. Readiness
        // comes from that clock, not from the presence of a danger interval.
        foreach(var key in covered)
            if(sources.TryGetValue(key.EntityId,out int source)){seen[source]=true;known[source]=true;}
            else unsupported="Observed Hit source is outside the compiled source table.";
        foreach(var danger in dangers)
        {
            var hazard=danger.Hazard;
            if(!sources.TryGetValue(hazard.Source.EntityId,out int source))
            {unsupported="Observed Hit source is outside the compiled source table.";continue;}
            seen[source]=true;
            known[source]&=danger.Certainty!=MechanicForecastCertainty.Unknown;
            if(hazard.Footprint is not { } shape || hazard.SourceVelocity is { } velocity&&velocity!=Vector2.Zero ||
                hazard.RotationVelocity is { } turn&&turn!=0)
            {unsupported="Observed motion has no compiled spatial channel.";continue;}
            bool matched=false;
            foreach(int id in conditionsBySource[source])
            {
                var condition=course.Asset.Conditions[id];
                if(condition.Shape is not { } compiled)continue;
                bool unknown=danger.Certainty==MechanicForecastCertainty.Unknown;
                if(condition.UnknownEnvelope!=unknown || !(unknown?ContainsEnvelope(compiled,shape):SameShape(compiled,shape)))continue;
                blocked[id]?.Add(new(hazard.ActiveStart-BoundaryPadding,hazard.ActiveEnd+BoundaryPadding));matched=true;
            }
            if(!matched)unsupported="Observed Hit footprint is outside the compiled envelopes.";
        }
        var result=new ConditionWindows[blocked.Length];
        var omitted=new ConditionWindows(false,[new(now,horizon)]);
        foreach(var condition in course.Asset.Conditions)
        {
            if(blocked[condition.Id]==null){result[condition.Id]=omitted;continue;}
            bool observed=seen[condition.Source]&&known[condition.Source];
            // Missing sources are not absent hazards. Their entire compiled
            // envelope remains blocked until this round has relevant evidence.
            if(!seen[condition.Source])blocked[condition.Id].Add(new(now,horizon));
            result[condition.Id]=new(observed,Windows.Merge(blocked[condition.Id]));
        }
        return new(session,revision,now,horizon,result,unsupported);
    }
    internal static bool SameShape(MechanicFootprint a,MechanicFootprint b)
    {
        // The phase model uses the same copied numeric asset as the compiler.
        // No position snapping or nearest phase substitutes for a missing pose.
        return a.Kind==b.Kind&&a.Origin==b.Origin&&a.Endpoint==b.Endpoint&&a.Rotation==b.Rotation&&
            a.Radius==b.Radius&&a.ForwardMin==b.ForwardMin&&a.ForwardMax==b.ForwardMax&&
            a.HalfWidth==b.HalfWidth&&a.HalfAngle==b.HalfAngle&&a.TargetRadius==b.TargetRadius;
    }
    private static bool ContainsEnvelope(MechanicFootprint envelope,MechanicFootprint observed)
    {
        if(SameShape(envelope,observed))return true;
        if(envelope.Kind!=MechanicFootprintKind.Charge||observed.Kind!=MechanicFootprintKind.Charge ||
            envelope.TargetRadius<observed.TargetRadius)return false;
        float margin=envelope.HalfWidth-observed.HalfWidth;
        if(margin<0)return false;
        var axis=envelope.Endpoint-envelope.Origin;float length=axis.LengthSquared();
        bool Inside(Vector2 point)
        {
            float t=length>0?Math.Clamp(Vector2.Dot(point-envelope.Origin,axis)/length,0,1):0;
            return Vector2.Distance(point,envelope.Origin+t*axis)<=margin+1e-5f;
        }
        return Inside(observed.Origin)&&Inside(observed.Endpoint);
    }
}
