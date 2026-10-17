using System.Collections.Immutable;
using System.Numerics;
using MrBean.FallGuys.Mechanics;

namespace MrBean.FallGuys;

internal sealed class KnightObservations : ICourseObservations
{
    private const string Name="KnightFever";
    private readonly CompiledCourse course;
    private readonly long session;
    private readonly MechanicTimeline? timeline;
    private readonly KnightCycleProfiles.Profile[] profiles;
    private readonly KnightRecordedCycle[] cycles;
    private readonly MechanicActionState[] actions;
    private readonly MechanicActionKey[] keys;
    private readonly bool[] known;
    private readonly Condition[][] channels;
    private readonly KnightBridgeSupport bridges;
    private string? fault;
    public long Revision {get;private set;}
    public KnightObservations(CompiledCourse course,long session)
    {
        this.course=course;this.session=session;profiles=KnightCycleProfiles.All.ToArray();bridges=new();
        var catalog=MechanicCatalog.LoadEmbedded();timeline=new(catalog);
        cycles=new KnightRecordedCycle[profiles.Length];actions=new MechanicActionState[profiles.Length];
        channels=Enumerable.Range(0,profiles.Length).Select(n=>course.Asset.Conditions.Where(c=>c.Source==n).ToArray()).ToArray();
        keys=new MechanicActionKey[profiles.Length];known=new bool[profiles.Length];
        for(int n=0;n<profiles.Length;++n)
        {
            var profile=profiles[n];catalog.TryGet(Name,profile.Action,out var definition);
            actions[n]=new(definition!);cycles[n]=new(profile,KnightCycleProfiles.Shape(profile,0,catalog));
            keys[n]=new(new(Name,0),profile.Action);
        }
    }
    private KnightObservations(KnightObservations source)
    {
        course=source.course;session=source.session;profiles=source.profiles;fault=source.fault;Revision=source.Revision;
        cycles=source.cycles.Select(c=>c.Copy()).ToArray();keys=(MechanicActionKey[])source.keys.Clone();known=(bool[])source.known.Clone();channels=source.channels;
        // Forecasting consumes only the immutable definition and pending cast.
        // The mutable live timeline and receipt queues never enter the worker.
        actions=source.actions.Select(a=>new MechanicActionState(a.Definition){PendingCast=a.PendingCast}).ToArray();
        bridges=source.bridges.Copy();timeline=null;
    }
    public ICourseForecast Capture()=>new KnightObservations(this);
    public void Release(double at){bridges.BeginRound(at);++Revision;}
    public void Apply(MechanicObservation observation,Vector3? position=null,float? heading=null)
    {
        if(observation.Session!=session)return;
        if(timeline==null)throw new InvalidOperationException("A captured forecast does not accept receipts.");
        if(observation is MechanicMapObservation map&&bridges.Apply(map)){++Revision;return;}
        timeline.Apply(Name,observation,position,heading);
        uint action=observation switch{MechanicCastObservation c=>c.ActionId,MechanicEffectObservation e=>e.ActionId,_=>0};
        if(action is not (34716 or 34717 or 34718))return;
        uint entity=observation switch{MechanicCastObservation c=>c.EntityId,MechanicEffectObservation e=>e.EntityId,_=>0};
        var key=new MechanicActionKey(new(Name,entity),action);
        if(!timeline.Actions.TryGetValue(key,out var state)||!timeline.Sources.TryGetValue(key.Source,out var source)||
            MechanicFootprints.Build(state,source,out var shape)!=MechanicFootprintResult.Candidate)
        {fault="Knight source has no attributable observed pose.";++Revision;return;}
        if(shape.Origin.X>=-130||shape.Origin.Y>=0)return;
        var profile=KnightCycleProfiles.Match(action,shape);
        int index=profile==null?-1:Array.IndexOf(profiles,profile);
        if(index<0){fault="Knight source differs from the compiled pose table.";++Revision;return;}
        if(observation is MechanicEffectObservation effect&&profile!.BodyRadius is { } radius&&effect.SourceRadius!=radius)
        {fault="Knight spike native body radius differs from the compiled candidate.";++Revision;return;}
        actions[index]=state;keys[index]=key;
        bool anchor=action==34717?observation is MechanicCastObservation:observation is MechanicEffectObservation;
        if(anchor)
        {
            cycles[index].Observe(shape,observation.Seconds,observation is MechanicCastObservation cast?cast.Duration:0);
            known[index]=true;
        }
        ++Revision;
    }
    public CourseObservation Snapshot(double now,double horizon,float frame,WindowScope? scope=null)
    {
        var blocked=new List<TimeWindow>[course.Asset.Conditions.Length];
        var ready=new bool[blocked.Length];for(int n=0;n<blocked.Length;++n)blocked[n]=[];
        var dangers=ImmutableArray.CreateBuilder<MechanicDanger>();
        var diagnostics=ImmutableArray.CreateBuilder<MechanicForecastDiagnostic>();
        for(int n=0;n<profiles.Length;++n)
        {
            if(scope!=null&&!channels[n].Any(c=>scope.Conditions[c.Id]))continue;
            dangers.Clear();diagnostics.Clear();cycles[n].Append(keys[n],actions[n],now,horizon,frame,dangers,diagnostics);
            bool current=known[n]&&!dangers.Any(d=>d.Certainty==MechanicForecastCertainty.Unknown);
            foreach(var channel in channels[n])ready[channel.Id]=current;
            foreach(var danger in dangers)
            {
                bool matched=false;
                foreach(var channel in channels[n])
                    if(danger.CandidateFootprint is { } shape&&HitObservations.SameShape(channel.Shape!.Value,shape))
                    {blocked[channel.Id].Add(new(danger.Hazard.ActiveStart,danger.Hazard.ActiveEnd));matched=true;}
                if(!matched)fault="Knight phase produced a pose outside its compiled spatial channels.";
            }
        }
        foreach(var support in bridges.Snapshot(now,horizon,frame))
        {
            int id=Array.FindIndex(course.Asset.Conditions,c=>c.SupportId==support.LayoutId);
            if(id<0){fault="Knight bridge has no compiled support dependency.";continue;}
            if(scope!=null&&!scope.Conditions[id])continue;
            blocked[id].AddRange(Windows.Complement(Windows.Merge(support.Available.Select(w=>new TimeWindow(w.Start,w.End)).ToList()),now,horizon));
            ready[id]=true;
        }
        var omitted=new ConditionWindows(false,[new(now,horizon)]);
        return new(session,Revision,now,horizon,blocked.Select((b,n)=>scope!=null&&!scope.Conditions[n]?omitted:
            new ConditionWindows(ready[n],Windows.Merge(b))).ToArray(),fault);
    }
}
