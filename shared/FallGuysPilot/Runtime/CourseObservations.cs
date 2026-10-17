using System.Numerics;
using MrBean.FallGuys.Mechanics;

namespace MrBean.FallGuys;

internal interface ICourseForecast
{
    long Revision {get;}
    CourseObservation Snapshot(double now,double horizon,float frame,WindowScope? scope=null);
}
internal interface ICourseObservations : ICourseForecast
{
    void Apply(MechanicObservation observation,Vector3? position=null,float? heading=null);
    void Release(double at);
    ICourseForecast Capture();
}

// The compiler enumerates the finite Crown poses. Live code binds only their
// observed phase intervals; it never builds a trajectory or intersects space.
internal sealed class CrownObservations : ICourseObservations
{
    private readonly CompiledCourse course;
    private readonly long session;
    private readonly CrownPhaseTracker phases;
    private readonly Dictionary<uint,Condition[]> byAction;
    public long Revision=>phases.Revision;
    public CrownObservations(CompiledCourse course,long session)
    {
        this.course=course;this.session=session;phases=new();
        byAction=course.Asset.Conditions.GroupBy(c=>course.Asset.Sources[c.Source].Action)
            .ToDictionary(g=>g.Key,g=>g.ToArray());
    }
    private CrownObservations(CrownObservations source)
    {course=source.course;session=source.session;phases=source.phases.Copy();byAction=source.byAction;}
    public ICourseForecast Capture()=>new CrownObservations(this);
    public void Apply(MechanicObservation observation,Vector3? position=null,float? heading=null)
    {
        if(observation.Session!=session)return;
        if(observation is MechanicEffectObservation effect)
            phases.Observe(effect,position,course.Asset.Law.MaximumFrameSeconds);
    }
    public void Release(double at)=>phases.Begin(at);
    public CourseObservation Snapshot(double now,double horizon,float frame,WindowScope? scope=null)
    {
        if(scope?.ConditionCount==0)return new(session,Revision,now,horizon,
            Enumerable.Repeat(new ConditionWindows(false,[new(now,horizon)]),course.Asset.Conditions.Length).ToArray(),phases.Fault);
        var forecast=phases.Snapshot(session,now,horizon,frame);
        var blocked=new List<TimeWindow>[course.Asset.Conditions.Length];
        for(int n=0;n<blocked.Length;++n)if(scope==null||scope.Conditions[n])blocked[n]=[];
        string? unsupported=phases.Fault;
        foreach(var hazard in forecast.Hazards)
        {
            if(hazard.Footprint is not { } shape||!byAction.TryGetValue(hazard.ActionId,out var channels))
            {unsupported??="Observed Crown hazard has no compiled spatial channel.";continue;}
            bool matched=false;
            foreach(var channel in channels)
                if(channel.Shape is { } compiled&&HitObservations.SameShape(compiled,shape))
                {if(blocked[channel.Id]!=null)blocked[channel.Id].Add(new(hazard.ActiveStart,hazard.ActiveEnd));matched=true;}
            if(!matched)unsupported??="Observed Crown pose differs from its compiled source table.";
        }
        // All still-possible opening phases are included in the clock union.
        // A source need not produce a receipt before its conservative union is
        // available. Faults and missing release remain explicit.
        var omitted=new ConditionWindows(false,[new(now,horizon)]);
        return new(session,Revision,now,horizon,blocked.Select(b=>b==null?omitted:
            new ConditionWindows(phases.Release!=null,Windows.Merge(b))).ToArray(),unsupported);
    }
}
