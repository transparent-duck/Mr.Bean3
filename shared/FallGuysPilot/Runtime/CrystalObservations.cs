using System.Numerics;
using MrBean.FallGuys.Mechanics;

namespace MrBean.FallGuys;

internal sealed class CrystalObservations : ICourseObservations
{
    private sealed class Clock(CrystalProfiles.Source source,CrystalProfiles.Phase phase,int condition)
    {
        public readonly CrystalProfiles.Source Source=source;
        public readonly CrystalProfiles.Phase Phase=phase;
        public readonly int Condition=condition;
        public readonly PeriodicReceiptClock Receipt=new(phase.Minimum,phase.Maximum,source.Error,1);
        public double Last=double.NegativeInfinity;
        public float CastDuration;
        public Clock(Clock source):this(source.Source,source.Phase,source.Condition)
        {Receipt=source.Receipt.Copy();Last=source.Last;CastDuration=source.CastDuration;}
    }
    private CrystalObservations(CrystalObservations source)
    {
        course=source.course;session=source.session;fault=source.fault;Revision=source.Revision;
        clocks=source.clocks.Select(c=>new Clock(c)).ToArray();
        byAction=clocks.GroupBy(c=>c.Source.Action).ToDictionary(g=>g.Key,g=>g.ToArray());
    }
    public ICourseForecast Capture()=>new CrystalObservations(this);
    private readonly CompiledCourse course;
    private readonly long session;
    private readonly Clock[] clocks;
    private readonly Dictionary<uint,Clock[]> byAction;
    private string? fault;
    public long Revision {get;private set;}
    public CrystalObservations(CompiledCourse course,long session)
    {
        this.course=course;this.session=session;int condition=0;
        clocks=CrystalProfiles.Embedded.Sources.SelectMany(s=>s.Phases.Select(p=>new Clock(s,p,condition++))).ToArray();
        byAction=clocks.GroupBy(c=>c.Source.Action).ToDictionary(g=>g.Key,g=>g.ToArray());
        foreach(var clock in clocks)
            if(!HitObservations.SameShape(course.Asset.Conditions[clock.Condition].Shape!.Value,clock.Phase.Shape))
                throw new InvalidDataException("Crystal source clock does not match its compiled channel.");
    }
    public void Release(double at){++Revision;}
    public void Apply(MechanicObservation observation,Vector3? position=null,float? heading=null)
    {
        if(observation.Session!=session)return;
        uint action=observation switch{MechanicCastObservation c=>c.ActionId,MechanicEffectObservation e=>e.ActionId,_=>0};
        if(!byAction.TryGetValue(action,out var possible))return;
        if(action==34800&&observation is not MechanicCastObservation||action!=34800&&observation is not MechanicEffectObservation)return;
        Vector3? origin=observation is MechanicEffectObservation effect?effect.Origin??position:position;
        float? rotation=observation switch{MechanicCastObservation c=>c.Rotation,MechanicEffectObservation e=>e.Rotation,_=>heading};
        if(origin is not { } at||rotation is not { } angle)
        {fault="Crystal receipt lacks its actual source pose.";++Revision;return;}
        if(at.X>=-130||at.Z<=100)return;
        Clock? matched=null;
        foreach(var clock in possible)
        {
            var shape=clock.Phase.Shape;
            if(Vector2.Distance(new(at.X,at.Z),shape.Origin)>MechanicSourceBounds.PoseTolerance)continue;
            if(action==34774)
            {
                if(observation is not MechanicEffectObservation {PacketTargetPosition:{ } target}||
                    Vector2.Distance(new(target.X,target.Z),shape.Endpoint)>MechanicSourceBounds.PoseTolerance)continue;
            }
            else if(Math.Abs(MathF.IEEERemainder(angle-shape.Rotation,MathF.Tau))>=.001f)continue;
            matched=clock;break;
        }
        if(matched==null){fault="Crystal receipt has no compiled spatial channel.";++Revision;return;}
        if(observation.Seconds<=matched.Last)return;
        matched.Last=observation.Seconds;
        if(observation is MechanicCastObservation cast)
        {
            if(cast.Duration<=0||cast.Duration>=matched.Phase.Minimum)
            {fault="Crystal cast duration is outside its supported cycle.";++Revision;return;}
            matched.CastDuration=cast.Duration;
        }
        matched.Receipt.Observe(observation.Seconds);++Revision;
    }
    public CourseObservation Snapshot(double now,double horizon,float frame,WindowScope? scope=null)
    {
        var result=new ConditionWindows[course.Asset.Conditions.Length];
        Array.Fill(result,new(false,[new(now,horizon)]));
        foreach(var clock in clocks)
        {
            if(scope!=null&&!scope.Conditions[clock.Condition])continue;
            bool ready=clock.Receipt.Valid&&now<=clock.Last+clock.Phase.Maximum+2*clock.Source.Error;
            if(!ready){result[clock.Condition]=new(false,[new(now,horizon)]);continue;}
            var blocked=new List<TimeWindow>();int first=Math.Max(0,clock.Receipt.Count-1);
            int count=(int)Math.Ceiling((horizon-now)/clock.Phase.Minimum)+3;
            for(int n=first;n<=first+count;++n)
            {
                var predicted=clock.Receipt.Predict(n)!.Value;
                double begin=predicted.Start+clock.CastDuration-clock.Source.Before-frame;
                double end=predicted.End+clock.CastDuration+clock.Source.After+frame;
                if(begin>=horizon)break;
                if(end>now)blocked.Add(new(Math.Max(now,begin),Math.Min(horizon,end)));
            }
            result[clock.Condition]=new(true,Windows.Merge(blocked));
        }
        return new(session,Revision,now,horizon,result,fault);
    }
}
