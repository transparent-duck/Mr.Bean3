using System.Numerics;
using MrBean.FallGuys.Mechanics;

namespace MrBean.FallGuys;

internal sealed class RingObservations : ICourseObservations
{
    private sealed class CastClock(MechanicDefinition definition)
    {
        public readonly MechanicDefinition Definition=definition;
        public readonly PeriodicReceiptClock Clock=new(definition.InitialCastPeriod!.Value.Minimum,
            definition.InitialCastPeriod!.Value.Maximum,RingProfiles.ReceiptClockError,1);
        public double Last=double.NegativeInfinity;
        public float Duration;
        public CastClock(CastClock source):this(source.Definition)
        {Clock=source.Clock.Copy();Last=source.Last;Duration=source.Duration;}
    }
    private readonly CompiledCourse course;
    private readonly long session;
    private readonly CastClock[] casts;
    private readonly Dictionary<uint,(double At,float Heading)> winds;
    private readonly int[][] sourceConditions;
    private string? fault;
    private bool released;
    public long Revision {get;private set;}
    public RingObservations(CompiledCourse course,long session)
    {
        this.course=course;this.session=session;winds=[];var catalog=MechanicCatalog.LoadEmbedded();
        casts=RingProfiles.Embedded.Sources.Select(s=>catalog.TryGet("RingSpinner",s.Action,out var d)?new CastClock(d):
            throw new InvalidDataException("Ring cast clock is missing.")).ToArray();
        sourceConditions=course.Asset.Sources.Select(s=>course.Asset.Conditions.Where(c=>c.Source==s.Id).Select(c=>c.Id).ToArray()).ToArray();
    }
    private RingObservations(RingObservations source)
    {
        course=source.course;session=source.session;casts=source.casts.Select(c=>new CastClock(c)).ToArray();
        winds=new(source.winds);sourceConditions=source.sourceConditions;fault=source.fault;released=source.released;Revision=source.Revision;
    }
    public ICourseForecast Capture()=>new RingObservations(this);
    public void Release(double at){released=true;++Revision;}
    public void Apply(MechanicObservation observation,Vector3? position=null,float? heading=null)
    {
        if(observation.Session!=session)return;
        if(observation is MechanicEffectObservation {ActionId:34808} wind)
        {
            var origin=wind.Origin??position;
            if(origin is not {X:>150 and <250,Z:<0})return;
            if(Vector2.Distance(new(origin.Value.X,origin.Value.Z),RingProfiles.Center)>MechanicSourceBounds.PoseTolerance)
            {fault="Ring wind source differs from the compiled center.";++Revision;return;}
            if(winds.TryGetValue(wind.EntityId,out var prior))
            {
                double dt=wind.Seconds-prior.At;if(dt<=0)return;
                double delta=Math.IEEERemainder(wind.Rotation-prior.Heading,Math.Tau);
                double uncertainty=2*RingProfiles.ReceiptClockError*Math.Abs(RingProfiles.Embedded.WindMinimum)+Math.Tau/65535;
                if(delta<RingProfiles.Embedded.WindMinimum*dt-uncertainty||delta>RingProfiles.Embedded.WindMaximum*dt+uncertainty)
                    fault="Ring wind contradicted its supported observed rate interval.";
            }
            winds[wind.EntityId]=(wind.Seconds,wind.Rotation);++Revision;return;
        }
        if(observation is not MechanicCastObservation cast||cast.ActionId is not (34797 or 34798 or 34805 or 35409))return;
        var at=position??cast.Position;
        if(at.X is <150 or >250||at.Z>=0)return;
        bool matched=false;
        for(int n=0;n<casts.Length;++n)
        {
            var source=RingProfiles.Embedded.Sources[n];if(source.Action!=cast.ActionId)continue;
            bool applies=source.Action==35409?
                source.Shapes.Any(s=>Vector2.Distance(new(cast.Position.X,cast.Position.Z),s.Origin)<=MechanicSourceBounds.PoseTolerance||
                    Vector2.Distance(new(cast.Position.X,cast.Position.Z),s.Endpoint)<=MechanicSourceBounds.PoseTolerance):
                Vector2.Distance(new(at.X,at.Z),source.Slot)<=MechanicSourceBounds.PoseTolerance&&
                    (source.Action==34805||Math.Abs(MathF.IEEERemainder(cast.Rotation-source.Shapes[0].Rotation,MathF.Tau))<.001f);
            if(!applies)continue;
            matched=true;var clock=casts[n];
            if(cast.Seconds<=clock.Last)continue;
            clock.Last=cast.Seconds;clock.Duration=cast.Duration;clock.Clock.Observe(cast.Seconds);
            ++Revision;
        }
        if(!matched){fault="Ring cast has no compiled source pose or charge endpoint.";++Revision;}
    }
    public CourseObservation Snapshot(double now,double horizon,float frame,WindowScope? scope=null)
    {
        var result=new ConditionWindows[course.Asset.Conditions.Length];
        Array.Fill(result,new(false,[new(now,horizon)]));
        for(int n=0;n<casts.Length;++n)
        {
            if(scope!=null&&!sourceConditions[n].Any(id=>scope.Conditions[id]))continue;
            var source=casts[n];var blocked=new List<TimeWindow>();
            var definition=source.Definition;
            if(!source.Clock.Valid||now>source.Last+definition.InitialCastPeriod!.Value.Maximum+2*RingProfiles.ReceiptClockError)
                blocked.Add(new(now,horizon));
            else
            {
                int first=Math.Max(0,source.Clock.Count-1);
                int count=(int)Math.Ceiling((horizon-now)/definition.InitialCastPeriod!.Value.Minimum)+3;
                double tail=Math.Max(definition.CastReceiptExcessBound??0,definition.CastCompletionTail)+course.Asset.Law.MaximumFrameSeconds;
                for(int k=first;k<=first+count;++k)
                {
                    var predicted=source.Clock.Predict(k)!.Value;
                    double from=predicted.Start+source.Duration,to=predicted.End+source.Duration+tail;
                    if(from>horizon)break;
                    if(to>=now)blocked.Add(new(Math.Max(now,from),Math.Min(horizon,to)));
                }
            }
            var windows=new ConditionWindows(released,Windows.Merge(blocked));
            foreach(int id in sourceConditions[n])if(scope==null||scope.Conditions[id])result[id]=windows;
        }
        var windConditions=sourceConditions[^1];
        for(int bin=0;bin<windConditions.Length;++bin)
        {
            if(scope!=null&&!scope.Conditions[windConditions[bin]])continue;
            var blocked=new List<TimeWindow>();
            if(winds.Count!=2)blocked.Add(new(now,horizon));
            else foreach(var wind in winds.Values)
            {
                double half=Math.PI/RingProfiles.WindBins;
                double center=bin*Math.Tau/RingProfiles.WindBins;
                double slow=-RingProfiles.Embedded.WindMaximum,fast=-RingProfiles.Embedded.WindMinimum;
                double angleError=Math.Tau/65535;
                int first=(int)Math.Floor((wind.Heading-fast*(horizon-wind.At)-center)/Math.Tau)-1;
                int last=(int)Math.Ceiling((wind.Heading-center)/Math.Tau)+1;
                for(int cycle=first;cycle<=last;++cycle)
                {
                    double low=center+cycle*Math.Tau-half-angleError,high=center+cycle*Math.Tau+half+angleError;
                    double from=wind.At+(wind.Heading-high)/fast-RingProfiles.ReceiptClockError;
                    double to=wind.At+(wind.Heading-low)/slow+RingProfiles.ReceiptClockError;
                    if(to<now||from>horizon)continue;
                    blocked.Add(new(Math.Max(now,from),Math.Min(horizon,to)));
                }
            }
            result[windConditions[bin]]=new(released,Windows.Merge(blocked));
        }
        return new(session,Revision,now,horizon,result,fault);
    }
}
