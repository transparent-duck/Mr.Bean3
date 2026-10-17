using System;
using System.Collections.Immutable;
using System.Numerics;

namespace MrBean.FallGuys.Mechanics;

// Installed CN 2026.09.15 assets: ManagedSG146 indices14..17; brg01.sgb
// CollisionBox child10; brgc1.pcb support. See the maintained navigation topic.
// Copied numeric support data; see the maintained reuse register.
internal sealed class KnightBridgeSupport
{
    internal readonly record struct Bridge(uint LayoutId,uint BasePlaneId,byte Index,Vector3 Origin);
    internal static readonly Bridge[] Bridges=[
        new(10078795,10617577,14,new(-210.523f,13.09402f,-337.9914f)),
        new(10078796,10617578,15,new(-203.5f,13.09402f,-338)),
        new(10078797,10617580,16,new(-196.5f,13.09402f,-338)),
        new(10078798,10617581,17,new(-189.5f,13.09402f,-338))];
    private static readonly TemporalSupport[] Dependencies=BuildDependencies();
    private static TemporalSupport[] BuildDependencies()
    {
        var result=new TemporalSupport[Bridges.Length];
        for(int i=0;i<result.Length;++i)
        {
            var bridge=Bridges[i];
            result[i]=new(bridge.LayoutId,bridge.Origin+new Vector3(-3,-.025f,-15.600001f),
                bridge.Origin+new Vector3(3,.975f,15.6f),[],0,0);
        }
        return result;
    }
    private sealed class Phase
    {
        // Period and opposite-phase offsets are measured offline. One actual
        // transition can anchor a bounded forecast of the other transition;
        // it cannot establish unobserved support before that forecast begins.
        private static PeriodicReceiptClock Clock()
        {var p=KnightCycleProfiles.Bridge;return new(p.Minimum,p.Maximum,p.Error,1);}
        public readonly PeriodicReceiptClock On,Off;
        public bool? LastOn,FirstOn;
        public double LastAt=double.NegativeInfinity;
        public bool Fault;
        public double? InitialOnAt;
        public Phase(){On=Clock();Off=Clock();}
        public Phase(Phase source)
        {
            On=source.On.Copy();Off=source.Off.Copy();LastOn=source.LastOn;FirstOn=source.FirstOn;
            LastAt=source.LastAt;Fault=source.Fault;InitialOnAt=source.InitialOnAt;
        }
    }
    private readonly Phase[] phases;
    private double? release;
    public KnightBridgeSupport()=>phases=[new(),new(),new(),new()];
    private KnightBridgeSupport(KnightBridgeSupport source)
    {phases=source.phases.Select(p=>new Phase(p)).ToArray();release=source.release;Revision=source.Revision;}
    public KnightBridgeSupport Copy()=>new(this);
    public void BeginRound(double at)
    {if(release==null&&double.IsFinite(at)){release=at;++Revision;}}
    internal static bool IntersectsDependency(Vector3 from,Vector3 to)
    {
        foreach(var support in Dependencies)if(support.Intersect(from,to,out _,out _))return true;
        return false;
    }
    public long Revision {get;private set;}
    public bool Apply(MechanicMapObservation observation)
    {
        if(observation.EventId!=0x8004002B||observation.Index is <14 or >17)return false;
        ++Revision;var phase=phases[observation.Index-14];
        // The pre-round command establishes only opening support. It does not
        // identify which alternating pair drops first or seed a cycle clock.
        if(observation.State==1&&observation.TimelineIndex==1&&phase.LastOn==null&&
            (release==null||observation.Seconds<=release))
        {phase.InitialOnAt=observation.Seconds;return true;}
        bool? on=(observation.State,observation.TimelineIndex) switch{(1,2)=>true,(4,8)=>false,_=>null};
        if(on==null||observation.Seconds<=phase.LastAt||on==phase.LastOn)
        {phase.Fault=true;return true;}
        phase.FirstOn??=on;phase.LastOn=on;phase.LastAt=observation.Seconds;
        (on.Value?phase.On:phase.Off).Observe(observation.Seconds);return true;
    }
    public ImmutableArray<TemporalSupport> Snapshot(double now,double end,double frame)
    {
        var result=ImmutableArray.CreateBuilder<TemporalSupport>(4);
        for(int i=0;i<4;++i)
        {
            var bridge=Bridges[i];var phase=phases[i];var available=ImmutableArray.CreateBuilder<SupportWindow>();
            // Both pair assignments share this initial interval. 4.4s is below
            // the first-off training minimum minus a 0.1s receipt allowance.
            // No initial command means no inferred floor. Collider timing is
            // still a live-acceptance boundary, as for the cyclic map commands.
            if(!phase.Fault&&(phase.Off.Count==0||phase.Off.Valid)&&release is { } released&&phase.InitialOnAt is { } initial&&initial<=released)
            {
                double begin=Math.Max(now,released+2*frame),finish=Math.Min(end,released+4.4-2*frame);
                if(phase.Off.Count>0)finish=Math.Min(finish,phase.Off.Predict(0)!.Value.Start-2*frame);
                if(begin<finish)available.Add(new(begin,finish));
            }
            if(!phase.Fault&&(phase.On.Valid||phase.Off.Valid)&&
                (phase.On.Count==0||phase.On.Valid)&&(phase.Off.Count==0||phase.Off.Valid))
            {
                var prior=KnightCycleProfiles.Bridge;
                SupportWindow Shift(SupportWindow value,SupportWindow offset)=>new(value.Start+offset.Start,value.End+offset.End);
                int offOffset=phase.FirstOn==true?0:1;
                for(int cycle=Math.Max(0,phase.On.Count-1);cycle<phase.On.Count+Math.Ceiling((end-now)/KnightCycleProfiles.Bridge.Minimum)+2;++cycle)
                {
                    var on=phase.On.Valid?phase.On.Predict(cycle)!.Value:
                        Shift(phase.Off.Predict(cycle)!.Value,prior.OffToOn);
                    var off=phase.Off.Valid?phase.Off.Predict(cycle+offOffset)!.Value:
                        Shift(phase.On.Predict(cycle)!.Value,prior.OnToOff);
                    double begin=Math.Max(now,on.End+2*frame),finish=Math.Min(end,off.Start-2*frame);
                    if(begin<finish)available.Add(new(begin,finish));
                }
            }
            // Conservative support-dependence envelope: native body radius .5
            // about the asset bounds. This is unrelated to attack target radius.
            result.Add(Dependencies[i] with{Available=available.ToImmutable(),Begin=now,End=end});
        }
        return result.MoveToImmutable();
    }
}
