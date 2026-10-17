using System;
using System.Collections.Immutable;
using System.Numerics;

namespace MrBean.FallGuys.Mechanics;

// Observations only narrow opening choices. A released round starts with every
// choice possible; no current-recording prefix or future event is preloaded.
internal sealed class CrownPhaseTracker
{
    private readonly CrownPhaseProfiles profiles;
    private readonly int[] possible;
    private readonly Anchor?[] anchors;
    private readonly record struct Anchor(uint Entity,int Ordinal,double At,int Choice);
    public double? Release {get;private set;}
    public string? Fault {get;private set;}
    public long Revision {get;private set;}
    public int? Configuration
    {
        get
        {
            if(Release==null||Fault!=null)return null;
            Span<int> choices=stackalloc int[possible.Length];
            for(int i=0;i<possible.Length;++i)
            {int mask=possible[i];if(mask==0||(mask&(mask-1))!=0)return null;choices[i]=BitOperations.TrailingZeroCount((uint)mask);}
            return profiles.Encode(choices);
        }
    }
    public CrownPhaseTracker(CrownPhaseProfiles? profiles=null)
    {
        this.profiles=profiles??CrownPhaseProfiles.Embedded;
        possible=new int[this.profiles.Radices.Length];anchors=new Anchor?[this.profiles.Sources.Length];
        for(int i=0;i<possible.Length;++i)possible[i]=(1<<this.profiles.Radices[i])-1;
    }
    public void Begin(double release)
    {
        if(!double.IsFinite(release))throw new ArgumentException("Invalid release clock.");
        if(Release!=null)throw new InvalidOperationException("A Crown round cannot be rebased after release.");
        Release=release;++Revision;
    }
    public CrownPhaseTracker Copy()
    {
        var result=new CrownPhaseTracker(profiles){Release=Release,Fault=Fault,Revision=Revision};
        Array.Copy(possible,result.possible,possible.Length);Array.Copy(anchors,result.anchors,anchors.Length);
        return result;
    }
    public static bool Owns(uint action)=>action is 29795 or 34796 or 34801 or 34802 or 34804 or 34812;
    public void Retire(uint entity)
    {
        foreach(var anchor in anchors)if(anchor?.Entity==entity){Fault="Observed Crown source retired during the active round.";++Revision;return;}
    }
    public void Observe(MechanicEffectObservation effect,Vector3? observed,float frame)
    {
        if(!Owns(effect.ActionId)||effect.ActionId==34812||Release is not { } release||Fault!=null)return;
        var origin=effect.Origin??observed;if(origin is not { } point)return;
        var xz=new Vector2(point.X,point.Z);int found=-1;
        for(int i=0;i<profiles.Sources.Length;++i)
        {
            var profile=profiles.Sources[i];if(profile.ActionId!=effect.ActionId)continue;
            foreach(var pose in profile.Poses)
                if(Vector2.Distance(pose.Origin,xz)<=2*2000f/65535)
                {if(found>=0&&found!=i){Fault="Ambiguous Crown source orbit.";++Revision;return;}found=i;break;}
        }
        if(found<0){Fault="Crown receipt does not belong to a compiled source orbit.";++Revision;return;}
        var source=profiles.Sources[found];int allowed=0,matchedOrdinal=-1,matchedChoice=-1;
        double elapsed=effect.Seconds-release;
        for(int choice=0;choice<profiles.Radices[source.Factor];++choice)
        {
            if((possible[source.Factor]&(1<<choice))==0)continue;
            double period=(source.PeriodMinimum+source.PeriodMaximum)/2;
            int center=Math.Max(0,(int)((elapsed-profiles.Opening(source,choice))/period));
            for(int cycle=Math.Max(0,center-1);cycle<=center+1;++cycle)
            for(int phase=0;phase<source.Poses.Length;++phase)
            {
                int ordinal=cycle*source.Poses.Length+phase;var pose=profiles.Pose(source,choice,ordinal);
                if(Vector2.Distance(pose.Origin,xz)>2*2000f/65535)continue;
                var time=Receipt(found,choice,ordinal);
                if(effect.Seconds<time.earliest-frame||effect.Seconds>time.latest+frame)continue;
                allowed|=1<<choice;matchedOrdinal=ordinal;matchedChoice=choice;
            }
        }
        if(allowed==0){Fault=FormattableString.Invariant($"Crown receipt contradicts phase/clock candidates: action={effect.ActionId}, source={found}, elapsed={elapsed:F6}, mask={possible[source.Factor]}, pose={xz}.");++Revision;return;}
        int previous=possible[source.Factor];possible[source.Factor]&=allowed;
        if(previous!=possible[source.Factor])++Revision;
        if((allowed&(allowed-1))==0)
        {
            var prior=anchors[found];
            if(prior is { } old&&matchedOrdinal<=old.Ordinal)return;
            anchors[found]=new(effect.EntityId,matchedOrdinal,effect.Seconds,matchedChoice);++Revision;
        }
    }
    private (double earliest,double latest) Receipt(int index,int choice,int ordinal)
    {
        var source=profiles.Sources[index];
        if(anchors[index] is not { } anchor||anchor.Choice!=choice)
        {
            var initial=profiles.Receipt(source,choice,ordinal);
            return(Release!.Value+initial.earliest,Release.Value+initial.latest);
        }
        int n=source.Poses.Length,cycles=ordinal/n-anchor.Ordinal/n;
        double offset=source.Offsets[ordinal%n]-source.Offsets[anchor.Ordinal%n];
        double a=cycles*source.PeriodMinimum,b=cycles*source.PeriodMaximum;
        // Two packet residuals enter an anchor-to-future difference.
        double error=2*source.ClockError;
        return(anchor.At+offset+Math.Min(a,b)-error,anchor.At+offset+Math.Max(a,b)+error);
    }
    public MechanicDangerSnapshot Snapshot(long session,double now,double end,float frame)
    {
        var hazards=ImmutableArray.CreateBuilder<TemporalHazard>();
        var issues=ImmutableArray.CreateBuilder<MechanicForecastDiagnostic>();
        if(Fault is { } fault)
        {
            hazards.Add(new(new("CrownMountain",0),0,null,now,end,now));
            issues.Add(new(null,MechanicForecastIssue.InvalidObservation,fault));
        }
        else for(int i=0;i<profiles.Sources.Length;++i)
        {
            var source=profiles.Sources[i];uint entity=anchors[i]?.Entity??(uint)(i+1);
            if(Release==null)
            {
                foreach(var pose in source.Poses)hazards.Add(new(new("CrownMountain",entity),source.ActionId,pose,now,end,now));
                continue;
            }
            for(int choice=0;choice<profiles.Radices[source.Factor];++choice)
            {
                if((possible[source.Factor]&(1<<choice))==0)continue;
                int first=Math.Max(0,(int)((now-Release.Value)/source.PeriodMaximum)-2)*source.Poses.Length;
                for(int ordinal=first;;++ordinal)
                {
                    var time=Receipt(i,choice,ordinal);double begin=time.earliest-source.Before-frame,until=time.latest+frame;
                    if(begin>end)break;if(until<now)continue;
                    hazards.Add(new(new("CrownMountain",entity),source.ActionId,profiles.Pose(source,choice,ordinal),Math.Max(now,begin),Math.Min(end,until),now));
                }
            }
        }
        if(Release==null)issues.Add(new(null,MechanicForecastIssue.MissingTiming,"Waiting for authoritative Crown start-lock release."));
        return new(session,now,end,hazards.ToImmutable(),[],[],issues.ToImmutable(),[]);
    }
}
