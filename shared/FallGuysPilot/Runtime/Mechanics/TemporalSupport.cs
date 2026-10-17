using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Numerics;

namespace MrBean.FallGuys.Mechanics;

internal readonly record struct SupportWindow(double Start,double End);

// A floor obligation, separate from damage geometry. Grounded movement must
// remain within one available interval; airborne motion has no such obligation.
// The caller supplies takeoff's grounded prefix and the actual landing point.
internal sealed record TemporalSupport(uint LayoutId,Vector3 Min,Vector3 Max,
    ImmutableArray<SupportWindow> Available,double Begin,double End)
{
    public bool Intersect(Vector3 from,Vector3 to,out double enter,out double exit)
    {
        double first=0,last=1;
        bool Axis(double a,double b,double min,double max)
        {
            double delta=b-a;
            if(delta==0)return a>=min&&a<=max;
            double lo=(min-a)/delta,hi=(max-a)/delta;
            if(lo>hi)(lo,hi)=(hi,lo);
            first=Math.Max(first,lo);last=Math.Min(last,hi);return first<=last;
        }
        bool found=Axis(from.X,to.X,Min.X,Max.X)&&Axis(from.Y,to.Y,Min.Y,Max.Y)&&Axis(from.Z,to.Z,Min.Z,Max.Z);
        enter=first;exit=last;return found;
    }

    public bool Clear(Vector3 from,Vector3 to,double start,double end)
    {
        if(!Intersect(from,to,out double enter,out double exit))return true;
        double a=start+(end-start)*enter,b=start+(end-start)*exit;
        foreach(var window in Available)if(a>=window.Start&&b<=window.End)return true;
        return false;
    }
    public bool JumpClear(Vector3 from,Vector3 landing,Vector2 velocity,double start,double end,double delay)
    {
        double prefix=Math.Min(delay,end-start);
        return Clear(from,from+new Vector3(velocity.X,0,velocity.Y)*(float)prefix,start,start+prefix)
            &&Clear(landing,landing,end,end);
    }

    public IEnumerable<SupportWindow> Unavailable()
    {
        double at=Begin;
        foreach(var window in Available)
        {if(window.Start>at)yield return new(at,window.Start);at=Math.Max(at,window.End);}
        if(at<End)yield return new(at,End);
    }

    public TemporalSupport Reserve(double seconds)
    {
        var result=ImmutableArray.CreateBuilder<SupportWindow>();
        foreach(var window in Available)
            if(window.End-window.Start>2*seconds)result.Add(new(window.Start+seconds,window.End-seconds));
        return this with{Available=result.ToImmutable()};
    }
}

internal static class TemporalSupports
{
    public static bool JumpClear(this ImmutableArray<TemporalSupport> supports,Vector3 from,Vector3 landing,
        Vector2 velocity,double start,double end,double delay)
    {
        foreach(var support in supports)if(!support.JumpClear(from,landing,velocity,start,end,delay))return false;
        return true;
    }

    public static void BlockJumpDepartures(this ImmutableArray<TemporalSupport> supports,Vector3 from,Vector3 landing,
        Vector2 velocity,double offset,double duration,double delay,List<(double Start,double End)> blocked)
    {
        double prefix=Math.Min(delay,duration);
        supports.BlockDepartures(from,from+new Vector3(velocity.X,0,velocity.Y)*(float)prefix,offset,prefix,blocked);
        supports.BlockDepartures(landing,landing,offset+duration,0,blocked);
    }
    public static bool Clear(this ImmutableArray<TemporalSupport> supports,Vector3 from,Vector3 to,double start,double end)
    {foreach(var support in supports)if(!support.Clear(from,to,start,end))return false;return true;}

    public static bool TrajectoryClear(this ImmutableArray<TemporalSupport> supports,ReadOnlySpan<TemporalTrajectoryPoint> points)
    {
        for(int i=1;i<points.Length;++i)
            if(!supports.Clear(points[i-1].Position,points[i].Position,points[i-1].Seconds,points[i].Seconds))return false;
        return true;
    }

    // Add forbidden departure intervals for this grounded segment. The same
    // operation serves graph edges and selected-route schedule repair.
    public static void BlockDepartures(this ImmutableArray<TemporalSupport> supports,Vector3 from,Vector3 to,
        double offset,double duration,List<(double Start,double End)> blocked)
    {
        foreach(var support in supports)
            if(support.Intersect(from,to,out double enter,out double exit))
                foreach(var window in support.Unavailable())
                    blocked.Add((window.Start-offset-exit*duration,window.End-offset-enter*duration));
    }
}
