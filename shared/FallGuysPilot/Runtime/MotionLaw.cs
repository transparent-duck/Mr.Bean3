using System.Numerics;

namespace MrBean.FallGuys;

// Physical values and execution limits are different inputs. The former come
// from the captured client model; MaximumFrameSeconds remains the compiler's
// integration budget. The user-requested live experiment removes runtime frame
// and phase ceilings; it does not change these compiled families or physics.
public sealed record MotionLaw(float WalkingSpeed,float SteeringSpeed,float SteeringTurnRate,
    float JumpVerticalSpeed,float Gravity,float BodyRadius,float BodyCenter,
    float MaximumFrameSeconds,float GeometryPrecision,float NativePositionBudget)
{
    public float WalkingStep => WalkingSpeed*MaximumFrameSeconds;
    public bool AcceptsTargetSpeed(EntryDomain entry,float observed)
        =>float.IsFinite(observed)&&(entry.ClampsGroundSpeed?observed>=entry.RequiredTargetSpeed:SameSpeed(observed,entry.RequiredTargetSpeed));
    // With target-directed input and step <= 2r, a step starting outside r
    // cannot cross the target and finish outside r on the opposite side.
    // The next eligible callback therefore captures within half a maximum step.
    public float CaptureRadius => WalkingStep/2;
    public double InputStartupSeconds => 2*MaximumFrameSeconds;
    // Input may consume the frame interval that preceded its dispatch. Its
    // wall timestamp may also be up to one frame after the frame's clock
    // sample. A pose can therefore lead elapsed wall time by two frames;
    // observing the integrated pose can lag it by one frame. The adapter
    // verifies unscaled frame time and the current callback's clock phase.
    public double ClockLeadSeconds => 2*MaximumFrameSeconds;
    public double ClockLagSeconds => MaximumFrameSeconds;
    // Admission waits for a fresh native frame; its input callback can occur
    // one clock phase later. Reserve both bounds in the departure window.
    public double DispatchSeconds => MaximumFrameSeconds+ClockLagSeconds;
    public double EarliestObservation(double physicalSeconds)=>Math.Max(0,physicalSeconds-ClockLeadSeconds);
    public bool AcceptsFrame(float seconds)=>float.IsFinite(seconds)&&seconds>0;
    public bool AcceptsClock(MotionObservation actual)=>actual.UnscaledClock&&
        double.IsFinite(actual.FramePhaseSeconds)&&actual.FramePhaseSeconds>=0;
    // Native launch writes this exact float target, and ordinary release writes
    // zero. Neither speed comparison borrows the mesh's distance budget.
    public bool SameSpeed(float actual,float expected)=>actual==expected;
    // Unit-vector input passes through float rotation/normalization. This is a
    // separate arithmetic budget, propagated into the jump's spatial family.
    public float InputDirectionPrecision=>8*(MathF.BitIncrement(1f)-1f);
    public bool AtRest(float speed,Vector2 external)=>speed==0&&external==Vector2.Zero;
}

public static class SteeringPassControl
{
    // Compare actual (step <= T) and an exact continuous reference at a common Z.
    // Gate delay delta is in [0,T]. Right-end quadrature has weight w in
    // [0,T], so their combined variation kernel is w-delta, in [-T,T].
    // Charging both absolute errors separately would count the same phase
    // range twice. Thus each component costs v*T*TV(direction).
    // Project Z displacement to section arrival time.
    // This bound is used only when both families settle before the next gate.
    public static (float Lateral,double Arrival,float Heading) SectionTurn(float from,float to,MotionLaw law)
    {
        double end=from+MathF.IEEERemainder(to-from,MathF.Tau);
        double lo=Math.Min(from,end),hi=Math.Max(from,end);
        double Variation(bool sine)
        {
            double Value(double a)=>sine?Math.Sin(a):Math.Cos(a);
            double offset=sine?Math.PI/2:0,last=Value(lo),sum=0;
            for(double a=offset+Math.Ceiling((lo-offset)/Math.PI)*Math.PI;a<hi;a+=Math.PI)
            {double value=Value(a);sum+=Math.Abs(value-last);last=value;}
            return sum+Math.Abs(Value(hi)-last);
        }
        double cmin=Math.Min(-Math.Cos(lo),-Math.Cos(hi));
        if(cmin<=0)throw new ArgumentOutOfRangeException(nameof(to),"Section steering must progress north.");
        double scale=law.SteeringSpeed*law.MaximumFrameSeconds;
        double bx=scale*Variation(true),bz=scale*Variation(false);
        double slope=Math.Max(Math.Abs(Math.Tan(lo)),Math.Abs(Math.Tan(hi)));
        return ((float)(bx+slope*bz),bz/(law.SteeringSpeed*cmin),
            (float)Math.Min(hi-lo,2*law.SteeringTurnRate*law.MaximumFrameSeconds/cmin));
    }
    public static bool TryCorrection(SteeringCorrection correction,Vector3 feet,float heading,out Vector3 target)
    {
        target=feet;
        if(!float.IsFinite(feet.X)||!float.IsFinite(feet.Y)||!float.IsFinite(feet.Z)||!float.IsFinite(heading)||
            feet.Z<correction.MinimumZ||feet.Z>correction.MaximumZ||
            Math.Abs(MathF.IEEERemainder(heading-correction.Heading,MathF.Tau))>
                8*Math.Abs(MathF.BitIncrement(correction.Heading)-correction.Heading))return false;
        float slope=MathF.Sin(correction.Heading)/MathF.Cos(correction.Heading);
        float beforeX=correction.Before.X+(feet.Z-correction.Before.Z)*slope;
        float beforeRadius=correction.Section?correction.BeforeRadius:correction.BeforeRadius/Math.Abs(MathF.Cos(correction.Heading));
        if(Math.Abs(feet.X-beforeX)>beforeRadius)return false;
        target=feet with{X=correction.Anchor.X+(feet.Z-correction.Anchor.Z)*slope};
        float distance=Vector3.Distance(feet,target);
        return float.IsFinite(distance)&&distance<.3f&&distance<=correction.MaximumDisplacement;
    }
    public static float NextHeading(float current,float target,float turnRate,float seconds)
        =>current+Math.Clamp(MathF.IEEERemainder(target-current,MathF.Tau),-turnRate*seconds,turnRate*seconds);
    public static bool Inside(SteeringPassLeg pass,Vector3 feet,double elapsed,MotionLaw law)
    {
        if(feet.Y<pass.MinimumHeight||feet.Y>pass.MaximumHeight)return false;
        var samples=pass.Envelope;
        double phase=pass.MaximumTimeRadius;
        double first=Math.Max(0,elapsed-law.InputStartupSeconds-law.ClockLagSeconds-phase);
        double last=Math.Min(samples[^1].Seconds,elapsed+law.ClockLeadSeconds+phase);
        if(first>last)return false;
        int lo=0,hi=samples.Length-1;
        while(lo+1<hi){int middle=(lo+hi)/2;if(samples[middle].Seconds<=first)lo=middle;else hi=middle;}
        var p=new Vector2(feet.X,feet.Z);
        for(int n=lo+1;n<samples.Length&&samples[n-1].Seconds<=last;++n)
        {
            var a=samples[n-1];var b=samples[n];double span=b.Seconds-a.Seconds;
            if(a.Section!=null&&b.Section!=null)
            {
                float dz=b.Position.Z-a.Position.Z;
                if(dz==0)continue; // The correction sweep is checked at dispatch.
                float f=(feet.Z-a.Position.Z)/dz;
                if(f>1||f<0&&(n!=1||feet.Z-a.Position.Z>pass.EntryRadius))continue;
                var expected=Vector3.Lerp(a.Position,b.Position,f);
                double at=a.Seconds+span*f,uncertainty=Math.Max(a.TimeRadius,b.TimeRadius)+Math.Max(a.Section.InterpolationTime,b.Section.InterpolationTime);
                float lateral=Math.Max(a.Radius,b.Radius)+Math.Max(a.Section.InterpolationLateral,b.Section.InterpolationLateral);
                if(Math.Abs(feet.X-expected.X)<=lateral&&
                    elapsed>=at-law.ClockLeadSeconds-uncertainty&&
                    elapsed<=at+law.InputStartupSeconds+law.ClockLagSeconds+uncertainty)return true;
                continue;
            }
            double localPhase=Math.Max(a.TimeRadius,b.TimeRadius);
            double localFirst=Math.Max(first,elapsed-law.InputStartupSeconds-law.ClockLagSeconds-localPhase);
            double localLast=Math.Min(last,elapsed+law.ClockLeadSeconds+localPhase);
            if(b.Seconds<localFirst||a.Seconds>localLast)continue;
            var start=Vector3.Lerp(a.Position,b.Position,span>0?(float)Math.Clamp((localFirst-a.Seconds)/span,0,1):0);
            var end=Vector3.Lerp(a.Position,b.Position,span>0?(float)Math.Clamp((localLast-a.Seconds)/span,0,1):1);
            var q=new Vector2(start.X,start.Z);var delta=new Vector2(end.X-start.X,end.Z-start.Z);
            float t=delta.LengthSquared()>0?Math.Clamp(Vector2.Dot(p-q,delta)/delta.LengthSquared(),0,1):0;
            if(Vector2.Distance(p,q+delta*t)<=Math.Max(a.Radius,b.Radius))return true;
        }
        return false;
    }
}

// The straight-to-target control contracts toward Target. For every admitted
// start in the entry disc its path stays in the segment capsule. Earliest and
// Latest cover that complete family, including the final input callback.
public sealed record GroundSegment(Vector3 Start,Vector3 Target,float EntryRadius,float ArrivalRadius,
    double EarliestStart,double LatestEnd,float MinimumHeight,float MaximumHeight,Vector3? EntryLast=null,
    float CorridorRadius=0);

public static class GroundControl
{
    public static float HorizontalDistance(Vector3 a,Vector3 b)
        =>Vector2.Distance(new(a.X,a.Z),new(b.X,b.Z));
    public static bool Inside(GroundSegment segment,Vector3 feet)
    {
        var a=new Vector2(segment.Start.X,segment.Start.Z);
        var b=new Vector2(segment.Target.X,segment.Target.Z);
        var delta=b-a;var p=new Vector2(feet.X,feet.Z);
        float t=delta.LengthSquared()>0?Math.Clamp(Vector2.Dot(p-a,delta)/delta.LengthSquared(),0,1):0;
        float radius=Math.Max(segment.CorridorRadius,Math.Max(segment.EntryRadius,segment.ArrivalRadius));
        return Vector2.Distance(p,a+delta*t)<=radius&&feet.Y>=segment.MinimumHeight&&feet.Y<=segment.MaximumHeight;
    }
    public static Vector2 Direction(Vector3 feet,Vector3 target)
    {
        var remaining=new Vector2(target.X-feet.X,target.Z-feet.Z);
        return remaining.LengthSquared()>0?Vector2.Normalize(remaining):Vector2.Zero;
    }
}
