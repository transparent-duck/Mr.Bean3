using System;
using System.Collections.Immutable;
using System.Numerics;

namespace MrBean.FallGuys.Mechanics;

// A declared sequence of attack poses, anchored by cast or pose announcements.
// Map/action identity and cycle length are catalog data. This predicts attack
// windows only; no visual actor or native GetRadius creates a solid obstacle.
internal sealed class MechanicPhaseCycle(int phases,double? receiptExcessBound,bool poseDriven=false,MechanicReceiptWindow? receiptWindow=null,bool castEndActivation=false,MechanicCastPeriod? initialPeriod=null,double completionTail=0,double receiptClockError=.2)
{
    private readonly MechanicFootprint?[] poses=new MechanicFootprint?[phases];
    private readonly double[] receiptMaximum=new double[phases];
    private readonly int[] receiptCount=new int[phases];
    private readonly double ReceiptClockError=receiptClockError;
    private PeriodicReceiptClock[] clocks=NewClocks(phases,receiptClockError);
    private PeriodicReceiptClock[] receiptClocks=NewClocks(phases,receiptClockError);
    private static PeriodicReceiptClock[] NewClocks(int count,double error)
    {
        var result=new PeriodicReceiptClock[count];
        for(int i=0;i<count;++i)result[i]=new(.001,120,error);
        return result;
    }
    private readonly record struct Anchor(double Seconds,double Duration);
    private Anchor? last;
    private MechanicFootprint? observedPose;
    private double lastReceipt=double.NegativeInfinity;
    private int phase=-1;
    private const float PositionQuantum=2000f/65535;

    public void Observe(MechanicActionState action,MechanicSourceState source)
    {
        if(action.LastCast is not { } cast||!double.IsFinite(cast.Seconds)||
            !float.IsFinite(cast.Duration)||cast.Duration<=0||last?.Seconds>=cast.Seconds||
            MechanicFootprints.Build(action,source,out var shape)!=MechanicFootprintResult.Candidate)return;
        Observe(shape,cast.Seconds,cast.Duration);
    }

    public bool ObservePose(MechanicActionState action,MechanicSourceState source,double at)
    {
        if(!poseDriven||source.Rotation is not { } rotation||
            MechanicFootprints.Build(action,source,out var shape)!=MechanicFootprintResult.Candidate)return false;
        shape=shape with{Rotation=rotation};
        var previous=observedPose;observedPose=shape;
        // A first sample has no known onset. Only a pose transition supplies
        // the announcement clock; repeated framework samples cannot reset it.
        if(previous==null||SamePose(previous.Value,shape))return false;
        Observe(shape,at,0);return true;
    }

    public void ObserveReceipt(MechanicActionState action,MechanicSourceState source,double at)
    {
        if(receiptWindow==null||!double.IsFinite(at)||at<=lastReceipt||
            MechanicFootprints.Build(action,source,out var shape)!=MechanicFootprintResult.Candidate)return;
        Observe(shape,at,0);Receipt(at);
    }

    public void Receipt(double at)
    {
        if(last is { } anchor&&phase>=0&&at>=anchor.Seconds&&lastReceipt<anchor.Seconds)
        {
            receiptMaximum[phase]=Math.Max(receiptMaximum[phase],at-anchor.Seconds-anchor.Duration);++receiptCount[phase];
            receiptClocks[phase].Observe(at);
        }
        lastReceipt=Math.Max(lastReceipt,at);
    }

    public bool Initialized
    {
        get
        {
            int count=0;
            for(int p=0;p<poses.Length;++p){if(poses[p]==null)return false;count+=clocks[p].Count;}
            return count>poses.Length;
        }
    }

    private void Observe(MechanicFootprint shape,double at,double duration)
    {
        if(!double.IsFinite(at)||last?.Seconds>=at)return;
        // A charge traverses the same closed corridor in either direction.
        // Canonical endpoints keep the periodic spatial identity unchanged.
        if(shape.Kind==MechanicFootprintKind.Charge&&(shape.Origin.X>shape.Endpoint.X||
            shape.Origin.X==shape.Endpoint.X&&shape.Origin.Y>shape.Endpoint.Y))
            shape=shape with{Origin=shape.Endpoint,Endpoint=shape.Origin};
        int next=(phase+1)%poses.Length;
        if(poses[next] is { } expected&&!SamePose(expected,shape))
        {
            Array.Clear(poses);
            Array.Clear(receiptMaximum);Array.Clear(receiptCount);
            clocks=NewClocks(poses.Length,ReceiptClockError);receiptClocks=NewClocks(poses.Length,ReceiptClockError);
            phase=-1;next=0;last=null;
        }
        poses[next]=shape;phase=next;last=new(at,duration);clocks[next].Observe(at);
    }

    private static bool SamePose(MechanicFootprint a,MechanicFootprint b)
        =>a.Kind==b.Kind&&Vector2.Distance(a.Origin,b.Origin)<=2*PositionQuantum&&
            (a.Kind!=MechanicFootprintKind.Charge||Vector2.Distance(a.Endpoint,b.Endpoint)<=2*PositionQuantum)&&
            (a.Kind==MechanicFootprintKind.Circle||Math.Abs(MathF.IEEERemainder(a.Rotation-b.Rotation,MathF.Tau))<=2*MathF.Tau/65535);

    public bool Append(MechanicActionKey key,MechanicActionState action,double now,double end,
        double? frameSeconds,ImmutableArray<MechanicDanger>.Builder dangers,
        ImmutableArray<MechanicForecastDiagnostic>.Builder issues)
    {
        if(last is not { } cast||phase<0||poses[phase] is not { } current||poseDriven&&!Initialized)return false;
        if(receiptWindow is { } pulse)
            return AppendReceiptWindows(key,now,end,pulse,dangers,issues);
        bool frameKnown=frameSeconds is >0&&double.IsFinite(frameSeconds.Value);
        double frame=frameKnown?frameSeconds!.Value:0;
        double excess=receiptExcessBound??double.NegativeInfinity;
        if(!poseDriven)foreach(double value in action.CastReceiptExcess)if(double.IsFinite(value))excess=Math.Max(excess,value);
        bool fitted=frameKnown&&double.IsFinite(excess);
        double Delay(int p)=>cast.Duration+Math.Max(completionTail,poseDriven&&receiptCount[p]>=2
            ?Math.Min(excess,receiptMaximum[p]+ReceiptClockError):excess)+frame;
        double delay=Delay(phase),activationOffset=castEndActivation?cast.Duration:0;
        void Issue(MechanicForecastIssue issue,string detail)=>issues.Add(new(key,issue,detail));
        void Add(MechanicFootprint shape,double first,double until,bool unknown=false)
        {
            first=Math.Max(now,first);until=Math.Min(end,until);if(until<first)return;
            dangers.Add(new(new(key.Source,key.ActionId,shape,first,until,now),
                unknown?MechanicForecastCertainty.Unknown:MechanicForecastCertainty.Candidate,
                unknown?MechanicDangerPolicy.CastCycleUninitialized:MechanicDangerPolicy.CastCycleWindow,
                MechanicDangerEnd.EffectOrExplicitCancellation,shape,cast.Seconds,cast.Seconds,null));
        }
        Issue(MechanicForecastIssue.UncalibratedActivationWindow,
            "Announcement-anchored receipt envelope is a candidate; recipient geometry and judgment lag are calibrated separately.");
        if(completionTail>0|| (poseDriven?lastReceipt<cast.Seconds:action.PendingCast!=null))
        {
            double until=fitted?cast.Seconds+delay:end;
            if(now>until&&(poseDriven?lastReceipt<cast.Seconds:action.PendingCast!=null))
            {until=end;Issue(MechanicForecastIssue.PendingCastPastDeclaredEnd,"Overdue announcement remains pending until effect or explicit cancellation.");}
            // Activation and completion are independently declared. A charge
            // can remain active after its launch receipt until endpoint arrival.
            Add(current,cast.Seconds+activationOffset,until);
        }
        bool initialized=fitted&&Initialized;
        bool clockReady=true,clockFault=false;
        foreach(var clock in clocks){clockReady&=clock.Valid;clockFault|=clock.Count>=3&&!clock.Valid;}
        initialized&=clockReady&&!clockFault;
        if(!initialized&&fitted&&initialPeriod is { } initial&&!clockFault)
        {
            // Cold-start bounds are declared evidence, not an action-wide
            // midpoint. Each live cast reanchors them; the observed clock takes
            // over as soon as it is fitted. Contradictions retain the envelope.
            for(int n=1;n<=128;++n)
            {
                double first=cast.Seconds+n*initial.Minimum-ReceiptClockError;
                if(first+activationOffset>end)return true;
                Add(current,first+activationOffset,cast.Seconds+n*initial.Maximum+ReceiptClockError+delay);
            }
        }
        if(!initialized)
        {
            Issue(MechanicForecastIssue.MissingTiming,clockFault?"Observed phases contradict the bounded periodic clock.":"The bounded periodic clock needs three observations of each phase; known attack poses remain blocked.");
            foreach(var pose in poses)if(pose is { } shape)Add(shape,now,end,true);
            return true;
        }
        for(int p=0;p<clocks.Length;++p)
        {
            var clock=clocks[p];bool exhausted=true;
            for(int future=clock.Count;future<clock.Count+128;++future)
            {
                var predicted=clock.Predict(future)!.Value;
                if(predicted.Start>end){exhausted=false;break;}
                double first=predicted.Start,last=predicted.End;
                if(phases==1&&initialPeriod is { } bound)
                {
                    int count=future-clock.Count+1;
                    first=Math.Max(first,cast.Seconds+count*bound.Minimum-ReceiptClockError);
                    last=Math.Min(last,cast.Seconds+count*bound.Maximum+ReceiptClockError);
                    if(first>last)
                    {Add(poses[p]!.Value,now,end,true);Issue(MechanicForecastIssue.MissingTiming,"Observed phase contradicts declared period bounds.");return true;}
                }
                Add(poses[p]!.Value,first+activationOffset,last+Delay(p));
            }
            if(exhausted)Add(poses[p]!.Value,now,end,true);
        }
        return true;
    }

    private bool AppendReceiptWindows(MechanicActionKey key,double now,double end,MechanicReceiptWindow window,
        ImmutableArray<MechanicDanger>.Builder dangers,ImmutableArray<MechanicForecastDiagnostic>.Builder issues)
    {
        bool ready=Initialized;
        foreach(var clock in receiptClocks)ready&=clock.Valid;
        void Add(MechanicFootprint shape,double first,double until,bool unknown=false)
        {
            first=Math.Max(now,first);until=Math.Min(end,until);if(until<first)return;
            dangers.Add(new(new(key.Source,key.ActionId,shape,first,until,now),
                unknown?MechanicForecastCertainty.Unknown:MechanicForecastCertainty.Candidate,
                unknown?MechanicDangerPolicy.CastCycleUninitialized:MechanicDangerPolicy.ReceiptCycleWindow,
                MechanicDangerEnd.CallerHorizon,shape,last?.Seconds,last?.Seconds,null));
        }
        issues.Add(new(key,MechanicForecastIssue.UncalibratedActivationWindow,
            "Visual/local-receipt attack window candidate; packet-clock uncertainty is separate from the declared sweep window."));
        if(!ready)
        {
            foreach(var pose in poses)if(pose is { } shape)Add(shape,now,end,true);
            return true;
        }
        for(int p=0;p<receiptClocks.Length;++p)
        {
            var clock=receiptClocks[p];bool exhausted=true;
            // Include the last receipt's tail and the next receipt's warning
            // interval, even while a later pose announcement is still pending.
            for(int ordinal=clock.Count-1;ordinal<clock.Count+128;++ordinal)
            {
                var time=clock.Predict(ordinal)!.Value;
                if(time.Start-window.Before>end){exhausted=false;break;}
                Add(poses[p]!.Value,time.Start-window.Before,time.End+window.After);
            }
            if(exhausted)Add(poses[p]!.Value,now,end,true);
        }
        return true;
    }
}
