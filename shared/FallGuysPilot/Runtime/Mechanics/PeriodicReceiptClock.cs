using System;
using System.Collections.Generic;

namespace MrBean.FallGuys.Mechanics;

// Feasible (origin,period) polygon. Every received phase constrains the SAME
// clock; interval uncertainty does not compound once per predicted transition.
// Bounds describe packet receipts, not physical activation or server authority.
internal sealed class PeriodicReceiptClock(double minimumPeriod,double maximumPeriod,double receiptError,int minimumObservations=3)
{
    private readonly record struct Vertex(double Origin,double Period);
    private List<Vertex> polygon=[];
    private double origin;
    public int Count {get;private set;}
    public bool Valid=>Count>=minimumObservations&&polygon.Count>0;
    public PeriodicReceiptClock Copy()=>new(minimumPeriod,maximumPeriod,receiptError,minimumObservations)
        {polygon=[..polygon],origin=origin,Count=Count};
    public void Observe(double at)
        =>Observe(at,Count,0,0);
    // Several linked sources may constrain the same cycle. Their known
    // ordinal and within-cycle offset do not increment the cycle separately.
    public void Observe(double at,int ordinal,double offsetMinimum,double offsetMaximum)
    {
        if(!double.IsFinite(at)||ordinal<0||!double.IsFinite(offsetMinimum)||
            !double.IsFinite(offsetMaximum)||offsetMinimum>offsetMaximum)
        {polygon.Clear();++Count;return;}
        if(Count==0)
        {
            origin=at;
            polygon=[new(-offsetMaximum-receiptError-ordinal*minimumPeriod,minimumPeriod),
                new(-offsetMinimum+receiptError-ordinal*minimumPeriod,minimumPeriod),
                new(-offsetMinimum+receiptError-ordinal*maximumPeriod,maximumPeriod),
                new(-offsetMaximum-receiptError-ordinal*maximumPeriod,maximumPeriod)];
        }
        else
        {
            Clip(1,ordinal,at-origin-offsetMinimum+receiptError);
            Clip(-1,-ordinal,-(at-origin)+offsetMaximum+receiptError);
        }
        ++Count;
    }
    private void Clip(double x,double y,double limit)
    {
        if(polygon.Count==0)return;
        var output=new List<Vertex>();var previous=polygon[^1];
        double before=previous.Origin*x+previous.Period*y-limit;
        foreach(var current in polygon)
        {
            double after=current.Origin*x+current.Period*y-limit;
            if((before<=0)!=(after<=0))
            {
                double t=before/(before-after);
                output.Add(new(previous.Origin+(current.Origin-previous.Origin)*t,
                    previous.Period+(current.Period-previous.Period)*t));
            }
            if(after<=0)output.Add(current);
            previous=current;before=after;
        }
        polygon=output;
    }
    public SupportWindow? Predict(int ordinal)
    {
        if(!Valid||ordinal<0)return null;
        double min=double.PositiveInfinity,max=double.NegativeInfinity;
        foreach(var vertex in polygon)
        {double value=origin+vertex.Origin+ordinal*vertex.Period;min=Math.Min(min,value);max=Math.Max(max,value);}
        return new(min-receiptError,max+receiptError);
    }
}
