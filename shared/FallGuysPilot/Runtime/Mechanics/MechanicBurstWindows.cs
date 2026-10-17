using System;
using System.Collections.Immutable;

namespace MrBean.FallGuys.Mechanics;

internal readonly record struct MechanicBurstWindow(double Start,double End);
internal enum MechanicBurstWindowIssue { None,InvalidClock,MissingOrdinal,MissingPhases,InvalidFrame,WindowBudgetExceeded }
internal readonly record struct MechanicBurstWindowResult(bool Initialized,
    ImmutableArray<MechanicBurstWindow> Windows,MechanicBurstWindowIssue Issue);

// EXPERIMENTAL receipt-clock candidate, only for confirmed eight-pulse34773.
// Phases0..6 are the seven within-burst intervals; phase7 is the observed
// eighth→next-first off-gap. No action-wide period/anchor or future hit instant
// is created. This does not prove immunity or bound physical post-receipt tail,
// server latency, animation timing or model error. Unknown input falls back to
// the caller's conservative policy. The supplied actual frame is an explicit
// one-frame observation envelope, not calibrated server duration or a fixed pad.
internal static class MechanicBurstWindows
{
    public static MechanicBurstWindowResult Candidate(double lastReceipt,
        ReadOnlySpan<double> phaseMinimums,ReadOnlySpan<double> phaseMaximums,int? observedOrdinal,
        double horizonStart,double horizonEnd,double frameSeconds,int maximumWindows=256)
    {
        static MechanicBurstWindowResult Unknown(MechanicBurstWindowIssue issue)=>new(false,[],issue);
        if(!double.IsFinite(lastReceipt)||!double.IsFinite(horizonStart)||!double.IsFinite(horizonEnd)||
            horizonEnd<horizonStart||!double.IsFinite(horizonEnd-horizonStart))return Unknown(MechanicBurstWindowIssue.InvalidClock);
        if(observedOrdinal is not (>=1 and <=8))return Unknown(MechanicBurstWindowIssue.MissingOrdinal);
        if(phaseMinimums.Length!=8||phaseMaximums.Length!=8)return Unknown(MechanicBurstWindowIssue.MissingPhases);
        if(!double.IsFinite(frameSeconds)||frameSeconds<=0)return Unknown(MechanicBurstWindowIssue.InvalidFrame);
        double withinMin=0,withinMax=0,cycleMin=0,cycleMax=0;
        for(int i=0;i<8;++i)
        {
            if(!double.IsFinite(phaseMinimums[i])||!double.IsFinite(phaseMaximums[i])||
                phaseMinimums[i]<=0||phaseMaximums[i]<phaseMinimums[i])return Unknown(MechanicBurstWindowIssue.MissingPhases);
            cycleMin+=phaseMinimums[i];cycleMax+=phaseMaximums[i];
            if(i<7){withinMin+=phaseMinimums[i];withinMax+=phaseMaximums[i];}
        }
        if(!double.IsFinite(cycleMax)||maximumWindows<=0)return Unknown(MechanicBurstWindowIssue.WindowBudgetExceeded);
        int ordinal=observedOrdinal.Value;
        double firstEarliest=lastReceipt,eighthEarliest=lastReceipt,eighthLatest=lastReceipt;
        for(int i=0;i<ordinal-1;++i)firstEarliest-=phaseMaximums[i];
        for(int i=ordinal-1;i<7;++i){eighthEarliest+=phaseMinimums[i];eighthLatest+=phaseMaximums[i];}
        double nextFirstEarliest=eighthEarliest+phaseMinimums[7];
        double nextFirstLatest=eighthLatest+phaseMaximums[7];
        if(!double.IsFinite(firstEarliest)||!double.IsFinite(nextFirstLatest)||
            !double.IsFinite(nextFirstEarliest-frameSeconds))return Unknown(MechanicBurstWindowIssue.InvalidClock);
        // Bound allocation/work before iterating; failure exposes no partial
        // clear interval. This is a query budget, not a hard motion limit.
        double futureCount=Math.Max(0,Math.Floor((horizonEnd+frameSeconds-nextFirstEarliest)/cycleMin)+1);
        if(!double.IsFinite(futureCount)||futureCount+1>maximumWindows)
            return Unknown(MechanicBurstWindowIssue.WindowBudgetExceeded);
        var result=ImmutableArray.CreateBuilder<MechanicBurstWindow>((int)futureCount+1);
        bool Add(double first,double last)
        {
            first-=frameSeconds;last+=frameSeconds;
            if(!double.IsFinite(first)||!double.IsFinite(last))return false;
            first=Math.Max(first,horizonStart);last=Math.Min(last,horizonEnd);
            if(first<=last)
            {
                // Merge overlapping candidate uncertainty envelopes.
                if(result.Count>0&&first<=result[^1].End)
                    result[^1]=result[^1] with {End=Math.Max(result[^1].End,last)};
                else result.Add(new(first,last));
            }
            return true;
        }
        if(!Add(firstEarliest,eighthLatest))return Unknown(MechanicBurstWindowIssue.InvalidClock);
        for(int i=0;i<(int)futureCount;++i)
        {
            if(!Add(nextFirstEarliest,nextFirstLatest+withinMax))return Unknown(MechanicBurstWindowIssue.InvalidClock);
            nextFirstEarliest+=cycleMin;nextFirstLatest+=cycleMax;
        }
        return new(true,result.ToImmutable(),MechanicBurstWindowIssue.None);
    }
}
