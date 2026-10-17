using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.IO;
using System.Numerics;
using System.Text.Json;

namespace MrBean.FallGuys.Mechanics;

internal sealed record CrownPhaseProfile(uint ActionId,int Factor,string PhaseMode,
    MechanicFootprint[] Poses,double PeriodMinimum,double PeriodMaximum,double[] Offsets,
    double First,double ClockError,double Before,int LaneIndex,double[] OpeningOffsets,int BasePhase);
internal sealed record CrownStartCase(int Index,Vector3 Position,bool Recorded);

// Finite spatial choices and ordinal clocks are separate. In particular,
// changing 34801's opening pose does not rotate its long-pause clock.
// This is the common owned input for offline enumeration and live matching.
internal sealed class CrownPhaseProfiles
{
    public int[] Radices {get;}
    public int ConfigurationCount {get;}
    public CrownPhaseProfile[] Sources {get;}
    public CrownStartCase[] Starts {get;}
    private static readonly Lazy<CrownPhaseProfiles> embedded=new(LoadEmbedded);
    public static CrownPhaseProfiles Embedded=>embedded.Value;

    private CrownPhaseProfiles(int[] radices,CrownPhaseProfile[] sources,CrownStartCase[] starts)
    {
        Radices=radices;Sources=sources;Starts=starts;ConfigurationCount=1;
        foreach(int radix in radices)ConfigurationCount=checked(ConfigurationCount*radix);
    }
    private static CrownPhaseProfiles LoadEmbedded()
    {
        using var stream=typeof(CrownPhaseProfiles).Assembly.GetManifestResourceStream("MrBean.FallGuys.Data.crown-phases.json")
            ??throw new InvalidOperationException("Crown phase profiles are missing.");
        return Read(stream);
    }
    public static CrownPhaseProfiles Read(Stream stream)
    {
        using var document=JsonDocument.Parse(stream);var root=document.RootElement;
        if(root.GetProperty("schemaVersion").GetInt32()!=1||root.GetProperty("course").GetString()!="CrownMountain")
            throw new InvalidDataException("Unsupported Crown phase profiles.");
        var radices=new List<int>();
        foreach(var item in root.GetProperty("factors").EnumerateArray())radices.Add(item.GetProperty("count").GetInt32());
        var profiles=new List<CrownPhaseProfile>();
        foreach(var item in root.GetProperty("profiles").EnumerateArray())
        {
            uint action=item.GetProperty("action").GetUInt32();var geometry=item.GetProperty("geometry");
            var poses=new List<MechanicFootprint>();
            foreach(var pose in item.GetProperty("cycle").EnumerateArray())
            {
                var p=pose[0];var origin=new Vector2(p[0].GetSingle(),p[2].GetSingle());
                float angle=pose[1].GetSingle();
                poses.Add(geometry.GetProperty("shape").GetString()=="circle"
                    ?new(MechanicFootprintKind.Circle,origin,angle,geometry.GetProperty("radius").GetSingle(),0,0,0,0,default)
                    :new(MechanicFootprintKind.Rectangle,origin,angle,0,0,geometry.GetProperty("length").GetSingle(),
                        geometry.GetProperty("halfWidth").GetSingle(),0,default));
            }
            var period=item.GetProperty("period");var offsets=new List<double>();
            foreach(var offset in item.GetProperty("offsets").EnumerateArray())offsets.Add(offset.GetDouble());
            var openings=new List<double>();
            if(item.TryGetProperty("openingOffsets",out var opening))foreach(var offset in opening.EnumerateArray())openings.Add(offset.GetDouble());
            int factor=item.GetProperty("factor").GetInt32();
            var profile=new CrownPhaseProfile(action,factor,item.GetProperty("phaseMode").GetString()!,poses.ToArray(),
                period[0].GetDouble(),period[1].GetDouble(),offsets.ToArray(),
                item.GetProperty("first").ValueKind==JsonValueKind.Number?item.GetProperty("first").GetDouble():0,
                item.GetProperty("clockError").GetDouble(),item.GetProperty("before").GetDouble(),
                item.TryGetProperty("laneIndex",out var lane)?lane.GetInt32():0,openings.ToArray(),item.GetProperty("basePhase").GetInt32());
            if(factor<0||factor>=radices.Count||radices[factor]<=0||poses.Count==0||offsets.Count!=poses.Count||
                !double.IsFinite(profile.PeriodMaximum)||profile.PeriodMinimum<=0||profile.PeriodMaximum<profile.PeriodMinimum||
                profile.ClockError<0||profile.Before<0||profile.PhaseMode is not ("pose" or "lane" or "group")||
                profile.PhaseMode=="pose"&&radices[factor]!=poses.Count||
                profile.PhaseMode!="pose"&&openings.Count!=radices[factor])throw new InvalidDataException("Invalid Crown source profile.");
            profiles.Add(profile);
        }
        var starts=new List<CrownStartCase>();
        foreach(var item in root.GetProperty("starts").EnumerateArray())
        {
            var p=item.GetProperty("position");
            starts.Add(new(item.GetProperty("index").GetInt32(),new(p[0].GetSingle(),p[1].GetSingle(),p[2].GetSingle()),
                item.GetProperty("recording").ValueKind==JsonValueKind.Number));
        }
        var result=new CrownPhaseProfiles(radices.ToArray(),profiles.ToArray(),starts.ToArray());
        if(result.ConfigurationCount!=root.GetProperty("states").GetInt32())throw new InvalidDataException("Crown state count does not match its factors.");
        return result;
    }

    public void Decode(int configuration,Span<int> choices)
    {
        if(configuration<0||configuration>=ConfigurationCount||choices.Length!=Radices.Length)throw new ArgumentOutOfRangeException(nameof(configuration));
        for(int i=0;i<Radices.Length;++i){choices[i]=configuration%Radices[i];configuration/=Radices[i];}
    }
    public int Encode(ReadOnlySpan<int> choices)
    {
        if(choices.Length!=Radices.Length)throw new ArgumentException("Wrong Crown factor count.");
        int result=0,multiplier=1;
        for(int i=0;i<choices.Length;++i)
        {
            if(choices[i]<0||choices[i]>=Radices[i])throw new ArgumentOutOfRangeException(nameof(choices));
            result+=choices[i]*multiplier;multiplier*=Radices[i];
        }
        return result;
    }
    public double Opening(CrownPhaseProfile source,int choice)
        =>source.PhaseMode=="pose"?source.First:source.OpeningOffsets[(source.LaneIndex-choice+Radices[source.Factor])%Radices[source.Factor]];
    public MechanicFootprint Pose(CrownPhaseProfile source,int choice,int ordinal)
        =>source.Poses[(ordinal+(source.PhaseMode=="pose"?choice:source.BasePhase))%source.Poses.Length];
    public (double earliest,double latest) Receipt(CrownPhaseProfile source,int choice,int ordinal)
    {
        int cycles=ordinal/source.Poses.Length,phase=ordinal%source.Poses.Length;
        double first=Opening(source,choice)+source.Offsets[phase];
        return(first+cycles*source.PeriodMinimum-source.ClockError,first+cycles*source.PeriodMaximum+source.ClockError);
    }
    public MechanicDangerSnapshot Snapshot(int configuration,double release,double now,double end,float frame)
    {
        if(!double.IsFinite(release)||now<release||end<=now||!float.IsFinite(frame)||frame<=0)throw new ArgumentException("Invalid Crown clock.");
        Span<int> choices=stackalloc int[Radices.Length];Decode(configuration,choices);
        var hazards=ImmutableArray.CreateBuilder<TemporalHazard>();
        for(int i=0;i<Sources.Length;++i)
        {
            var source=Sources[i];int choice=choices[source.Factor];
            for(int ordinal=0;;++ordinal)
            {
                var time=Receipt(source,choice,ordinal);
                double first=release+time.earliest-source.Before-frame,last=release+time.latest+frame;
                if(first>end)break;if(last<now)continue;
                hazards.Add(new(new("CrownMountain",(uint)(i+1)),source.ActionId,Pose(source,choice,ordinal),
                    Math.Max(now,first),Math.Min(end,last),now));
            }
        }
        return new(0,now,end,hazards.ToImmutable(),[],[],[],[]);
    }
}
