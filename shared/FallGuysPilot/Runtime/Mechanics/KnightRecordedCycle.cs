using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.IO;
using System.Numerics;
using System.Text.Json;

namespace MrBean.FallGuys.Mechanics;

// Compiled source geometry and period bounds are prior knowledge. Every clock
// is separately anchored by this round's actual cast/receipt. In particular no
// source ID, opening time or held-out event is embedded in a runtime clock.
internal sealed class KnightCycleProfiles
{
    internal readonly record struct Pose(Vector2 Position,float Heading);
    internal sealed record BodyChannel(float First,float Last,SupportWindow[] PhaseWindows);
    internal sealed record Profile(uint Action,Pose[] Poses,double Minimum,double Maximum,
        double OffsetMinimum,double OffsetMaximum,double Error,float? BodyRadius,BodyChannel[] BodyChannels);
    private static readonly Lazy<KnightCycleProfiles> embedded=new(Load);
    private readonly Dictionary<uint,Profile[]> profiles;
    internal readonly record struct BridgeClock(double Minimum,double Maximum,double Error,SupportWindow OnToOff,SupportWindow OffToOn);
    private readonly BridgeClock bridge;
    private KnightCycleProfiles(Dictionary<uint,Profile[]> profiles,BridgeClock bridge)
    {this.profiles=profiles;this.bridge=bridge;}
    internal static BridgeClock Bridge=>embedded.Value.bridge;
    internal static IEnumerable<Profile> All=>embedded.Value.profiles.Values.SelectMany(p=>p);
    internal static MechanicFootprint Shape(Profile profile,int pose,MechanicCatalog catalog)
    {
        if(!catalog.TryGet("KnightFever",profile.Action,out var definition))throw new InvalidDataException("Missing Knight definition.");
        var p=profile.Poses[pose];
        if(MechanicFootprints.Build(new(definition),new(){Position=new(p.Position.X,0,p.Position.Y),Rotation=p.Heading},out var shape)
            !=MechanicFootprintResult.Candidate)throw new InvalidDataException("Knight profile has no fixed spatial candidate.");
        return shape with{Rotation=p.Heading};
    }
    internal static MechanicFootprint BodyShape(Profile profile,BodyChannel channel)
        =>new(MechanicFootprintKind.Charge,
            Vector2.Lerp(profile.Poses[0].Position,profile.Poses[1].Position,channel.First),
            0,0,0,0,profile.BodyRadius!.Value+MechanicSourceBounds.PoseTolerance,0,
            Vector2.Lerp(profile.Poses[0].Position,profile.Poses[1].Position,channel.Last));
    public static Profile? Match(uint action,MechanicFootprint shape)
    {
        if(!embedded.Value.profiles.TryGetValue(action,out var candidates))return null;
        foreach(var profile in candidates)
            foreach(var pose in profile.Poses)if(Matches(pose,shape))return profile;
        return null;
    }
    internal static bool Matches(Pose pose,MechanicFootprint shape)
        =>Vector2.DistanceSquared(pose.Position,shape.Origin)<=MechanicSourceBounds.PoseTolerance*MechanicSourceBounds.PoseTolerance&&
          (shape.Kind==MechanicFootprintKind.Circle||MathF.Abs(MathF.IEEERemainder(pose.Heading-shape.Rotation,MathF.Tau))<.001f);
    private static KnightCycleProfiles Load()
    {
        using var stream=typeof(KnightCycleProfiles).Assembly.GetManifestResourceStream("MrBean.FallGuys.Data.knight-cycles.json")
            ??throw new InvalidDataException("Knight cycle profiles are missing.");
        using var document=JsonDocument.Parse(stream);var root=document.RootElement;
        if(root.GetProperty("schemaVersion").GetInt32()!=1||root.GetProperty("course").GetString()!="KnightFever")
            throw new InvalidDataException("Unsupported Knight cycle profiles.");
        var groups=new Dictionary<uint,List<Profile>>();int count=0;
        foreach(var row in root.GetProperty("profiles").EnumerateArray())
        {
            uint action=row.GetProperty("actionId").GetUInt32();var poses=new List<Pose>();
            foreach(var p in row.GetProperty("poses").EnumerateArray())
                poses.Add(new(new(p.GetProperty("x").GetSingle(),p.GetProperty("z").GetSingle()),p.GetProperty("heading").GetSingle()));
            var period=row.GetProperty("period");var offset=row.GetProperty("phaseOffset");
            float? bodyRadius=null;var bodyChannels=new List<BodyChannel>();
            if(row.TryGetProperty("body",out var body))
            {
                bodyRadius=body.GetProperty("radius").GetSingle();
                foreach(var channel in body.GetProperty("channels").EnumerateArray())
                {
                    var fraction=channel.GetProperty("fraction");
                    var windows=channel.GetProperty("phaseWindows").EnumerateArray()
                        .Select(w=>new SupportWindow(w[0].GetDouble(),w[1].GetDouble())).ToArray();
                    float first=fraction[0].GetSingle(),last=fraction[1].GetSingle();
                    if(first<0||last<=first||last>1||windows.Length!=2||
                        windows.Any(w=>w.Start<0||!double.IsFinite(w.End)||w.End<w.Start||w.End>period[1].GetDouble()))
                        throw new InvalidDataException("Invalid Knight body channel.");
                    bodyChannels.Add(new(first,last,windows));
                }
            }
            var profile=new Profile(action,[..poses],period[0].GetDouble(),period[1].GetDouble(),
                offset[0].GetDouble(),offset[1].GetDouble(),row.GetProperty("clockError").GetDouble(),bodyRadius,[..bodyChannels]);
            if(action==34717&&(!(bodyRadius>0)||!float.IsFinite(bodyRadius.Value)||bodyChannels.Count==0||
                bodyChannels[0].First!=0||bodyChannels[^1].Last!=1||
                bodyChannels.Zip(bodyChannels.Skip(1),(a,b)=>a.Last!=b.First).Any(gap=>gap)))
                throw new InvalidDataException("Knight spike body coverage is incomplete.");
            if(action is not (34716 or 34717 or 34718)||poses.Count!=2||!(profile.Minimum>0)||
                !double.IsFinite(profile.Maximum)||profile.Maximum<profile.Minimum||
                !(profile.OffsetMinimum>0)||profile.OffsetMaximum<profile.OffsetMinimum||profile.OffsetMaximum>=profile.Maximum||
                !(profile.Error>0)||!double.IsFinite(profile.Error))throw new InvalidDataException("Invalid Knight source cycle.");
            foreach(var pose in poses)if(!float.IsFinite(pose.Position.X)||!float.IsFinite(pose.Position.Y)||!float.IsFinite(pose.Heading))
                throw new InvalidDataException("Invalid Knight source pose.");
            if(!groups.TryGetValue(action,out var group))groups[action]=group=[];
            group.Add(profile);++count;
        }
        if(count!=64)throw new InvalidDataException("Incomplete Knight source cycles.");
        var result=new Dictionary<uint,Profile[]>();foreach(var pair in groups)result[pair.Key]=[..pair.Value];
        var bridgeRow=root.GetProperty("bridgeClock");var bridgePeriod=bridgeRow.GetProperty("period");
        var onToOff=bridgeRow.GetProperty("onToOff");var offToOn=bridgeRow.GetProperty("offToOn");
        var bridge=new BridgeClock(bridgePeriod[0].GetDouble(),bridgePeriod[1].GetDouble(),bridgeRow.GetProperty("clockError").GetDouble(),
            new(onToOff[0].GetDouble(),onToOff[1].GetDouble()),new(offToOn[0].GetDouble(),offToOn[1].GetDouble()));
        if(!(bridge.Minimum>0)||!double.IsFinite(bridge.Maximum)||bridge.Maximum<bridge.Minimum||
            !(bridge.Error>0)||!double.IsFinite(bridge.Error))throw new InvalidDataException("Invalid Knight bridge clock.");
        foreach(var offset in new[]{bridge.OnToOff,bridge.OffToOn})
            if(!(offset.Start>0)||!double.IsFinite(offset.End)||offset.End<offset.Start||offset.End>=bridge.Maximum)
                throw new InvalidDataException("Invalid Knight bridge phase offset.");
        return new(result,bridge);
    }
}

internal sealed class KnightRecordedCycle
{
    private readonly KnightCycleProfiles.Profile profile;
    private readonly MechanicFootprint[] shapes;
    private readonly MechanicFootprint[] bodyShapes;
    private readonly PeriodicReceiptClock[] clocks;
    private int phase=-1;
    private double last=double.NegativeInfinity,duration;
    private bool fault;
    public KnightRecordedCycle(KnightCycleProfiles.Profile profile,MechanicFootprint shape)
    {
        this.profile=profile;shapes=new MechanicFootprint[2];clocks=new PeriodicReceiptClock[2];
        bodyShapes=profile.BodyChannels.Select(c=>KnightCycleProfiles.BodyShape(profile,c)).ToArray();
        for(int i=0;i<2;++i)
        {
            shapes[i]=shape with{Origin=profile.Poses[i].Position,Rotation=profile.Poses[i].Heading};
            // One observation is sufficient only because this source has an
            // independently compiled period bound. Generic clocks retain 3.
            clocks[i]=new(profile.Minimum,profile.Maximum,profile.Error,1);
        }
    }
    public void Observe(MechanicFootprint shape,double at,double castDuration=0)
    {
        if(fault)return;
        int next=-1;for(int i=0;i<2;++i)if(KnightCycleProfiles.Matches(profile.Poses[i],shape)){next=i;break;}
        if(next<0||!double.IsFinite(at)||at<=last||next==phase){fault=true;return;}
        phase=next;last=at;duration=castDuration;clocks[next].Observe(at);
        if(!clocks[next].Valid)fault=true;
    }
    private KnightRecordedCycle(KnightRecordedCycle source)
    {
        profile=source.profile;shapes=source.shapes;bodyShapes=source.bodyShapes;clocks=source.clocks.Select(c=>c.Copy()).ToArray();
        phase=source.phase;last=source.last;duration=source.duration;fault=source.fault;
    }
    public KnightRecordedCycle Copy()=>new(this);
    public void Append(MechanicActionKey key,MechanicActionState action,double now,double end,double frame,
        ImmutableArray<MechanicDanger>.Builder dangers,ImmutableArray<MechanicForecastDiagnostic>.Builder issues)
    {
        void Add(MechanicFootprint shape,double from,double until,bool unknown=false)
        {
            from=Math.Max(now,from);until=Math.Min(end,until);if(until<from)return;
            dangers.Add(new(new(key.Source,key.ActionId,shape,from,until,now),
                unknown?MechanicForecastCertainty.Unknown:MechanicForecastCertainty.Candidate,
                unknown?MechanicDangerPolicy.CastCycleUninitialized:MechanicDangerPolicy.RecordedCycleWindow,
                MechanicDangerEnd.CallerHorizon,shape,last,last,null));
        }
        if(fault||phase<0||now>last+profile.Maximum+2*profile.Error)
        {
            foreach(var shape in shapes)Add(shape,now,end,true);
            foreach(var shape in bodyShapes)Add(shape,now,end,true);
            issues.Add(new(key,MechanicForecastIssue.MissingTiming,"The source contradicted or outlived its recorded cycle; retain both attack poses."));
            return;
        }
        var definition=action.Definition;
        double before=definition.ReceiptWindow?.Before??(profile.Action==34718?definition.PoseReceiptBound??0:0);
        double after=definition.ReceiptWindow?.After??(profile.Action==34717?duration+(definition.CastReceiptExcessBound??0):0);
        before+=profile.Action==34718?frame:0;after+=frame;
        for(int p=0;p<2;++p)
        {
            bool own=clocks[p].Count>0;int reference=own?p:1-p;var clock=clocks[reference];
            double offsetLow=own?0:p==1?profile.OffsetMinimum:-profile.OffsetMaximum;
            double offsetHigh=own?0:p==1?profile.OffsetMaximum:-profile.OffsetMinimum;
            int first=Math.Max(0,clock.Count-1),limit=first+(int)Math.Ceiling((end-now)/profile.Minimum)+3;
            for(int n=first;n<=limit;++n)
            {
                var predicted=clock.Predict(n)!.Value;
                double begin=predicted.Start+offsetLow-before,finish=predicted.End+offsetHigh+after;
                if(begin>end)break;
                Add(shapes[p],begin,finish);
                // A cast's endpoint AoE does not describe the translating body.
                // The compiler has intersected these finite swept channels;
                // live code binds only their source-relative clock windows.
                for(int channel=0;channel<bodyShapes.Length;++channel)
                {
                    var span=profile.BodyChannels[channel].PhaseWindows[p];
                    Add(bodyShapes[channel],predicted.Start+offsetLow+span.Start-frame,
                        predicted.End+offsetHigh+span.End+frame);
                }
            }
        }
        if(profile.Action==34717&&action.PendingCast is { } pending&&now>pending.Seconds+duration+(definition.CastReceiptExcessBound??0)+frame)
        {
            Add(shapes[phase],now,end,true);
            issues.Add(new(key,MechanicForecastIssue.PendingCastPastDeclaredEnd,"An overdue cast remains blocked until its receipt or explicit cancellation."));
        }
    }
}
