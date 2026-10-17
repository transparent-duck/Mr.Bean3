using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Numerics;
using System.Text.Json;

namespace MrBean.FallGuys.Mechanics;

internal readonly record struct MechanicSourceBounds(Vector2 Min,Vector2 Max)
{
    // ActorMove's position quantization is source-pose uncertainty. It is not
    // recipient radius or an inferred physical body of the mechanism.
    public const float PoseTolerance=2*2000f/65535;
    public bool Contains(Vector3 point)=>point.X>=Min.X-PoseTolerance&&point.X<=Max.X+PoseTolerance&&
        point.Z>=Min.Y-PoseTolerance&&point.Z<=Max.Y+PoseTolerance;
    public MechanicFootprint Envelope(MechanicFootprint shape,bool rotating)
    {
        if(rotating)
        {
            var circle=MechanicFootprints.RotationEnvelope(shape);
            return circle with{Origin=(Min+Max)/2,Radius=circle.Radius+(Max-Min).Length()/2+PoseTolerance};
        }
        if(shape.Kind!=MechanicFootprintKind.Circle)throw new InvalidDataException("Source motion bounds require a circle candidate.");
        var half=(Max-Min)/2+new Vector2(shape.Radius+PoseTolerance);
        return new(MechanicFootprintKind.Rectangle,(Min+Max)/2,0,0,-half.Y,half.Y,half.X,0,default){TargetRadius=shape.TargetRadius};
    }
}

internal readonly record struct MechanicCastPeriod(double Minimum,double Maximum);
internal readonly record struct MechanicReceiptWindow(double Before,double After);

internal sealed record MechanicDefinition(string Course,uint ActionId,string Label,
    string Role,char TimingKind,int CastType,float EffectRange,float AxisModifier,
    JsonElement Geometry,JsonElement Timing)
{
    public int CastOriginCycle {get;init;}
    public double? CastReceiptExcessBound {get;init;}
    public double CastCompletionTail {get;init;}
    public bool CastEndActivation {get;init;}
    public MechanicCastPeriod? InitialCastPeriod {get;init;}
    public bool FixedOriginRotationEnvelope {get;init;}
    public int PoseCycle {get;init;}
    public double? PoseReceiptBound {get;init;}
    public bool SelfCastOrigin {get;init;}
    public MechanicReceiptWindow? ReceiptWindow {get;init;}
    public int ReceiptCycle {get;init;}
    public double ReceiptClockError {get;init;}=.2;
}

// Parse once. Evidence and per-source families remain owned; no action-wide
// midpoint timer is created from the descriptive ranges in the catalog.
internal sealed class MechanicCatalog
{
    private readonly Dictionary<(string,uint),MechanicDefinition> entries=[];
    private readonly HashSet<(string,uint)> sources=[];
    private readonly Dictionary<(string,uint),List<(MechanicDefinition Definition,MechanicSourceBounds Bounds)>> bindings=[];
    public IEnumerable<MechanicDefinition> Entries=>entries.Values;
    public int Count=>entries.Count;
    public bool TryGet(string course,uint action,out MechanicDefinition definition)
        =>entries.TryGetValue((course,action),out definition!);
    public bool AcceptsSource(string course,uint baseId)=>sources.Contains((course,baseId));
    public bool TryBind(string course,uint baseId,Vector3 position,out MechanicDefinition definition,out MechanicSourceBounds bounds)
    {
        if(bindings.TryGetValue((course,baseId),out var candidates))foreach(var candidate in candidates)
            if(candidate.Bounds.Contains(position)){definition=candidate.Definition;bounds=candidate.Bounds;return true;}
        definition=null!;bounds=default;return false;
    }
    private static readonly Lazy<MechanicCatalog> embedded=new(ReadEmbedded);
    public static MechanicCatalog LoadEmbedded()=>embedded.Value;
    private static MechanicCatalog ReadEmbedded()
    {
        using var stream=typeof(MechanicCatalog).Assembly.GetManifestResourceStream("MrBean.FallGuys.Data.recorded-mechanics.json")
            ??throw new InvalidOperationException("Fall Guys mechanics catalog is missing.");
        return Read(stream);
    }
    public static MechanicCatalog Read(Stream stream)
    {
        using var json=JsonDocument.Parse(stream);
        if(json.RootElement.GetProperty("schemaVersion").GetInt32()!=1)
            throw new InvalidDataException("Unsupported Fall Guys mechanics catalog schema.");
        var result=new MechanicCatalog();
        foreach(var item in json.RootElement.GetProperty("entries").EnumerateArray())
        {
            var definition=new MechanicDefinition(item.GetProperty("course").GetString()!,
                item.GetProperty("actionId").GetUInt32(),item.GetProperty("label").GetString()!,
                item.GetProperty("receiptRole").GetString()!,item.GetProperty("timingKind").GetString()![0],
                item.GetProperty("castType").GetInt32(),item.GetProperty("effectRange").GetSingle(),
                item.GetProperty("xAxisModifier").GetSingle(),
                item.GetProperty("geometryCandidate").Clone(),item.GetProperty("timing").Clone());
            if(definition.Geometry.TryGetProperty("origin",out var origin))
            {
                if(origin.GetString()!="self-cast-location"||definition.TimingKind!='C')
                    throw new InvalidDataException("Invalid cast-origin policy.");
                definition=definition with{SelfCastOrigin=true};
            }
            if(item.TryGetProperty("castPrediction",out var prediction))
            {
                int phases=prediction.GetProperty("originCycle").GetInt32();
                if(phases is <1 or >16||definition.TimingKind!='C')throw new InvalidDataException("Invalid cast-pose cycle.");
                double bound=prediction.GetProperty("receiptExcessUpperBound").GetDouble();
                if(!double.IsFinite(bound)||bound<0)throw new InvalidDataException("Invalid cast receipt bound.");
                definition=definition with{CastOriginCycle=phases,CastReceiptExcessBound=bound};
                if(prediction.TryGetProperty("completionTail",out var completion))
                {
                    double tail=completion.GetDouble();
                    if(!double.IsFinite(tail)||tail<0)throw new InvalidDataException("Invalid cast completion tail.");
                    definition=definition with{CastCompletionTail=tail};
                }
                if(prediction.TryGetProperty("activation",out var activation))
                {
                    if(activation.GetString()!="cast-end")throw new InvalidDataException("Unknown cast activation policy.");
                    definition=definition with{CastEndActivation=true};
                }
                if(prediction.TryGetProperty("initialPeriod",out var period))
                {
                    double minimum=period[0].GetDouble(),maximum=period[1].GetDouble();
                    if(phases!=1||!double.IsFinite(minimum)||!double.IsFinite(maximum)||minimum<=0||maximum<minimum)
                        throw new InvalidDataException("Invalid initial cast period bounds.");
                    definition=definition with{InitialCastPeriod=new(minimum,maximum)};
                }
            }
            if(item.TryGetProperty("spatialPrediction",out var spatial))
            {
                if(spatial.GetString()!="fixed-origin-full-rotation"||definition.TimingKind!='M')
                    throw new InvalidDataException("Invalid spatial prediction policy.");
                definition=definition with{FixedOriginRotationEnvelope=true};
            }
            if(item.TryGetProperty("posePrediction",out var pose))
            {
                int phases=pose.GetProperty("cycle").GetInt32();
                double bound=pose.GetProperty("receiptUpperBound").GetDouble();
                if(phases is <1 or >16||!definition.FixedOriginRotationEnvelope||!double.IsFinite(bound)||bound<0)
                    throw new InvalidDataException("Invalid source-pose cycle.");
                definition=definition with{PoseCycle=phases,PoseReceiptBound=bound};
            }
            if(item.TryGetProperty("receiptPrediction",out var receipt))
            {
                int phases=receipt.GetProperty("cycle").GetInt32();
                if(phases is <1 or >16)throw new InvalidDataException("Invalid receipt-pose cycle.");
                definition=definition with{ReceiptCycle=phases};
                if(receipt.TryGetProperty("clockError",out var clockError))
                {
                    double error=clockError.GetDouble();
                    if(!double.IsFinite(error)||error<=0)throw new InvalidDataException("Invalid receipt clock error.");
                    definition=definition with{ReceiptClockError=error};
                }
            }
            if(item.TryGetProperty("receiptWindow",out var window))
            {
                double before=window.GetProperty("before").GetDouble(),after=window.GetProperty("after").GetDouble();
                if(definition.ReceiptCycle==0||!double.IsFinite(before)||!double.IsFinite(after)||before<0||after<0)
                    throw new InvalidDataException("Invalid receipt-anchored attack window.");
                definition=definition with{ReceiptWindow=new(before,after)};
            }
            if(definition.ReceiptCycle>0&&definition.ReceiptWindow==null)
                throw new InvalidDataException("Receipt-pose prediction needs an explicit attack window.");
            if(!result.entries.TryAdd((definition.Course,definition.ActionId),definition))
                throw new InvalidDataException("Duplicate course/action in Fall Guys mechanics catalog.");
            if(item.TryGetProperty("actorBaseIds",out var baseIds))
                foreach(var baseId in baseIds.EnumerateArray())
                {
                    var key=(definition.Course,baseId.GetUInt32());result.sources.Add(key);
                    if(!item.TryGetProperty("sourceBounds",out var sourceBounds))continue;
                    if(!result.bindings.TryGetValue(key,out var boundEntries))result.bindings.Add(key,boundEntries=[]);
                    foreach(var row in sourceBounds.EnumerateArray())
                    {
                        var bound=new MechanicSourceBounds(new(row[0].GetSingle(),row[1].GetSingle()),new(row[2].GetSingle(),row[3].GetSingle()));
                        if(!float.IsFinite(bound.Min.X)||!float.IsFinite(bound.Min.Y)||!float.IsFinite(bound.Max.X)||!float.IsFinite(bound.Max.Y)||
                            bound.Max.X<bound.Min.X||bound.Max.Y<bound.Min.Y)throw new InvalidDataException("Invalid source motion bound.");
                        boundEntries.Add((definition,bound));
                    }
                }
        }
        return result;
    }
}
