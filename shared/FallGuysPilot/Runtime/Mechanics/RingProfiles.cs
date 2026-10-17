using System.Numerics;
using System.Text.Json;

namespace MrBean.FallGuys.Mechanics;

// Individually reviewed scalar facts extracted from three retained captures.
// Receipt-rate limits define the supported observation family, not an exact
// native rotation law or calibrated server hit boundary.
internal static class RingProfiles
{
    internal sealed record Source(uint Action,Vector2 Slot,MechanicFootprint[] Shapes);
    internal sealed record Profile(Source[] Sources,Vector3[] Starts,float WindMinimum,float WindMaximum);
    public const int WindBins=720;
    public const float WindLength=100,WindHalfWidth=4;
    public const double ReceiptClockError=.2;
    public static readonly Vector2 Center=new(200,-125);
    public static readonly Profile Embedded=Load();
    private static Profile Load()
    {
        using var stream=typeof(RingProfiles).Assembly.GetManifestResourceStream("MrBean.FallGuys.Data.ring-poses.json")
            ??throw new InvalidDataException("Ring source poses are missing.");
        using var doc=JsonDocument.Parse(stream);var root=doc.RootElement;
        if(root.GetProperty("schemaVersion").GetInt32()!=1||root.GetProperty("course").GetString()!="Ring")
            throw new InvalidDataException("Unsupported Ring poses.");
        static Vector3 Point(JsonElement p)=>new(p[0].GetSingle(),p[1].GetSingle(),p[2].GetSingle());
        var sources=new List<Source>();
        foreach(var row in root.GetProperty("fixed").EnumerateArray())
        {
            uint action=row.GetProperty("action").GetUInt32();var p=Point(row.GetProperty("position"));var slot=new Vector2(p.X,p.Z);
            var shape=action==34805?new MechanicFootprint(MechanicFootprintKind.Circle,slot,0,6,0,0,0,0,default):
                new(MechanicFootprintKind.Cone,slot,row.GetProperty("heading").GetSingle(),8,0,0,0,MathF.PI/2,default);
            sources.Add(new(action,slot,[shape]));
        }
        foreach(var row in root.GetProperty("charges").EnumerateArray())
        {
            var shapes=new List<MechanicFootprint>();var center=Point(row.GetProperty("center"));
            foreach(var segment in row.GetProperty("segments").EnumerateArray())
            {
                var a=Point(segment.GetProperty("start"));var b=Point(segment.GetProperty("end"));
                shapes.Add(new(MechanicFootprintKind.Charge,new(a.X,a.Z),0,0,0,0,
                    1.5f+MechanicSourceBounds.PoseTolerance,0,new(b.X,b.Z)));
            }
            sources.Add(new(35409,new(center.X,center.Z),[..shapes]));
        }
        var rates=root.GetProperty("observedWindRateRange");
        return new([..sources],root.GetProperty("starts").EnumerateArray().Select(Point).ToArray(),
            MathF.BitDecrement(rates[0].GetSingle()),MathF.BitIncrement(rates[1].GetSingle()));
    }
    public static MechanicFootprint WindShape(int bin)
    {
        float angle=bin*MathF.Tau/WindBins;
        // A rotating rectangle in this bin remains within this inflated
        // fixed rectangle. The radius bounds its farthest corner.
        float error=MathF.Sqrt(WindLength*WindLength+WindHalfWidth*WindHalfWidth)*MathF.PI/WindBins;
        return new(MechanicFootprintKind.Rectangle,Center,angle,0,-error,WindLength+error,WindHalfWidth+error,0,default);
    }
}
