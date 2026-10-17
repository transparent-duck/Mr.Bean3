using System.Numerics;
using System.Text.Json;

namespace MrBean.FallGuys.Mechanics;

internal static class CrystalProfiles
{
    internal sealed record Phase(MechanicFootprint Shape,double Minimum,double Maximum);
    internal sealed record Source(uint Action,Vector2 Slot,Phase[] Phases,double Before,double After,double Error);
    internal sealed record Goal(ushort Carry,uint LayoutId,Vector3 Position);
    internal sealed record Profile(Source[] Sources,Vector3[] Starts,Goal[] Goals);
    public static readonly Profile Embedded=Load();
    private static Profile Load()
    {
        using var stream=typeof(CrystalProfiles).Assembly.GetManifestResourceStream("MrBean.FallGuys.Data.crystal-profiles.json")
            ??throw new InvalidDataException("Crystal source profiles are missing.");
        using var doc=JsonDocument.Parse(stream);var root=doc.RootElement;
        if(root.GetProperty("schemaVersion").GetInt32()!=1||root.GetProperty("course").GetString()!="Crystal")
            throw new InvalidDataException("Unsupported Crystal profile.");
        static Vector2 XZ(JsonElement p)=>new(p[0].GetSingle(),p[1].GetSingle());
        static Vector3 XYZ(JsonElement p)=>new(p[0].GetSingle(),p[1].GetSingle(),p[2].GetSingle());
        var catalog=MechanicCatalog.LoadEmbedded();var sources=new List<Source>();
        foreach(var row in root.GetProperty("sources").EnumerateArray())
        {
            uint action=row.GetProperty("action").GetUInt32();
            if(!catalog.TryGet("CrystalDelivery",action,out var definition))throw new InvalidDataException("Crystal action is unknown.");
            var phases=new List<Phase>();
            foreach(var p in row.GetProperty("phases").EnumerateArray())
            {
                var origin=XZ(p.GetProperty("origin"));float heading=p.GetProperty("heading").GetSingle();
                var shape=action==34774?new MechanicFootprint(MechanicFootprintKind.Charge,origin,0,0,0,0,
                    1.5f+MechanicSourceBounds.PoseTolerance,0,XZ(p.GetProperty("end"))):
                    new(MechanicFootprintKind.Rectangle,origin,heading,0,action==34716?-1.5f:0,
                        action==34716?1.5f:action==34799?4:31,action==34716?1:action==34799?2:6,0,default);
                var period=p.GetProperty("period");double low=period[0].GetDouble(),high=period[1].GetDouble();
                if(low<=0||!double.IsFinite(high)||high<low)throw new InvalidDataException("Invalid Crystal phase interval.");
                phases.Add(new(shape,low,high));
            }
            sources.Add(new(action,XZ(row.GetProperty("slot")),[..phases],definition.ReceiptWindow?.Before??0,
                definition.ReceiptWindow?.After??definition.CastReceiptExcessBound??0,definition.ReceiptClockError));
        }
        return new([..sources],root.GetProperty("starts").EnumerateArray().Select(XYZ).ToArray(),
            root.GetProperty("goals").EnumerateArray().Select(g=>new Goal(g.GetProperty("carry").GetUInt16(),
                g.GetProperty("id").GetUInt32(),XYZ(g.GetProperty("position")))).ToArray());
    }
}
