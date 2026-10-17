using System.Text.Json;

namespace MrBean.FallGuys.Mechanics;

// Linked receipt clocks for the three pendulums in one physical row. This
// does not merge the six independently assigned opening batches or certify
// server activation. Contradictions remove the grouped refinement.
internal sealed class HitPendulumClockGroups
{
    private sealed record Member(float X,SupportWindow[] Offsets);
    private sealed record Definition(string Id,float Z,double Minimum,double Maximum,double Error,SupportWindow Opening,Member[] Members);
    private sealed class Row(Definition definition)
    {
        public readonly Definition Definition=definition;
        public PeriodicReceiptClock Clock=new(definition.Minimum,definition.Maximum,0,1);
        public readonly Dictionary<float,uint> Owners=[];
        public bool Disabled;
        public int? Batch;
    }
    private readonly Dictionary<float,Row> rows;
    private readonly bool sharedBatches;
    private PeriodicReceiptClock common;
    private bool commonDisabled;
    private double? release;
    private HitPendulumClockGroups(IEnumerable<Definition> definitions,bool sharedBatches=false)
    {
        rows=definitions.ToDictionary(d=>d.Z,d=>new Row(d));this.sharedBatches=sharedBatches;
        common=new(rows.Values.Max(r=>r.Definition.Minimum),rows.Values.Min(r=>r.Definition.Maximum),0,1);
    }
    public void BeginRound(double at)=>release??=at;
    public static HitPendulumClockGroups Read(Stream stream)
    {
        using var document=JsonDocument.Parse(stream);
        if(document.RootElement.GetProperty("schemaVersion").GetInt32()!=1)throw new InvalidDataException("Unknown Hit linked-clock format.");
        var definitions=document.RootElement.GetProperty("groups").EnumerateArray().Select(g=>new Definition(
            g.GetProperty("id").GetString()!,g.GetProperty("z").GetSingle(),
            g.GetProperty("periodMinimum").GetDouble(),g.GetProperty("periodMaximum").GetDouble(),g.GetProperty("receiptError").GetDouble(),
            g.TryGetProperty("opening",out var opening)?new(opening[0].GetDouble(),opening[1].GetDouble()):new(0,0),
            g.GetProperty("members").EnumerateArray().Select(m=>new Member(m.GetProperty("x").GetSingle(),
                m.GetProperty("offsets").EnumerateArray().Select(o=>new SupportWindow(o[0].GetDouble(),o[1].GetDouble())).ToArray())).ToArray())).ToArray();
        if(definitions.Length!=6||definitions.Any(d=>d.Members.Length!=3||d.Minimum<=0||d.Maximum<d.Minimum||
            !double.IsFinite(d.Maximum)||!double.IsFinite(d.Error)||d.Error<0||
            d.Members.Any(m=>m.Offsets.Length!=2||m.Offsets.Any(o=>!double.IsFinite(o.Start)||!double.IsFinite(o.End)||o.Start>o.End))))
            throw new InvalidDataException("Incomplete Hit linked-clock definitions.");
        bool shared=document.RootElement.TryGetProperty("sharedBatches",out var mode)&&mode.GetBoolean();
        if(shared&&(definitions.Max(d=>d.Minimum)>definitions.Min(d=>d.Maximum)||
            document.RootElement.GetProperty("batchSpacing").GetDouble()!=3))throw new InvalidDataException("Unsupported common Hit batch clock.");
        return new(definitions,shared);
    }
    public HitPendulumClockGroups Copy()
    {
        var copy=new HitPendulumClockGroups(rows.Values.Select(r=>r.Definition),sharedBatches)
            {common=common.Copy(),commonDisabled=commonDisabled,release=release};
        foreach(var pair in rows)
        {
            var target=copy.rows[pair.Key];target.Clock=pair.Value.Clock.Copy();target.Disabled=pair.Value.Disabled;target.Batch=pair.Value.Batch;
            foreach(var owner in pair.Value.Owners)target.Owners.Add(owner.Key,owner.Value);
        }
        return copy;
    }
    public void Reset()
    {
        foreach(var row in rows.Values)
        {row.Clock=new(row.Definition.Minimum,row.Definition.Maximum,0,1);row.Owners.Clear();row.Disabled=false;row.Batch=null;}
        common=new(rows.Values.Max(r=>r.Definition.Minimum),rows.Values.Min(r=>r.Definition.Maximum),0,1);commonDisabled=false;release=null;
    }
    public void Invalidate(HitSourceProfile profile)
    {if(rows.TryGetValue(profile.SlotZ,out var row))row.Disabled=true;commonDisabled=true;}
    public void Observe(HitSourceProfile profile,uint entity,int phase,int ordinal,double at,bool first,bool contiguous)
    {
        if(!rows.TryGetValue(profile.SlotZ,out var row)||row.Disabled)return;
        var member=Array.Find(row.Definition.Members,m=>m.X==profile.SlotX);
        if(member==null||first&&(phase!=0||ordinal!=0)||!first&&!contiguous||
            row.Owners.TryGetValue(profile.SlotX,out uint owner)&&owner!=entity)
        {row.Disabled=true;return;}
        if(first)
        {
            if(release is not { } round){row.Disabled=true;return;}
            var candidates=new List<(int Batch,int Cycle)>();
            var firstOffset=member.Offsets[0];double residual=profile.CycleAnchorError/2;
            int maximumCycle=(int)Math.Clamp((at-round)/row.Definition.Minimum,0,64);
            for(int candidateBatch=0;candidateBatch<6;++candidateBatch)
                for(int cycle=0;cycle<=maximumCycle;++cycle)
                    if(at-round>=3*candidateBatch+cycle*row.Definition.Minimum+row.Definition.Opening.Start+firstOffset.Start-residual&&
                        at-round<=3*candidateBatch+cycle*row.Definition.Maximum+row.Definition.Opening.End+firstOffset.End+residual)
                        candidates.Add((candidateBatch,cycle));
            // Receipt plus actual release can identify an opening even when
            // historical logs omitted actor samples. Test missing-cycle
            // alternatives too; do not assign the nearest six-batch template.
            if(candidates.Count!=1||candidates[0].Cycle!=0||row.Batch is { } prior&&prior!=candidates[0].Batch)
            {row.Disabled=true;return;}
            if(rows.Values.Any(r=>r!=row&&r.Batch==candidates[0].Batch))commonDisabled=true;
            row.Batch=candidates[0].Batch;
        }
        row.Owners[profile.SlotX]=entity;
        var offset=member.Offsets[phase];
        double error=profile.CycleAnchorError/2;
        row.Clock.Observe(at,ordinal,offset.Start-error,offset.End+error);
        if(!row.Clock.Valid)row.Disabled=true;
        if(!sharedBatches||commonDisabled)return;
        if(row.Batch is not { } batch)return;
        common.Observe(at,ordinal,3*batch+row.Definition.Opening.Start+offset.Start-error,
            3*batch+row.Definition.Opening.End+offset.End+error);
        if(!common.Valid)commonDisabled=true;
    }
    public SupportWindow? Predict(HitSourceProfile profile,int phase,int ordinal)
    {
        if(!rows.TryGetValue(profile.SlotZ,out var row)||row.Disabled||!row.Owners.ContainsKey(profile.SlotX)||
            row.Clock.Predict(ordinal) is not { } clock)return null;
        var offset=Array.Find(row.Definition.Members,m=>m.X==profile.SlotX)!.Offsets[phase];
        double error=profile.CycleAnchorError/2;
        var result=new SupportWindow(clock.Start+offset.Start-error,clock.End+offset.End+error);
        if(sharedBatches&&!commonDisabled&&row.Batch is { } batch&&common.Predict(ordinal) is { } shared)
        {
            double first=shared.Start+3*batch+row.Definition.Opening.Start+offset.Start-error;
            double last=shared.End+3*batch+row.Definition.Opening.End+offset.End+error;
            result=new(Math.Max(result.Start,first),Math.Min(result.End,last));
        }
        return result;
    }
    public int DisabledRows=>rows.Values.Count(r=>r.Disabled);
    public bool DisabledCommon=>commonDisabled;
}
