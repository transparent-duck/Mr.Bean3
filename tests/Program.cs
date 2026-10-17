using System.Net;
using System.Text;
using System.Text.Json;
using MrBean.FallGuys;
using MrBean.Modules.FallGuysPilot;
using Mr.Bean3;

var now = new DateTimeOffset(2026,10,17,10,0,0,TimeSpan.Zero);
var world = new WorldIdentity(101,"陆行鸟",1042,"测试世界");
var actor = new CharacterIdentity("18446744073709551615","測試角色",world,world);
RunSummaryBuilder New(bool partial=false) => new(actor,"7.55.3.0",new string('a',64),partial,now);
PilotObservation Event(string kind,CourseId course,long scene,double seconds,int hits=0,bool manual=false)
    => new(kind,scene,course,now.AddSeconds(seconds),seconds,true,kind is "qualified" or "winner",
        kind=="eliminated",manual,kind is "qualified" or "winner"?1u:null,hits,2,1);
void Assert(bool value,string name) { if(!value) throw new Exception(name); }
int passed=0;

var full=New();
Assert(full.Initial.Revision==0 && full.Initial.Stages.Length==0,"start is one empty duty summary");
full.Observe(Event("qualified",CourseId.Hit,1,70));
full.Observe(Event("qualified",CourseId.Crystal,2,55));
Assert(full.Observe(Event("winner",CourseId.Crown,3,60))=="won","winner evidence");
var won=full.Finish("won",now.AddMinutes(4));
Assert(won.Stages.Length==3 && won.Stages.All(s=>s.Outcome is "qualified" or "won") && won.Revision==1,"three stages in one final summary"); ++passed;

var checkpoint=New();
checkpoint.Observe(Event("scene-end",CourseId.Ring,1,30,2));
checkpoint.Observe(Event("start",CourseId.Ring,2,0));
checkpoint.Observe(Event("eliminated",CourseId.Ring,2,20,1));
var lost=checkpoint.Finish("eliminated",now.AddSeconds(50));
Assert(lost.Stages.Length==1 && lost.Stages[0].Hits==3 && lost.Stages[0].DurationMs==50000,"checkpoint accumulation"); ++passed;

var frozen=New();
frozen.Observe(Event("qualified",CourseId.Hit,1,70,1));
frozen.Observe(Event("scene-end",CourseId.Hit,1,95,1));
Assert(frozen.Finish("unknown",now.AddSeconds(100)).Stages[0].DurationMs==70000,"qualification duration excludes waiting"); ++passed;

var manual=New();
Assert(manual.Observe(Event("yield",CourseId.Crystal,1,10,0,true))=="cancelled","manual outcome");
var cancelled=manual.Finish("cancelled",now.AddSeconds(10));
Assert(cancelled.ManualIntervention && cancelled.Stages[0].Outcome=="cancelled","manual run marked");
Assert(manual.Observe(Event("winner",CourseId.Crown,2,60))==null,"manual continuation cannot change final result"); ++passed;

var partial=New(true).Finish("unknown",now);
Assert(partial.PartialDuty,"partial duty excluded from automatic metrics"); ++passed;

var repeat=New();
repeat.Observe(Event("scene-end",CourseId.Hit,1,30,2));
repeat.Observe(Event("scene-end",CourseId.Hit,1,30,2));
Assert(repeat.Finish("unknown",now.AddSeconds(30)).Stages[0].Hits==2,"same scene does not double count"); ++passed;

var interrupted=New();
interrupted.Observe(Event("release",CourseId.Hit,1,0));
Assert(interrupted.Finish("disconnected",now.AddSeconds(5)).Stages[0].Outcome=="unknown","no inferred qualification"); ++passed;

var json=JsonSerializer.Serialize(won,UsageOutbox.Json);
using(var document=JsonDocument.Parse(json)) {
    Assert(document.RootElement.GetProperty("character").GetProperty("cid").GetString()==actor.Cid,"CID stays a lossless string");
    Assert(document.RootElement.GetProperty("stages").GetArrayLength()==3,"one nested stage array");
} ++passed;

var path=Path.Combine(Path.GetTempPath(),"mrbean3-outbox-test-"+Guid.NewGuid().ToString("N"));
try {
    using(var box=new UsageOutbox(path,new Uri("https://example.invalid/v1/runs"),_=>{},new OfflineHandler())) {
        box.Enqueue(full.Initial); box.Enqueue(won);
    }
    Assert(Directory.GetFiles(path,"*.json").Length==2,"shutdown persists both revisions without requiring network"); ++passed;
    var handler=new AckHandler();
    using(var box=new UsageOutbox(path,new Uri("https://example.invalid/v1/runs"),_=>{},handler)) {
        for(int i=0;i<100 && Directory.GetFiles(path,"*.json").Length!=0;++i) await Task.Delay(50);
        Assert(Directory.GetFiles(path,"*.json").Length==0 && handler.Count==2,"restart retries and acknowledges both revisions");
    } ++passed;
} finally { if(Directory.Exists(path)) Directory.Delete(path,true); }
Console.WriteLine($"Mr.Bean3 summary/outbox tests: {passed} passed");

sealed class OfflineHandler:HttpMessageHandler {
    protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request,CancellationToken token)
        => Task.FromException<HttpResponseMessage>(new HttpRequestException("offline test"));
}
sealed class AckHandler:HttpMessageHandler {
    public int Count;
    protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request,CancellationToken token) {
        using var body=JsonDocument.Parse(await request.Content!.ReadAsStringAsync(token));
        ++Count;
        return new HttpResponseMessage(HttpStatusCode.OK) {Content=new StringContent(JsonSerializer.Serialize(new {
            accepted=true,runId=body.RootElement.GetProperty("runId").GetString(),revision=body.RootElement.GetProperty("revision").GetInt32()
        }),Encoding.UTF8,"application/json")};
    }
}
