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
var statisticsPath=Path.Combine(Path.GetTempPath(),"mrbean3-statistics-test-"+Guid.NewGuid().ToString("N"));
try {
    void Error(Exception error) => throw new Exception("Statistics persistence failed",error);
    var totals=new LocalStatistics(statisticsPath,Error);
    Assert(totals.TotalRuns==0 && totals.WinRate==0,"empty statistics");
    totals.BeginSession(false);
    totals.UpdateTerritory(1197,false);
    totals.Stop();
    Assert(totals.TotalRuns==0,"stopping in lobby does not count a duty"); ++passed;

    totals.BeginSession(false);
    totals.UpdateTerritory(1165,true);
    Assert(totals.TotalRuns==0,"loading does not start a duty");
    totals.UpdateTerritory(1165,false);
    totals.Observe(Event("qualified",CourseId.Hit,1,70));
    totals.UpdateTerritory(1165,false);
    totals.Observe(Event("qualified",CourseId.Crystal,2,55));
    totals.Observe(Event("winner",CourseId.Crown,3,60));
    totals.Observe(Event("winner",CourseId.Crown,3,60));
    totals.Stop();
    totals.Stop();
    Assert(totals.TotalRuns==1 && totals.Wins==1 && totals.Interrupted==0,"three stages count once; terminal and stop are idempotent"); ++passed;

    totals=new LocalStatistics(statisticsPath,Error);
    Assert(totals.TotalRuns==1 && totals.Wins==1,"totals persist across reloads");
    totals.BeginSession(false);
    totals.UpdateTerritory(1165,false);
    totals.Observe(Event("eliminated",CourseId.Ring,2,30));
    totals.UpdateTerritory(1197,false);
    totals.UpdateTerritory(1165,false);
    totals.Observe(Event("yield",CourseId.Hit,1,10,manual:true));
    totals.Observe(Event("winner",CourseId.Crown,3,60));
    totals.Stop();
    Assert(totals.TotalRuns==3 && totals.Wins==1 && totals.Failures==1 && totals.Interrupted==1 && totals.WinRate==.5,
        "loop entries accumulate; manual takeover is excluded from win rate"); ++passed;

    totals.BeginSession(false);
    totals.UpdateTerritory(1165,false);
    totals.UpdateTerritory(1197,true);
    totals.UpdateTerritory(1165,false);
    Assert(totals.TotalRuns==4 && totals.Interrupted==1,"same-duty checkpoint does not create another run");
    totals.Stop();
    Assert(totals.Interrupted==2,"explicit close counts interruption"); ++passed;

    totals.BeginSession(false);
    totals.UpdateTerritory(1165,false);
    totals=new LocalStatistics(statisticsPath,Error);
    Assert(totals.TotalRuns==5 && totals.Interrupted==3,"unfinished duty recovers as interrupted after crash");
    totals=new LocalStatistics(statisticsPath,Error);
    Assert(totals.Interrupted==3,"repeated reload does not recount interruption"); ++passed;

    totals.BeginSession(true);
    totals.UpdateTerritory(1165,false);
    totals.Observe(Event("winner",CourseId.Crown,1,60));
    totals.UpdateTerritory(1197,false);
    totals.UpdateTerritory(1165,false);
    totals.Observe(Event("winner",CourseId.Crown,3,60));
    totals.Stop();
    Assert(totals.TotalRuns==7 && totals.Wins==2 && totals.Failures==1 && totals.Interrupted==4,
        "mid-duty start is excluded; next full duty can win"); ++passed;

    totals.BeginSession(false);
    totals.UpdateTerritory(1165,false);
    totals.UpdateTerritory(1197,false);
    totals.Stop();
    Assert(totals.TotalRuns==8 && totals.Interrupted==5 && totals.WinRate==2d/3,
        "leaving without a known result counts interruption"); ++passed;
    File.WriteAllText(Path.Combine(statisticsPath,"statistics.json"),"{");
    var readErrors=new List<Exception>();
    totals=new LocalStatistics(statisticsPath,readErrors.Add);
    Assert(readErrors.Count==1 && totals.TotalRuns==8 && totals.Wins==2 && totals.Interrupted==5,
        "damaged primary statistics recover from local backup"); ++passed;
} finally { if(Directory.Exists(statisticsPath)) Directory.Delete(statisticsPath,true); }
Console.WriteLine($"Mr.Bean3 summary/outbox/statistics tests: {passed} passed");
if (args.Length == 2 && args[0] == "--package") PackageAssets.Check(args[1]);

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
