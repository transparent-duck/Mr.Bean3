using MrBean.FallGuys;
using MrBean.Modules.FallGuysPilot;
using Mr.Bean3;

var now = new DateTimeOffset(2026,10,17,10,0,0,TimeSpan.Zero);
PilotObservation Event(string kind,CourseId course,long scene,double seconds,int hits=0,bool manual=false)
    => new(kind,scene,course,now.AddSeconds(seconds),seconds,true,kind is "qualified" or "winner",
        kind=="eliminated",manual,kind is "qualified" or "winner"?1u:null,hits,2,1);
void Assert(bool value,string name) { if(!value) throw new Exception(name); }
int passed=0;

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
Console.WriteLine($"Mr.Bean3 local statistics tests: {passed} passed");
if (args.Length == 2 && args[0] == "--package") PackageAssets.Check(args[1]);
