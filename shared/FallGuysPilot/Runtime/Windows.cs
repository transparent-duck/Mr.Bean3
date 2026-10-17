namespace MrBean.FallGuys;

public sealed record ConditionWindows(bool Observed, TimeWindow[] Blocked);
public sealed record CourseObservation(long Session, long Revision, double At, double Horizon,
    ConditionWindows[] Conditions, string? Unsupported);
public sealed record ScheduledProgram(int Program, double Departure, double LatestArrival, double LatestDeparture);
public abstract record RoutingResult;
public sealed record AwaitStatusRemoval(ushort Status,TimeWindow Hold) : RoutingResult;
public sealed record FullSchedule(ScheduledProgram[] Programs, double Arrival) : RoutingResult;
public sealed record ObserveThen(ScheduledProgram[] Approach, ObservationPolicy Policy,
    TimeWindow Hold, double DecisionBy, ScheduledProgram? Timeout) : RoutingResult;
public sealed record PolicyContinuation(ObservationBranch Branch,ScheduledProgram[] Programs,
    ObserveThen? Next) : RoutingResult;
public sealed record BlockedHold(string Reason, TimeWindow? Hold) : RoutingResult;

internal static class Windows
{
    public static TimeWindow[] Merge(List<TimeWindow> input)
    {
        input.Sort((a,b) => a.Start.CompareTo(b.Start));
        var result = new List<TimeWindow>();
        foreach (var item in input)
        {
            if (item.End <= item.Start) continue;
            if (result.Count > 0 && item.Start <= result[^1].End)
                result[^1] = new(result[^1].Start, Math.Max(result[^1].End, item.End));
            else result.Add(item);
        }
        return [.. result];
    }
    public static TimeWindow[] Complement(TimeWindow[] blocked, double start, double end)
    {
        var result = new List<TimeWindow>();
        double at = start;
        foreach (var item in blocked)
        {
            if (item.End <= at) continue;
            if (item.Start >= end) break;
            if (item.Start > at) result.Add(new(at, Math.Min(item.Start,end)));
            at = Math.Max(at,item.End);
        }
        if (at < end) result.Add(new(at,end));
        return [.. result];
    }
}

// One pass over precompiled crossings binds current windows. Query complexity
// depends on the finite program graph, never on a terrain-grid resolution.
public sealed class BoundWindows
{
    private readonly CompiledCourse course;
    public CourseObservation Observation { get; }
    public TimeWindow[][] SafePorts { get; }
    private readonly TimeWindow[]?[] blockedDepartures;
    public bool[] Ready { get; }
    public bool[] SourcesReady { get; }
    public BoundWindows(CompiledCourse course, CourseObservation observation,WindowScope? scope=null)
    {
        this.course=course;
        Observation = observation;
        if (observation.Conditions.Length != course.Asset.Conditions.Length)
            throw new ArgumentException("Observation does not belong to this compiled condition table.");
        SafePorts = new TimeWindow[course.Asset.Ports.Length][];
        Array.Fill(SafePorts,Array.Empty<TimeWindow>());
        blockedDepartures = new TimeWindow[course.Asset.Programs.Length][];
        Ready = new bool[course.Asset.Programs.Length];
        SourcesReady=new bool[course.Asset.Sources.Length];
        for(int n=0;n<SourcesReady.Length;n++)SourcesReady[n]=observation.Unsupported==null&&(scope==null||scope.Sources[n]);
        foreach(var condition in course.Asset.Conditions)
            if(scope==null||scope.Conditions[condition.Id])SourcesReady[condition.Source]&=observation.Conditions[condition.Id].Observed;
        foreach (int id in scope?.Ports??Enumerable.Range(0,course.Asset.Ports.Length))
        {
            var port=course.Asset.Ports[id];
            var occupied = new List<TimeWindow>();
            foreach (int condition in port.OccupiedConditions)
                occupied.AddRange(observation.Conditions[condition].Blocked);
            SafePorts[port.Id] = Windows.Complement(Windows.Merge(occupied),observation.At,observation.Horizon);
        }
        foreach (int id in scope?.Programs??Enumerable.Range(0,course.Asset.Programs.Length))
        {
            var program=course.Asset.Programs[id];
            bool ready = observation.Unsupported == null;
            foreach (int condition in program.RequiredConditions) ready &= observation.Conditions[condition].Observed;
            Ready[program.Id] = ready;
        }
    }

    // Departure queries belong to admission/routing at a receiving port.
    // Running control reads the current raw conditions in ClearRemainder;
    // it does not need to rebuild every unrelated program's departure table.
    private TimeWindow[] Departures(int program)
    {
        if(blockedDepartures[program] is { } cached)return cached;
        var blocked=new List<TimeWindow>();
        foreach(var crossing in course.Asset.Programs[program].Crossings)
            foreach(var window in Observation.Conditions[crossing.Condition].Blocked)
                blocked.Add(new(window.Start-crossing.Exit,window.End-crossing.Enter));
        return blockedDepartures[program]=Windows.Merge(blocked);
    }

    public bool Clear(int program, double departure)
    {
        if (!Ready[program] || departure+course.Asset.Programs[program].MaximumDuration>=Observation.Horizon) return false;
        foreach (var blocked in Departures(program))
        {
            if (departure < blocked.Start) break;
            if (departure < blocked.End) return false;
        }
        return true;
    }
    public bool ClearRemainder(int program,double started,double now)
    {
        if(Observation.Unsupported!=null)return false;
        var item=course.Asset.Programs[program];
        if(started+item.MaximumDuration>=Observation.Horizon)return false;
        foreach(var crossing in item.Crossings)
        {
            double end=started+crossing.Exit;
            if(end<now)continue;
            double start=Math.Max(now,started+crossing.Enter);
            var condition=Observation.Conditions[crossing.Condition];
            if(!condition.Observed)return false;
            foreach(var blocked in condition.Blocked)
                if(start<blocked.End&&end>=blocked.Start)return false;
        }
        return true;
    }
    public double LatestDeparture(int program,double departure)
    {
        if(!Clear(program,departure))return departure;
        var item=course.Asset.Programs[program];
        double latest=Observation.Horizon-item.MaximumDuration;
        foreach(var blocked in Departures(program))
            if(blocked.Start>departure){latest=Math.Min(latest,blocked.Start);break;}
        var hold=Array.Find(SafePorts[item.From],w=>w.Contains(departure));
        latest=Math.Min(latest,hold.End);
        var receiving=Array.Find(SafePorts[item.To],w=>w.Contains(departure+item.MinimumDuration,departure+item.MaximumDuration));
        return Math.Min(latest,receiving.End-item.MaximumDuration);
    }
    public double NextDeparture(int program, double earliest,out double clearUntil)
    {
        clearUntil=double.PositiveInfinity;
        foreach (var blocked in Departures(program))
        {
            if (earliest < blocked.Start){clearUntil=blocked.Start;break;}
            if (earliest < blocked.End) earliest = blocked.End;
        }
        return earliest;
    }
}
