namespace MrBean.FallGuys;

public sealed class CourseRouter(CompiledCourse course)
{
    private readonly record struct State(int Port, int Interval,bool PendingTransition);
    private sealed record Label(State State, double Arrival, int Parent, int Program, double Departure,double LatestDeparture);

    public RoutingResult Route(int port, double now, BoundWindows windows, double? observationDeadline = null,
        double? actorTransitionAt = null,ushort actorTransitionStatus=0,int? destination=null,ushort groundInputStatus=0)
    {
        if (windows.Observation.Unsupported is { } issue) return new BlockedHold(issue,null);
        int initial = Array.FindIndex(windows.SafePorts[port],w => w.Contains(now));
        if (initial < 0) return new BlockedHold("Actual entry has no observed safe holding interval.",null);
        if(actorTransitionStatus==3802&&actorTransitionAt is { } expiry&&
            windows.SafePorts[port][initial].Contains(now,expiry+course.Asset.Law.MaximumFrameSeconds))
            return new AwaitStatusRemoval(3802,windows.SafePorts[port][initial]);
        var labels = new List<Label>();
        var best = new Dictionary<State,int>();
        var queue = new PriorityQueue<int,double>();
        void Offer(State state,double arrival,int parent,int program,double departure,double latestDeparture)
        {
            if (best.TryGetValue(state,out int old) && labels[old].Arrival <= arrival) return;
            int id = labels.Count;
            labels.Add(new(state,arrival,parent,program,departure,latestDeparture));best[state] = id;
            queue.Enqueue(id,arrival+course.Asset.RemainingCost[state.Port]);
        }
        Offer(new(port,initial,actorTransitionAt!=null),now,-1,-1,now,double.PositiveInfinity);
        int observing = -1;double observationScore = double.PositiveInfinity;
        bool survival=course.Asset.GoalKind==GoalKind.Survive;
        double survivalUntil=now+8;
        int longestHold=0;
        while (queue.TryDequeue(out int id,out _))
        {
            var label = labels[id];
            if (best[label.State] != id) continue;
            int current = label.State.Port;
            if ((destination is { } goal?current==goal:course.Asset.Ports[current].Goal)&&!label.State.PendingTransition)
                return new FullSchedule(Path(id),label.Arrival);
            var hold = windows.SafePorts[current][label.State.Interval];
            if(survival&&!label.State.PendingTransition)
            {
                // Survive through a bounded temporal horizon. No port is a
                // fabricated finish and no completed lap grants qualification.
                if(hold.End>survivalUntil)return new FullSchedule(Path(id),label.Arrival);
                var previous=labels[longestHold];
                if(hold.End>windows.SafePorts[previous.State.Port][previous.State.Interval].End)longestHold=id;
                if(label.Arrival>=survivalUntil)continue;
            }
            if (course.Observations[current] is { } policy &&
                TryObservation(policy,label.Arrival,hold,windows,observationDeadline,label.State.PendingTransition?actorTransitionAt:null,out _,out _) &&
                label.Arrival+course.Asset.RemainingCost[current] < observationScore)
            { observing = id;observationScore = label.Arrival+course.Asset.RemainingCost[current]; }
            // A future actor status is an observation boundary, never an
            // assumed receipt propagated through a precomputed schedule.
            if(course.Observations[current] is {RequiredStatus:>0} statusPolicy&&
                (label.Parent>=0||groundInputStatus!=statusPolicy.RequiredStatus))continue;
            foreach (int programId in course.Outgoing[current])
            {
                var program = course.Asset.Programs[programId];
                for (int next = 0; next < windows.SafePorts[program.To].Length; ++next)
                {
                    var receiving = windows.SafePorts[program.To][next];
                    if(!TryStep(program,label.Arrival,hold,receiving,windows,label.State.PendingTransition,
                        actorTransitionAt,actorTransitionStatus,out var step))continue;
                    Offer(new(program.To,next,label.State.PendingTransition&&program.Transition==null),step.LatestArrival,id,programId,step.Departure,step.LatestDeparture);
                }
            }
        }
        if(survival&&longestHold!=0)
            return new FullSchedule(Path(longestHold),labels[longestHold].Arrival);
        if (observing >= 0)
        {
            var label = labels[observing];var policy = course.Observations[label.State.Port]!;
            var hold = windows.SafePorts[label.State.Port][label.State.Interval];
            if (TryObservation(policy,label.Arrival,hold,windows,observationDeadline,label.State.PendingTransition?actorTransitionAt:null,out double decisionBy,out var timeout))
                return new ObserveThen(Path(observing),policy,hold,decisionBy,timeout);
        }
        var blockedUntil=windows.SafePorts[port][initial];
        if(actorTransitionAt is { } limit)
            return new BlockedHold("Known pending native transition has no authored complete continuation.",
                blockedUntil with{End=Math.Min(blockedUntil.End,limit)});
        return new BlockedHold("No covered program or observation continuation in the current windows.",blockedUntil);

        ScheduledProgram[] Path(int id)
        {
            var path = new List<ScheduledProgram>();
            while (labels[id] is {Parent:>=0} label)
            { path.Add(new(label.Program,label.Departure,label.Arrival,label.LatestDeparture));id = label.Parent; }
            path.Reverse();return [.. path];
        }
    }

    // Once an observation policy is active, only its declared cases can be
    // selected. New evidence does not invoke the whole-course route search.
    public RoutingResult Follow(ObserveThen active,double now,BoundWindows windows,double? actorTransitionAt,ushort actorTransitionStatus,ushort groundInputStatus=0)
    {
        if(now>=active.DecisionBy)return new BlockedHold("Observation receipt window has expired.",null);
        if(windows.Observation.Unsupported is { } issue)return new BlockedHold(issue,null);
        PolicyContinuation? selected=null;double best=double.PositiveInfinity;
        if((active.Policy.RequiredStatus==0||active.Policy.RequiredStatus==groundInputStatus)&&
            (active.Policy.Need.Length==0||active.Policy.Need.Any(source=>windows.SourcesReady[source])))
            foreach(var branch in active.Policy.Cases)
            {
                if(branch.Need.Any(source=>!windows.SourcesReady[source]))continue;
                if(ScheduleBranch(branch,now,windows,actorTransitionAt,actorTransitionStatus,groundInputStatus) is not { } candidate)continue;
                double score=candidate.Programs[^1].LatestArrival+course.Asset.RemainingCost[branch.NextPort];
                if(score>=best)continue;
                best=score;selected=candidate;
            }
        return selected is not null?selected:new BlockedHold("Waiting for evidence and a valid time window for a declared observation branch.",active.Hold);
    }

    // Rebind one finite, already selected chain at a real receiving state. The
    // chain includes any intermediate ports that cannot hold indefinitely.
    public PolicyContinuation? ScheduleBranch(ObservationBranch branch,double now,BoundWindows windows,double? actorTransitionAt,ushort actorTransitionStatus,ushort groundInputStatus=0)
    {
        if(branch.ProgramChoices.Length==0||windows.Observation.Unsupported!=null)return null;
        int from=course.Asset.Programs[branch.ProgramChoices[0][0]].From;
        int initial=Array.FindIndex(windows.SafePorts[from],w=>w.Contains(now));
        if(initial<0)return null;
        var states=new Dictionary<(int Interval,bool Pending),(double Arrival,ScheduledProgram[] Path)>{[(initial,actorTransitionAt!=null)]=(now,[])};
        foreach(var choices in branch.ProgramChoices)
        {
            var nextStates=new Dictionary<(int Interval,bool Pending),(double Arrival,ScheduledProgram[] Path)>();
            foreach(int id in choices)
            {
                var program=course.Asset.Programs[id];
                if(program.Entry.RequiredStatus!=0&&(program.From!=from||program.Entry.RequiredStatus!=groundInputStatus))continue;
                foreach(var(current,state) in states)
                {
                    var hold=windows.SafePorts[program.From][current.Interval];
                    for(int interval=0;interval<windows.SafePorts[program.To].Length;++interval)
                    {
                        var receiving=windows.SafePorts[program.To][interval];
                        if(!TryStep(program,state.Arrival,hold,receiving,windows,current.Pending,actorTransitionAt,actorTransitionStatus,out var step))continue;
                        var next=(interval,current.Pending&&program.Transition==null);
                        if(nextStates.TryGetValue(next,out var old)&&old.Arrival<=step.LatestArrival)continue;
                        nextStates[next]=(step.LatestArrival,[..state.Path,step]);
                    }
                }
            }
            if(nextStates.Count==0)return null;
            states=nextStates;
        }
        foreach(var(current,state) in states.OrderBy(p=>p.Value.Arrival))
        {
            if(course.Asset.Ports[branch.NextPort].Goal&&!current.Pending)return new(branch,state.Path,null);
            if(course.Observations[branch.NextPort] is not { } next)continue;
            var hold=windows.SafePorts[branch.NextPort][current.Interval];
            if(!TryObservation(next,state.Arrival,hold,windows,null,current.Pending?actorTransitionAt:null,out double deadline,out var timeout))continue;
            return new(branch,state.Path,new([],next,hold,deadline,timeout));
        }
        return null;
    }

    // Both whole-course routing and fixed-policy choices use the same timing
    // and pending-warning semantics. No control is substituted by the driver.
    private bool TryStep(MovementProgram program,double arrival,TimeWindow hold,TimeWindow receiving,
        BoundWindows windows,bool pending,double? transitionAt,ushort transitionStatus,out ScheduledProgram step)
    {
        step=null!;
        if(program.Recovery||!windows.Ready[program.Id])return false;
        if(program.Transition!=null&&(!pending||transitionAt==null||transitionStatus is not (3699 or 3700 or 3701)))return false;
        double earliest=Math.Max(arrival,receiving.Start-program.MinimumDuration);
        if(program.Transition is { } handled)
            earliest=Math.Max(earliest,transitionAt!.Value-handled.LatestActivation+course.Asset.Law.MaximumFrameSeconds);
        double limit=Math.Min(hold.End,Math.Min(receiving.End,windows.Observation.Horizon)-program.MaximumDuration);
        if(pending&&transitionAt is { } transition)
            limit=Math.Min(limit,transition-course.Asset.Law.MaximumFrameSeconds-(program.Transition==null?program.MaximumDuration:0));
        while(earliest<limit)
        {
            double departure=windows.NextDeparture(program.Id,earliest,out double clearUntil);
            double latest=departure+course.Asset.Law.DispatchSeconds;
            if(!course.Asset.Ports[program.From].CanHold&&departure>arrival)return false;
            if(latest<Math.Min(clearUntil,limit))
            {
                // The downstream label includes the full permitted dispatch
                // delay, so a preceding program cannot consume that reserve.
                step=new(program.Id,departure,latest+program.MaximumDuration,latest);return true;
            }
            if(clearUntil>=limit)return false;
            earliest=clearUntil;
        }
        return false;
    }

    private bool TryObservation(ObservationPolicy policy,double arrival,TimeWindow hold,BoundWindows windows,
        double? observationDeadline,double? actorTransitionAt,out double deadline,out ScheduledProgram? timeout)
    {
            deadline = Math.Min(observationDeadline ?? arrival+policy.DecisionSeconds,hold.End);
            timeout = null;
            if (deadline <= arrival) return false;
            if (policy.Timeout == TimeoutKind.StopAtHold)
            {
                // This disposition is valid only at an unconditional hold port.
                // A finite safe window is not permission to stop there forever.
                if (actorTransitionAt!=null||course.Asset.Ports[policy.Port].OccupiedConditions.Length != 0) return false;
                return true;
            }
            var escape = course.Asset.Programs[policy.TimeoutProgram];
            deadline = Math.Min(deadline,hold.End-escape.MaximumDuration);
            if (deadline <= arrival || !windows.Clear(escape.Id,deadline)) return false;
            double end = deadline+escape.MaximumDuration;
            double earliestArrival=deadline+escape.MinimumDuration;
            if (!Array.Exists(windows.SafePorts[escape.To],w => w.Contains(earliestArrival,end))) return false;
            timeout = new(escape.Id,deadline,end,windows.LatestDeparture(escape.Id,deadline));return true;
    }
}
