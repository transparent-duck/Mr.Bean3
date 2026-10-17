using System.Numerics;

namespace MrBean.FallGuys;

public readonly record struct ActorSession(long Scene, ulong Actor);
public sealed record MotionObservation(ActorSession Session, long Serial,uint NativeFrame, double At, Vector3 Feet,
    float Heading, Vector2 Velocity, MotionKind Motion, bool Grounded,
    float NativeSpeed,float AvailableTargetSpeed,float FrameSeconds,Vector2 ExternalVelocity,double? TransitionAt,ushort TransitionStatus,
    double FramePhaseSeconds,bool UnscaledClock,
    float? TimelineSpeed=null,ushort GroundInputStatus=0);
public enum ReceiptKind { Activated, Released, Interrupted, Replaced }
public sealed record NativeEpisode(MotionKind Kind, long FirstObservation, Vector3 Origin,
    Vector3? Target, ushort Timeline, double? InitialClock);
public sealed record NativeReceipt(ActorSession Session, long Attempt, ReceiptKind Kind,
    long Observation, NativeEpisode Episode);
public enum ExecutionState { AtPort, WaitingDeparture, Preparing, Running, Observing, Stopped, SessionEnded, NativeHolding }
public enum ExecutionValidity { Valid, Contradicted, ObservationLost, OutsideCoverage }
public enum DriverControl { Release, Hold, Walk, Jump, Steering, Arrow, GroundVelocity, ObserveNative,ForcedHeading, Ice }
public sealed record ControlCommand(DriverControl Kind, Vector2 Input, float? Heading, float? Speed,Vector3? PositionCorrection=null);
public sealed record DriverFrame(long Commit, long Attempt, int Leg, double LegStartedAt, double Now,
    MovementProgram Program, MotionObservation Observation, NativeEpisode? Episode, bool ActivationPending);
public sealed record DriverResult(ControlCommand Control, bool Ready, bool LegCompleted,
    bool AwaitActivation, NativeReceipt? Receipt, string? Unsupported);

// The adapter may interpret native state and derive control for the accepted
// leg. It cannot choose a different path, clear a commitment or advance a cursor.
public interface IProgramDriver
{
    DriverResult Prepare(MovementProgram program, MotionObservation observation, double startedAt);
    DriverResult Step(DriverFrame frame);
    void Apply(ControlCommand command, long attempt, MotionObservation observation);
    // True only when an armed request was cancelled before native submission.
    // External triggers and already submitted actions remain observable.
    bool Revoke();
}

public sealed record ExecutionStatus(ExecutionState State, ExecutionValidity Validity, int Port,
    int? Program, int Leg, long Commit, NativeEpisode? Native, bool ActivationPending,
    double? DecisionBy, string? Issue);
public sealed record Admission(long ExpectedCommit, ActorSession Session,uint NativeFrame, long ForecastRevision,
    ScheduledProgram First, RoutingResult Plan, double ValidUntil,NativeEpisode? RetainedEpisode=null);

public sealed class ExecutionOwner
{
    private readonly CompiledCourse course;
    private readonly IProgramDriver driver;
    private readonly ActorSession session;
    private readonly HashSet<int> visitedObservations = [];
    private ExecutionState state = ExecutionState.AtPort;
    private ExecutionValidity validity;
    private int port,leg,pathIndex;
    private long commit,attempt;
    private MovementProgram? program;
    private ScheduledProgram[] path = [];
    private ObserveThen? observation;
    private PolicyContinuation? branchPlan;
    private NativeEpisode? native;
    private long lastReceipt;
    private bool activationPending;
    private double legStartedAt,preparationAt;
    private string? issue;
    private Admission? pending;
    private AwaitStatusRemoval? nativeHold;
    private ExecutionState admissionState;
    private double? continuationBy;
    public ExecutionStatus Status => new(state,validity,port,program?.Id,leg,commit,native,
        activationPending,DecisionBy,issue);
    public bool NeedsRoute => state is ExecutionState.AtPort or ExecutionState.Observing;
    public ExecutionState State=>state;
    public int? CurrentProgram=>program?.Id;
    public int? PendingProgram=>pending?.First.Program;
    public double? PendingHorizon=>pending is { } p?
        p.ValidUntil+course.Asset.Programs[p.First.Program].MaximumDuration+course.Asset.Law.MaximumFrameSeconds:null;
    public long Commit=>commit;
    public double? ProgramStartedAt=>program!=null?preparationAt:null;
    public double? LegStartedAt=>program!=null?legStartedAt:null;
    // During commitment only its remaining occupancy can authorize control.
    // Include the final callback so equality at the receiving bound is covered.
    public double? CommittedHorizon => state is ExecutionState.Running or ExecutionState.Preparing&&program!=null?
        preparationAt+program.MaximumDuration+course.Asset.Law.MaximumFrameSeconds:null;
    public int Port => port;
    public long Attempt => attempt;
    public double? DecisionBy => continuationBy??(observation?.Policy.Port==port?observation.DecisionBy:null);
    public ObserveThen? ActiveObservation=>state==ExecutionState.Observing?observation:null;
    public ObservationBranch? PendingBranch=>branchPlan!=null&&pathIndex<path.Length?
        branchPlan.Branch with{ProgramChoices=branchPlan.Branch.ProgramChoices.Skip(pathIndex).ToArray()}:null;

    public ExecutionOwner(CompiledCourse course,IProgramDriver driver,ActorSession session,int port)
    { this.course=course;this.driver=driver;this.session=session;this.port=port; }

    // An empty observation policy can enter its hold here. Physical preparation
    // starts only after a complete program fits its current time windows.
    public Admission? Admit(RoutingResult result,MotionObservation actual,BoundWindows windows)
    {
        if (!NeedsRoute || actual.Session != session || activationPending ||
            native!=null&&!RetainArrowAt(port,actual) ||
            windows.Observation.Unsupported != null) return null;
        if(result is AwaitStatusRemoval wait)
        {
            if(native!=null||actual.TransitionStatus!=wait.Status||actual.Motion!=MotionKind.Ordinary||!actual.Grounded||
                !course.Asset.Law.AtRest(actual.NativeSpeed,actual.ExternalVelocity)||
                !wait.Hold.Contains(actual.At)||!Array.Exists(windows.SafePorts[port],w=>w.Contains(actual.At)))return null;
            nativeHold=wait;state=ExecutionState.NativeHolding;
            // Waiting for a status does not create another observation visit.
            // Retain its policy, selected branch and fixed departure limits;
            // clearing it while retaining visitedObservations strands the owner.
            issue=null;return null;
        }
        var steps = result switch { FullSchedule full=>full.Programs,ObserveThen observe=>observe.Approach,
            PolicyContinuation continuation=>continuation.Programs,_=>[] };
        if (steps.Length == 0)
        {
            if (result is ObserveThen observe && observe.Policy.Port == port &&
                (observation != null || !visitedObservations.Contains(port)))
            {
                observation ??= observe;visitedObservations.Add(port);state=ExecutionState.Observing;
            }
            else if (result is BlockedHold blocked) issue=blocked.Reason;
            return null;
        }
        var first=steps[0];var candidate=course.Asset.Programs[first.Program];
        // Calculation can finish after the nominal earliest departure. Bind
        // only inside its already reserved dispatch interval. Downstream
        // latest-arrival labels and the original latest departure stay fixed.
        double departure=Math.Max(first.Departure,actual.At);
        if(state==ExecutionState.Observing&&result is PolicyContinuation branch&&
            (observation==null||!observation.Policy.Cases.Contains(branch.Branch)))return null;
        issue=null;
        if(!course.Asset.Law.AcceptsFrame(actual.FrameSeconds))
        {issue="Current native callback period is outside the compiled entry domain.";return null;}
        if(!course.Asset.Law.AcceptsClock(actual))
        {issue="Native physics and observation clocks are outside the compiled phase domain.";return null;}
        if(!course.Asset.Law.AcceptsTargetSpeed(candidate.Entry,actual.AvailableTargetSpeed))
        {issue=$"Available native target speed {actual.AvailableTargetSpeed} differs from required {candidate.Entry.RequiredTargetSpeed}.";return null;}
        if(!BeforeTransition(candidate,departure,actual))
        {issue="Known native transition occurs before this program can reach its receiving state.";return null;}
        if(!EntryMatches(candidate,actual))
        {issue="Actual motion, rest, position or heading is outside the compiled entry domain.";return null;}
        if (candidate.From != port || actual.At>=first.LatestDeparture ||
            !windows.Clear(candidate.Id,departure)) return null;
        if (!Array.Exists(windows.SafePorts[port],w=>w.Contains(actual.At,departure))) return null;
        if (result is ObserveThen waiting && visitedObservations.Contains(waiting.Policy.Port) &&
            observation?.Policy.Port != waiting.Policy.Port) return null;
        double validUntil=Math.Min(first.LatestDeparture,DecisionBy??double.PositiveInfinity);
        if(validUntil<=departure)return null;
        return new(commit,session,actual.NativeFrame,windows.Observation.Revision,first with{Departure=departure},result,validUntil,native);
    }

    public bool Offer(Admission candidate)
    {
        if (!NeedsRoute || candidate.ExpectedCommit!=commit || candidate.Session!=session) return false;
        pending=candidate;admissionState=state;state=ExecutionState.WaitingDeparture;return true;
    }

    // Called at the appropriate input callback, without delaying steering
    // control until the following Framework.Update.
    public void ObserveCoverage(MotionObservation actual,BoundWindows windows)
    {
        if(actual.Session!=session){EndSession();return;}
        if(state is ExecutionState.Stopped or ExecutionState.SessionEnded)return;
        // This is an observation/revocation path. It cannot refresh input,
        // start a departure, advance a leg or consume a release receipt.
        // No committed moving family exists at a waiting receiving hold.
        // Its native callback checks rest/support and its eventual departure
        // still has the full entry/frame gate.
        if(program==null)return;
        if(!course.Asset.Law.AcceptsFrame(actual.FrameSeconds)||!actual.UnscaledClock)
        {Lose("Observed native physics step or rate is outside the compiled control domain.");return;}
        if(!BeforeTransition(program,preparationAt,actual))
        {Contradicted("Newly observed native transition invalidates the committed program's completion bound.");return;}
        if(!windows.ClearRemainder(program.Id,preparationAt,actual.At))
        {Contradicted("New observations invalidate the remaining committed program; no authored recovery applies.");return;}
        if(actual.At>preparationAt+program.MaximumDuration)Lose("Program exceeded its compiled timing coverage.");
    }
    public void Frame(MotionObservation actual,BoundWindows windows)
    {
        if (actual.Session!=session) { EndSession();return; }
        if (state is ExecutionState.Stopped or ExecutionState.SessionEnded) return;
        if(state!=ExecutionState.NativeHolding&&program?.Transition==null&&actual.TransitionAt is { } transition&&actual.At+course.Asset.Law.MaximumFrameSeconds>=transition)
        {Contradicted("Pending native transition has reached the end of ordinary-control coverage; no authored continuation applies.");return;}
        if(state is ExecutionState.AtPort or ExecutionState.Observing or ExecutionState.WaitingDeparture or ExecutionState.NativeHolding)
        {
            var holding=course.Asset.Ports[port];
            if(!actual.Grounded||actual.Motion!=holding.HoldingMotion||
                holding.HoldingMotion==MotionKind.Arrow&&!RetainArrowAt(port,actual)||
                !course.Asset.Law.AtRest(actual.NativeSpeed,actual.ExternalVelocity)||
                HorizontalDistance(actual.Feet,holding.Position)>holding.Radius||
                Math.Abs(actual.Feet.Y-holding.Position.Y)>holding.HeightTolerance||
                !Array.Exists(windows.SafePorts[port],w=>w.Contains(actual.At)))
            {Contradicted("Actual state no longer satisfies the holding contract.");return;}
            driver.Apply(new(DriverControl.Hold,default,null,null),attempt,actual);
            if(continuationBy is { } deadline&&actual.At>=deadline)
            {Stop("Selected continuation departure deadline reached at its receiving hold.");return;}
        }
        if(state==ExecutionState.NativeHolding)
        {
            if(DecisionBy is { } decision&&actual.At>=decision)
            {Stop("The original observation or continuation deadline expired during native status waiting.");return;}
            if(nativeHold==null||actual.At>=nativeHold.Hold.End)
            {Stop("Native status removal was not observed within the admitted safe holding interval.");return;}
            if(actual.TransitionStatus==nativeHold.Status)return;
            if(actual.TransitionAt!=null){Contradicted("A different native obligation replaced the held status.");return;}
            nativeHold=null;state=observation?.Policy.Port==port?ExecutionState.Observing:ExecutionState.AtPort;return;
        }
        if (state==ExecutionState.WaitingDeparture)
        {
            if (pending is not { } candidate) { Lose("Missing admitted candidate.");return; }
            // Scheduling and input dispatch must occupy separate native frames. A long
            // scheduling frame must fail admission while still at the holding
            // port, not start control using the earlier frame measurement.
            if(actual.NativeFrame==candidate.NativeFrame)return;
            if (actual.At<candidate.First.Departure) return;
            var first=course.Asset.Programs[candidate.First.Program];
            // A changed forecast is checked against this finite program. It
            // does not start a new route search or silently reuse a stale pass.
            if (candidate.ExpectedCommit!=commit || candidate.RetainedEpisode!=native || actual.At>=candidate.ValidUntil || !EntryMatches(first,actual) ||
                !windows.Clear(first.Id,actual.At)||!BeforeTransition(first,actual.At,actual))
            { pending=null;state=admissionState;issue="Admission no longer applies.";return; }
            path=candidate.Plan switch {FullSchedule full=>full.Programs,ObserveThen observe=>observe.Approach,
                PolicyContinuation continuation=>continuation.Programs,_=>[]};
            branchPlan=candidate.Plan as PolicyContinuation;
            observation=branchPlan?.Next??candidate.Plan as ObserveThen;pathIndex=0;pending=null;
            Begin(first,actual.At);
        }
        if (state==ExecutionState.Observing)
        {
            if (observation==null) { Lose("Missing observation contract.");return; }
            if (actual.At>=observation.DecisionBy)
            {
                if (observation.Timeout is { } timeout && windows.Clear(timeout.Program,actual.At) &&
                    EntryMatches(course.Asset.Programs[timeout.Program],actual))
                { path=[timeout];pathIndex=0;observation=null;Begin(course.Asset.Programs[timeout.Program],actual.At); }
                else Stop("Observation deadline reached; executing the specified stop-at-hold disposition.");
            }
            if (state is not (ExecutionState.Running or ExecutionState.Preparing)) return;
        }
        if (state==ExecutionState.AtPort) return;
        if (program==null) { Lose("Running state has no committed program.");return; }
        if(!BeforeTransition(program,preparationAt,actual))
        {Contradicted("Newly observed native transition invalidates the committed program's completion bound.");return;}
        if(!windows.ClearRemainder(program.Id,preparationAt,actual.At))
        { Contradicted("New observations invalidate the remaining committed program; no authored recovery applies.");return; }
        if(actual.At>preparationAt+program.MaximumDuration)
        { Lose("Program exceeded its compiled timing coverage.");return; }
        DriverResult result;
        if (state==ExecutionState.Preparing)
        {
            if (actual.At>preparationAt+program.Prepare.MaximumSeconds ||
                HorizontalDistance(actual.Feet,course.Asset.Ports[program.From].Position)>
                program.Entry.PositionRadius+program.Prepare.MaximumDisplacement)
            { Lose("Entry preparation exceeded its admitted domain.");return; }
            result=driver.Prepare(program,actual,preparationAt);
            if (result.Unsupported!=null) { Lose(result.Unsupported);return; }
            driver.Apply(result.Control,attempt,actual);
            if (!result.Ready) return;
            state=ExecutionState.Running;legStartedAt=actual.At;
        }
        // One native observation can finish a release and its already-resting
        // capture. Consume every such completion now: applying Release from a
        // completed capture removes the input owner and may leave no later walk
        // callback to finish it. Each iteration advances a finite authored leg.
        while(true)
        {
            result=driver.Step(new(commit,attempt,leg,legStartedAt,actual.At,program,actual,native,activationPending));
            if(result.AwaitActivation&&!AwaitActivation())return;
            if (result.Receipt is { } receipt) ObserveReceipt(receipt);
            if(state==ExecutionState.Stopped)return;
            if (result.Unsupported!=null) { Lose(result.Unsupported);return; }
            if(!result.LegCompleted)break;
            bool continues=leg+1<program.Legs.Length &&
                (program.Legs[leg+1] is ReleaseLeg release && native?.Kind==release.NativeMotion ||
                 native?.Kind==MotionKind.Arrow&&program.Legs[leg+1] is CaptureLeg&&
                 course.Asset.Ports[program.To].HoldingMotion==MotionKind.Arrow);
            continues|=leg+1==program.Legs.Length&&RetainArrowAt(program.To,actual);
            if ((native!=null || activationPending)&&!continues) { Lose("Leg ended without its native release observation.");return; }
            ++leg;legStartedAt=actual.At;
            if (leg==program.Legs.Length)
            {
                var receiving=course.Asset.Ports[program.To];
                if (!actual.Grounded || actual.Motion!=receiving.HoldingMotion ||
                    !course.Asset.Law.AtRest(actual.NativeSpeed,actual.ExternalVelocity)||
                    HorizontalDistance(actual.Feet,receiving.Position)>receiving.Radius ||
                    Math.Abs(actual.Feet.Y-receiving.Position.Y)>receiving.HeightTolerance)
                { Lose("Program has not reached its actual holdable receiving domain.");return; }
                port=program.To;program=null;driver.Revoke();
                // At a real receiving port the finite scheduler can bind the
                // continuation to actual arrival. Never shift a stale suffix.
                if(++pathIndex<path.Length)
                {
                    // Preserve the selected chain's next departure bound while
                    // rebinding at actual arrival. Repeated offers cannot renew it.
                    continuationBy=branchPlan!=null?path[pathIndex].LatestDeparture:null;
                    state=ExecutionState.AtPort;return;
                }
                branchPlan=null;
                state=observation==null?ExecutionState.AtPort:ExecutionState.Observing;
                if (observation!=null) visitedObservations.Add(port);
                return;
            }
            // Select the next authored leg in this same eligible callback.
        }
        driver.Apply(result.Control,attempt,actual);
    }

    private bool AwaitActivation()
    {
        if(activationPending||native!=null){Lose("A native activation is already owned.");return false;}
        activationPending=true;return true;
    }

    private void Begin(MovementProgram next,double now)
    {
        program=next;leg=0;++commit;++attempt;preparationAt=legStartedAt=now;
        continuationBy=null;
        state=next.Prepare.Kind==PreparationKind.None?ExecutionState.Running:ExecutionState.Preparing;
        validity=ExecutionValidity.Valid;issue=null;
    }
    public void ObserveReceipt(NativeReceipt receipt)
    {
        if (state==ExecutionState.SessionEnded || receipt.Session!=session || receipt.Attempt!=attempt ||
            receipt.Observation<=lastReceipt) return;
        if (receipt.Kind==ReceiptKind.Activated)
        {
            if (!activationPending || native!=null) return;
            native=receipt.Episode;activationPending=false;
        }
        else
        {
            if (native!=receipt.Episode) return;
            native=null;activationPending=false;++attempt;
            if (receipt.Kind is ReceiptKind.Interrupted or ReceiptKind.Replaced)
            { Contradicted(receipt.Kind==ReceiptKind.Replaced?"Native action was observably replaced.":"Native action was observably interrupted."); }
        }
        lastReceipt=receipt.Observation;
    }
    public void ObservationLost(string reason)
    { validity=ExecutionValidity.ObservationLost;Stop(reason); }
    public void Contradicted(string reason)
    {
        validity=ExecutionValidity.Contradicted;issue=reason;pending=null;
        // Removing writes does not cancel the native episode or claim safety.
        RevokeControl();state=ExecutionState.Stopped;
    }
    public void Stop(string reason)
    { pending=null;path=[];branchPlan=null;state=ExecutionState.Stopped;issue=reason;RevokeControl(); }
    private void RevokeControl()
    {if(driver.Revoke()&&native==null)activationPending=false;}
    public void EndSession()
    { driver.Revoke();pending=null;path=[];branchPlan=null;program=null;native=null;activationPending=false;state=ExecutionState.SessionEnded; }
    private void Lose(string reason)
    { validity=ExecutionValidity.OutsideCoverage;Stop(reason); }
    // Arrow no-input stops translation while its native direction clock keeps
    // running. A new compiled control can own input at this real receiving
    // state without inventing a release, new origin or replacement episode.
    private bool RetainArrowAt(int at,MotionObservation actual)
        =>native is {Kind:MotionKind.Arrow}&&actual.Motion==MotionKind.Arrow&&
            course.Asset.Ports[at].HoldingMotion==MotionKind.Arrow;
    private bool EntryMatches(MovementProgram candidate,MotionObservation actual)
    {
        var entry=candidate.Entry;var at=course.Asset.Ports[candidate.From].Position;
        return actual.Grounded && actual.Motion==entry.Motion && actual.Velocity.Length()<=entry.MaximumSpeed &&
            (entry.RequiredStatus==0||actual.GroundInputStatus==entry.RequiredStatus)&&
            (candidate.Transition==null||actual.TransitionStatus is 3699 or 3700 or 3701)&&
            actual.NativeSpeed<=entry.MaximumSpeed&&actual.ExternalVelocity==Vector2.Zero&&
            (!entry.RequiresRest||course.Asset.Law.AtRest(actual.NativeSpeed,actual.ExternalVelocity))&&
            course.Asset.Law.AcceptsTargetSpeed(entry,actual.AvailableTargetSpeed)&&
            course.Asset.Law.AcceptsFrame(actual.FrameSeconds)&&
            course.Asset.Law.AcceptsClock(actual)&&
            HorizontalDistance(actual.Feet,at)<=entry.PositionRadius && Math.Abs(actual.Feet.Y-at.Y)<=entry.HeightTolerance &&
            (entry.Heading==null || Math.Abs(MathF.IEEERemainder(actual.Heading-entry.Heading.Value,MathF.Tau))<=entry.HeadingTolerance);
    }
    private bool BeforeTransition(MovementProgram candidate,double started,MotionObservation actual)
        =>actual.TransitionAt is not { } transition||
            (candidate.Transition is { } handled?
                actual.TransitionStatus is 3699 or 3700 or 3701&&
                    transition-started<=handled.LatestActivation&&transition-started>course.Asset.Law.MaximumFrameSeconds:
                started+candidate.MaximumDuration+course.Asset.Law.MaximumFrameSeconds<transition);
    private static float HorizontalDistance(Vector3 a,Vector3 b)=>Vector2.Distance(new(a.X,a.Z),new(b.X,b.Z));
}
