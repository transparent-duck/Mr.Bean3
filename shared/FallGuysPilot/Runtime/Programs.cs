using System.Numerics;
using System.Text.Json.Serialization;
using MrBean.FallGuys.Mechanics;

namespace MrBean.FallGuys;

public enum CourseId { Hit, Knight, Ring, Crystal, Crown }
public enum MotionKind { Ordinary, Jump, Steering, Ice, Arrow, Wet, Locked, Forced, Transport }
public enum GoalKind { Finish, Survive, Deliver }
public enum ConditionKind { Hazard, Support }
public enum PreparationKind { None, FaceAndAccelerate }
public enum TimeoutKind { StopAtHold, TakeProgram }

public readonly record struct TimeWindow(double Start, double End)
{
    public bool Contains(double time) => time >= Start && time < End;
    public bool Contains(double start, double end) => start >= Start && end < End;
}

// A port is a physical receiving domain. It is never an arbitrary search cutoff.
public sealed record Port(int Id, Vector3 Position, float Radius, float HeightTolerance,
    int[] OccupiedConditions, bool CanHold, bool Goal,MotionKind HoldingMotion=MotionKind.Ordinary);
public sealed record EntryDomain(MotionKind Motion, float MaximumSpeed, float PositionRadius,
    float HeightTolerance, float? Heading, float HeadingTolerance,
    float RequiredTargetSpeed,float MaximumFrameSeconds,bool RequiresRest,bool ClampsGroundSpeed=false,ushort RequiredStatus=0);
public sealed record Preparation(PreparationKind Kind, double MaximumSeconds,
    float MaximumDisplacement, float? Heading);
public sealed record Crossing(int Condition, double Enter, double Exit);
public sealed record CompiledSource(int Id, uint Action, uint BaseId, Vector2 Slot);
public sealed record Condition(int Id, int Source, ConditionKind Kind, MechanicFootprint? Shape,
    uint SupportId=0, bool UnknownEnvelope=false);

// Every leg carries its required data. Missing motion data cannot become Walk.
[JsonPolymorphic(TypeDiscriminatorPropertyName = "kind")]
[JsonDerivedType(typeof(WalkLeg), "walk")]
[JsonDerivedType(typeof(JumpLeg), "jump")]
[JsonDerivedType(typeof(SteeringPassLeg), "steering-pass")]
[JsonDerivedType(typeof(IceLeg), "ice")]
[JsonDerivedType(typeof(ReleaseLeg), "release")]
[JsonDerivedType(typeof(CaptureLeg), "capture")]
[JsonDerivedType(typeof(CrownTransportLeg), "crown-transport")]
[JsonDerivedType(typeof(ArrowLeg), "arrow")]
public abstract record ProgramLeg(double Duration, Vector3 End);
public sealed record WalkLeg(double Duration, Vector3 End, Vector3[] Path, float Speed,
    float TrackingRadius,GroundSegment[] Segments,double MinimumSeconds,float EntryHeightBelow=0,
    bool ControlsForced=false,Vector2? CompensatedForce=null,float VelocityError=0,bool AllowsHigherTargetSpeed=false) : ProgramLeg(Duration, End);
public sealed record CaptureLeg(double Duration,Vector3 End,float Radius,float MaximumSpeed)
    : ProgramLeg(Duration,End);
public sealed record JumpLeg(double Duration, Vector3 End, Vector3 Takeoff, Vector2 Velocity,
    double ActivationMaximum, float LandingRadius, Vector3 LandingFirst,Vector3 LandingLast,
    float MaximumFrameSeconds,float MinimumOriginY,float MaximumOriginY,
    double MinimumSeconds,float OccupancyRadius) : ProgramLeg(Duration, End);
// Ordinary approach and native steering use the same northward speed. Status
// installation therefore changes ownership, not the spatial or timing law.
// Release can occur anywhere in the compiled terminal strip; its recovery and
// ordinary receiving control remain inside the same complete program.
// Section families compare positions at the same Z, rather than at the same
// time. Radius bounds X there; TimeRadius bounds arrival time there. A lateral
// write changes only the former. Null preserves existing circular assets.
public sealed record SteeringSection(float HeadingRadius,float InterpolationLateral=0,double InterpolationTime=0);
public sealed record SteeringEnvelope(double Seconds,Vector3 Position,float Radius,double TimeRadius=0,SteeringSection? Section=null);
public sealed record SteeringTurn(float AtZ,float Heading);
public sealed record SteeringCorrection(float AtZ,Vector3 Anchor,Vector3 Before,float Heading,float MaximumDisplacement,
    float BeforeRadius,double TimeRadius,float MinimumZ,float MaximumZ,bool Section=false);
public sealed record SteeringPassLeg(double Duration,Vector3 End,Vector3 Start,Vector3 ReleaseLast,
    float Speed,float EntryRadius,double ReleaseSeconds,double MinimumSeconds,SteeringEnvelope[] Envelope,
    float MinimumHeight,float MaximumHeight,SteeringTurn[] Turns,SteeringCorrection[]? Corrections=null,float ReleaseRadius=0)
    : ProgramLeg(Duration, End)
{
    [JsonIgnore] public double MaximumTimeRadius {get;}=Envelope.Max(s=>s.TimeRadius+(s.Section?.InterpolationTime??0));
    [JsonIgnore] public float TerminalRadius=>ReleaseRadius>0?ReleaseRadius:Envelope[^1].Radius;
}
public sealed record IceLeg(double Duration,Vector3 End,Vector3 Start,float EntryRadius,
    Vector3 OriginFirst,Vector3 OriginLast,float OriginRadius,float MinimumHeight,float MaximumHeight,
    double ActivationMaximum,double MinimumSeconds,float ReceivingRadius,WalkLeg? Approach,bool Exits)
    : ProgramLeg(Duration,End);
public sealed record ArrowLeg(double Duration,Vector3 End,WalkLeg Ground,bool Enters,bool Exits)
    : ProgramLeg(Duration,End);
public sealed record ReleaseLeg(double Duration, Vector3 End, MotionKind NativeMotion,
    double MinimumSeconds,float EndTick,float TicksPerSecond,Vector3 Last,float Radius) : ProgramLeg(Duration, End);
// The approach and server-triggered movement form one non-waitable program.
// ActivationSeconds is its supported receipt deadline, not a server latency law.
public sealed record CrownTransportLeg(double Duration,Vector3 End,Vector3 Destination,WalkLeg Approach,
    double ActivationSeconds,double MinimumSeconds,float FlightMinimum,float FlightMaximum,
    Vector3 OriginFirst,float OriginMinimumY,float OriginMaximumY,float OccupancyRadius,float ReceivingRadius)
    : ProgramLeg(Duration,End);

// A non-waitable chain is one program through its real receiving port.
// Crossings already include preparation, tracking and activation uncertainty.
public sealed record DirectionalTransition(double LatestActivation,double MaximumNativeSeconds);
public sealed record MovementProgram(int Id, int From, int To, EntryDomain Entry,
    Preparation Prepare, ProgramLeg[] Legs, double MinimumDuration, double MaximumDuration,
    Crossing[] Crossings, int[] RequiredConditions, bool Recovery,DirectionalTransition? Transition=null);
// The port sequence is fixed. Each step lists its finite authored controls,
// including any control that consumes a known directional warning.
public sealed record ObservationBranch(int Id,int[] Need,int[][] ProgramChoices,int NextPort);
public sealed record ObservationPolicy(int Port, string Goal, int[] Need,
    ObservationBranch[] Cases, double DecisionSeconds, TimeoutKind Timeout, int TimeoutProgram,ushort RequiredStatus=0);
public sealed record CourseAsset(CourseId Course, string Scene, string Provenance, GoalKind GoalKind,
    Port[] Ports, MovementProgram[] Programs, CompiledSource[] Sources, Condition[] Conditions,
    ObservationPolicy[] Observations, int[] StartPorts, double[] RemainingCost,MotionLaw Law,DeliveryGoal[]? DeliveryGoals=null);
public sealed record DeliveryGoal(ushort Carry,int Port,uint LayoutId);

// A query over compiled dependencies. An omitted condition is unavailable,
// never implicitly safe. Masks and IDs are prepared once with the asset.
public sealed class WindowScope
{
    public int[] Ports {get;}
    public int[] Programs {get;}
    internal bool[] Conditions {get;}
    internal bool[] Sources {get;}
    public int ConditionCount {get;}
    internal WindowScope(CourseAsset asset,int[] ports,int[] programs)
    {
        Ports=ports;Programs=programs;
        Conditions=new bool[asset.Conditions.Length];Sources=new bool[asset.Sources.Length];
        foreach(int port in ports)foreach(int id in asset.Ports[port].OccupiedConditions)Conditions[id]=true;
        foreach(int program in programs)
        {
            foreach(int id in asset.Programs[program].RequiredConditions)Conditions[id]=true;
            foreach(var crossing in asset.Programs[program].Crossings)Conditions[crossing.Condition]=true;
        }
        foreach(var condition in asset.Conditions)if(Conditions[condition.Id])Sources[condition.Source]=true;
        ConditionCount=Conditions.Count(v=>v);
    }
}

// This is the runtime's entire planning input. It contains no terrain grid,
// collision mesh, trajectory generator or callable spatial planner.
public sealed class CompiledCourse
{
    public CourseAsset Asset { get; }
    public int[][] Outgoing { get; }
    public ObservationPolicy?[] Observations { get; }
    public WindowScope[] ProgramScopes {get;}
    public WindowScope[] PortScopes {get;}
    public CompiledCourse(CourseAsset asset)
    {
        if(asset.Law.GeometryPrecision<=0||asset.Law.NativePositionBudget<=0)
            throw new InvalidDataException("Course lacks separate geometry and native-control bounds; recompile it.");
        Asset = asset;
        Outgoing = new int[asset.Ports.Length][];
        Observations = new ObservationPolicy?[asset.Ports.Length];
        var edges = new List<int>[asset.Ports.Length];
        for (int n = 0; n < edges.Length; ++n)
        {
            if (asset.Ports[n].Id != n) throw new InvalidDataException("Port IDs must be dense.");
            edges[n] = [];
        }
        foreach (var program in asset.Programs)
        {
            if (program.Id < 0 || program.Id >= asset.Programs.Length || asset.Programs[program.Id] != program ||
                program.From < 0 || program.From >= edges.Length || program.To < 0 || program.To >= edges.Length ||
                program.Legs.Length == 0 || program.MinimumDuration <= 0 ||
                program.MaximumDuration < program.MinimumDuration || !double.IsFinite(program.MaximumDuration))
                throw new InvalidDataException("Invalid movement program.");
            foreach(var pass in program.Legs.OfType<SteeringPassLeg>())
            {
                if(pass.Turns==null)throw new InvalidDataException("Steering control has no authored turn table.");
                double phase=0;SteeringSection? previousSection=null;bool firstSample=true;
                foreach(var sample in pass.Envelope)
                {
                    if(!double.IsFinite(sample.TimeRadius)||sample.TimeRadius<0||
                        !firstSample&&(previousSection!=null)==(sample.Section!=null)&&sample.TimeRadius<phase||
                        !float.IsFinite(sample.Radius)||sample.Radius<0||
                        sample.Section is { } section&&(!float.IsFinite(section.HeadingRadius)||section.HeadingRadius<0||
                            !float.IsFinite(section.InterpolationLateral)||section.InterpolationLateral<0||
                            !double.IsFinite(section.InterpolationTime)||section.InterpolationTime<0))
                        throw new InvalidDataException("Invalid steering position/time uncertainty.");
                    phase=sample.TimeRadius;previousSection=sample.Section;firstSample=false;
                }
                float previous=pass.Start.Z;
                foreach(var turn in pass.Turns)
                {
                    if(!float.IsFinite(turn.AtZ)||!float.IsFinite(turn.Heading)||turn.AtZ>=previous||MathF.Cos(turn.Heading)>=0)
                        throw new InvalidDataException("Steering turn gates are not a northward ordered control.");
                    previous=turn.AtZ;
                }
                previous=pass.Start.Z;
                foreach(var correction in pass.Corrections??[])
                {
                    if(!float.IsFinite(correction.AtZ)||correction.AtZ>=previous||correction.AtZ<=pass.End.Z||
                        correction.Anchor.Z!=correction.AtZ||correction.Before.Z!=correction.AtZ||
                        !float.IsFinite(correction.Anchor.X)||!float.IsFinite(correction.Before.X)||
                        !(correction.MaximumDisplacement>0&&correction.MaximumDisplacement<.3f)||
                        correction.MinimumZ>correction.AtZ||correction.MaximumZ<correction.AtZ||
                        !double.IsFinite(correction.TimeRadius)||correction.TimeRadius<0||MathF.Cos(correction.Heading)>=0)
                        throw new InvalidDataException("Invalid bounded lateral steering correction.");
                    previous=correction.AtZ;
                }
            }
            foreach (var crossing in program.Crossings)
                if (crossing.Condition < 0 || crossing.Condition >= asset.Conditions.Length ||
                    crossing.Enter < 0 || crossing.Exit < crossing.Enter || crossing.Exit > program.MaximumDuration)
                    throw new InvalidDataException("Invalid precomputed crossing.");
            edges[program.From].Add(program.Id);
        }
        for (int n = 0; n < edges.Length; ++n) Outgoing[n] = [.. edges[n]];
        ProgramScopes=asset.Programs.Select(p=>new WindowScope(asset,p.From==p.To?[p.From]:[p.From,p.To],[p.Id])).ToArray();
        PortScopes=asset.Ports.Select(p=>new WindowScope(asset,[p.Id],[])).ToArray();
        foreach (var policy in asset.Observations)
        {
            if (!asset.Ports[policy.Port].CanHold || policy.Need.Length == 0&&policy.RequiredStatus==0 || policy.DecisionSeconds <= 0 ||
                policy.Cases.Length == 0) throw new InvalidDataException("Invalid observation policy.");
            foreach (var branch in policy.Cases)
            {
                int at=policy.Port;
                foreach(var choices in branch.ProgramChoices)
                {
                    if(choices.Length==0||choices.Any(id=>id<0||id>=asset.Programs.Length))
                        throw new InvalidDataException("Observation step has no known program choices.");
                    int next=asset.Programs[choices[0]].To;
                    if(choices.Any(id=>asset.Programs[id].From!=at||asset.Programs[id].To!=next))
                        throw new InvalidDataException("Observation choices do not share the fixed port sequence.");
                    at=next;
                }
                if(at!=branch.NextPort||branch.ProgramChoices.Length==0||
                    !asset.Ports[at].Goal&&!asset.Observations.Any(p=>p.Port==at)||
                    branch.Need.Any(source=>!policy.Need.Contains(source)))
                    throw new InvalidDataException("Observation branch has no explicit continuation.");
            }
            if (policy.Timeout == TimeoutKind.TakeProgram &&
                (!asset.Programs[policy.TimeoutProgram].Recovery || asset.Programs[policy.TimeoutProgram].From != policy.Port))
                throw new InvalidDataException("Observation timeout has no applicable recovery.");
            Observations[policy.Port] = policy;
        }
        if (asset.RemainingCost.Length != asset.Ports.Length) throw new InvalidDataException("Missing compiled remaining costs.");
    }
}
