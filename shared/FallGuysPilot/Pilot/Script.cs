using System;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot;

internal sealed class PilotContext
{
    public PilotPose Pose;
    public double Now;
    public required Actuator Act { get; init; }
    public required HazardField Hazards { get; init; }
    public required Action<string, object> Log { get; init; }
    public double ReleasedAt = double.NaN;
    public double ExpectedReleaseAt = double.NaN;
    public double LastHitAt;
    public double LastWindHitAt = double.NegativeInfinity;
    public double LastDeliveryAt = double.NegativeInfinity;
    public required Water.WaterSession Water { get; init; }
    public Courses.RoutePlanningTimes PlanningTimes { get; init; } = new();
}

/// <summary>Axis-aligned box used to decide which step owns the actor's current position.</summary>
internal readonly record struct Zone(float XMin, float XMax, float ZMin, float ZMax, float YMin = -1000, float YMax = 1000)
{
    public static Zone ZBand(float zNear, float zFar, float yMin = -1000, float yMax = 1000, float halfWidth = 40)
        => new(-halfWidth, halfWidth, MathF.Min(zNear, zFar), MathF.Max(zNear, zFar), yMin, yMax);
    public bool Contains(Vector3 p) => p.X >= XMin && p.X <= XMax && p.Z >= ZMin && p.Z <= ZMax && p.Y >= YMin && p.Y <= YMax;
    public Zone Offset(float dx) => this with { XMin = XMin + dx, XMax = XMax + dx };
}

/// <summary>
/// One section of a course script. Tick must never "fail": it always issues the best command
/// for the actual state. Anything unexpected is resolved by relocalization over Zones.
/// </summary>
internal abstract class Step
{
    public required string Name { get; init; }
    public required Zone Zone { get; init; }
    /// <summary>Step to continue with after completion; null = the next step in the list.</summary>
    public string? Next { get; init; }
    public virtual void Enter(PilotContext c) { }
    public virtual void PrepareStart(PilotContext c) { }
    /// <summary>Observe native movement ownership before the engine holds input.</summary>
    public virtual bool ObserveMotion(PilotContext c) => false;
    public abstract bool Tick(PilotContext c);
    /// <summary>World direction native forced movement should take while this step owns the actor.</summary>
    public virtual Vector2? Aim(PilotContext c) => null;
}

internal sealed class WalkStep : Step
{
    public required Vector3[] Points { get; init; }
    public float ArriveRadius { get; init; } = 0.35f;
    private PointFollower follower = null!;
    public override void Enter(PilotContext c) => follower = new(Points, ArriveRadius);
    public override bool Tick(PilotContext c) => follower.Tick(c);
    public override Vector2? Aim(PilotContext c) => follower?.Aim(c);
}

/// <summary>Run to Takeoff, snap within the nudge bound, jump along Direction, and steer in the air.</summary>
internal sealed class JumpStep : Step
{
    public required Vector3 Takeoff { get; init; }
    public required Vector2 Direction { get; init; }
    public float TriggerRadius { get; init; } = 0.25f;
    private bool launched, airborne;
    private double launchedAt;

    public override void Enter(PilotContext c) { launched = airborne = false; c.Act.ClearJump(); }

    public override bool Tick(PilotContext c)
    {
        var pose = c.Pose; var dir = Vector2.Normalize(Direction);
        if (launched)
        {
            if (!pose.Grounded) { airborne = true; c.Act.Walk(dir); return false; }
            if (airborne) return true;
            if (c.Now - launchedAt < 0.5) { c.Act.Walk(dir); return false; }
            launched = false; // the jump never left the ground; approach again
        }
        var p = pose.XZ; var takeoff = Takeoff.XZ();
        float distance = Vector2.Distance(p, takeoff);
        // Passing the takeoff line along the jump direction also triggers (fast approach).
        bool passed = Vector2.Dot(p - takeoff, dir) >= 0 && distance < 0.6f;
        if (pose.Grounded && (distance <= TriggerRadius || passed))
        {
            if (distance > 0.05f) c.Act.Nudge(Takeoff with { Y = pose.Feet.Y }, c.Now, 0);
            c.Act.Jump(dir, c.Now);
            launched = true; airborne = false; launchedAt = c.Now;
            c.Log("jump", new { step = Name, at = c.Now, feet = pose.Feet, distance });
            return false;
        }
        c.Act.Walk(Geo.Toward(p, takeoff));
        return false;
    }

    public override Vector2? Aim(PilotContext c) => Vector2.Normalize(Direction);
}

/// <summary>
/// Go to Hold, wait until the planner finds a clean (passage, delay), then run it.
/// A failed search stays pending; it never launches a known-hit passage.
/// </summary>
internal sealed class GateStep : Step
{
    public required Vector3 Hold { get; init; }
    /// <summary>Optional: choose the waiting spot from the actual position on entry (e.g. keep the lane X).</summary>
    public Func<PilotPose, Vector3>? HoldSelector { get; init; }
    public required Passage[] Options { get; init; }
    public double PlanHorizon { get; init; } = 6;
    private enum Phase { Approach, Wait, Run }
    private Phase phase;
    private double waitStart, nextPlan;
    private PointFollower approach = null!;
    private Passage? running;
    private Vector3 hold;

    public override void Enter(PilotContext c)
    {
        phase = Phase.Approach; running = null; nextPlan = 0;
        hold = HoldSelector?.Invoke(c.Pose) ?? Hold;
        approach = new([hold], 0.2f);
    }

    public override bool Tick(PilotContext c)
    {
        if (phase == Phase.Approach)
        {
            if (!approach.Tick(c)) return false;
            c.Act.Nudge(hold with { Y = c.Pose.Feet.Y }, c.Now);
            phase = Phase.Wait; waitStart = c.Now;
        }
        if (phase == Phase.Wait)
        {
            c.Act.Hold();
            if (c.Now < nextPlan) return false;
            nextPlan = c.Now + 0.15;
            var plan = Planner.Choose(c.Hazards, Options, c.Pose.XZ, c.Now, PlanHorizon, PilotEngine.BodyRadius);
            if (plan == null || plan.Delay > 0.05 || !plan.Clean) return false;
            running = plan.Passage; running.Begin(c); phase = Phase.Run;
            c.Log("gate-go", new { step = Name, passage = running.Name, waited = c.Now - waitStart, plan.Hits, plan.Unknowns });
        }
        return running!.Follow(c);
    }

    public override Vector2? Aim(PilotContext c) => running?.Aim(c) ?? Geo.Toward(c.Pose.XZ, hold.XZ());
    public string Status => phase == Phase.Run ? $"run {running?.Name}" : phase.ToString().ToLowerInvariant();
}

/// <summary>Run into the goal and keep pressing toward its far point; never completes.</summary>
internal sealed class FinishStep : Step
{
    public required Vector3[] Points { get; init; }
    private PointFollower follower = null!;
    public override void Enter(PilotContext c) => follower = new(Points, 0.4f);
    public override bool Tick(PilotContext c) { if (follower.Tick(c)) c.Act.Hold(); return false; }
    public override Vector2? Aim(PilotContext c) => follower?.Aim(c);
}

internal abstract class CourseScript
{
    public abstract CourseId Course { get; }
    public abstract HazardSpec[] Hazards { get; }
    /// <summary>Built per round; steps may keep per-round state.</summary>
    public abstract Step[] Build(PilotPose start);
    /// <summary>Optional per-frame override (e.g. Crystal goal selection, Ring survival). Return true when handled.</summary>
    public virtual bool Override(PilotContext c, PilotEngine engine) => false;
}

/// <summary>Executes a script: global native-state handling, relocalization, step progression.</summary>
internal sealed class PilotEngine : IDisposable
{
    // Effective radius added to every AoE footprint. Waiting 5.48 m from a radius-5 water pulse
    // was never hit in Hit-20261014-093642, so the effective pad is below 0.48 m.
    public const float BodyRadius = 0.3f;
    private readonly Step[] steps;
    private readonly CourseScript script;
    private double outsideSince = double.NaN;
    public int Index { get; private set; }
    public Step? Current => Index < steps.Length ? steps[Index] : null;
    public int Count => steps.Length;
    public void Dispose()
    {
        foreach (var step in steps) if (step is IDisposable owned) owned.Dispose();
    }

    public PilotEngine(CourseScript script, PilotPose start, PilotContext c)
    {
        this.script = script;
        steps = script.Build(start);
        Index = Locate(start.Feet, 0) ?? 0;
        Current?.Enter(c);
    }

    public void Tick(PilotContext c)
    {
        var pose = c.Pose;
        if (Current?.ObserveMotion(c) == true) return;
        if (pose.Locked || pose.Has(3737) || pose.Motion is MotionKind.Forced or MotionKind.Ice) { c.Act.Hold(); return; }
        if (script.Override(c, this)) return;
        // Forward relocalization is immediate; going back (knockback, respawn) needs a dwell so
        // that a step ending a little early (arrival radius) does not bounce to its predecessor.
        if (Current is { } step && !step.Zone.Contains(pose.Feet))
        {
            if (double.IsNaN(outsideSince)) outsideSince = c.Now;
            if (pose.Grounded && Locate(pose.Feet, Index) is { } found && found != Index &&
                (found > Index || c.Now - outsideSince >= 1.0))
                MoveTo(c, found, "relocalize");
        }
        else outsideSince = double.NaN;
        if (Current is not { } active) { c.Act.Hold(); return; }
        if (active.Tick(c))
        {
            int next = active.Next is { } name ? Array.FindIndex(steps, s => s.Name == name) : Index + 1;
            MoveTo(c, next < 0 ? Index + 1 : next, "done");
        }
    }

    public void PrepareStart(PilotContext c) => Current?.PrepareStart(c);

    public void MoveTo(PilotContext c, int index, string reason)
    {
        c.Log("step", new { from = Current?.Name, to = index < steps.Length ? steps[index].Name : "end", reason, at = c.Now, feet = c.Pose.Feet });
        Index = index;
        outsideSince = double.NaN;
        Current?.Enter(c);
    }

    /// <summary>Nearby forward steps first (progress), then any step from the start (setback).</summary>
    private int? Locate(Vector3 feet, int from)
    {
        for (int i = from; i < Math.Min(steps.Length, from + 3); ++i) if (steps[i].Zone.Contains(feet)) return i;
        for (int i = 0; i < steps.Length; ++i) if (steps[i].Zone.Contains(feet)) return i;
        return null;
    }
}
