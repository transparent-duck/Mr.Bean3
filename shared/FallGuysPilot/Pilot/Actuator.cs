using System;
using System.Numerics;
using MrBean.Modules.Blunderville;
using MrBean.Modules.FallGuysPilot.Courses;
using MrBean.Host;
using DeepDungeon.Fsd.Dalamud.Utilities;
using FFXIVClientStructs.FFXIV.Client.Game;
using NativeCharacter = FFXIVClientStructs.FFXIV.Client.Game.Character.Character;
using OmenTools.Interop.Game.Models.Native;

namespace MrBean.Modules.FallGuysPilot;

/// <summary>
/// The only owner of movement writes. Every command is idempotent and may be issued every
/// frame; nothing here validates routes or stops the pilot. Native adapters are the ones
/// already proven in live runs (DirectMovementController, jump speed hook, status callbacks).
/// </summary>
internal sealed unsafe class Actuator : IDisposable
{
    public const float WalkSpeed = 6f;
    public const float NudgeLimit = 0.29f; // user-approved bound for direct position correction

    public readonly DirectMovementController Movement = new("Fall Guys pilot")
        { ClampFinalStep = false, Precision = 0, CancelOnManualInput = true };
    private readonly NativeJumpLaunch launch;
    private readonly NativeControlCallback callback = new();
    private double jumpArmedAt = double.NaN, lastNudgeAt;
    private bool jumpDispatched;
    private bool arrowEntryNotified, arrowRequested, arrowLatched;
    private double arrowNotBefore;
    private float? arrowBefore;
    private KnightArrowPolicy? arrowRegions;
    public float ArrowRate { get; set; } = -MathF.Tau;
    public long ArrowEntryWrites { get; private set; }
    public long ArrowTurnSamples { get; private set; }
    public double ArrowTurnSeconds { get; private set; }
    public double ArrowTurnRadians { get; private set; }

    /// <summary>Desired arrow heading. Native gain may write it once; later turns wait for rotation.</summary>
    public float? ArrowPreference { get; set; }
    // Chosen before course prefetch. Route targets must not overwrite the one
    // native entry direction; subsequent turns follow the observed arrow.
    public float? ArrowEntryHeading { get; set; }
    internal bool ArrowLatched => arrowLatched;
    public int Nudges { get; private set; }
    public float LargestNudge { get; private set; }
    public long Jumps { get; private set; }
    public bool ImmediateJump { get; set; }
    public bool CrownStairBoost { get; }
    public double JumpDispatchAt { get; private set; }
    public Vector3 JumpDispatchFeet { get; private set; }
    public bool JumpAccepted { get; private set; }
    public float JumpSpeedBefore { get; private set; }
    public float JumpTargetSpeed { get; private set; }
    public long AccelerationWrites => launch.AccelerationWrites;
    public double AccelerationAt => launch.AccelerationAt;
    public float AccelerationBefore => launch.AccelerationBefore;
    public float AccelerationTarget => launch.AccelerationTarget;
    public Vector3 AccelerationPosition => launch.AccelerationPosition;
    public string LastCommand { get; private set; } = "none";
    public bool CancelledByUser => Movement.WasCancelledByUser;
    public Exception? NativeFault => callback.Fault;

    public Actuator(bool crownStairBoost = false)
    {
        CrownStairBoost = crownStairBoost;
        launch = new(Movement);
        if (crownStairBoost) launch.DirectionalTargetSpeed = feet =>
            PilotCourseGeometry.CrownBoostRegion(feet) ? PilotCourseGeometry.CrownBoostSpeed : null;
        // Recreating the actuator inside an existing status is not a new entry.
        if (Service.LocalPlayer is { } player)
        {
            var manager = ((NativeCharacter*)player.Address)->GetStatusManager();
            arrowEntryNotified = manager != null && manager->GetStatusIndex(3694) >= 0;
        }
        Movement.PrepareDirectionalInput();
        Movement.ObserveUserInput = true;
        Movement.BeforeSteeringInput = callback.InputRead;
        callback.Step = OnEligibleInput;
        callback.ArrowGain = OnArrowGain;
        callback.Commit = ObserveArrowTurn;
        callback.ForcedGain = () =>
        {
            if (Movement.Enabled) Hold();
            return false;
        };
        callback.Enable();
    }

    public void Walk(Vector2 direction)
    {
        ClearArrowRegions();
        arrowRequested = false;
        arrowLatched = false;
        if (direction == Vector2.Zero) { Hold(); return; }
        Movement.SetWorldDirectionalInput(direction);
        LastCommand = "walk";
    }

    /// <summary>Walk so that the net ground velocity (input + external force) points along desired.</summary>
    public void WalkNet(Vector2 desired, Vector2 force)
        => Walk(Geo.CompensateForce(desired, force, WalkSpeed, out _));

    public void Hold()
    {
        ClearArrowRegions();
        arrowRequested = false;
        arrowLatched = false;
        ClearJump();
        Movement.Hold();
        LastCommand = "hold";
    }

    public void Face(float heading)
    {
        if (!float.IsFinite(heading)) throw new ArgumentException("Facing requires a finite heading.");
        Hold();
        // Hit's forced-march preparation uses an exact facing while stationary.
        // Clear ordinary turn input before writing so it cannot turn us back.
        if (Service.LocalPlayer is { } player && Math.Abs(Geo.WrapAngle(heading - player.Rotation)) > .0001f)
            ((NativeCharacter*)player.Address)->SetRotation(Geo.WrapAngle(heading));
        LastCommand = "face";
    }

    public void Arrow(Vector2 direction, double notBefore)
    {
        ClearArrowRegions();
        if (direction == Vector2.Zero) { Hold(); return; }
        float heading = Geo.Heading(direction);
        // A new graph heading needs a new native alignment, even if its turn
        // is smaller than the drift threshold. Carrying the previous latch
        // skipped the 0.02 s turn in Knight-055640, then cost a full rotation.
        if (!arrowRequested || Math.Abs(Geo.WrapAngle(heading - arrowRequestedHeading)) > .0001f)
            arrowLatched = false;
        ArrowPreference = arrowRequestedHeading = heading;
        arrowRequested = true; arrowNotBefore = notBefore;
        // The native input callback decides when the rotating arrow is aligned.
        // Holding here also prevents stale ordinary input before that callback.
        Movement.Hold(); LastCommand = "arrow-turn";
    }

    private float arrowRequestedHeading;

    internal void ArrowRegions(KnightArrowPolicy policy)
    {
        if (arrowRegions != policy)
        {
            ClearArrowRegions(); arrowRegions = policy;
            arrowRequested = arrowLatched = false; ArrowPreference = null;
            Movement.Hold();
        }
    }

    private void ClearArrowRegions()
    {
        arrowRegions?.Reset(); arrowRegions = null;
    }

    public void Steer(Vector3 feet, Vector2 direction)
    {
        ClearArrowRegions();
        if (direction == Vector2.Zero) { Hold(); return; }
        direction = Vector2.Normalize(direction);
        Movement.SetSteeringInput(feet + new Vector3(direction.X, 0, direction.Y) * 4, direction);
        LastCommand = "steer";
    }

    /// <summary>Jump along direction at full speed. The speed hook dispatches at the native step;
    /// a direct dispatch follows if the hook did not fire (for example a non-standard target speed).</summary>
    public void Jump(Vector2 direction, double now)
    {
        Walk(direction);
        jumpDispatched = false;
        jumpArmedAt = now;
        if (ImmediateJump)
        {
            launch.Cancel();
            TryImmediateJump();
        }
        else launch.Arm(Movement, null, 1e-3f, Vector2.Normalize(direction), () => true, DispatchJump);
        LastCommand = "jump";
    }

    public void Release()
    {
        ClearArrowRegions();
        ClearJump();
        Movement.Release();
        ArrowPreference = null;
        ArrowEntryHeading = null;
        arrowRequested = false;
        LastCommand = "release";
    }

    /// <summary>Direct position correction within the approved bound. Returns false when outside it.</summary>
    public bool Nudge(Vector3 target, double now, double minimumInterval = 0.25)
    {
        if (Service.LocalPlayer is not { IsDead: false } player) return false;
        float distance = Vector3.Distance(player.Position, target);
        if (distance >= NudgeLimit || now - lastNudgeAt < minimumInterval) return false;
        ((FFXIVClientStructs.FFXIV.Client.Game.Object.GameObject*)player.Address)->SetPosition(target.X, target.Y, target.Z);
        lastNudgeAt = now; ++Nudges; LargestNudge = MathF.Max(LargestNudge, distance);
        return true;
    }

    public void Update(double now)
    {
        if (Service.LocalPlayer is { } player)
        {
            var manager = ((NativeCharacter*)player.Address)->GetStatusManager();
            if (manager != null && manager->GetStatusIndex(3694) < 0) arrowEntryNotified = arrowLatched = false;
        }
        if (JumpPending && ImmediateJump) TryImmediateJump();
        else if (JumpPending && now - jumpArmedAt > 0.12)
        {
            launch.Cancel();
            TryImmediateJump();
        }
    }

    public bool JumpPending => !double.IsNaN(jumpArmedAt) && !jumpDispatched;
    public void ClearJump() { jumpArmedAt = double.NaN; launch.Cancel(); }

    private void TryImmediateJump()
    {
        // After a planned hold, Walk has not necessarily reached the native
        // controller yet. Ring-080750/081508 dispatched with target speed 0
        // and crossed at only 2 m/s. Retry after native input resumes, without
        // exceeding its target speed or using the timed fallback below it.
        var walk = PlayerMoveControllerWalk.Instance();
        JumpSpeedBefore = walk == null ? 0 : walk->CurrentSpeed;
        JumpTargetSpeed = walk == null ? 0 : walk->GetTargetSpeed();
        if (walk == null || Movement.DirectionalInputAllowed != true ||
            !float.IsFinite(JumpTargetSpeed) || JumpTargetSpeed <= 0) return;
        if (JumpTargetSpeed > walk->CurrentSpeed) walk->CurrentSpeed = JumpTargetSpeed;
        DispatchJump();
    }

    private void DispatchJump()
    {
        jumpDispatched = true;
        ++Jumps;
        JumpDispatchAt = PilotPose.Now;
        JumpDispatchFeet = Service.LocalPlayer?.Position ?? default;
        JumpAccepted = ActionManager.Instance()->UseAction(ActionType.GeneralAction, 2);
    }

    private void OnEligibleInput()
    {
        arrowBefore = null;
        if (Service.LocalPlayer is not { } player) return;
        var manager = ((NativeCharacter*)player.Address)->GetStatusManager();
        if (manager == null || manager->GetStatusIndex(3694) < 0) return;
        if (!Movement.TryReadForcedDirection(out float actual)) return;
        arrowBefore = actual;
        if (arrowRegions is { } policy && Movement.Enabled)
        {
            if (policy.Next(player.Position, actual, PilotPose.Now))
            { Movement.SetWorldDirectionalInput(Geo.FromHeading(actual)); LastCommand = "arrow-region-walk"; }
            else { Movement.Hold(); LastCommand = "arrow-region-turn"; }
            return;
        }
        if (!arrowRequested || ArrowPreference is not { } heading || !Movement.Enabled) return;
        if (RouteExecution.ArrowWalk(ref arrowLatched, actual, heading, PilotPose.Now,
            arrowNotBefore, callback.FrameSeconds, ArrowRate))
        {
            arrowLatched = true;
            Movement.SetWorldDirectionalInput(Geo.FromHeading(actual));
            LastCommand = "arrow-walk";
        }
        else { arrowLatched = false; Movement.Hold(); LastCommand = "arrow-turn"; }
    }

    private void OnArrowGain()
    {
        // Original gain can execute the native input callback before returning.
        // That observation must not consume this one permitted entry write.
        if (arrowEntryNotified) return;
        arrowEntryNotified = true;
        if (ArrowEntryHeading is { } heading && Movement.Enabled && Movement.TryWriteForcedDirection(heading)) ++ArrowEntryWrites;
    }

    private void ObserveArrowTurn()
    {
        if (arrowBefore is not { } before || Movement.DirectionalInputAllowed != false || callback.FrameSeconds <= 0 ||
            !Movement.TryReadForcedDirection(out float after)) return;
        float delta = Geo.WrapAngle(after - before);
        ++ArrowTurnSamples; ArrowTurnSeconds += callback.FrameSeconds; ArrowTurnRadians += delta;
    }

    public void Dispose()
    {
        Release();
        callback.Dispose();
        launch.Dispose();
        Movement.Dispose();
    }
}
