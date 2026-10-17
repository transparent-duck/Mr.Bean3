using System;
using System.Diagnostics;
using System.Numerics;
using MrBean.FallGuys;
using MrBean.Modules.Blunderville;
using DeepDungeon.Fsd.Dalamud.Utilities;

namespace MrBean.Modules.FallGuysPilot;

/// <summary>One owned snapshot of the local actor, read once per Framework tick.</summary>
internal readonly record struct PilotPose(
    double At,
    Vector3 Feet,
    float Facing,
    MotionKind Motion,
    bool Grounded,
    float Speed,
    Vector2 Force,
    NativeMechanicMotion Raw,
    NativeJumpMotion Jump,
    NativeCrownTransport Transport)
{
    public static double Now => (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
    public Vector2 XZ => new(Feet.X, Feet.Z);
    public bool Has(ushort status) => Raw.HasStatus(status);
    /// <summary>Native code owns movement (stun, recovery lock, transport); the pilot only holds.</summary>
    public bool Locked => Transport.InputLocked || Motion is MotionKind.Locked or MotionKind.Transport;

    public static bool TryRead(DirectMovementController movement, out PilotPose pose)
    {
        pose = default;
        if (!NativeMechanicMotion.TryRead(movement, out var raw) || !raw.StatusesAvailable) return false;
        if (!NativeJumpMotion.TryRead(out var jump)) return false;
        var transport = NativeCrownTransport.Read();
        var force = raw.ExternalVelocity is { } f ? new Vector2(f.X, f.Z) : Vector2.Zero;
        pose = new(Now, raw.Position, raw.Facing, Classify(raw, jump, transport), jump.Grounded,
            raw.Walk?.Speed ?? 0, force, raw, jump, transport);
        return true;
    }

    // Same native channel precedence the previous driver used; it was validated in live runs.
    private static MotionKind Classify(NativeMechanicMotion raw, NativeJumpMotion jump, NativeCrownTransport transport)
        => transport.Active || transport.Profile == 3 && transport.InputLocked ? MotionKind.Transport
        : !jump.Grounded ? MotionKind.Jump
        : raw.TimelineMove is { Active: true } or { Pending: true } ? MotionKind.Ice
        : raw.HasStatus(3737) ? MotionKind.Forced
        : raw.HasStatus(3702) ? MotionKind.Steering
        : raw.HasStatus(149) ? MotionKind.Locked
        : raw.SteeringClock is { CurrentTimelineId: 8514, CurrentTimestamp: null or < 65 } ? MotionKind.Locked
        : raw.HasStatus(3694) ? MotionKind.Arrow
        : raw.HasStatus(3703) ? MotionKind.Wet
        : MotionKind.Ordinary;
}

internal static class Geo
{
    public static Vector2 XZ(this Vector3 v) => new(v.X, v.Z);
    public static float Heading(Vector2 direction) => MathF.Atan2(direction.X, direction.Y);
    public static Vector2 FromHeading(float heading) => new(MathF.Sin(heading), MathF.Cos(heading));
    public static Vector2 Toward(Vector2 from, Vector2 to)
    {
        var d = to - from;
        return d.LengthSquared() > 1e-8f ? Vector2.Normalize(d) : Vector2.Zero;
    }
    public static float WrapAngle(float a) => MathF.IEEERemainder(a, MathF.Tau);

    /// <summary>
    /// Input direction u (unit) such that walkSpeed*u + force points along desired d.
    /// Returns the resulting net speed along d (may be small/negative when the force wins).
    /// </summary>
    public static Vector2 CompensateForce(Vector2 desired, Vector2 force, float walkSpeed, out float netSpeed)
    {
        if (force == Vector2.Zero || desired == Vector2.Zero) { netSpeed = walkSpeed; return desired; }
        var g = force / walkSpeed;
        float dg = Vector2.Dot(desired, g);
        float disc = dg * dg - g.LengthSquared() + 1;
        if (disc < 0) { netSpeed = 0; return desired; }
        float k = dg + MathF.Sqrt(disc);
        var u = desired * k - g;
        netSpeed = walkSpeed * k;
        return u.LengthSquared() > 1e-8f ? Vector2.Normalize(u) : desired;
    }
}
