using System;
using System.Diagnostics;
using System.Numerics;
using MrBean.FallGuys;
using MrBean.Host;
using Dalamud.Hooking;
using DeepDungeon.Fsd.Dalamud.Utilities;
using OmenTools.Interop.Game.Models;
using OmenTools.Interop.Game.Models.Native;

namespace MrBean.Modules.Blunderville;

// Copied and reduced from the measured launch primitive. PlayerMoveControllerWalk
// exposes the state, but not this speed-update function in the current references.
internal sealed unsafe class NativeJumpLaunch : IDisposable
{
    private delegate void SpeedUpdate(PlayerMoveControllerWalk* walk);
    private readonly Hook<SpeedUpdate> hook;
    private readonly DirectMovementController? instantMovement;
    private DirectMovementController? movement;
    private Func<bool>? permitted;
    private Action? applied;
    private float expectedSpeed,directionPrecision;
    private System.Numerics.Vector2 expectedDirection;
    private bool holdGroundVelocity;
    private bool allowHigherGroundTarget;
    private bool holdLaunchSpeed;
    private bool nativeLaunch;
    private Vector2 groundVelocity;
    private Vector2 permittedForce;
    private float? zeroForceScalarSpeed;
    public string? GroundFault { get; private set; }
    public long GroundWrites { get; private set; }
    public bool Pending => applied != null;
    public float PreviousSpeed { get; private set; }
    public float AppliedSpeed { get; private set; }
    public long AccelerationWrites { get; private set; }
    public double AccelerationAt { get; private set; }
    public float AccelerationBefore { get; private set; }
    public float AccelerationTarget { get; private set; }
    public Vector3 AccelerationPosition { get; private set; }
    public double MaximumCallbackSeconds { get; private set; }
    // Local speed-hook entry, before Original. These scalars identify its
    // ordering against the existing input-output counters and Framework pose.
    public uint SpeedFrame { get; private set; }
    public double SpeedAt { get; private set; }
    public Vector3 SpeedPosition { get; private set; }
    public NativeJumpLaunch(DirectMovementController? instantMovement = null)
    {
        this.instantMovement = instantMovement;
        // Install while the scene is locked. Arming and cancellation only
        // change owned state; patching hooks during movement can stall a frame.
        hook=new CompSig("40 53 48 83 EC 50 80 79 3C 00 48 8B D9 ?? ?? ?? ?? ?? ?? 48 89 7C 24 68")
            .GetHook<SpeedUpdate>(OnSpeed);
        hook.Enable();
    }
    public void Arm(DirectMovementController controller,float? speed,float precision,System.Numerics.Vector2 direction,Func<bool> guard,Action callback,
        bool retainClampedSpeed=false)
    {
        Cancel();
        movement = controller; permitted = guard; applied = callback;
        nativeLaunch=!speed.HasValue;
        expectedSpeed=speed.GetValueOrDefault();directionPrecision=precision;expectedDirection=direction;
        holdLaunchSpeed=retainClampedSpeed;allowHigherGroundTarget=retainClampedSpeed;
    }
    public void HoldGroundVelocity(DirectMovementController controller,float nativeSpeed,float precision,
        Vector2 velocity,Vector2 force,Func<bool> guard,bool allowHigherTarget=false,float? zeroForceSpeed=null)
    {
        Cancel();GroundFault=null;GroundWrites=0;
        movement=controller;permitted=guard;expectedSpeed=nativeSpeed;
        directionPrecision=precision;groundVelocity=velocity;
        permittedForce=force;holdGroundVelocity=true;allowHigherGroundTarget=allowHigherTarget;zeroForceScalarSpeed=zeroForceSpeed;
    }
    public void UpdateGroundVelocity(Vector2 velocity,float? zeroForceSpeed=null)
    {groundVelocity=velocity;zeroForceScalarSpeed=zeroForceSpeed;}
    private void OnSpeed(PlayerMoveControllerWalk* walk)
    {
        long started=Stopwatch.GetTimestamp();
        if(walk!=null&&walk==PlayerMoveControllerWalk.Instance()&&Service.LocalPlayer is { } actor)
        {
            var framework=FFXIVClientStructs.FFXIV.Client.System.Framework.Framework.Instance();
            SpeedFrame=framework!=null?framework->FrameCounter:0;
            SpeedAt=(double)started/Stopwatch.Frequency;SpeedPosition=actor.Position;
        }
        try{OnSpeedCore(walk);}
        finally{MaximumCallbackSeconds=Math.Max(MaximumCallbackSeconds,Stopwatch.GetElapsedTime(started).TotalSeconds);}
    }
    private void OnSpeedCore(PlayerMoveControllerWalk* walk)
    {
        hook!.Original(walk);
        if (instantMovement?.DirectionalInputAllowed == true && walk != null &&
            walk == PlayerMoveControllerWalk.Instance() && walk->Parent != null)
        {
            float target = walk->GetTargetSpeed();
            if (float.IsFinite(target) && target > walk->CurrentSpeed)
            {
                AccelerationAt = SpeedAt;
                AccelerationBefore = walk->CurrentSpeed;
                AccelerationTarget = target;
                AccelerationPosition = SpeedPosition;
                walk->CurrentSpeed = target;
                ++AccelerationWrites;
            }
        }
        if (applied == null&&!holdGroundVelocity&&!holdLaunchSpeed || walk == null) return;
        if (movement?.Enabled != true||permitted?.Invoke()!=true) { Cancel(); return; }
        if (walk != PlayerMoveControllerWalk.Instance() || walk->Parent == null) return;
        float speed = walk->GetTargetSpeed();
        if (!float.IsFinite(speed)||(nativeLaunch?speed<=0:
            (holdGroundVelocity||holdLaunchSpeed)&&allowHigherGroundTarget?speed<expectedSpeed:speed!=expectedSpeed))
        {if(holdGroundVelocity)FailGround("Native target speed changed during ground velocity control.");return;}
        float inputSpeed=speed;
        var requiredDirection=expectedDirection;
        if(holdGroundVelocity)
        {
            if(Service.LocalPlayer is not {IsDead:false} actor)
            {FailGround("The ground velocity actor is unavailable.");return;}
            var character=(FFXIVClientStructs.FFXIV.Client.Game.Character.Character*)actor.Address;
            if(NativeMechanicMotion.ReadExternalVelocity(character) is not {Y:0} force ||
                force!=Vector3.Zero&&force!=new Vector3(permittedForce.X,0,permittedForce.Y))
            {FailGround("Native external force is outside the ground velocity law.");return;}
            // Current client: speed update 141829480 precedes movement
            // 141828A90 -> 14181A9B0. The latter reads CurrentSpeed, then adds
            // the same controller's force at 14181AE4C on this native thread.
            if(!GroundInputAuthority.TryResolve(groundVelocity,new(force.X,force.Z),speed,
                permittedForce==Vector2.Zero?zeroForceScalarSpeed:null,out inputSpeed,out requiredDirection))
            {FailGround("The requested net ground speed exceeds native input authority.");return;}
        }
        var direction=walk->MovementDirectionWorld;
        float error=Vector2.Distance(new(direction.X,direction.Z),requiredDirection);
        if(!float.IsFinite(error)||error>directionPrecision)
        {if(holdGroundVelocity)FailGround("Native input direction changed during ground velocity control.");return;}
        if(holdGroundVelocity)
        {
            PreviousSpeed=walk->CurrentSpeed;AppliedSpeed=inputSpeed;
            walk->CurrentSpeed=inputSpeed;++GroundWrites;return;
        }
        PreviousSpeed=walk->CurrentSpeed;
        AppliedSpeed=nativeLaunch?walk->CurrentSpeed:holdLaunchSpeed?expectedSpeed:speed;
        if(!nativeLaunch)walk->CurrentSpeed=AppliedSpeed;
        var callback = applied;
        if(holdLaunchSpeed)applied=null;else Cancel();
        callback?.Invoke();
    }
    private void FailGround(string reason)
    {
        GroundFault=reason;var controller=movement;Cancel();controller?.Release();
    }
    public void Cancel()
    {
        applied = null; permitted = null; movement = null;holdGroundVelocity=false;holdLaunchSpeed=false;nativeLaunch=false;zeroForceScalarSpeed=null;
    }
    public void Dispose() { Cancel(); hook.Dispose(); }
}
