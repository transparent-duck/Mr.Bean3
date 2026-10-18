using System.Diagnostics;
using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Hooking;
using FFXIVClientStructs.FFXIV.Client.Game.Control;
using OmenTools.Info.Models;
using OmenTools.Interop.Game.Models;
using OmenTools.Interop.Game.Models.Native;

namespace DeepDungeon.Fsd.Dalamud.Utilities;

public readonly record struct DirectNativeMotion(float Speed,float TargetSpeed,float BaseSpeed,Vector3 Direction,float VerticalSpeed,bool Jumping,bool HasInput);

/// <summary>Shared direct-input owner. Callers select destinations; this class does not find paths.</summary>
public sealed unsafe class DirectMovementController : IDisposable
{
    private static Backend? backend;
    private static Exception? initializationError;
    private static DirectMovementController? owner;
    private static int instances;
    private static readonly HashSet<DirectMovementController> inputObservers = [];
    private static readonly HashSet<DirectMovementController> steeringObservers = [];
    public bool ObserveUserInput
    {
        get => inputObservers.Contains(this);
        set
        {
            ObjectDisposedException.ThrowIf(disposed,this);
            if(value){EnsureBackend();inputObservers.Add(this);backend!.EnableObservation();}
            else{inputObservers.Remove(this);if(owner==null)backend?.Disable();}
        }
    }
    private bool disposed;
    private long validUntil;
    private Vector3 destination;
    private bool hold;
    private bool? directionalInput;
    private Vector3? steeringWorldInput;
    private Vector2? ordinaryWorldInput;
    private float? facingInput;
    public bool IsAutoMove { get; set; } = true;
    public float Precision { get; set; } = 0.01f;
    public bool ClampFinalStep { get; set; }
    public bool CancelOnManualInput { get; set; }
    public bool WasCancelledByUser { get; private set; }
    public Vector2 LastUserInput { get; private set; }
    public Vector2 LastAppliedInput { get; private set; }
    public double LastInputTimestamp { get; private set; }
    public double MaximumWalkInputSeconds { get; private set; }
    public double MaximumDirectionSeconds { get; private set; }
    // A scoped input observer remains active after movement ownership is
    // released. This callback cannot by itself authorize any movement write.
    private Action? beforeSteeringInput;
    public Action? BeforeSteeringInput
    {
        get=>beforeSteeringInput;
        set
        {
            ObjectDisposedException.ThrowIf(disposed,this);
            if(value!=null){EnsureBackend();backend!.PrepareDirectionGate();steeringObservers.Add(this);}
            else steeringObservers.Remove(this);
            beforeSteeringInput=value;backend?.UpdateDirectionGate();
        }
    }
    // Optional bounded owner step immediately before ordinary input is consumed.
    public Action? BeforeWalkInput { get; set; }
    // Optional final permission at the actual ordinary-output write. A denied
    // owner releases its channel and contributes no automatic input.
    public Func<bool>? CanWriteWalkInput { get; set; }
    // Called only after this owner writes the native ordinary-input output.
    public Action? AfterWalkInput { get; set; }
    public static string? ActiveOwner => Current?.Name;
    public static bool TryReadNativeMotion(out DirectNativeMotion motion)
    {
        motion=default;
        try
        {
            var controller=PlayerMoveControllerWalk.Instance();
            if(controller==null||controller->Parent==null)return false;
            motion=new(controller->CurrentSpeed,controller->GetTargetSpeed(),controller->BaseMovementSpeed,controller->MovementDirectionWorld,controller->Parent->VerticalVelocity,controller->IsJumping,controller->IsMovementInput);
            return float.IsFinite(motion.Speed)&&motion.Speed>=0&&float.IsFinite(motion.TargetSpeed);
        }
        catch{return false;}
    }
    public bool? DirectionalInputAllowed => Enabled ? directionalInput : null;
    public bool ControlsAvailable => Enabled && backend?.ControlsAvailable==true;
    public string Name { get; }
    public bool Enabled
    {
        get => !disposed && Current == this;
        set
        {
            ObjectDisposedException.ThrowIf(disposed, this);
            if (!value) { Release(); return; }
            if (Current is { } current && current != this) throw new InvalidOperationException($"Direct movement is owned by {current.Name}.");
            EnsureBackend();
            owner = this;
            Refresh();
            try { backend!.Enable(); }
            catch { Release(); throw; }
        }
    }
    public Vector3 DesiredPosition
    {
        get => destination;
        set
        {
            if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z)) throw new ArgumentException("Invalid movement destination.");
            destination = value; hold = false; directionalInput = null; steeringWorldInput = null; ordinaryWorldInput = null; facingInput = null; Refresh();
        }
    }

    public DirectMovementController(string name = "direct movement") { Name = name; ++instances; }
    /// <summary>Resolve directional control hooks without acquiring input ownership.</summary>
    public void PrepareDirectionalInput()
    {
        ObjectDisposedException.ThrowIf(disposed,this);
        EnsureBackend();backend!.PrepareDirectionGate();
    }
    public void Refresh() => validUntil = Stopwatch.GetTimestamp() + Stopwatch.Frequency / 4;
    public void Hold()
    {
        hold = true;
        steeringWorldInput = null;
        ordinaryWorldInput = null;
        facingInput = null;
        directionalInput = false;
        Refresh();
        Enabled = true;
        backend!.EnableDirectionGate();
    }
    /// <summary>The caller has already evaluated the world-space arrow and the complete segment.</summary>
    public void SetDirectionalInput(Vector3 target, bool allow)
    {
        DesiredPosition = target;
        hold = !allow;
        directionalInput = allow;
        Refresh();
        Enabled = true;
        backend!.EnableDirectionGate();
    }
    /// <summary>Ordinary world input without a position-to-direction round trip.</summary>
    public void SetWorldDirectionalInput(Vector2 direction)
    {
        if (!float.IsFinite(direction.X) || !float.IsFinite(direction.Y) || direction == Vector2.Zero)
            throw new ArgumentException("Ordinary movement requires a finite world direction.");
        ordinaryWorldInput = Vector2.Normalize(direction);
        facingInput = null;
        steeringWorldInput = null;
        hold = false;
        directionalInput = true;
        Refresh();
        Enabled = true;
        backend!.EnableDirectionGate();
    }
    public void SetFacingInput(float heading)
    {
        if (!float.IsFinite(heading)) throw new ArgumentException("Facing requires a finite heading.");
        facingInput = heading;
        steeringWorldInput = null;
        ordinaryWorldInput = null;
        hold = false;
        directionalInput = true;
        Refresh();
        Enabled = true;
        backend!.EnableDirectionGate();
    }
    /// <summary>World input consumed by the native limited-turn status processor.</summary>
    public void SetSteeringInput(Vector3 target, Vector2 direction)
    {
        SetDirectionalInput(target, true);
        if (!float.IsFinite(direction.X) || !float.IsFinite(direction.Y) || direction == Vector2.Zero)
            throw new ArgumentException("Steering requires a finite world direction.");
        direction = Vector2.Normalize(direction);
        steeringWorldInput = new(direction.X, 0, direction.Y);
    }
    public bool TryReadForcedDirection(out float radians)
    {
        EnsureBackend();
        return backend!.TryReadForcedDirection(out radians);
    }
    /// <summary>Commit a validated native heading under the current input owner.</summary>
    public bool TryWriteForcedDirection(float radians)
        =>Enabled&&float.IsFinite(radians)&&backend!.TryWriteForcedDirection(radians);
    public void ClearCancellation() => WasCancelledByUser = false;
    public void Release()
    {
        if (owner != this) return;
        owner = null;
        hold = false;
        directionalInput = null;
        steeringWorldInput = null;
        ordinaryWorldInput = null;
        facingInput = null;
        backend?.Disable();
    }
    public void Dispose()
    {
        if (disposed) return;
        BeforeSteeringInput=null;
        ObserveUserInput=false;
        Release();
        disposed = true;
        if (--instances == 0) { backend?.Dispose(); backend = null; initializationError = null; }
    }
    private static void EnsureBackend()
    {
        if (backend != null) return;
        if (initializationError != null) throw new InvalidOperationException("Direct movement input is unavailable for this client.", initializationError);
        try { backend = new Backend(); }
        catch (Exception e) { initializationError = e; throw; }
    }
    private static DirectMovementController? Current => owner is { disposed: false } c && Stopwatch.GetTimestamp() <= c.validUntil ? c : null;

    private sealed class Backend : IDisposable
    {
        // Native predicates return AL only. A marshalled four-byte BOOL can
        // treat a false AL with nonzero upper EAX bits as true.
        private delegate byte InputEnabled(void* self);
        private delegate void Walk(void* self, float* left, float* forward, float* turn, byte* strafe, byte* unknown, byte additive);
        private delegate void Fly(void* self, FlyInput* result);
        private delegate byte InputActive(void* self, byte flags);
        private readonly Hook<Walk> walk;
        private readonly Hook<Fly> fly;
        private readonly InputEnabled enabled1;
        private readonly InputEnabled enabled2;
        private Hook<InputActive>? gate;
        private float* forcedDirection;
        private bool directionResolved;
        private Exception? gateError;
        public bool ControlsAvailable { get; private set; }
        public Backend()
        {
            // Current ClientStructs does not expose these input functions. Reuse the existing Omen signatures.
            enabled1 = new CompSig("E8 ?? ?? ?? ?? 84 C0 75 10 38 43 3C").GetDelegate<InputEnabled>();
            enabled2 = new CompSig("E8 ?? ?? ?? ?? 84 C0 75 03 88 47 3F").GetDelegate<InputEnabled>();
            walk = new CompSig("E8 ?? ?? ?? ?? 80 7B 3E 00 48 8D 3D").GetHook<Walk>(OnWalk);
            try { fly = new CompSig("E8 ?? ?? ?? ?? 0F B6 0D ?? ?? ?? ?? B8").GetHook<Fly>(OnFly); }
            catch { walk.Dispose(); throw; }
        }
        public void Enable() { walk.Enable(); fly.Enable(); }
        public void EnableObservation() => walk.Enable();
        public void Disable() { UpdateDirectionGate(); fly.Disable(); if(inputObservers.Count==0)walk.Disable(); }
        public void UpdateDirectionGate()
        {if(gate==null)return;if(steeringObservers.Count>0||owner!=null)gate.Enable();else gate.Disable();}
        public void PrepareDirectionGate()
        {
            if (gateError != null) throw new InvalidOperationException("Movement stop/arrow input gate is unavailable.", gateError);
            try { gate ??= new CompSig("E8 ?? ?? ?? ?? 84 C0 74 09 84 DB 74 1A").GetHook<InputActive>(OnInputActive); }
            catch (Exception e) { gateError = e; throw; }
        }
        public void EnableDirectionGate(){PrepareDirectionGate();gate!.Enable();}
        public bool TryReadForcedDirection(out float radians)
        {
            if (!directionResolved)
            {
                directionResolved = true;
                if (Service.Scanner.TryGetStaticAddressFromSig("F3 0F 11 0D ?? ?? ?? ?? 48 85 DB", out var address)) forcedDirection = (float*)address;
            }
            radians = forcedDirection == null ? float.NaN : *forcedDirection;
            return float.IsFinite(radians) && MathF.Abs(radians) <= MathF.Tau + 0.01f;
        }
        public bool TryWriteForcedDirection(float radians)
        {
            if(!TryReadForcedDirection(out _))return false;
            *forcedDirection=MathF.IEEERemainder(radians,MathF.Tau);return true;
        }
        private byte OnInputActive(void* self, byte flags)
        {
            foreach(var observer in steeringObservers)observer.beforeSteeringInput?.Invoke();
            var c = Current;
            if (c?.hold == true) return 0;
            if (c?.steeringWorldInput is { } world)
            {
                // 3702 reads Walk+A0 and Walk+B0 immediately after this gate
                // (1408A3123..368E in CN 2026.09.15). In its forced channel,
                // overriding OnWalk outputs alone leaves this input stale.
                // Do not write the arrow angle: native retains its turn limit.
                var controller = PlayerMoveControllerWalk.Instance();
                if (controller != null)
                {
                    controller->MovementDirectionWorld = world;
                    controller->RotationDirection = 0;
                }
            }
            if (c?.directionalInput is { } allow) return allow ? (byte)1 : (byte)0;
            return gate!.Original(self, flags);
        }
        private void OnWalk(void* self, float* left, float* forward, float* turn, byte* strafe, byte* unknown, byte additive)
        {
            long started=Stopwatch.GetTimestamp();var measured=Current;
            try{OnWalkCore(self,left,forward,turn,strafe,unknown,additive);}
            finally
            {
                if(measured!=null)measured.MaximumWalkInputSeconds=Math.Max(measured.MaximumWalkInputSeconds,Stopwatch.GetElapsedTime(started).TotalSeconds);
            }
        }
        private void OnWalkCore(void* self, float* left, float* forward, float* turn, byte* strafe, byte* unknown, byte additive)
        {
            walk.Original(self, left, forward, turn, strafe, unknown, additive);
            if(additive!=0)return;
            double now=(double)Stopwatch.GetTimestamp()/Stopwatch.Frequency;
            foreach(var observer in inputObservers)
            {observer.LastUserInput=new(*left,*forward);observer.LastInputTimestamp=now;}
            var c = Current;
            if (c == null) return;
            c.LastUserInput = new(*left, *forward);
            if (c.CancelOnManualInput && (*left != 0 || *forward != 0 || c.facingInput != null && *turn != 0) && (c.directionalInput == null || c.ObserveUserInput))
            {
                c.WasCancelledByUser = true;
                c.Release();
                return;
            }
            c.LastInputTimestamp=(double)Stopwatch.GetTimestamp()/Stopwatch.Frequency;
            c.BeforeWalkInput?.Invoke();
            if(Current!=c)return;
            if(c.CanWriteWalkInput?.Invoke()==false)
            {c.Release();*left=*forward=*turn=0;return;}
            if (c.hold) { *left = *forward = *turn = 0;c.LastAppliedInput=Vector2.Zero;c.AfterWalkInput?.Invoke(); return; }
            ControlsAvailable=enabled1(self)!=0&&enabled2(self)!=0;
            if(c.steeringWorldInput!=null)
            {
                // Steering input belongs exclusively to the native status
                // processor's world-input gate. During its first frame the
                // ordinary channel can still be enabled; sending the requested
                // turn direction there bypasses the native angular limit.
                *left=*forward=*turn=0;c.LastAppliedInput=Vector2.Zero;c.AfterWalkInput?.Invoke();return;
            }
            if (!ControlsAvailable) {c.LastAppliedInput=new(*left,*forward);return;}
            if (c.facingInput is { } heading && Service.LocalPlayer is { } turning)
            {
                float error = MathF.IEEERemainder(heading - turning.Rotation, MathF.Tau);
                var framework = FFXIVClientStructs.FFXIV.Client.System.Framework.Framework.Instance();
                float dt = framework != null ? framework->FrameDeltaTime : 1f / 60;
                *left = *forward = 0;
                *turn = Math.Abs(error) <= .015f ? 0 : Math.Clamp(error / Math.Max(.02f, 2 * dt), -1, 1);
                c.LastAppliedInput = Vector2.Zero;
                c.AfterWalkInput?.Invoke();
                return;
            }
            if (!c.IsAutoMove && (*left != 0 || *forward != 0)) return;
            if(c.ordinaryWorldInput is { } world && Service.LocalPlayer is { } actor)
            {
                // Same native input reference as destination control, but retain
                // the supplied vector instead of encoding it in a float position.
                float reference = OmenTools.DService.Instance().GameConfig.UiControl.TryGetUInt("MoveMode",out var mode)&&mode==1
                    ? ((CameraExtra*)CameraManager.Instance()->GetActiveCamera())->DirH+MathF.PI
                    : actor.Rotation;
                var input=(Angle.FromDirection(world)-new Angle(reference)).ToDirection();
                *left=input.X;*forward=input.Y;c.LastAppliedInput=input;c.AfterWalkInput?.Invoke();return;
            }
            // Walking stops by horizontal distance, including while jumping above a landing point.
            var target = c.destination;
            if (Service.LocalPlayer is { } player) target.Y = player.Position.Y;
            long directionStarted=Stopwatch.GetTimestamp();
            bool hasDirection=Angle.TryGetDirectionToDestination(target, false, out var dir, c.Precision);
            c.MaximumDirectionSeconds=Math.Max(c.MaximumDirectionSeconds,Stopwatch.GetElapsedTime(directionStarted).TotalSeconds);
            if (hasDirection)
            {
                var input = dir.H.ToDirection();
                if(c.ClampFinalStep&&Service.LocalPlayer is { } local)
                {
                    float distance=Vector2.Distance(new(local.Position.X,local.Position.Z),new(target.X,target.Z));
                    var controller=PlayerMoveControllerWalk.Instance();
                    var framework=FFXIVClientStructs.FFXIV.Client.System.Framework.Framework.Instance();
                    float step=controller!=null&&framework!=null?Math.Max(controller->CurrentSpeed,controller->BaseMovementSpeed)*framework->FrameDeltaTime:0;
                    if(step>.001f&&distance<step)input*=distance/step;
                }
                *left = input.X;
                *forward = input.Y;
            }
            else if (c.IsAutoMove) { *left = *forward = 0; }
            c.LastAppliedInput=new(*left,*forward);
            if(hasDirection||c.IsAutoMove)c.AfterWalkInput?.Invoke();
        }
        private void OnFly(void* self, FlyInput* result)
        {
            fly.Original(self, result);
            var c = Current;
            if (c == null) return;
            if (c.hold) { result->Left = result->Forward = result->Up = 0; return; }
            if (!c.IsAutoMove && (result->Left != 0 || result->Forward != 0 || result->Up != 0)) return;
            if (Angle.TryGetDirectionToDestination(c.destination, true, out var dir, c.Precision))
            {
                var input = dir.H.ToDirection();
                result->Left = input.X; result->Forward = input.Y; result->Up = dir.V.Rad;
            }
        }
        public void Dispose() { gate?.Dispose(); fly.Dispose(); walk.Dispose(); }
        [StructLayout(LayoutKind.Explicit, Size = 0x18)]
        private struct FlyInput
        {
            [FieldOffset(0)] public float Forward;
            [FieldOffset(4)] public float Left;
            [FieldOffset(8)] public float Up;
        }
    }
}
