using System.Numerics;
using Dalamud.Hooking;
using FFXIVClientStructs.FFXIV.Client.Game;
using FFXIVClientStructs.FFXIV.Client.Game.Character;
using FFXIVClientStructs.FFXIV.Client.Game.Object;

namespace MrBean.Host.Modules;

// Only the receive notification required by PacketObservations is hosted here.
// The PvP ActionTracker and its action execution hooks are not imported.
public sealed unsafe class ActionTracker : IDisposable
{
    public delegate void RawActionEffectObservedHandler(uint actorId, Character* casterPtr,
        Vector3* targetPos, ActionEffectHandler.Header* header,
        ActionEffectHandler.TargetEffects* effects, GameObjectId* targetEntityIds);
    public event RawActionEffectObservedHandler? RawActionEffectReceiving;
    private readonly Hook<ActionEffectHandler.Delegates.Receive> hook;

    public ActionTracker()
    {
        hook = Service.GameInteropProvider.HookFromAddress<ActionEffectHandler.Delegates.Receive>(
            ActionEffectHandler.MemberFunctionPointers.Receive, OnReceive);
        try { hook.Enable(); }
        catch { hook.Dispose(); throw; }
    }

    private void OnReceive(uint actorId, Character* caster, Vector3* position,
        ActionEffectHandler.Header* header, ActionEffectHandler.TargetEffects* effects, GameObjectId* targets)
    {
        try { RawActionEffectReceiving?.Invoke(actorId, caster, position, header, effects, targets); }
        catch (Exception error) { Service.Log.Error(error, "[Mr.Bean3] Effect observation failed."); }
        hook.Original(actorId, caster, position, header, effects, targets);
    }

    public void Dispose() => hook.Dispose();
}
