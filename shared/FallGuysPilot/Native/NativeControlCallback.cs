using System;
using Dalamud.Hooking;
using FFXIVClientStructs.FFXIV.Client.Game;
using FFXIVClientStructs.FFXIV.Client.Game.Character;
using OmenTools.Interop.Game.Models;
using MrBean.Host;

namespace MrBean.Modules.Blunderville;

// Current CN client: StatusManager movement processor 1408A2810, called with
// only RCX by Character update at 140906341. The maintained StatusManager API
// does not expose this function. A scoped input read proves that the processor
// passed its movement eligibility predicate; status presence alone does not.
internal sealed unsafe class NativeControlCallback : IDisposable
{
    private delegate void Process(StatusManager* manager);
    private delegate void Gain(StatusManager* manager,ushort status,float seconds,ushort parameter,ulong source,bool refresh);
    private Hook<Process>? hook;
    private Hook<Gain>? gain;
    private bool processing,consumed;
    public Action? Step { get; set; }
    public Action? ForcedStep { get; set; }
    public Func<bool>? ForcedGain { get; set; }
    public Action? ArrowGain { get; set; }
    public Action? Commit { get; set; }
    public float FrameSeconds { get; private set; }
    public Exception? Fault { get; private set; }
    public void Enable()
    {
        hook??=new CompSig("4C 8B DC 55 49 8D AB 88 FE FF FF 48 81 EC 70 02 00 00 48 8B 05 ?? ?? ?? ?? 48 33 C4")
            .GetHook<Process>(OnProcess);
        // Current CN gain handler14089F010 is not exposed by StatusManager.
        // Unlike the periodic processor, it submits the first forced target
        // immediately, after capturing facing plus the status parameter offset.
        gain??=new CompSig("48 8B C4 55 57 41 54 41 56 48 8D 68 B8 48 81 EC 28 01 00 00 48 83 39 00 48 8B F9")
            .GetHook<Gain>(OnGain);
        gain.Enable();
        hook.Enable();
    }
    private void OnGain(StatusManager* manager,ushort status,float seconds,ushort parameter,ulong source,bool refresh)
    {
        Character* actor=null;float facing=0;
        try
        {
            if(status==3737&&parameter is 2 or 4 or 8&&Fault==null&&
                Service.LocalPlayer is {IsDead:false} player&&manager==((Character*)player.Address)->GetStatusManager())
            {
                // The native slot is already installed. The same owner checks
                // its admitted leg, warning, support and deadlines before any
                // direction is used. No owner command means native gain stays intact.
                bool submitted=false;
                try{submitted=ForcedGain?.Invoke()==true;}catch(Exception error){Fault=error;}
                if(submitted)
                {
                    actor=(Character*)player.Address;facing=actor->Rotation;
                    float offset=parameter switch{2=>MathF.PI,4=>MathF.PI/2,_=>-MathF.PI/2};
                    float prepared=MathF.IEEERemainder(manager->SpecialStatusTimerOrDirection-offset,2*MathF.PI);
                    actor->SetRotation(prepared);
                }
            }
            gain!.Original(manager,status,seconds,parameter,source,refresh);
            if (status == 3694 && Fault == null && Service.LocalPlayer is { } arrowPlayer &&
                manager == ((Character*)arrowPlayer.Address)->GetStatusManager())
                try { ArrowGain?.Invoke(); } catch (Exception error) { Fault = error; }
        }
        finally
        {
            // Gain has captured its angle and submitted its native target.
            // This temporary facing does not become ordinary movement input.
            if(actor!=null)actor->SetRotation(facing);
        }
    }
    private void OnProcess(StatusManager* manager)
    {
        bool local=Service.LocalPlayer is { } player&&manager==((Character*)player.Address)->GetStatusManager();
        bool forced=local&&manager->GetStatusIndex(3737)>=0;
        processing=local&&(manager->GetStatusIndex(3702)>=0||manager->GetStatusIndex(3694)>=0);consumed=false;
        if(processing||forced)
        {
            var framework=FFXIVClientStructs.FFXIV.Client.System.Framework.Framework.Instance();
            bool unscaled=manager->Owner!=null&&manager->Owner->ObjectIndex is >=200 and <=448;
            FrameSeconds=framework->FrameDeltaTime*(unscaled?1:framework->GameSpeedMultiplier);
        }
        try
        {
            if(forced&&Fault==null)
                try{ForcedStep?.Invoke();}catch(Exception error){Fault=error;}
            hook!.Original(manager);
            if(processing&&consumed&&Fault==null)
                try{Commit?.Invoke();}catch(Exception error){Fault=error;}
        }
        finally{processing=false;}
    }
    public void InputRead()
    {
        if(!processing||consumed||Fault!=null)return;
        consumed=true;
        try{Step?.Invoke();}catch(Exception error){Fault=error;}
    }
    public void Reset(){processing=false;consumed=false;Fault=null;}
    public void Disable(){gain?.Disable();hook?.Disable();Reset();}
    public void Dispose(){gain?.Dispose();hook?.Dispose();}
}
