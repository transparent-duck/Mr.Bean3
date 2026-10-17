using System;
using MrBean.Host;

namespace MrBean.Modules.Blunderville;

// Current 2026.09.15 native dispatcher, independently checked against the game
// executable. Read the Character mirror used by Character.IsJumping itself.
// The old Omen Walk instance is the input controller, not this physics block.
internal readonly record struct NativeJumpMotion(int State,float Elapsed,float FlightClock,float VerticalSpeed,float OriginY,float ApexClock)
{
    public bool Grounded=>State is 0 or 5;
    public static unsafe bool TryRead(out NativeJumpMotion motion)
    {
        motion=default;if(Service.LocalPlayer is not { } player)return false;
        var character=(FFXIVClientStructs.FFXIV.Client.Game.Character.Character*)player.Address;
        byte* p=(byte*)&character->MoveController+0x10;
        motion=new(*(int*)(p+0xC0),*(float*)(p+0xC4),*(float*)(p+0xC8),*(float*)(p+0xCC),*(float*)(p+0xDC),*(float*)(p+0xEC));
        return motion.State is >=0 and <=5;
    }
}
