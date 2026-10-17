using System.Numerics;
using MrBean.Host;
using FFXIVClientStructs.FFXIV.Client.Game.Character;

namespace MrBean.Modules.Blunderville;

// Owned scalars from the current GimmickJump/profile3 path. Target, row and
// sequence are written by14181EEF0; normalized clock by14181B406.
internal readonly record struct NativeCrownTransport(byte Profile,byte Row,uint Sequence,
    bool InputLocked,Vector3 Target,float Duration,float Progress,float PreviousProgress,
    float Distance,int Easing,float GravityOverride)
{
    public bool Active=>Profile==3&&Row==1;
    public static unsafe NativeCrownTransport Read()
    {
        if(Service.LocalPlayer is not { } actor)return default;
        byte* p=(byte*)&((Character*)actor.Address)->MoveController;
        return new(p[0x101],p[0x102],*(uint*)(p+0x104),(p[0x439]&0x20)!=0,
            *(Vector3*)(p+0xB0),*(float*)(p+0x1E4),*(float*)(p+0x218),*(float*)(p+0x21C),
            *(float*)(p+0x214),*(int*)(p+0x1EC),*(float*)(p+0xC0));
    }
}
