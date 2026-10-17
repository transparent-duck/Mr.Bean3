using System.Numerics;

namespace MrBean.FallGuys;

// Shared by the native speed sink and saved-data arithmetic diagnostics.
// The admitted leg owns scalar speed; a rounded vector owns only direction.
internal static class GroundInputAuthority
{
    public static bool TryResolve(Vector2 netVelocity,Vector2 actualForce,float availableSpeed,
        float? zeroForceSpeed,out float inputSpeed,out Vector2 direction)
    {
        inputSpeed=0;direction=default;
        var input=netVelocity-actualForce;
        float reconstructed=input.Length();
        if(!float.IsFinite(availableSpeed)||availableSpeed<=0||
            !float.IsFinite(reconstructed)||reconstructed<=0)return false;
        if(actualForce==Vector2.Zero&&zeroForceSpeed is { } authored)
        {
            if(!float.IsFinite(authored)||authored<=0||authored>availableSpeed)return false;
            inputSpeed=authored;
            direction=Vector2.Normalize(input);
        }
        else
        {
            if(reconstructed>availableSpeed)return false;
            inputSpeed=reconstructed;
            direction=input/reconstructed;
        }
        return true;
    }
}
