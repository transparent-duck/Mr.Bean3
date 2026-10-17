using System.Numerics;

namespace MrBean.FallGuys;

// Current native row1: Height100 / 10, default gravity30, profile3.
// Curve3's three cubic pieces were read from the current movement table and
// traced through 14181B406 -> 14181B4E0 -> 141834329 (2026-10-14 client).
public static class CrownTransportLaw
{
    public const float Height=10,Gravity=30,CleanupSeconds=.0664f;
    public static float VerticalSpeed(float originY,float targetY)
        =>MathF.Sqrt(2*Gravity*(Height+Math.Max(0,targetY-originY)));
    public static float FlightSeconds(float originY,float targetY)
    {
        float rise=Height+Math.Max(0,targetY-originY);
        return MathF.Sqrt(2*rise/Gravity)+MathF.Sqrt(2*(originY+rise-targetY)/Gravity);
    }
    public static float Progress(float fraction)
    {
        float scaled=Math.Clamp(fraction,0,1)*3;
        int segment=Math.Min(2,(int)scaled);float t=scaled-segment;
        return segment switch
        {
            0=>((-0.06666669249534607f*t)*t+0.6666667461395264f)*t,
            1=>((0.03333338350057602f*t-0.2000000774860382f)*t+0.4666666090488434f)*t+0.6000000238418579f,
            _=>((0.03333330899477005f*t-0.09999992698431015f)*t+0.1666666567325592f)*t+0.8999999761581421f,
        };
    }
    public static Vector3 Position(Vector3 origin,Vector3 target,float seconds)
    {
        float duration=FlightSeconds(origin.Y,target.Y);
        float t=Math.Clamp(seconds,0,duration);
        var position=Vector3.Lerp(origin,target,Progress(t/duration));
        position.Y=origin.Y+VerticalSpeed(origin.Y,target.Y)*t-.5f*Gravity*t*t;
        return position;
    }
}
