namespace MrBean.FallGuys;

// Individually reviewed row3788/C117/TMFC7 values. The native timeline owns
// origin, target and time; this law never advances or resets that clock.
public static class IceMotionLaw
{
    public const ushort Timeline=3788;
    public const float Distance=5;
    public const float EndTick=50;
    public const float TicksPerSecond=30;
    public const double Seconds=50d/30;
    public static float Progress(float tick)
    {float u=Math.Clamp(tick*.02f,0,1);return u*u*(3-2*u);}
}
