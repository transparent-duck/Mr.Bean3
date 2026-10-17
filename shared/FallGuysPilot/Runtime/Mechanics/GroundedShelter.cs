using System;
using System.Numerics;

namespace MrBean.FallGuys.Mechanics;

// A supported region immediately behind a physical stop. Contact can still
// occur here; this certificate bounds its displacement, not the recipient set.
public readonly record struct GroundedShelter(Vector3 Min,Vector3 Max,float MaximumDisplacement)
{
    public bool Contains(Vector3 point,float error)
        =>float.IsFinite(error)&&error>=0&&point.X-error>=Min.X&&point.X+error<=Max.X&&
          point.Z-error>=Min.Z&&point.Z+error<=Max.Z&&MathF.Abs(point.Y-Min.Y)<=.001f;
}
