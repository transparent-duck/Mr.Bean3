using System;
using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.Blunderville;

// Territory 1165's disjoint course areas select the recording and script.
// Ring's 10107969 is a MapRange, unlike the other courses' EventRanges.
// A range-registry lookup must not silently suppress an entire live course.
internal static class CourseScene
{
    public static CourseId? Detect(Vector3 position)
    {
        CourseId? candidate=position.Z<0?
            position.X < -130?CourseId.Knight:position.X>130?CourseId.Ring:
            Math.Abs(position.X)<70?CourseId.Hit:null:
            position.Z>100?(position.X < -130?CourseId.Crystal:
            Math.Abs(position.X)<70?CourseId.Crown:null):null;
        return candidate;
    }
}
