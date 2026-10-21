using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

/// <summary>
/// Course registry. A null script means the pilot only records that course (no control).
/// Rounds: 1 = Hit or Knight (race, top ~50% qualify), 2 = Ring (survival) or Crystal
/// (deliveries), 3 = Crown (single winner). See doc/FallGuys-Pilot.md for the per-course specs.
/// </summary>
internal static class CourseScripts
{
    public static CourseScript? Create(CourseId course, bool crownStairBoost = false) => course switch
    {
        CourseId.Hit => new HitCourse(),
        CourseId.Knight => new KnightCourse(),
        CourseId.Ring => new RingCourse(),
        CourseId.Crystal => new CrystalCourse(),
        CourseId.Crown => new CrownCourse(crownStairBoost),
        _ => null,
    };
}
