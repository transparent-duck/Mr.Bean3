using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal sealed class HitCourse : GraphCourse
{
    public HitCourse() => _ = HitCourseGeometry.Load();

    public override CourseId Course => CourseId.Hit;

    // AoE data: shapes from recorded-mechanics/hit-parade-phases; periods from live receipts.
    // TODO(GPT): calibrate Lead/Tail and shape sizes from local-hit evidence (see doc/FallGuys-Pilot.md).
    public override HazardSpec[] Hazards { get; } =
    [
        new(34773, HazardShape.Circle, 5f, Period: 6.10, Lead: 0.25, Tail: 0.2, Note: "water pulse rows X=-10/0/10, Z=-202..-242; 8 pulses then pause"),
        new(34774, HazardShape.Charge, 1.5f, Period: 4.19, Lead: 0.6, Tail: 0.4, EndpointDwell: 2.2, Area: new(-30, 30, -260, -240), Note: "opening charging columns Z=-248.5/-251.5"),
        new(34774, HazardShape.Charge, 1.5f, Period: 12.84, Lead: 0.6, Tail: 0.4, EndpointDwell: 1.7, Area: new(-30, 30, -380, -340), Note: "wet-slope columns Z=-347.8/-355.8/-363.8"),
        new(34775, HazardShape.Square, 1.5f, Period: 4.27, Lead: 0.2, Tail: 0.15, Note: "pendulum stations; rows -281/-301/-321 at X=0.5/3.5/6.5, rows -288/-308/-328 at X=-0.5/-3.5/-6.5"),
        new(34776, HazardShape.Circle, 2.3f, Period: 10.2, Lead: 0.25, Tail: 0.2, Quantum: 1f, Note: "rolling balls on lanes X=+-1.83/+-5.5"),
    ];

}
