using System.Numerics;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

/// <summary>
/// Hit Parade (round 1). The opening searches captured beams and native jump
/// connections from the actual pose. Each later area has its own timed graph.
///
/// Sections: opening beams and jump choices (-108..-156), directional floor with forced 3737
/// movement (-156..-195), water pulse rows 34773 (-202..-242), jump + charging columns 34774
/// (-239..-252), native steering 3702 through pendulum rows 34775 (-270..-332), wet slope
/// 3703 with rolling balls 34776 and columns 34774 (-339..-367), goal Z &lt;= -373 (Y 18.94).
/// </summary>
internal sealed class HitCourse : CourseScript
{
    public HitCourse()
    {
        // Scene detection happens during the start lock. Construct static
        // geometry here instead of on the frame that enters each hazard area.
        HitOpeningStep.Prepare(); HitColumnStep.Prepare(); HitSteeringStep.Prepare(); HitWetStep.Prepare();
    }

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

    private const float Deck = 10f;

    private static Vector3 V(float x, float y, float z) => new(x, y, z);

    public override Step[] Build(PilotPose start)
    {
        return
        [
            new HitOpeningStep { Name = "opening", Zone = Zone.ZBand(-194.9f, -95f, 9f, 12f) },
            new HitWaterRouteStep { Name = "water-route", Zone = Zone.ZBand((float)Water.WaterRoutes.ExitZ, -194.9f, 9f, 12f) },
            new HitColumnStep { Name = "water-exit", Zone = Zone.ZBand(-269.0f, (float)Water.WaterRoutes.ExitZ, 9f, 12f) },
            new HitSteeringStep { Name = "steering", Zone = Zone.ZBand(-332.6f, -269.0f, 9f, 12f) },
            new HitWetStep { Name = "wet-slope", Zone = Zone.ZBand(-372.6f, -332.6f, 9f, 20f) },
            new FinishStep { Name = "finish", Zone = Zone.ZBand(-400f, -372.6f, 17f, 22f), Points = [V(0, 18.94f, -380f)] },
            // Knocked off the steering deck onto a side catwalk (Y~6): walk back to the steering start.
            new WalkStep
            {
                Name = "catwalk-left", Zone = new(-16f, -9.5f, -335f, -262f, 4f, 9.4f), Next = "steering",
                Points = [V(-13.6f, 6f, -271.5f), V(-13.2f, 6f, -267.5f), V(-7.95f, 9.5f, -268.5f), V(-3.3f, Deck, -269.0f)],
            },
            new WalkStep
            {
                Name = "catwalk-right", Zone = new(9.5f, 16f, -335f, -262f, 4f, 9.4f), Next = "steering",
                Points = [V(13.6f, 6f, -271.5f), V(13.2f, 6f, -267.5f), V(7.95f, 9.5f, -268.5f), V(3.3f, Deck, -269.0f)],
            },
        ];
    }

}
