using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Goal safety is independent of the current attack phase. This field is used
// only to select endpoints; the route keeps its live, timed hazard field.
internal sealed class RingGoalSelection
{
    private readonly (int Node, float Heading)[] candidates;

    public RingGoalSelection(PilotCourseGeometry data)
    {
        var footprints = new HazardField([]);
        foreach (var group in PilotCourseGeometry.Read<CourseHazards.Group[]>("ring-hazards.json.gz"))
        {
            foreach (var shape in group.Shapes) shape.Station(footprints, group.Action);
            foreach (var body in group.Bodies) body.Shape.Station(footprints, group.Action);
        }
        // Both half-discs together cover the full 8 m circle at each source.
        Circle(34797, new(180, -125), 8);
        Circle(34797, new(220, -125), 8);
        // All four 34805 positions in the retained DAT corpus, including both
        // radial positions of each source. Do not wait for the first live cast.
        Circle(34805, new(183.0294f, -108.0294f), 6);
        Circle(34805, new(216.9706f, -108.0294f), 6);
        Circle(34805, new(188.6863f, -113.6863f), 6);
        Circle(34805, new(211.3137f, -113.6863f), 6);

        // Walk completion accepts feet within 0.12 m of the waypoint.
        candidates = Enumerable.Range(0, data.Nodes.Length)
            .Where(i => data.Nodes[i].CanWait &&
                !footprints.Stations.Any(s => s.Contains(data.Nodes[i].Feet.XZ(), s.Spec.Pad + .12f)))
            .Select(i => (i, Geo.Heading(data.Nodes[i].Feet.XZ() - new Vector2(200, -125))))
            .ToArray();

        void Circle(uint action, Vector2 origin, float radius)
            => footprints.StationFor(new(action, HazardShape.Circle, radius), origin, 0, origin);
    }

    public HashSet<int> Select(float target)
    {
        float nearestError = float.PositiveInfinity;
        foreach (var candidate in candidates)
            nearestError = Math.Min(nearestError, Math.Abs(Geo.WrapAngle(candidate.Heading - target)));
        // Keep equivalent radial choices at the nearest safe angle when the
        // original target band has no safe node. Never fall back to an unsafe node.
        float tolerance = Math.Max(.13f, nearestError + .001f);
        var goals = new HashSet<int>();
        foreach (var candidate in candidates)
            if (Math.Abs(Geo.WrapAngle(candidate.Heading - target)) <= tolerance) goals.Add(candidate.Node);
        return goals;
    }
}
