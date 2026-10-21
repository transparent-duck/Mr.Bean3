using System;
using System.Linq;

namespace MrBean.Modules.FallGuysPilot.Courses;

internal static class KnightCourseGeometry
{
    // Handoffs sit on ordinary stopping rows, outside the ice/arrow interiors.
    // Native slides and bridge jumps stay inside their owning section.
    internal static void Sections(PilotCourseGeometry data)
    {
        // Opening axes end before the first column row. Keep the ordinary
        // columns and ice columns together so entry lanes include their slides.
        string[] names = ["opening-axes", "columns", "axes", "arrows", "bomb", "bridges"];
        float[] rows = [-158, -210, -244, -298, -320];
        var ids = Enumerable.Range(0, data.Nodes.Length).ToArray();
        float start = data.Nodes.Max(n => n.Feet.Z);
        int[] entries = ids.Where(i => data.Nodes[i].Feet.Z >= start - .01f).ToArray();
        data.Sections = new PilotCourseGeometry.Section[names.Length];
        float high = float.PositiveInfinity;
        for (int section = 0; section < names.Length; ++section)
        {
            bool last = section == names.Length - 1;
            float low = last ? float.NegativeInfinity : rows[section];
            var exits = ids.Where(i => last ? data.Nodes[i].Goal : Math.Abs(data.Nodes[i].Feet.Z - low) < .025f).ToArray();
            var nodes = ids.Where(i => data.Nodes[i].Feet.Z <= high + .025f && data.Nodes[i].Feet.Z >= low - .025f).ToArray();
            data.Sections[section] = new(names[section], nodes, entries, exits);
            entries = exits; high = low;
        }
    }
}
