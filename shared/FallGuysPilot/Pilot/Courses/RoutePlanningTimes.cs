using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using MrBean.FallGuys;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Main-thread samples measure request -> usable result, including the queue,
// snapshot, result polling and validation. Waiting for a future handoff is not
// computation time. Keep cold graph construction out of the warm distribution.
internal sealed class RoutePlanningTimes
{
    public sealed class Samples
    {
        public double Cold { get; set; }
        public double[] Warm { get; set; } = [];
        public double Reserve { get; set; }
    }
    private readonly Dictionary<string, Samples> courses = [];
    internal double Lead(CourseId course, bool cold, double frame)
    {
        courses.TryGetValue(course.ToString(), out var sample);
        double seconds = cold ? Math.Max(sample?.Cold ?? 0, sample?.Reserve ?? 0) : sample?.Reserve ?? 0;
        return Math.Max(.15, seconds * 1.25 + 2 * Math.Clamp(frame, 1d / 240, .1));
    }
    internal void Observe(CourseId course, bool cold, double seconds)
    {
        if (!double.IsFinite(seconds) || seconds <= 0) return;
        if (!courses.TryGetValue(course.ToString(), out var sample)) courses.Add(course.ToString(), sample = new());
        if (cold) { sample.Cold = Math.Max(seconds, sample.Cold * .95); return; }
        sample.Warm = sample.Warm.TakeLast(63).Append(seconds).ToArray();
        var sorted = sample.Warm.Order().ToArray();
        double high = sorted[(int)Math.Ceiling(sorted.Length * .95) - 1];
        // Increase immediately on a slow sample, decrease only with sustained
        // faster work. The bounded sample window eventually forgets old spikes.
        sample.Reserve = Math.Max(seconds, Math.Max(high, sample.Reserve * .98));
    }
    internal string Serialize() => JsonSerializer.Serialize(courses);
    internal static RoutePlanningTimes Read(string json)
    {
        var result = new RoutePlanningTimes();
        foreach (var (name, sample) in JsonSerializer.Deserialize<Dictionary<string, Samples>>(json) ?? [])
        {
            if (!Enum.TryParse<CourseId>(name, out _) || sample == null) continue;
            sample.Cold = double.IsFinite(sample.Cold) && sample.Cold > 0 ? sample.Cold : 0;
            sample.Reserve = double.IsFinite(sample.Reserve) && sample.Reserve > 0 ? sample.Reserve : 0;
            sample.Warm = (sample.Warm ?? []).Where(n => double.IsFinite(n) && n > 0).TakeLast(64).ToArray();
            result.courses.Add(name, sample);
        }
        return result;
    }
}
