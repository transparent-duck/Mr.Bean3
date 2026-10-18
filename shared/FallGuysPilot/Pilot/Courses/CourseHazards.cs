using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using MrBean.FallGuys;
using MrBean.FallGuys.Mechanics;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Scalar source geometry and periods, re-anchored by this round's receipts.
// A missing/mismatched source leaves its prediction unresolved; it never stops
// execution. No retired forecaster, runtime router or admission state is used.
internal sealed class CourseHazards
{
    public sealed class Shape
    {
        public string Kind { get; set; } = "Circle";
        public float[] Origin { get; set; } = [];
        public float Rotation { get; set; }
        public float Size { get; set; }
        public float Length { get; set; }
        public float ForwardMin { get; set; }
        public float[] Endpoint { get; set; } = [];
        public HazardField.Station Station(HazardField field, uint action, double lead = .15, double tail = .10, string note = "")
            => field.StationFor(new(action, Enum.Parse<HazardShape>(Kind), Size, Length, Lead: lead, Tail: tail, ForwardMin: ForwardMin, Note: note),
                new(Origin[0], Origin[1]), Rotation, new(Endpoint[0], Endpoint[1]));
    }
    public sealed class Body
    {
        public Shape Shape { get; set; } = new();
        public double[][] Windows { get; set; } = [];
    }
    public sealed class Group
    {
        public uint Action { get; set; }
        public double Period { get; set; }
        public double[] Offsets { get; set; } = [];
        public double[] OffsetErrors { get; set; } = [];
        public double[][] OffsetChoices { get; set; } = [];
        public bool MatchRotation { get; set; }
        public bool MatchDestination { get; set; }
        public double PeriodError { get; set; }
        public Shape[] Shapes { get; set; } = [];
        public Body[] Bodies { get; set; } = [];
        public double Lead { get; set; } = .15;
        public double Tail { get; set; } = .10;
        public bool Permanent { get; set; }
        public double Epoch = double.NaN;
        internal double EpochError;
        public HazardField.Station[] Stations = [], BodyStations = [];
        internal double[] ChoiceEpochs = [];
        internal bool[] ActiveChoices = [];
        internal bool UsesChoices => OffsetChoices.Length > 0 || MatchDestination;
    }
    private readonly CourseId course;
    private readonly HazardField field;
    private readonly Group[] groups;
    private readonly HazardField.Station[] bridges = new HazardField.Station[4];
    private readonly double[] bridgeOff = Enumerable.Repeat(double.NaN, 4).ToArray();
    private readonly HazardField.Station[] wind = new HazardField.Station[72];
    private sealed class RingColumn
    {
        public HazardField.Station Origin = null!;
        public HazardField.Station? Destination;
        public double StopAt = double.PositiveInfinity;
    }
    private readonly Dictionary<uint, RingColumn> ringColumns = [];
    private readonly List<(HazardField.Station From, HazardField.Station To)> ringColumnSites = [];
    private readonly List<HazardField.Station> pendingRingColumnSites = [];

    private HazardField.Station ColumnBody(Vector2 position) => field.StationFor(
        new(0xE2000000, HazardShape.Circle, 1.5f, Note: "column-body"), position, 0, position);

    private void RevealRingColumnSite(Vector2 position)
    {
        HazardField.Station? opposite = null;
        bool ambiguous = false;
        foreach (var (from, to) in ringColumnSites)
        {
            HazardField.Station other;
            if (Vector2.DistanceSquared(position, from.Origin) < .75f * .75f) other = to;
            else if (Vector2.DistanceSquared(position, to.Origin) < .75f * .75f) other = from;
            else continue;
            if (opposite != null && opposite != other) ambiguous = true;
            opposite = other;
        }
        for (int i = pendingRingColumnSites.Count - 1; i >= 0; --i)
        {
            var station = pendingRingColumnSites[i];
            if (Vector2.DistanceSquared(position, station.Origin) >= .75f * .75f &&
                (ambiguous || station != opposite)) continue;
            pendingRingColumnSites.RemoveAt(i);
            field.Changed(station);
        }
    }

    private void ColumnMove(uint entity, Vector2 from, Vector2 to, double receipt)
    {
        if (!ringColumns.TryGetValue(entity, out var column)) ringColumns[entity] = column = new();
        if (column.Origin != null) field.Changed(column.Origin);
        if (column.Destination != null) field.Changed(column.Destination);
        column.Origin = ColumnBody(from); column.Destination = ColumnBody(to);
        column.StopAt = receipt + .85;
        field.Changed(column.Origin); field.Changed(column.Destination);
        RevealRingColumnSite(from); RevealRingColumnSite(to);
    }

    public static HazardSpec[] Specs(CourseId course) => course switch
    {
        CourseId.Knight => [new(34716, HazardShape.Rect, 1, 1.5f, Lead: .35, Tail: .35, ForwardMin: -1.5f),
            new(34717, HazardShape.Circle, 3, CastWarning: true), new(34718, HazardShape.Rect, 5, 4)],
        // Half-disc effects report caster facing; casts report the left/right footprint direction.
        CourseId.Ring => [new(34797, HazardShape.HalfDisc, 8, CastWarning: true, EffectRotationOffset: MathF.PI / 2),
            new(34798, HazardShape.HalfDisc, 8, CastWarning: true, EffectRotationOffset: -MathF.PI / 2),
            new(34805, HazardShape.Circle, 6, CastWarning: true),
            new(35409, HazardShape.Charge, 1.5f, Tail: .85)],
        CourseId.Crystal => [new(34716, HazardShape.Rect, 1, 1.5f, ForwardMin: -1.5f),
            new(34774, HazardShape.Charge, 1.5f, Tail: .85), new(34799, HazardShape.Rect, 2, 2, Tail: 2.1, ForwardMin: -2),
            new(34800, HazardShape.Rect, 6, 31, Lead: .15, CastWarning: true)],
        _ => [new(29795, HazardShape.Circle, 3), new(34796, HazardShape.Circle, 5),
            new(34801, HazardShape.Circle, 6), new(34802, HazardShape.Circle, 5), new(34804, HazardShape.Rect, 3, 3, ForwardMin: -3),
            new(34812, HazardShape.Rect, 3, 3, ForwardMin: -3)],
    };
    public static void Install(CourseId course, HazardField field)
    {
        if (field.ExtraObservation?.Target is CourseHazards installed && installed.course == course) return;
        _ = new CourseHazards(course, field);
    }
    private CourseHazards(CourseId course, HazardField field)
    {
        this.course = course;
        this.field = field;
        groups = PilotCourseGeometry.Read<Group[]>($"{course.ToString().ToLowerInvariant()}-hazards.json.gz");
        uint custom = 0xF0000000;
        foreach (var group in groups)
        {
            int choices = group.UsesChoices ? Math.Max(1, group.OffsetChoices.Length) * group.Shapes.Length : 0;
            group.ChoiceEpochs = new double[choices];
            group.ActiveChoices = new bool[choices];
            group.Stations = group.Shapes.Select(s => s.Station(field, group.Action, group.Lead, group.Tail,
                group.Permanent ? course == CourseId.Ring ? "column-body" : "hammer-body" :
                    course == CourseId.Crystal && group.Action == 34800 ? "crystal-wind" : "")).ToArray();
            group.BodyStations = group.Bodies.Select(b => b.Shape.Station(field, custom++, note: "column-body")).ToArray();
            if (course == CourseId.Ring && group.Action == 35409)
                foreach (var station in group.Stations)
                {
                    var from = ColumnBody(station.Origin); var to = ColumnBody(station.Endpoint);
                    ringColumnSites.Add((from, to));
                    if (!pendingRingColumnSites.Contains(from)) pendingRingColumnSites.Add(from);
                    if (!pendingRingColumnSites.Contains(to)) pendingRingColumnSites.Add(to);
                }
        }
        if (course == CourseId.Knight)
            for (int i = 0; i < bridges.Length; ++i)
                bridges[i] = field.StationFor(new(custom++, HazardShape.Rect, 3, 15.6f, ForwardMin: -15.6f, Note: "support"),
                    new(i == 0 ? -210.523f : -203.5f + (i - 1) * 7, -338), 0, default);
        if (course == CourseId.Ring)
            for (int i = 0; i < wind.Length; ++i)
            {
                // Ring occupancy lies within 30 m. Inflate by the largest
                // angular error in a 5-degree bin; time remains continuous.
                float pad = MathF.Sqrt(30 * 30 + 4 * 4) * MathF.PI / wind.Length;
                wind[i] = field.StationFor(new(custom++, HazardShape.Rect, 4 + pad, 30 + pad, ForwardMin: -pad, Note: "ring-wind"),
                    new(200, -125), i * MathF.Tau / wind.Length, default);
            }
        field.ExtraObservation = Observe;
        field.ExtraWindows = Prepare;
        if (course == CourseId.Crystal) field.NextWind = after =>
        {
            foreach (var group in groups)
                if (group.Action == 34800 && double.IsFinite(group.Epoch) && group.Period > 0)
                    return group.Epoch + Math.Ceiling((after - group.Epoch) / group.Period) * group.Period;
            return null;
        };
    }
    private bool Observe(MechanicObservation observation)
    {
        if (course == CourseId.Ring && observation is MechanicActorObservation
            { BaseId: 16350, ActorEvent: "despawn" } removed && ringColumns.Remove(removed.EntityId, out var gone))
        {
            field.Changed(gone.Origin); gone.Origin.KnownUntil = double.PositiveInfinity;
            if (gone.Destination != null)
            { field.Changed(gone.Destination); gone.Destination.KnownUntil = double.PositiveInfinity; }
            return true;
        }
        if (course == CourseId.Ring && observation is MechanicActorObservation
            { BaseId: 16350, Position: { } actorPosition, ActorEvent: "stationary-pose" } actor)
        {
            if (!ringColumns.TryGetValue(actor.EntityId, out var column))
                ringColumns[actor.EntityId] = column = new();
            // A pose at the old endpoint must not cancel an announced move.
            if (double.IsFinite(column.StopAt) && actor.Seconds < column.StopAt + .2) return true;
            if (column.Origin != null) field.Changed(column.Origin);
            if (column.Destination != null) field.Changed(column.Destination);
            column.Origin = ColumnBody(actorPosition.XZ()); column.Destination = null;
            column.StopAt = double.PositiveInfinity; field.Changed(column.Origin);
            RevealRingColumnSite(actorPosition.XZ());
            return true;
        }
        if (course == CourseId.Knight && observation is MechanicMapObservation { EventId: 0x8004002B, Index: >= 14 and <= 17 } map)
        {
            if (map.State == 4 && map.TimelineIndex == 8) bridgeOff[map.Index - 14] = map.Seconds;
            else if (map.State == 1 && map.TimelineIndex == 2) bridgeOff[map.Index - 14] = map.Seconds + 4.5;
            field.Changed(bridges[map.Index - 14]);
            return true;
        }
        if (course == CourseId.Ring && observation is MechanicEffectObservation { ActionId: 34808 } gust)
        {
            // One radial wind band. The interleaved source receipts follow one
            // continuous heading; retain the full angle, including its direction.
            float rate = -.191f, heading = gust.Rotation; // retained live startup prior
            bool changed = true;
            if (field.RingWind is { } previous)
            {
                rate = previous.Rate;
                float delta = Geo.WrapAngle(gust.Rotation - previous.Heading);
                heading = previous.Heading + delta;
                changed = Math.Abs(Geo.WrapAngle(delta -
                    previous.Rate * (float)(gust.Seconds - previous.At))) > .08;
                if (gust.Seconds <= previous.At + .02) return true;
                rate = .8f * rate + .2f * delta / (float)(gust.Seconds - previous.At);
            }
            if (changed) foreach (var station in wind) field.Changed(station);
            field.RingWind = (gust.Seconds, heading, rate); return true;
        }
        if (course == CourseId.Ring && observation is MechanicCastObservation { ActionId: 35409 } charge)
        {
            // This cast location is the destination, not the caster origin.
            // Match either endpoint of the retained moving-column corridor.
            foreach (var group in groups)
                foreach (var station in group.Stations)
                    if (Vector2.DistanceSquared(charge.Position.XZ(), station.Origin) < .75f * .75f ||
                        Vector2.DistanceSquared(charge.Position.XZ(), station.Endpoint) < .75f * .75f)
                    {
                        station.CastEnd = HazardField.CastActivation(charge);
                        field.Changed(station);
                        var chargeFrom = Vector2.DistanceSquared(charge.Position.XZ(), station.Origin) < .75f * .75f
                            ? station.Endpoint : station.Origin;
                        if (ringColumns.TryGetValue(charge.EntityId, out var known))
                            chargeFrom = (charge.Seconds >= known.StopAt ? known.Destination ?? known.Origin : known.Origin).Origin;
                        ColumnMove(charge.EntityId, chargeFrom, charge.Position.XZ(), station.CastEnd.Value);
                    }
            return true;
        }
        if (course == CourseId.Ring && observation is MechanicEffectObservation
            { ActionId: 35409, Origin: { } from, PacketTargetPosition: { } to } columnEffect)
        {
            ColumnMove(columnEffect.EntityId, from.XZ(), to.XZ(), columnEffect.Seconds);
            foreach (var group in groups)
                foreach (var station in group.Stations)
                    if (Vector2.DistanceSquared(from.XZ(), station.Origin) < .75f * .75f ||
                        Vector2.DistanceSquared(from.XZ(), station.Endpoint) < .75f * .75f)
                    { station.CastEnd = columnEffect.Seconds; field.Changed(station); }
            return true;
        }
        uint action; Vector3 position; float rotation; double at;
        if (observation is MechanicEffectObservation { Origin: { } origin } effect)
        { action = effect.ActionId; position = origin; rotation = effect.Rotation; at = effect.Seconds; }
        else if (observation is MechanicCastObservation cast)
        { action = cast.ActionId; position = cast.Position; rotation = cast.Rotation; at = HazardField.CastActivation(cast); }
        else return false;
        if (course == CourseId.Hit && observation is MechanicCastObservation) return false;
        // 34812 casts at the rear edge (187.4), then its receipt reports the
        // first square's centre (190.4). The five lanes share this geometry.
        if (course == CourseId.Crown && action == 34812 && observation is MechanicCastObservation)
        {
            var offset = Geo.FromHeading(rotation) * 3;
            position += new Vector3(offset.X, 0, offset.Y);
        }
        Group? selected = null; int phase = 0, phaseMask = 0; double nearest = .75 * .75;
        foreach (var group in groups)
        {
            if (group.Action != action) continue;
            for (int i = 0; i < group.Stations.Length; ++i)
            {
                var station = group.Stations[i];
                double distance = group.MatchDestination && observation is MechanicEffectObservation
                    { PacketTargetPosition: { } destination } ? Vector2.DistanceSquared(station.Endpoint, destination.XZ()) :
                    Vector2.DistanceSquared(station.Origin, position.XZ());
                if ((group.MatchRotation || station.Spec.Shape == HazardShape.Rect) &&
                    Math.Abs(Geo.WrapAngle(group.Shapes[i].Rotation - rotation)) > .2) continue;
                if (station.Spec.Shape == HazardShape.Charge && observation is MechanicEffectObservation
                    { PacketTargetPosition: { } endpoint } && Vector2.DistanceSquared(station.Endpoint, endpoint.XZ()) >= .75f * .75f) continue;
                if (group == selected && Math.Abs(distance - nearest) < 1e-6)
                { phaseMask |= 1 << i; continue; }
                if (distance >= nearest) continue;
                selected = group; phase = i; phaseMask = 1 << i; nearest = distance;
            }
        }
        if (selected == null) return false;
        if (selected.UsesChoices)
        {
            bool changed = false;
            int remaining = 0;
            int count = selected.Shapes.Length;
            for (int i = 0; i < selected.ChoiceEpochs.Length; ++i)
            {
                if (!selected.ActiveChoices[i]) continue;
                var offsets = selected.OffsetChoices.Length > 0 ? selected.OffsetChoices[i / count] : selected.Offsets;
                double error = double.PositiveInfinity, choiceEpoch = 0;
                for (int observed = 0; observed < count; ++observed)
                    if ((phaseMask & (1 << observed)) != 0)
                    {
                        double candidate = at - offsets[observed];
                        double residual = Math.Abs(Math.IEEERemainder(candidate - selected.ChoiceEpochs[i], selected.Period));
                        if (residual < error) { error = residual; choiceEpoch = candidate; }
                    }
                changed |= error > .15;
                if (error > .2)
                { selected.ActiveChoices[i] = false; changed = true; continue; }
                selected.ChoiceEpochs[i] = choiceEpoch; ++remaining;
            }
            if (remaining == 0)
            {
                for (int i = 0; i < selected.ChoiceEpochs.Length; ++i)
                {
                    int observed = i % count;
                    selected.ActiveChoices[i] = (phaseMask & (1 << observed)) != 0;
                    var offsets = selected.OffsetChoices.Length > 0 ? selected.OffsetChoices[i / count] : selected.Offsets;
                    selected.ChoiceEpochs[i] = at - offsets[observed];
                }
                changed = true;
            }
            if (changed)
            {
                foreach (var station in selected.Stations) field.Changed(station);
                foreach (var station in selected.BodyStations) field.Changed(station);
            }
            return true;
        }
        double epoch = at - selected.Offsets[phase];
        double epochError = phase < selected.OffsetErrors.Length ? selected.OffsetErrors[phase] : 0;
        if (observation is MechanicCastObservation || !double.IsFinite(selected.Epoch) || selected.Period <= 0 ||
            epochError > selected.EpochError || Math.Abs(Math.IEEERemainder(epoch - selected.Epoch, selected.Period)) > .15)
        {
            foreach (var station in selected.Stations) field.Changed(station);
            foreach (var station in selected.BodyStations) field.Changed(station);
        }
        selected.Epoch = epoch; selected.EpochError = epochError;
        return true;
    }
    private void Prepare(double now, double end)
    {
        foreach (var station in pendingRingColumnSites)
        { station.Windows.Add((now, end)); station.KnownUntil = end; }
        foreach (var column in ringColumns.Values)
        {
            // Retain the source body through transit; the full corridor also
            // remains dangerous. Destination occupancy persists until the next
            // observed move, including idle time before another cast appears.
            if (column.StopAt >= now) column.Origin.Windows.Add((now, Math.Min(end, column.StopAt)));
            column.Origin.KnownUntil = end;
            if (column.Destination is { } destination)
            {
                if (column.StopAt <= end) destination.Windows.Add((Math.Max(now, column.StopAt), end));
                destination.KnownUntil = end;
            }
        }
        foreach (var group in groups)
        {
            if (group.Permanent)
            {
                foreach (var station in group.Stations)
                { station.Windows.Add((now, end)); station.KnownUntil = double.PositiveInfinity; }
                continue;
            }
            if (group.UsesChoices)
            {
                for (int choice = 0; choice < group.ChoiceEpochs.Length; ++choice)
                    if (group.ActiveChoices[choice])
                        for (int phase = 0; phase < group.Stations.Length; ++phase)
                        {
                            var offsets = group.OffsetChoices.Length > 0 ? group.OffsetChoices[choice / group.Shapes.Length] : group.Offsets;
                            double anchor = group.ChoiceEpochs[choice] + offsets[phase];
                            Repeat(group.Stations[phase], anchor - group.Lead, anchor + group.Tail, group.Period, now, end, group.PeriodError);
                            for (int body = 0; body < group.Bodies.Length; ++body)
                            {
                                var span = group.Bodies[body].Windows[phase];
                                if (span.Length < 2) continue;
                                Repeat(group.BodyStations[body], anchor + span[0], anchor + span[1], group.Period, now, end, group.PeriodError);
                            }
                        }
                continue;
            }
            if (!double.IsFinite(group.Epoch)) continue;
            for (int phase = 0; phase < group.Stations.Length; ++phase)
            {
                var station = group.Stations[phase];
                double anchor = group.Epoch + group.Offsets[phase];
                double error = group.EpochError + (phase < group.OffsetErrors.Length ? group.OffsetErrors[phase] : 0);
                Repeat(station, anchor - group.Lead - error, anchor + group.Tail + error, group.Period, now, end, group.PeriodError);
                for (int b = 0; b < group.Bodies.Length; ++b)
                {
                    var span = group.Bodies[b].Windows[phase];
                    if (span.Length < 2) continue;
                    Repeat(group.BodyStations[b], anchor + span[0], anchor + span[1], group.Period, now, end, group.PeriodError);
                }
            }
        }
        if (course == CourseId.Knight)
            for (int i = 0; i < bridges.Length; ++i)
                if (double.IsFinite(bridgeOff[i])) Repeat(bridges[i], bridgeOff[i] - .1, bridgeOff[i] + 1.6, 6.026, now, end);
        if (course != CourseId.Ring || field.RingWind is not { } axis) return;
        var (windAt, windHeading, windRate) = axis;
        for (int i = 0; i < wind.Length; ++i)
        {
            float angle = i * MathF.Tau / wind.Length;
            double half = MathF.PI / wind.Length;
            if (Math.Abs(windRate) < .001)
            {
                if (Math.Abs(Geo.WrapAngle(angle - windHeading)) <= half) wind[i].Windows.Add((now, end));
                continue;
            }
            double center = windAt + Geo.WrapAngle(angle - windHeading) / windRate;
            double width = half / Math.Abs(windRate);
            Repeat(wind[i], center - width, center + width, MathF.Tau / Math.Abs(windRate), now, end);
        }
    }
    private static void Repeat(HazardField.Station station, double start, double finish, double period, double now, double end, double periodError = 0)
    {
        int first = (int)Math.Floor((now - finish) / period);
        for (int n = first; start + n * period - Math.Abs(n) * periodError <= end; ++n)
        {
            double error = Math.Abs(n) * periodError;
            double a = start + n * period - error, b = finish + n * period + error;
            if (b >= now) station.Windows.Add((a, b));
        }
        station.KnownUntil = end;
    }
}
