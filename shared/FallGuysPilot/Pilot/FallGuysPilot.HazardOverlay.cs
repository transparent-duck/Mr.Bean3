using System;
using System.Collections.Generic;
using System.Numerics;
using MrBean.FallGuys;
using MrBean.Modules.FallGuysPilot.Courses;
using MrBean.Host;
using Dalamud.Bindings.ImGui;

namespace MrBean.Modules.FallGuysPilot;

public sealed partial class FallGuysPilot
{
    private sealed class HazardDrawing(Vector3[] points, Vector3 center)
    {
        public readonly Vector3[] Points = points;
        public readonly Vector3 Center = center;
        public uint Color;
        public bool Active, Visible;
        public string Label = "";
    }
    private bool hazardOverlay = true;
    private readonly Dictionary<HazardField.Station, HazardDrawing> hazardDraw = [];
    private HitGround? overlayGround;
    private int overlayActive, overlayUnknown, overlayProjected;
    private double nextHazardOverlay;

    private void ResetHazardOverlay()
    {
        hazardDraw.Clear(); overlayGround = null; nextHazardOverlay = 0;
        overlayActive = overlayUnknown = overlayProjected = 0;
    }

    // Sample on the update thread after execution. Every planner prepares its
    // own query windows before reading them; drawing never mutates the model.
    // Static outlines are cached per station; timing/text refresh at 5 Hz.
    private void UpdateHazardOverlay(double now)
    {
        if (!hazardOverlay || context == null || course == null || now < nextHazardOverlay) return;
        nextHazardOverlay = now + .2;
        overlayGround ??= course == CourseId.Hit ? HitGround.Loaded.Value :
            PilotCourseGeometry.Read<HitGround>($"{course.Value.ToString().ToLowerInvariant()}-ground.json.gz");
        var field = context.Hazards;
        field.Prepare(now, 12);
        overlayActive = overlayUnknown = 0;
        foreach (var station in field.Stations)
        {
            if (!hazardDraw.TryGetValue(station, out var item))
            {
                float height = course == CourseId.Crystal ? 6 : overlayGround.At(station.Origin) ?? context.Pose.Feet.Y;
                var points = HazardOutline(station);
                var world = new List<Vector3>();
                for (int i = 0; i < points.Length; ++i)
                {
                    var a = points[i]; var b = points[(i + 1) % points.Length];
                    int count = course == CourseId.Crown ? Math.Max(1, (int)MathF.Ceiling(Vector2.Distance(a, b))) : 1;
                    for (int j = 0; j < count; ++j)
                    {
                        var p = Vector2.Lerp(a, b, (float)j / count);
                        float y = course == CourseId.Crown ? overlayGround.At(p) ?? height : height;
                        world.Add(new(p.X, y + .08f, p.Y));
                    }
                }
                item = new(world.ToArray(), new(station.Origin.X, height + .12f, station.Origin.Y));
                hazardDraw.Add(station, item);
            }
            double next = double.PositiveInfinity, remaining = 0;
            bool active = false;
            foreach (var window in station.Windows)
            {
                if (window.Start <= now && window.End >= now)
                { active = true; remaining = Math.Max(remaining, window.End - now); }
                if (window.Start > now) next = Math.Min(next, window.Start - now);
            }
            bool unknown = !active && !double.IsFinite(next) && station.KnownUntil <= now;
            item.Active = active;
            item.Visible = (station.Spec.Note != "ring-wind" || active) &&
                !(course == CourseId.Hit && station.Spec.ActionId == 34773 && water.HasPhase);
            item.Color = active ? 0xff4080ffu : next <= 1 ? 0xff50c0ffu : unknown ? 0xffffb060u : 0xffa0a0a0u;
            string name = station.Spec.Note switch
            { "ring-wind" => "風帶", "support" => "支撐關閉", "hammer-body" => "錘心", "column-body" => "柱身", _ => station.Spec.ActionId.ToString() };
            item.Label = station.Spec.Note == "hammer-body" ? "錘心 常駐" :
                $"{name} {(active ? $"剩 {remaining:F1}s" : double.IsFinite(next) ? $"後 {next:F1}s" : unknown ? "時序未知" : "等待")}";
            if (active) ++overlayActive;
            if (unknown) ++overlayUnknown;
        }
    }

    private static Vector2[] HazardOutline(HazardField.Station station)
    {
        var spec = station.Spec; var origin = station.Origin;
        var forward = Geo.FromHeading(station.Rotation); var side = new Vector2(forward.Y, -forward.X);
        if (spec.Shape == HazardShape.Square)
            return [origin + new Vector2(-spec.Size, -spec.Size), origin + new Vector2(spec.Size, -spec.Size),
                origin + new Vector2(spec.Size, spec.Size), origin + new Vector2(-spec.Size, spec.Size)];
        if (spec.Shape == HazardShape.Rect)
            return [origin + forward * spec.ForwardMin - side * spec.Size, origin + forward * spec.Length - side * spec.Size,
                origin + forward * spec.Length + side * spec.Size, origin + forward * spec.ForwardMin + side * spec.Size];
        if (spec.Shape == HazardShape.Charge)
        {
            var axis = station.Endpoint - origin;
            float heading = axis.LengthSquared() > .0001f ? Geo.Heading(axis) : 0;
            var points = new Vector2[34];
            for (int i = 0; i <= 16; ++i)
            {
                points[i] = station.Endpoint + Geo.FromHeading(heading - MathF.PI / 2 + i * MathF.PI / 16) * spec.Size;
                points[i + 17] = origin + Geo.FromHeading(heading + MathF.PI / 2 + i * MathF.PI / 16) * spec.Size;
            }
            return points;
        }
        bool half = spec.Shape == HazardShape.HalfDisc;
        var ring = new Vector2[half ? 18 : 32];
        int count = half ? 17 : 32;
        for (int i = 0; i < count; ++i)
            ring[i] = origin + Geo.FromHeading(half ? station.Rotation - MathF.PI / 2 + i * MathF.PI / 16 : i * MathF.Tau / 32) * spec.Size;
        if (half) ring[^1] = origin;
        return ring;
    }

    private void DrawHazardOverlay()
    {
        overlayProjected = 0;
        if (!hazardOverlay || course == null || Service.LocalPlayer is not { } player) return;
        var draw = ImGui.GetBackgroundDrawList();
        foreach (var item in hazardDraw.Values)
        {
            if (!item.Visible || Vector3.DistanceSquared(player.Position, item.Center) > 120 * 120) continue;
            bool centerVisible = Service.GameGui.WorldToScreen(item.Center, out var center, out _);
            bool previousVisible = Service.GameGui.WorldToScreen(item.Points[^1], out var previous, out _);
            bool projected = false;
            foreach (var world in item.Points)
            {
                bool visible = Service.GameGui.WorldToScreen(world, out var point, out _);
                if (visible && previousVisible)
                {
                    projected = true;
                    if (item.Active && centerVisible)
                        draw.AddTriangleFilled(center, previous, point, (item.Color & 0xffffffu) | 0x28000000u);
                    draw.AddLine(previous, point, item.Color, item.Active ? 2 : 1);
                }
                previous = point; previousVisible = visible;
            }
            if (projected) ++overlayProjected;
            if (centerVisible && Vector3.DistanceSquared(player.Position, item.Center) < 35 * 35)
            {
                var size = ImGui.CalcTextSize(item.Label);
                draw.AddRectFilled(center - new Vector2(3, 2), center + size + new Vector2(3, 2), 0xb0202020, 2);
                draw.AddText(center, item.Color, item.Label);
            }
        }
    }
}
