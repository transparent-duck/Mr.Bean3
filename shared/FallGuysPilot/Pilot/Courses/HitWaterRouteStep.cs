using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using MrBean.Modules.FallGuysPilot.Water;

namespace MrBean.Modules.FallGuysPilot.Courses;

/// <summary>One executor for observed and manually loaded water phases.</summary>
internal sealed class HitWaterRouteStep : Step
{
    public bool UsePreparedPlan { get; init; }
    private PathPassage? passage;
    private double departAt, seenHit, nextRetry;
    private bool begun;
    private int followedEdges;

    public override void Enter(PilotContext c)
    {
        seenHit = c.LastHitAt;
        c.Log("water-cache", new { loadMs = c.Water.LoadMs, routes = c.Water.Cache.Routes.Length,
            nodes = c.Water.Cache.Nodes.Length, edges = c.Water.Cache.Edges.Length });
        if (UsePreparedPlan) SetPassage(c);
        else Select(c);
    }

    public override bool Tick(PilotContext c)
    {
        if (c.Water.Plan == null && c.Now >= nextRetry || seenHit != c.LastHitAt && c.Pose.Grounded)
        {
            seenHit = c.LastHitAt;
            Select(c);
        }
        if (c.Water.Plan == null) { c.Act.Hold(); return false; }
        if (c.Water.Plan.Edges.Length == 0) return true;
        if (c.Now < departAt) { c.Act.Hold(); return false; }
        if (!begun) { passage!.Begin(c); begun = true; }
        if (!passage!.Follow(c)) return false;
        if (c.Water.Plan == null || c.Water.Plan.Edges.Length <= followedEdges) return true;
        c.Water.Advance(followedEdges, new(c.Pose.Feet.X, c.Pose.Feet.Z), c.Now);
        SetPassage(c);
        if (c.Now >= departAt)
        {
            passage!.Begin(c); begun = true; passage.Follow(c);
        }
        else c.Act.Hold();
        return false;
    }

    public override Vector2? Aim(PilotContext c) => passage?.Aim(c);

    private void Select(PilotContext c)
    {
        c.Water.Observe(c);
        c.Water.Calculate(new(c.Pose.Feet.X, c.Pose.Feet.Z), c.Now);
        nextRetry = c.Now + .25;
        SetPassage(c);
        c.Log("water-route-plan", c.Water.Snapshot(c.Now));
    }

    private void SetPassage(PilotContext c)
    {
        if (c.Water.Plan is { Edges.Length: > 0 } plan)
        {
            departAt = plan.Departures[0] + c.Water.Epoch;
            var points = new List<Vector3>();
            var jumps = new List<int>();
            followedEdges = 0;
            for (int i = 0; i < plan.Edges.Length; ++i)
            {
                if (i > 0 && plan.Departures[i] > plan.Departures[i - 1] + plan.Edges[i - 1].Duration + 1e-9) break;
                var edge = plan.Edges[i];
                if (edge.Jump >= 0) jumps.Add(points.Count + edge.Jump);
                points.AddRange(edge.Points.Select(p => new Vector3((float)p.X, 10, (float)p.Z)));
                ++followedEdges;
            }
            passage = new PathPassage { Name = "water route", Points = points.ToArray(), JumpFrom = jumps.ToArray(), ArriveRadius = 0.08f };
        }
        else passage = null;
        begun = false;
    }
}
