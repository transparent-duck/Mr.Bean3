using System.Collections.Generic;
using System.Numerics;
using MrBean.FallGuys;
using MrBean.FallGuys.Mechanics;
using MrBean.Host;

namespace MrBean.Modules.FallGuysPilot;

public sealed partial class FallGuysPilot
{
    private readonly Dictionary<uint, (Vector3 Last, Vector3? Published, double Seen)> ringSourcePoses = [];
    private readonly List<uint> removedRingSources = [];
    private double nextSourcePose;

    private void ObserveRingSources(double now)
    {
        if (course != CourseId.Ring || now < nextSourcePose) return;
        nextSourcePose = now + .2;
        foreach (var actor in Service.GameObjects)
        {
            if (actor.BaseId != 16350) continue;
            var position = actor.Position;
            bool known = ringSourcePoses.TryGetValue(actor.EntityId, out var previous);
            var published = previous.Published;
            // Two stable samples identify an idle body. Do not accumulate
            // geometry stations along the moving actor's intermediate poses.
            if (known && Vector3.DistanceSquared(position, previous.Last) < .01f &&
                (published == null || Vector3.DistanceSquared(position, published.Value) > .01f))
            {
                var observation = new MechanicActorObservation(packets.Session, now, actor.EntityId,
                    16350, position, actor.Rotation, "stationary-pose", null, actor.HitboxRadius);
                Write("observation", observation); context!.Hazards.Observe(observation);
                published = position;
            }
            ringSourcePoses[actor.EntityId] = (position, published, now);
        }
        removedRingSources.Clear();
        foreach (var (entity, pose) in ringSourcePoses)
            if (now - pose.Seen > .4) removedRingSources.Add(entity);
        foreach (uint entity in removedRingSources)
        {
            var observation = new MechanicActorObservation(packets.Session, now, entity,
                16350, null, null, "despawn", null);
            Write("observation", observation); context!.Hazards.Observe(observation);
            ringSourcePoses.Remove(entity);
        }
    }
}
