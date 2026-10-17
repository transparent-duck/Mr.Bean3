using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision;

namespace MrBean.Modules.FallGuysPilot;

// Explicit read-only snapshot on the framework thread; no movement or capture loop.
internal static class PilotCollisionCapture
{
    [DllImport("kernel32.dll")] private static extern unsafe void AcquireSRWLockShared(nint* address);
    [DllImport("kernel32.dll")] private static extern unsafe void ReleaseSRWLockShared(nint* address);
    internal readonly record struct Triangle(Vector3 A, Vector3 B, Vector3 C, ulong ObjectId, ulong Material)
    {
        public Vector3 Min => Vector3.Min(A, Vector3.Min(B, C));
        public Vector3 Max => Vector3.Max(A, Vector3.Max(B, C));
    }
    internal readonly record struct Bounds(Vector3 Min, Vector3 Max);
    public static unsafe object Capture(Vector3 min, Vector3 max)
    {
        var module = FFXIVClientStructs.FFXIV.Client.System.Framework.Framework.Instance()->BGCollisionModule;
        if (module == null || module->SceneManager == null) throw new InvalidOperationException("Collision scene unavailable.");
        var triangles = new List<Triangle>(); var unknown = new List<Bounds>(); int count = 0;
        min -= new Vector3(2); max += new Vector3(2);
        void Add(Vector3 a, Vector3 b, Vector3 c, ulong id, ulong material)
        {
            var t = new Triangle(a, b, c, id, material);
            if (Overlaps(t.Min, t.Max, min, max) && Vector3.Cross(b - a, c - a).LengthSquared() > 1e-12f) triangles.Add(t);
        }
        AcquireSRWLockShared(&module->UpdateTaskLock);
        try
        {
        foreach (var wrapper in module->SceneManager->Scenes)
        {
            if (wrapper->Scene == null) continue;
            foreach (var collider in wrapper->Scene->Colliders)
            {
                if ((collider->VisibilityFlags & 1) == 0 || (collider->LayerMask & 1) == 0) continue;
                var type = collider->GetColliderType();
                if (type == ColliderType.Streamed) continue; // Its loaded mesh children are separate colliders.
                FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.AABB bounds = default;
                collider->GetWorldBB(&bounds);
                if (!Overlaps(bounds.Min, bounds.Max, min, max)) continue;
                ++count;
                Matrix4x4 world = default; collider->GetWorldTransform(&world);
                ulong value = collider->ObjectMaterialValue, mask = collider->ObjectMaterialMask;
                ulong EffectiveMaterial(ulong primitive) => mask == 0 ? primitive : (primitive & ~mask) | value;
                bool Material(ulong primitive) => (EffectiveMaterial(primitive) & 0x2000) != 0;
                if (type == ColliderType.Mesh)
                {
                    var mesh = (ColliderMesh*)collider;
                    if (!mesh->Loaded || mesh->Mesh == null) { unknown.Add(new(bounds.Min, bounds.Max)); continue; }
                    if (!mesh->MeshIsSimple)
                    {
                        var pending = new Stack<nint>(); pending.Push((nint)((MeshPCB*)mesh->Mesh)->RootNode);
                        while (pending.TryPop(out nint pointer))
                        {
                            var node = (MeshPCB.FileNode*)pointer; if (node == null) continue;
                            if (node->Child1 != null) pending.Push((nint)node->Child1);
                            if (node->Child2 != null) pending.Push((nint)node->Child2);
                            foreach (var primitive in node->Primitives)
                                if (Material(primitive.Material))
                                    Add(Vector3.Transform(node->Vertex(primitive.V1), world), Vector3.Transform(node->Vertex(primitive.V2), world),
                                        Vector3.Transform(node->Vertex(primitive.V3), world), collider->LayoutObjectId, EffectiveMaterial(primitive.Material));
                        }
                    }
                    else
                    {
                        int vertices = (int)mesh->Mesh->GetNumVertices();
                        var positions = new Vector3[vertices];
                        fixed (Vector3* p = positions) mesh->Mesh->GatherVertices(&world, p);
                        var primitives = mesh->Mesh->GetPrimitives();
                        for (int i = 0; i < mesh->Mesh->GetNumPrimitives(); ++i)
                            if (Material(primitives[i].Material)) Add(positions[primitives[i].V1], positions[primitives[i].V2], positions[primitives[i].V3], collider->LayoutObjectId, EffectiveMaterial(primitives[i].Material));
                    }
                }
                else if ((value & 0x2000) != 0 && type is ColliderType.Box or ColliderType.Plane or ColliderType.PlaneTwoSided)
                {
                    var vertices = new Vector3[8];
                    for (int i = 0; i < 8; ++i) vertices[i] = Vector3.Transform(new((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1,
                        type == ColliderType.Box ? ((i & 4) == 0 ? -1 : 1) : 0), world);
                    int[] indices = type == ColliderType.Box ? [0,2,3,0,3,1,4,5,7,4,7,6,0,1,5,0,5,4,2,6,7,2,7,3,0,4,6,0,6,2,1,3,7,1,7,5] : [0,1,3,0,3,2];
                    for (int i = 0; i < indices.Length; i += 3) Add(vertices[indices[i]], vertices[indices[i+1]], vertices[indices[i+2]], collider->LayoutObjectId, value);
                }
                else if ((value & 0x2000) != 0) unknown.Add(new(bounds.Min, bounds.Max));
            }
        }
        }
        finally { ReleaseSRWLockShared(&module->UpdateTaskLock); }
        if (triangles.Count == 0) throw new InvalidOperationException("No character collision triangles captured.");
        return new { Triangles = triangles, Unknown = unknown, Colliders = count };
    }

    internal static bool Overlaps(Vector3 amin, Vector3 amax, Vector3 bmin, Vector3 bmax)
        => amin.X <= bmax.X && amax.X >= bmin.X && amin.Y <= bmax.Y && amax.Y >= bmin.Y && amin.Z <= bmax.Z && amax.Z >= bmin.Z;
}
