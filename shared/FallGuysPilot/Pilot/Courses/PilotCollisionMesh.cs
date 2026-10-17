using System;
using System.Collections.Generic;
using System.Numerics;

namespace MrBean.Modules.FallGuysPilot.Courses;

// Immutable BVH. Walking sweeps the native 0.5 m body sphere at feet + 1.01 m.
// Keep full 3D faces: projecting slanted blocks or slope sides loses collision.
internal sealed class PilotCollisionMesh
{
    private readonly record struct Triangle(Vector3 A, Vector3 B, Vector3 C)
    {
        public Vector3 Min => Vector3.Min(A, Vector3.Min(B, C));
        public Vector3 Max => Vector3.Max(A, Vector3.Max(B, C));
    }
    private readonly record struct Node(Vector3 Min, Vector3 Max, int Start, int Count, int Left, int Right);
    private readonly Triangle[] triangles;
    private readonly Node[] nodes;
    public PilotCollisionMesh(float[][] faces)
    {
        triangles = new Triangle[faces.Length];
        for (int i = 0; i < faces.Length; i++)
        { var p = faces[i]; triangles[i] = new(new(p[0],p[1],p[2]), new(p[3],p[4],p[5]), new(p[6],p[7],p[8])); }
        var built = new List<Node>();
        int Build(int start, int count)
        {
            var min = new Vector3(float.MaxValue); var max = new Vector3(float.MinValue);
            for (int i = start; i < start + count; i++) { min = Vector3.Min(min, triangles[i].Min); max = Vector3.Max(max, triangles[i].Max); }
            int id = built.Count; built.Add(default);
            if (count <= 8) { built[id] = new(min,max,start,count,-1,-1); return id; }
            var span = max-min; int axis = span.X > span.Z ? 0 : 2;
            float Center(Triangle t) => axis == 0 ? t.A.X+t.B.X+t.C.X : t.A.Z+t.B.Z+t.C.Z;
            Array.Sort(triangles,start,count,Comparer<Triangle>.Create((a,b) => Center(a).CompareTo(Center(b))));
            int left = Build(start,count/2), right = Build(start+count/2,count-count/2);
            built[id] = new(min,max,start,0,left,right); return id;
        }
        Build(0,triangles.Length); nodes = built.ToArray();
    }
    public bool Clear(Vector3 a, Vector3 b, float radius, Vector3? contactSource = null)
    {
        var extent = new Vector3(radius); var min = Vector3.Min(a,b)-extent; var max = Vector3.Max(a,b)+extent;
        Span<int> stack = stackalloc int[64]; int count = 1; stack[0] = 0;
        while (count > 0)
        {
            var n = nodes[stack[--count]];
            if (min.X > n.Max.X || max.X < n.Min.X || min.Y > n.Max.Y || max.Y < n.Min.Y || min.Z > n.Max.Z || max.Z < n.Min.Z) continue;
            if (n.Count == 0) { stack[count++] = n.Left; stack[count++] = n.Right; continue; }
            for (int i = n.Start; i < n.Start+n.Count; i++)
            {
                var t = triangles[i];
                if (SegmentTriangleDistanceSquared(a,b,t) >= radius*radius) continue;
                if (contactSource is { } source)
                {
                    var outward = a-Closest(a,t);
                    // Only leave an existing shallow contact. Distance to a
                    // convex triangle cannot decrease along this direction.
                    float initial = Vector3.Distance(source,Closest(source,t));
                    if (initial < radius && radius-initial < .29f && outward.Length() >= initial-.0001f && Vector3.Dot(outward,b-a) >= 0) continue;
                }
                return false;
            }
        }
        return true;
    }    private static float SegmentTriangleDistanceSquared(Vector3 a,Vector3 b,Triangle t)
    {
        var n=Vector3.Cross(t.B-t.A,t.C-t.A);var d=b-a;float denom=Vector3.Dot(n,d);
        if(MathF.Abs(denom)>1e-9f)
        {
            float u=Vector3.Dot(n,t.A-a)/denom;
            if(u>=0&&u<=1) { var p=a+d*u;var v0=t.B-t.A;var v1=t.C-t.A;var v2=p-t.A;
                float aa=Vector3.Dot(v0,v0),ab=Vector3.Dot(v0,v1),bb=Vector3.Dot(v1,v1),ap=Vector3.Dot(v0,v2),bp=Vector3.Dot(v1,v2);
                float det=aa*bb-ab*ab,x=(bb*ap-ab*bp)/det,y=(aa*bp-ab*ap)/det;if(x>=0&&y>=0&&x+y<=1)return 0; }
        }
        return MathF.Min(MathF.Min(Vector3.DistanceSquared(a,Closest(a,t)),Vector3.DistanceSquared(b,Closest(b,t))),
            MathF.Min(SegmentDistance(a,b,t.A,t.B),MathF.Min(SegmentDistance(a,b,t.B,t.C),SegmentDistance(a,b,t.C,t.A))));
    }
    private static Vector3 Closest(Vector3 p,Triangle t)
    {
        var ab=t.B-t.A;var ac=t.C-t.A;var ap=p-t.A;float d1=Vector3.Dot(ab,ap),d2=Vector3.Dot(ac,ap);
        if(d1<=0&&d2<=0)return t.A;
        var bp=p-t.B;float d3=Vector3.Dot(ab,bp),d4=Vector3.Dot(ac,bp);if(d3>=0&&d4<=d3)return t.B;
        float vc=d1*d4-d3*d2;if(vc<=0&&d1>=0&&d3<=0)return t.A+ab*(d1/(d1-d3));
        var cp=p-t.C;float d5=Vector3.Dot(ab,cp),d6=Vector3.Dot(ac,cp);if(d6>=0&&d5<=d6)return t.C;
        float vb=d5*d2-d1*d6;if(vb<=0&&d2>=0&&d6<=0)return t.A+ac*(d2/(d2-d6));
        float va=d3*d6-d5*d4;if(va<=0&&d4-d3>=0&&d5-d6>=0)return t.B+(t.C-t.B)*((d4-d3)/((d4-d3)+(d5-d6)));
        return t.A+(ab*vb+ac*vc)/(va+vb+vc);
    }
    private static float SegmentDistance(Vector3 a,Vector3 b,Vector3 c,Vector3 d)
    {
        var u=b-a;var v=d-c;var w=a-c;float aa=Vector3.Dot(u,u),bb=Vector3.Dot(u,v),cc=Vector3.Dot(v,v),dd=Vector3.Dot(u,w),ee=Vector3.Dot(v,w);
        float denom=aa*cc-bb*bb,s=denom>1e-12f?Math.Clamp((bb*ee-cc*dd)/denom,0,1):0;
        float q=cc>1e-12f?(bb*s+ee)/cc:0;
        if(q<0){q=0;s=aa>1e-12f?Math.Clamp(-dd/aa,0,1):0;}else if(q>1){q=1;s=aa>1e-12f?Math.Clamp((bb-dd)/aa,0,1):0;}
        return Vector3.DistanceSquared(a+u*s,c+v*q);
    }
}
