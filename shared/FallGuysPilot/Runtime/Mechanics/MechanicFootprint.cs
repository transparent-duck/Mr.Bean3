using System;
using System.Collections.Immutable;
using System.Numerics;
using System.Text.Json;

namespace MrBean.FallGuys.Mechanics;

public enum MechanicFootprintKind { Circle,Cone,Rectangle,Charge }
internal enum MechanicFootprintResult { Candidate,NoSpatialShape,MissingSource,MissingDirection,MissingDimensions,MissingEndpoint }

// Candidate geometry, explicitly separate from calibrated server hit volumes.
// Width/range stay in the original data; this is not a max-hit-distance fit.
public readonly record struct MechanicFootprint(MechanicFootprintKind Kind,Vector2 Origin,
    float Rotation,float Radius,float ForwardMin,float ForwardMax,float HalfWidth,
    float HalfAngle,Vector2 Endpoint)
{
    // Mechanic recipient extent, independent of the terrain collision capsule.
    // Zero is the point-target candidate. A nonzero value requires calibration
    // for this mechanic; actor GetRadius() is not evidence for that value.
    public float TargetRadius {get;init;}
    public ImmutableArray<GroundedShelter> GroundedShelters {get;init;}=[];
    public bool Sheltered(Vector3 from,Vector3 to,float error=0)
    {
        if(GroundedShelters.IsDefaultOrEmpty)return false;
        foreach(var shelter in GroundedShelters)
            if(shelter.Contains(from,error)&&shelter.Contains(to,error))return true;
        return false;
    }
    public float Distance(Vector2 point)=>ShapeDistance(point)-TargetRadius;

    private float ShapeDistance(Vector2 point)
    {
        var relative=point-Origin;
        if(Kind==MechanicFootprintKind.Circle)return relative.Length()-Radius;
        if(Kind==MechanicFootprintKind.Charge)
        {
            var direction=Endpoint-Origin;float length=direction.LengthSquared();
            float fraction=length>0?Math.Clamp(Vector2.Dot(relative,direction)/length,0,1):0;
            return Vector2.Distance(point,Origin+direction*fraction)-HalfWidth;
        }
        float forward=relative.X*MathF.Sin(Rotation)+relative.Y*MathF.Cos(Rotation);
        float lateral=relative.X*MathF.Cos(Rotation)-relative.Y*MathF.Sin(Rotation);
        if(Kind==MechanicFootprintKind.Rectangle)
        {
            var excess=new Vector2(MathF.Abs(lateral)-HalfWidth,
                MathF.Abs(forward-(ForwardMin+ForwardMax)/2)-(ForwardMax-ForwardMin)/2);
            return Vector2.Max(excess,Vector2.Zero).Length()+MathF.Min(MathF.Max(excess.X,excess.Y),0);
        }
        // Intersection of the range disk and the two oriented cone half-planes.
        // For the verified visual fan180, cos(halfAngle)=0 and this is a half-disk.
        float side=MathF.Abs(lateral)*MathF.Cos(HalfAngle)-forward*MathF.Sin(HalfAngle);
        return MathF.Max(relative.Length()-Radius,side);
    }
}

internal static class MechanicFootprints
{
    // The union of every orientation of an attack at a fixed origin.
    // This is an AoE envelope, not a collision volume of its actor.
    public static MechanicFootprint RotationEnvelope(MechanicFootprint shape)
    {
        float radius=shape.Kind switch
        {
            MechanicFootprintKind.Circle or MechanicFootprintKind.Cone=>shape.Radius,
            MechanicFootprintKind.Rectangle=>MathF.Sqrt(
                MathF.Max(shape.ForwardMin*shape.ForwardMin,shape.ForwardMax*shape.ForwardMax)+shape.HalfWidth*shape.HalfWidth),
            _=>throw new InvalidOperationException("A translating charge has no fixed-origin rotation envelope.")
        };
        return new(MechanicFootprintKind.Circle,shape.Origin,0,radius,0,0,0,0,default){TargetRadius=shape.TargetRadius};
    }

    private static float? Number(JsonElement element,string key)
        =>element.TryGetProperty(key,out var property)&&property.ValueKind==JsonValueKind.Number
            ?property.GetSingle():null;
    private static Vector2 XZ(Vector3 point)=>new(point.X,point.Z);

    public static MechanicFootprintResult Build(MechanicActionState action,MechanicSourceState source,
        out MechanicFootprint result)
    {
        var built=BuildShape(action,source,out result);
        if(built!=MechanicFootprintResult.Candidate)return built;
        float radius=Number(action.Definition.Geometry,"targetRadius")??0;
        if(!float.IsFinite(radius)||radius<0)return MechanicFootprintResult.MissingDimensions;
        result=result with{TargetRadius=radius};return built;
    }

    private static MechanicFootprintResult BuildShape(MechanicActionState action,MechanicSourceState source,
        out MechanicFootprint result)
    {
        result=default;var geometry=action.Definition.Geometry;
        string? shape=geometry.GetProperty("shape").GetString();
        if(shape=="none")return MechanicFootprintResult.NoSpatialShape;
        if(source.Position is not { } position)return MechanicFootprintResult.MissingSource;
        var origin=XZ(position);
        if(action.Definition.SelfCastOrigin&&action.LastCast is { } cast)
        {
            if(cast.TargetEntityId!=cast.EntityId||!float.IsFinite(cast.Position.X)||!float.IsFinite(cast.Position.Z))
                return MechanicFootprintResult.MissingSource;
            // Only catalog-validated self-target actions use this location.
            // The companion actor pose can precede the cast by several frames.
            origin=XZ(cast.Position);
        }
        if(shape=="moving-charge-corridor")
        {
            // Ring35409 announces Q in its cast. Hit/Crystal34774 announce
            // the current Q in ActionEffect, including packets with zero targets.
            if(action.ChargeDestination is not { } destination)return MechanicFootprintResult.MissingEndpoint;
            if(Number(geometry,"halfWidth") is not { } half)return MechanicFootprintResult.MissingDimensions;
            result=new(MechanicFootprintKind.Charge,origin,0,0,0,0,half,0,XZ(destination));
            return MechanicFootprintResult.Candidate;
        }
        if(shape=="circle")
        {
            if(Number(geometry,"radius") is not { } radius)return MechanicFootprintResult.MissingDimensions;
            result=new(MechanicFootprintKind.Circle,origin,0,radius,0,0,0,0,default);
            return MechanicFootprintResult.Candidate;
        }
        if((action.Facing??(action.Definition.FixedOriginRotationEnvelope?source.Rotation??0:null)) is not { } direction)
            return MechanicFootprintResult.MissingDirection;
        if(shape=="cone")
        {
            if(Number(geometry,"radius") is not { } radius||Number(geometry,"halfAngleRadians") is not { } angle)
                return MechanicFootprintResult.MissingDimensions;
            result=new(MechanicFootprintKind.Cone,origin,direction,radius,0,0,0,angle,default);
            return MechanicFootprintResult.Candidate;
        }
        if(shape=="rectangle")
        {
            if(Number(geometry,"length") is not { } length||Number(geometry,"halfWidth") is not { } half)
                return MechanicFootprintResult.MissingDimensions;
            // 34716's dense pre-knockback trace disproves the forward-only
            // anchor. Use the same-size centered candidate, and retain its
            // uncalibrated status; no enlarged range or extra margin is added.
            float first=action.Definition.ActionId==34716?-length/2:0;
            result=new(MechanicFootprintKind.Rectangle,origin,direction,0,first,first+length,half,0,default);
            return MechanicFootprintResult.Candidate;
        }
        return MechanicFootprintResult.MissingDimensions;
    }
}
