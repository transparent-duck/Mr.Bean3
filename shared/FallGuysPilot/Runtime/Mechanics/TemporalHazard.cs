using System;
using System.Numerics;

namespace MrBean.FallGuys.Mechanics;

internal enum TemporalCollisionKind { Clear,PossibleContact,Unresolved }
internal enum TemporalChargeEndpointMotion { WorldFixed,SourceRelative }
internal enum TemporalCollisionIssue
{
    None,UnknownGeometry,InvalidGeometry,InvalidTime,InvalidMotion,InvalidPlayerRadius,
    InvalidFrameDuration,InvalidBudget,InvalidTrajectory,IncompleteTrajectory,
    InvalidTrajectoryError,SweepBudgetExceeded,NumericOverflow
}

[Flags]
internal enum TemporalCollisionProvenance
{
    None=0,InactiveWindow=1,CandidateFootprint=2,LinearRelativeDisk=4,
    RectangleSlabsAndCorners=8,ChargeCapsule=16,NativeFrameSweep=32,
    ConeHalfPlaneBound=64,RotationDisplacementBound=128,TrajectoryChordBound=256,
    WorldFixedChargeEndpoint=512,SourceRelativeChargeEndpoint=1024,ChargeEndpointDisplacementBound=2048,
    BoundingDiskExclusion=4096,GroundedDisplacementBound=8192
}

// Seconds use the same absolute clock as the hazard. Y is retained for the
// caller's motion evidence; this XZ query does not infer jump/AoE immunity.
internal readonly record struct TemporalTrajectoryPoint(double Seconds,Vector3 Position);

internal readonly record struct TemporalCollisionResult(TemporalCollisionKind Kind,
    TemporalCollisionProvenance Provenance,TemporalCollisionIssue Issue,
    double? EarliestPossibleContact,double? ContactBracketEnd,double CheckedUntil,
    int Steps,float SpatialExpansion)
{
    public bool IsClear=>Kind==TemporalCollisionKind.Clear;
}

// A motion MODEL over a finite active window, not a calibrated server hit volume.
// Null velocities mean a fixed snapshot model. The caller owns motion/timing
// authority. WorldFixed charge motion advances P while packet Q stays fixed;
// rotation does not rotate Q. SourceRelative is an explicit alternative model
// which translates/rotates the entire snapshot charge corridor.
// Rotation increases in the existing FFXIV XZ convention: forward=(sin,cos).
internal readonly record struct TemporalHazard(MechanicSourceKey Source,uint ActionId,
    MechanicFootprint? Footprint,double ActiveStart,double ActiveEnd,double SnapshotAt,
    Vector2? SourceVelocity=null,float? RotationVelocity=null,
    TemporalChargeEndpointMotion ChargeEndpointMotion=TemporalChargeEndpointMotion.WorldFixed)
{
    public const int DefaultMaximumSteps=256;
    private const double Roundoff=2.2204460492503131e-16;
    // The disk discriminant contains fourth powers. This bound leaves arithmetic
    // headroom; it is not a spatial calibration or a game-world limit.
    private static readonly double SafeArithmeticMagnitude=Math.Sqrt(Math.Sqrt(double.MaxValue))/32;

    // Owned pose projection for rasterization/diagnostics. This does not grant
    // activity outside the window or convert the candidate to server geometry.
    public bool TryFootprintAt(double at,out MechanicFootprint result)
    {
        result=default;
        if(!double.IsFinite(at)||!double.IsFinite(SnapshotAt)||Footprint is not { } footprint||
            !Valid(footprint)||SourceVelocity is { } velocity&&!Finite(velocity)||
            RotationVelocity is { } angular&&!float.IsFinite(angular)||
            footprint.Kind==MechanicFootprintKind.Charge&&ChargeEndpointMotion is not
                (TemporalChargeEndpointMotion.WorldFixed or TemporalChargeEndpointMotion.SourceRelative)||
            !Pose(at,out var origin,out double rotation,out var direction))return false;
        var position=new Vector2((float)origin.X,(float)origin.Z);
        if(!Finite(position))return false;
        result=footprint with {Origin=position};
        if(footprint.Kind is MechanicFootprintKind.Rectangle or MechanicFootprintKind.Cone)
            result=result with {Rotation=(float)Math.IEEERemainder(rotation,Math.Tau)};
        else if(footprint.Kind==MechanicFootprintKind.Charge&&ChargeEndpointMotion==TemporalChargeEndpointMotion.SourceRelative)
        {
            var end=origin+direction;var endpoint=new Vector2((float)end.X,(float)end.Z);
            if(!Finite(endpoint)){result=default;return false;}
            result=result with {Endpoint=endpoint};
        }
        return true;
    }

    // The receiver is a coordinate point plus Footprint.TargetRadius. The
    // query expansion describes trajectory uncertainty, never terrain body size.
    // Actual P -> Q motion is linear in time. The frame duration is supplied by
    // the caller; only cone/rotating geometry needs the bounded frame sweep.
    public TemporalCollisionResult Query(Vector3 from,Vector3 to,double departure,double arrival,
        float trajectoryRadius,double nativeFrameSeconds,int maximumSteps=DefaultMaximumSteps)
    {
        var validation=Validate(departure,arrival,trajectoryRadius,nativeFrameSeconds,maximumSteps,
            out double first,out double last);
        if(validation is { } early)return early;
        if(!Finite(from)||!Finite(to)||departure==arrival&&from!=to)
            return Unknown(TemporalCollisionIssue.InvalidTrajectory,first);
        if(SourceVelocity is null&&RotationVelocity is null&&Footprint is { } sheltered&&
            sheltered.Sheltered(Vector3.Lerp(from,to,(float)Fraction(first,departure,arrival)),
                Vector3.Lerp(from,to,(float)Fraction(last,departure,arrival)),trajectoryRadius))
            return Clear(last,0,0,TemporalCollisionProvenance.GroundedDisplacementBound);
        var p=Lerp(from,to,Fraction(first,departure,arrival));
        var q=Lerp(from,to,Fraction(last,departure,arrival));
        int steps=0;float expansion=0;var provenance=TemporalCollisionProvenance.CandidateFootprint;
        return Segment(p,q,first,last,trajectoryRadius,0,nativeFrameSeconds,maximumSteps,
            ref steps,ref expansion,ref provenance);
    }

    // The samples specify a piecewise-linear XZ prediction. For a curved XZ
    // trajectory, planarChordError must bound its maximum deviation from EVERY
    // supplied chord. Zero asserts linear XZ motion; unbounded/unknown curves
    // must not be represented by zero. Ordinary ballistic Y alone needs no XZ
    // expansion. No sample extrapolation or endpoint-only jump check occurs.
    public TemporalCollisionResult QueryTrajectory(ReadOnlySpan<TemporalTrajectoryPoint> trajectory,
        double departure,double arrival,float trajectoryRadius,double nativeFrameSeconds,
        float planarChordError,int maximumSteps=DefaultMaximumSteps)
    {
        // A sampled native action or jump does not grant grounded shelter.
        if(Footprint is {GroundedShelters.IsDefaultOrEmpty:false} shape)
            return (this with{Footprint=shape with{GroundedShelters=[]}}).QueryTrajectory(
                trajectory,departure,arrival,trajectoryRadius,nativeFrameSeconds,planarChordError,maximumSteps);
        var validation=Validate(departure,arrival,trajectoryRadius,nativeFrameSeconds,maximumSteps,
            out double first,out double last);
        if(validation is { } early)return early;
        if(!float.IsFinite(planarChordError)||planarChordError<0)
            return Unknown(TemporalCollisionIssue.InvalidTrajectoryError,first);
        if(trajectory.Length==0)return Unknown(TemporalCollisionIssue.IncompleteTrajectory,first);
        // Validation itself is bounded, including samples outside the active window.
        if((long)trajectory.Length>maximumSteps+1L)
            return Unknown(TemporalCollisionIssue.SweepBudgetExceeded,first);
        for(int i=0;i<trajectory.Length;++i)
            if(!double.IsFinite(trajectory[i].Seconds)||!Finite(trajectory[i].Position)||
                i>0&&trajectory[i].Seconds<=trajectory[i-1].Seconds)
                return Unknown(TemporalCollisionIssue.InvalidTrajectory,first);
        if(trajectory[0].Seconds>departure||trajectory[^1].Seconds<arrival)
            return Unknown(TemporalCollisionIssue.IncompleteTrajectory,first);
        int steps=0;float expansion=0;var provenance=TemporalCollisionProvenance.CandidateFootprint;
        if(planarChordError>0)provenance|=TemporalCollisionProvenance.TrajectoryChordBound;
        if(trajectory.Length==1)
        {
            if(departure!=arrival)return Unknown(TemporalCollisionIssue.IncompleteTrajectory,first);
            var point=XZ(trajectory[0].Position);
            return Segment(point,point,first,last,trajectoryRadius,planarChordError,nativeFrameSeconds,
                maximumSteps,ref steps,ref expansion,ref provenance);
        }
        for(int i=1;i<trajectory.Length;++i)
        {
            var a=trajectory[i-1];var b=trajectory[i];
            if(b.Seconds<first)continue;
            if(a.Seconds>last)break;
            double start=Math.Max(first,a.Seconds),end=Math.Min(last,b.Seconds);
            var p=Lerp(a.Position,b.Position,Fraction(start,a.Seconds,b.Seconds));
            var q=Lerp(a.Position,b.Position,Fraction(end,a.Seconds,b.Seconds));
            var result=Segment(p,q,start,end,trajectoryRadius,planarChordError,nativeFrameSeconds,
                maximumSteps,ref steps,ref expansion,ref provenance);
            if(!result.IsClear)return result;
            if(end==last)return Clear(last,steps,expansion,provenance);
        }
        return Unknown(TemporalCollisionIssue.IncompleteTrajectory,first,steps,expansion,provenance);
    }

    private TemporalCollisionResult? Validate(double departure,double arrival,float trajectoryRadius,
        double frame,int budget,out double first,out double last)
    {
        first=departure;last=arrival;
        if(!double.IsFinite(departure)||!double.IsFinite(arrival)||arrival<departure||
            !double.IsFinite(arrival-departure)||!double.IsFinite(ActiveStart)||
            !double.IsFinite(ActiveEnd)||ActiveEnd<ActiveStart||!double.IsFinite(ActiveEnd-ActiveStart))
            return Unknown(TemporalCollisionIssue.InvalidTime,double.NaN);
        first=Math.Max(departure,ActiveStart);last=Math.Min(arrival,ActiveEnd);
        // Closed windows also cover an instantaneous activation and tangency.
        if(first>last)return Clear(arrival,0,0,TemporalCollisionProvenance.InactiveWindow);
        if(Footprint is not { } footprint)return Unknown(TemporalCollisionIssue.UnknownGeometry,first);
        if(!Valid(footprint))return Unknown(TemporalCollisionIssue.InvalidGeometry,first);
        if(!double.IsFinite(SnapshotAt)||SourceVelocity is { } velocity&&!Finite(velocity)||
            RotationVelocity is { } rotation&&!float.IsFinite(rotation)||
            footprint.Kind==MechanicFootprintKind.Charge&&ChargeEndpointMotion is not
                (TemporalChargeEndpointMotion.WorldFixed or TemporalChargeEndpointMotion.SourceRelative))
            return Unknown(TemporalCollisionIssue.InvalidMotion,first);
        if(!float.IsFinite(trajectoryRadius)||trajectoryRadius<0)
            return Unknown(TemporalCollisionIssue.InvalidPlayerRadius,first);
        if(!double.IsFinite(frame)||frame<=0)return Unknown(TemporalCollisionIssue.InvalidFrameDuration,first);
        if(budget<=0)return Unknown(TemporalCollisionIssue.InvalidBudget,first);
        return null;
    }

    private TemporalCollisionResult Segment(Point p,Point q,double first,double last,float radius,
        float chordError,double frame,int budget,ref int steps,ref float maximumExpansion,
        ref TemporalCollisionProvenance provenance)
    {
        var footprint=Footprint!.Value;
        radius+=footprint.TargetRadius;
        bool fixedCharge=footprint.Kind==MechanicFootprintKind.Charge&&
            ChargeEndpointMotion==TemporalChargeEndpointMotion.WorldFixed;
        var sourceVelocity=SourceVelocity??Vector2.Zero;
        double sourceSpeed=new Point(sourceVelocity.X,sourceVelocity.Y).Length;
        bool changesEndpoint=fixedCharge&&sourceSpeed>0;
        bool rotates=!fixedCharge&&footprint.Kind!=MechanicFootprintKind.Circle&&(RotationVelocity??0)!=0;
        bool sweep=rotates||changesEndpoint||footprint.Kind==MechanicFootprintKind.Cone;
        if(footprint.Kind==MechanicFootprintKind.Charge)
            provenance|=fixedCharge?TemporalCollisionProvenance.WorldFixedChargeEndpoint:
                TemporalCollisionProvenance.SourceRelativeChargeEndpoint;
        if(sweep&&!fixedCharge)
        {
            // Rotation preserves the whole shape's distance from its source.
            // An excluded relative chord proves clearance for every rotation,
            // before the native-frame work budget is charged. Intersection
            // only retains the existing exact/conservative shape query below.
            double bound=Extent(footprint)+radius+chordError;
            // Square roots and radius sums can round inward at a tangent.
            // Keep this enclosing disk outward of their arithmetic envelope.
            bound=Math.BitIncrement(bound+bound*8*Roundoff);
            if(!double.IsFinite(bound)||bound>SafeArithmeticMagnitude||
                !Pose(first,out var firstOrigin,out _,out _)||
                !Pose(last,out var lastOrigin,out _,out _))
                return Unknown(TemporalCollisionIssue.NumericOverflow,first,steps,maximumExpansion,provenance);
            var a=p-firstOrigin;var b=q-lastOrigin;
            if(!SafeArithmetic(a)||!SafeArithmetic(b))
                return Unknown(TemporalCollisionIssue.NumericOverflow,first,steps,maximumExpansion,provenance);
            if(!Disk(a,b,bound,out _,out _))
            {
                maximumExpansion=Math.Max(maximumExpansion,chordError);
                return Clear(last,steps,maximumExpansion,provenance|TemporalCollisionProvenance.BoundingDiskExclusion);
            }
        }
        double count=sweep?Math.Max(1,Math.Ceiling((last-first)/frame)):1;
        if(!double.IsFinite(count)||count>budget-steps)
            return Unknown(TemporalCollisionIssue.SweepBudgetExceeded,first,steps,maximumExpansion,provenance);
        int slices=(int)count;
        if(sweep)provenance|=TemporalCollisionProvenance.NativeFrameSweep;
        for(int slice=0;slice<slices;++slice)
        {
            double start=first+(last-first)*((double)slice/slices);
            double end=slice+1==slices?last:first+(last-first)*((double)(slice+1)/slices);
            var a=Lerp(p,q,(double)slice/slices);var b=Lerp(p,q,(double)(slice+1)/slices);
            double extra=chordError;
            if(changesEndpoint)
            {
                // Relative Q changes by -sourceVelocity * dt. Each point of
                // the varying-length corridor is within this displacement of
                // the frozen corridor. World-space packet Q never moves.
                extra+=sourceSpeed*(end-start);
                provenance|=TemporalCollisionProvenance.ChargeEndpointDisplacementBound;
            }
            if(rotates)
            {
                double angularTravel=Math.Abs((double)RotationVelocity!.Value)*(end-start);
                // Every point in a bounded shape moves at most this chord under
                // any intermediate rotation. This includes the charge endpoint.
                extra+=2*Extent(footprint)*Math.Sin(Math.Min(angularTravel,Math.PI)/2);
                provenance|=TemporalCollisionProvenance.RotationDisplacementBound;
            }
            double expanded=radius+extra;
            if(!double.IsFinite(expanded)||extra>float.MaxValue||
                !Pose(start,out var origin,out double rotation,out var chargeDirection)||
                !Origin(end,out var finalOrigin))
                return Unknown(TemporalCollisionIssue.NumericOverflow,start,steps,maximumExpansion,provenance);
            maximumExpansion=Math.Max(maximumExpansion,(float)extra);
            // Subtract both source positions: the player/source relative chord
            // is analytic even when both world-space objects move.
            a-=origin;b-=finalOrigin;++steps;
            if(!SafeArithmetic(a)||!SafeArithmetic(b)||expanded>SafeArithmeticMagnitude)
                return Unknown(TemporalCollisionIssue.NumericOverflow,start,steps,maximumExpansion,provenance);
            if(Intersect(footprint,a,b,rotation,chargeDirection,expanded,out double fraction,
                out var method))
            {
                provenance|=method;
                double contact=start+(end-start)*fraction;
                bool bound=sweep||chordError>0;
                return new(TemporalCollisionKind.PossibleContact,provenance,TemporalCollisionIssue.None,
                    contact,bound?end:contact,contact,steps,maximumExpansion);
            }
            provenance|=method;
        }
        return Clear(last,steps,maximumExpansion,provenance);
    }

    private bool Origin(double at,out Point origin)
    {
        var footprint=Footprint!.Value;var velocity=SourceVelocity??Vector2.Zero;
        double elapsed=at-SnapshotAt;
        origin=new(footprint.Origin.X+velocity.X*elapsed,footprint.Origin.Y+velocity.Y*elapsed);
        return Finite(origin);
    }

    private bool Pose(double at,out Point origin,out double rotation,out Point chargeDirection)
    {
        bool valid=Origin(at,out origin);var footprint=Footprint!.Value;
        rotation=0;chargeDirection=default;
        if(footprint.Kind==MechanicFootprintKind.Circle)return valid;
        if(footprint.Kind==MechanicFootprintKind.Charge&&ChargeEndpointMotion==TemporalChargeEndpointMotion.WorldFixed)
        {
            chargeDirection=new Point(footprint.Endpoint.X,footprint.Endpoint.Y)-origin;
            return valid&&SafeArithmetic(chargeDirection);
        }
        double turn=(RotationVelocity??0)*(at-SnapshotAt);
        rotation=footprint.Kind==MechanicFootprintKind.Charge?turn:footprint.Rotation+turn;
        if(footprint.Kind!=MechanicFootprintKind.Charge)return valid&&double.IsFinite(rotation);
        var delta=new Point((double)footprint.Endpoint.X-footprint.Origin.X,
            (double)footprint.Endpoint.Y-footprint.Origin.Y);
        double sin=Math.Sin(turn),cos=Math.Cos(turn);
        chargeDirection=new(delta.X*cos+delta.Z*sin,delta.Z*cos-delta.X*sin);
        return valid&&double.IsFinite(rotation)&&SafeArithmetic(chargeDirection);
    }

    // Entry/exit fractions on a stationary convex footprint. The graph uses
    // these spatial fractions to translate active windows into departure
    // windows, instead of reserving a whole corridor for the whole traversal.
    internal static bool StationaryOverlap(MechanicFootprint footprint,Vector3 from,Vector3 to,
        float radius,out double enter,out double exit,bool airborne=false)
    {
        if(!Valid(footprint)||!Finite(from)||!Finite(to)||!float.IsFinite(radius)||radius<0)
            throw new ArgumentException("Invalid stationary overlap geometry.");
        if(!airborne&&footprint.Sheltered(from,to,radius)){enter=exit=0;return false;}
        var origin=new Point(footprint.Origin.X,footprint.Origin.Y);
        var a=XZ(from)-origin;var b=XZ(to)-origin;
        var delta=new Point(footprint.Endpoint.X-footprint.Origin.X,footprint.Endpoint.Y-footprint.Origin.Y);
        radius+=footprint.TargetRadius;
        exit=0;
        if(!Intersect(footprint,a,b,footprint.Rotation,delta,radius,out enter,out _))return false;
        Intersect(footprint,b,a,footprint.Rotation,delta,radius,out double reverse,out _);
        exit=1-reverse;return true;
    }

    private static bool Intersect(MechanicFootprint footprint,Point from,Point to,double rotation,
        Point chargeDirection,double radius,out double first,out TemporalCollisionProvenance method)
    {
        first=0;
        if(footprint.Kind==MechanicFootprintKind.Circle)
        {
            method=TemporalCollisionProvenance.LinearRelativeDisk;
            return Disk(from,to,footprint.Radius+radius,out first,out _);
        }
        if(footprint.Kind==MechanicFootprintKind.Charge)
        {
            method=TemporalCollisionProvenance.ChargeCapsule;
            double length=chargeDirection.Length;
            if(length==0)return Disk(from,to,footprint.HalfWidth+radius,out first,out _);
            var direction=chargeDirection/length;
            var a=new Point(from.X*direction.Z-from.Z*direction.X,Dot(from,direction));
            var b=new Point(to.X*direction.Z-to.Z*direction.X,Dot(to,direction));
            double width=footprint.HalfWidth+radius;
            bool found=Box(a,b,-width,width,0,length,out first);
            found=EarlierDisk(a,b,new(0,0),width,ref first,found);
            return EarlierDisk(a,b,new(0,length),width,ref first,found);
        }
        double sin=Math.Sin(rotation),cos=Math.Cos(rotation);
        var localFrom=new Point(from.X*cos-from.Z*sin,from.X*sin+from.Z*cos);
        var localTo=new Point(to.X*cos-to.Z*sin,to.X*sin+to.Z*cos);
        if(footprint.Kind==MechanicFootprintKind.Rectangle)
        {
            method=TemporalCollisionProvenance.RectangleSlabsAndCorners;
            double half=footprint.HalfWidth,min=footprint.ForwardMin,max=footprint.ForwardMax;
            // Exact radius inflation: two rectangular strips plus corner disks.
            // A single expanded box would incorrectly include its outer corners.
            bool found=Box(localFrom,localTo,-half-radius,half+radius,min,max,out first);
            if(Box(localFrom,localTo,-half,half,min-radius,max+radius,out double entry)&&(!found||entry<first))
            {first=entry;found=true;}
            found=EarlierDisk(localFrom,localTo,new(-half,min),radius,ref first,found);
            found=EarlierDisk(localFrom,localTo,new(half,min),radius,ref first,found);
            found=EarlierDisk(localFrom,localTo,new(-half,max),radius,ref first,found);
            return EarlierDisk(localFrom,localTo,new(half,max),radius,ref first,found);
        }
        method=TemporalCollisionProvenance.ConeHalfPlaneBound;
        // Intersection of the expanded range disk and expanded cone half-planes
        // contains the radius-inflated fan. It is a conservative candidate at
        // side/range corners, not a calibrated distance-to-server-hit solution.
        if(!Disk(localFrom,localTo,footprint.Radius+radius,out double enter,out double exit))return false;
        double coneSin=Math.Sin(footprint.HalfAngle),coneCos=Math.Max(0,Math.Cos(footprint.HalfAngle));
        if(!HalfPlane(localFrom.Z*coneSin-localFrom.X*coneCos,
            localTo.Z*coneSin-localTo.X*coneCos,-radius,ref enter,ref exit)||
            !HalfPlane(localFrom.Z*coneSin+localFrom.X*coneCos,
                localTo.Z*coneSin+localTo.X*coneCos,-radius,ref enter,ref exit))return false;
        first=enter;return true;
    }

    private static bool EarlierDisk(Point a,Point b,Point center,double radius,ref double first,bool found)
    {
        if(Disk(a-center,b-center,radius,out double entry,out _)&&(!found||entry<first))
        {first=entry;return true;}
        return found;
    }

    private static bool Disk(Point a,Point b,double radius,out double enter,out double exit)
    {
        enter=0;exit=1;var delta=b-a;double aa=Dot(delta,delta),bb=Dot(a,delta);
        double cc=Dot(a,a)-radius*radius;
        if(aa==0)return cc<=0;
        double square=bb*bb,product=aa*cc;
        double discriminant=square-product;
        // Bound cancellation at a tangent by machine roundoff, not a gameplay
        // distance or launch-timing tolerance. A doubtful tangent is a contact.
        double numericalEnvelope=(Math.Abs(square)+Math.Abs(product))*8*Roundoff;
        if(discriminant < -numericalEnvelope)return false;
        double root=Math.Sqrt(Math.Max(0,discriminant+numericalEnvelope));
        enter=Math.Max(0,(-bb-root)/aa);exit=Math.Min(1,(-bb+root)/aa);
        return enter<=exit;
    }

    private static bool Box(Point a,Point b,double minX,double maxX,double minZ,double maxZ,out double enter)
    {
        enter=0;double exit=1;
        return Slab(a.X,b.X,minX,maxX,ref enter,ref exit)&&Slab(a.Z,b.Z,minZ,maxZ,ref enter,ref exit);
    }

    private static bool Slab(double a,double b,double min,double max,ref double enter,ref double exit)
        =>HalfPlane(a,b,min,ref enter,ref exit)&&HalfPlane(-a,-b,-max,ref enter,ref exit);

    private static bool HalfPlane(double a,double b,double min,ref double enter,ref double exit)
    {
        double delta=b-a;
        if(delta==0)return a>=min;
        double t=(min-a)/delta;
        if(delta>0)enter=Math.Max(enter,t);else exit=Math.Min(exit,t);
        return enter<=exit;
    }

    private static bool Valid(MechanicFootprint f)
    {
        if(!Finite(f.Origin)||!float.IsFinite(f.TargetRadius)||f.TargetRadius<0)return false;
        return f.Kind switch
        {
            MechanicFootprintKind.Circle=>float.IsFinite(f.Radius)&&f.Radius>=0,
            MechanicFootprintKind.Rectangle=>float.IsFinite(f.Rotation)&&float.IsFinite(f.HalfWidth)&&f.HalfWidth>=0&&
                float.IsFinite(f.ForwardMin)&&float.IsFinite(f.ForwardMax)&&f.ForwardMin<=f.ForwardMax,
            MechanicFootprintKind.Charge=>Finite(f.Endpoint)&&float.IsFinite(f.HalfWidth)&&f.HalfWidth>=0,
            MechanicFootprintKind.Cone=>float.IsFinite(f.Rotation)&&float.IsFinite(f.Radius)&&f.Radius>=0&&
                float.IsFinite(f.HalfAngle)&&f.HalfAngle>0&&f.HalfAngle<=MathF.PI/2,
            _=>false
        };
    }

    private static double Extent(MechanicFootprint f)=>f.Kind switch
    {
        MechanicFootprintKind.Rectangle=>Math.Sqrt((double)f.HalfWidth*f.HalfWidth+
            Math.Max((double)f.ForwardMin*f.ForwardMin,(double)f.ForwardMax*f.ForwardMax)),
        MechanicFootprintKind.Charge=>new Point((double)f.Endpoint.X-f.Origin.X,(double)f.Endpoint.Y-f.Origin.Y).Length+f.HalfWidth,
        _=>f.Radius
    };

    private static TemporalCollisionResult Clear(double until,int steps,float expansion,TemporalCollisionProvenance provenance)
        =>new(TemporalCollisionKind.Clear,provenance,TemporalCollisionIssue.None,null,null,until,steps,expansion);
    private static TemporalCollisionResult Unknown(TemporalCollisionIssue issue,double at,int steps=0,float expansion=0,
        TemporalCollisionProvenance provenance=TemporalCollisionProvenance.CandidateFootprint)
        =>new(TemporalCollisionKind.Unresolved,provenance,issue,double.IsFinite(at)?at:null,null,at,steps,expansion);
    private static double Fraction(double time,double first,double last)=>last==first?0:(time-first)/(last-first);
    private static Point XZ(Vector3 p)=>new(p.X,p.Z);
    private static Point Lerp(Vector3 a,Vector3 b,double t)=>Lerp(XZ(a),XZ(b),t);
    private static Point Lerp(Point a,Point b,double t)=>a+(b-a)*t;
    private static double Dot(Point a,Point b)=>a.X*b.X+a.Z*b.Z;
    private static bool Finite(Vector3 p)=>float.IsFinite(p.X)&&float.IsFinite(p.Y)&&float.IsFinite(p.Z);
    private static bool Finite(Vector2 p)=>float.IsFinite(p.X)&&float.IsFinite(p.Y);
    private static bool Finite(Point p)=>double.IsFinite(p.X)&&double.IsFinite(p.Z);
    private static bool SafeArithmetic(Point p)=>Finite(p)&&Math.Abs(p.X)<=SafeArithmeticMagnitude&&Math.Abs(p.Z)<=SafeArithmeticMagnitude;
    private readonly record struct Point(double X,double Z)
    {
        public double Length=>Math.Sqrt(X*X+Z*Z);
        public static Point operator +(Point a,Point b)=>new(a.X+b.X,a.Z+b.Z);
        public static Point operator -(Point a,Point b)=>new(a.X-b.X,a.Z-b.Z);
        public static Point operator *(Point a,double b)=>new(a.X*b,a.Z*b);
        public static Point operator /(Point a,double b)=>new(a.X/b,a.Z/b);
    }
}
