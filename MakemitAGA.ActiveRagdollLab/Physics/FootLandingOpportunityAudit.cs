using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // V142 observes every inactive pre-lifecycle sample, including unadopted
    // preparation. Criteria below are diagnostic screens, NEVER contact permission.
    // No native read, second clock, force, collider operation or per-step formatting.
    internal sealed class FootLandingOpportunityAudit
    {
        internal enum Phase { Unknown, Stance, Approach, Swing, Lift }
        internal enum SourceKind { Unknown, Idle, Motion }
        [Flags] internal enum Rejection
        {
            None=0, Geometry=1, Footprint=2, Motion=4, Age=8, Gap=16,
            FivePoint=32, Identity=64, Height=128, Descending=256, Projection=512
        }
        internal const float MinPositive=.001f, MaxPositive=.025f, MinDescending=.006f;
        internal struct Sample
        {
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal int Fixed, SupportId;
            internal Phase Gait;
            internal SourceKind SourceMotion;
            internal bool WasActive, NowActive, Strong, Identity, FivePoint, Adopted;
            internal int PrecontactOrigin;
            internal string FootSide;
            internal bool FirstMiss, MissInProgress, OrdinaryEnabled, RecoveryEnabled;
            internal int DepthLatchFixed;
            internal bool SwingMeasured;
            internal EarlyLandingWindow.Foot EarlyFoot;
            internal EarlyLandingWindow.Signal EarlySignal;
            internal int EarlyOrigin;
            internal SwingLandingWindow.Signal SwingSignal;
            internal SwingKneeCandidate.Result SwingCandidate;
            internal bool GeometryValid, MotionValid, TrackingValid, ParentValid;
            internal bool FullFootprint, SelectedBoxFallback;
            internal Vector3 FloorCenter, FloorHalf;
            internal float Min, PredictedMin, CornerVelocity, AnchorGap, SourceAge;
            internal float RawMin, ControlMin, AnkleDeltaY, RelativeErrorDegrees;
            internal float RawHipY, ReferenceHipY, ActualHipY;
            internal bool ParentHeightMeasured, ParentHeightRequested;
            internal int RightWarningOrigin;
            internal bool RightLandingObserved;
            internal RightFootLandingWindow.Readback RightLandingRead;
            internal RightThighWindow.Signal RightThigh;
            internal string SourcePhase;
            internal bool BoxReferencesValid, KneeHeightsValid, AnkleHeightsValid;
            internal Vector3 KneeHeights, AnkleHeights;
            internal bool ClockValid => Step.Valid && Source.Valid && Fixed>0 && Step.Dt<=.1f &&
                Source.Epoch==Step.Epoch && Source.Sequence==Step.SourceSequence;
            internal bool HeightValid => GeometryValid && SupportId!=0 && V(FloorCenter) && V(FloorHalf) &&
                FloorHalf.x>0f && FloorHalf.y>0f && FloorHalf.z>0f && F(Min);
            internal bool VelocityValid => HeightValid && MotionValid && F(CornerVelocity) && F(PredictedMin);
            internal bool TrackValid => HeightValid && TrackingValid && F(RawMin) && F(ControlMin) &&
                F(AnkleDeltaY) && F(RelativeErrorDegrees) && RelativeErrorDegrees>=0f;
            // Translating every control corner by the same actual ankle gap
            // commutes with min(): this isolates orientation at the ACTUAL ankle.
            internal float ControlAtActualAnkle => ControlMin+AnkleDeltaY;
            internal float FootRotationDelta => Min-ControlAtActualAnkle;
            internal bool ParentReady => ParentValid && F(RawHipY) && F(ReferenceHipY) && F(ActualHipY);
            internal bool StrongLanding => Strong && (Gait==Phase.Stance || Gait==Phase.Approach);
            internal bool PositiveDescending => VelocityValid && FullFootprint && Min>=MinPositive && Min<=MaxPositive && CornerVelocity < -MinDescending;
        }
        internal static SourceKind ClassifySource(string phase) => phase=="IDLE" ? SourceKind.Idle :
            phase=="START" || phase=="STOP" || phase=="TURN" || phase=="ACCEL" || phase=="DECEL" ||
            phase=="STEADY" || phase=="LOW_SPEED" ? SourceKind.Motion : SourceKind.Unknown;
        internal static Rejection Screen(in Sample s)
        {
            Rejection r=Rejection.None;
            if(!s.HeightValid)r|=Rejection.Geometry;
            if(!s.FullFootprint)r|=Rejection.Footprint;
            if(!s.VelocityValid)r|=Rejection.Motion;
            if(!F(s.SourceAge)||s.SourceAge<0f||s.SourceAge>FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds)r|=Rejection.Age;
            if(!F(s.AnchorGap)||s.AnchorGap<0f||s.AnchorGap>FootContactGeometryLimits.RecoveryMaxAnchorGapMeters)r|=Rejection.Gap;
            if(!s.FivePoint)r|=Rejection.FivePoint;
            if(!s.Identity)r|=Rejection.Identity;
            if(!F(s.Min)||s.Min<MinPositive||s.Min>MaxPositive)r|=Rejection.Height;
            if(!s.VelocityValid||s.CornerVelocity>=-MinDescending)r|=Rejection.Descending;
            if(!s.VelocityValid||s.PredictedMin < -FootContactGeometryLimits.OrdinaryPenetrationMeters)r|=Rejection.Projection;
            return r;
        }
        private sealed class Totals
        {
            internal int Count, Valid, Motion, Tracking, Parent, Adopted, Footprint, FivePoint, Identity, PositiveDescending, SelectedBoxFallback;
            internal int Screened, StrongScreened, ApproachScreened;
            internal readonly int[] Heights=new int[6], Phases=new int[5], Reject=new int[10];
            private double _min,_raw,_control,_trackActual,_ankle,_rotation,_rotationAbs,_angle,_parentReference,_parentResponse;
            internal void Reset()
            {
                Count=Valid=Motion=Tracking=Parent=Adopted=Footprint=FivePoint=Identity=PositiveDescending=SelectedBoxFallback=0;
                Screened=StrongScreened=ApproachScreened=0;
                Array.Clear(Heights,0,Heights.Length);Array.Clear(Phases,0,Phases.Length);Array.Clear(Reject,0,Reject.Length);
                _min=_raw=_control=_trackActual=_ankle=_rotation=_rotationAbs=_angle=_parentReference=_parentResponse=0;
            }
            internal void Add(in Sample s,Rejection rejection)
            {
                Count++; if(s.Adopted)Adopted++; Phases[(int)s.Gait]++;
                if(s.FullFootprint)Footprint++;if(s.FivePoint)FivePoint++;if(s.Identity)Identity++;
                if(s.HeightValid)
                {
                    Valid++;if(s.SelectedBoxFallback)SelectedBoxFallback++;_min+=s.Min;
                    Heights[s.Min<-.003f?0:s.Min<0f?1:s.Min<.001f?2:s.Min<=.025f?3:s.Min<=.080f?4:5]++;
                }
                if(s.VelocityValid)Motion++;
                if(s.PositiveDescending)PositiveDescending++;
                if(s.TrackValid){Tracking++;_raw+=s.RawMin;_control+=s.ControlMin;_trackActual+=s.Min;_ankle+=s.AnkleDeltaY;
                    _rotation+=s.FootRotationDelta;_rotationAbs+=Math.Abs(s.FootRotationDelta);_angle+=s.RelativeErrorDegrees;}
                if(s.ParentReady){Parent++;_parentReference+=s.ReferenceHipY-s.RawHipY;_parentResponse+=s.ActualHipY-s.ReferenceHipY;}
                for(int i=0;i<10;i++)if((((int)rejection)&(1<<i))!=0)Reject[i]++;
                if(rejection!=Rejection.None)return;
                Screened++;if(s.StrongLanding)StrongScreened++;
                if(s.StrongLanding&&s.Gait==Phase.Approach)ApproachScreened++;
            }
            internal string Text(string name) => "\nLANDING_OPPORTUNITY_TOTALS_V041420/group="+name+
                "/inactive-valid-missing-motion="+Count+"|"+Valid+"|"+(Count-Valid)+"|"+Motion+
                "/validSelectedBoxFallback="+SelectedBoxFallback+"/adopted-unadopted="+Adopted+"|"+(Count-Adopted)+
                "/phase(unknown-stance-approach-swing-lift)="+Ints(Phases)+
                "/heightBinsMm(ltMinus3-minus3To0-0To1-1To25-25To80-gt80)="+Ints(Heights)+
                "/fullFootprint-fivePoint-identity-positiveDescending="+Footprint+"|"+FivePoint+"|"+Identity+"|"+PositiveDescending+
                "/screened-strongLanding-strongApproach="+Screened+"|"+StrongScreened+"|"+ApproachScreened+
                "/rejectOverlap(geometry-footprint-motion-age-gap-fivePoint-identity-height-descending-projection)="+Ints(Reject)+
                "/minMeanMm="+Mean(_min,Valid,1000)+"/trackingValid="+Tracking+
                "/trackingMeanMm(raw-control-actual-ankleShift-footRotation-footRotationAbs)="+Mean(_raw,Tracking,1000)+"|"+Mean(_control,Tracking,1000)+"|"+Mean(_trackActual,Tracking,1000)+"|"+
                Mean(_ankle,Tracking,1000)+"|"+Mean(_rotation,Tracking,1000)+"|"+Mean(_rotationAbs,Tracking,1000)+
                "/relativeAngleMeanDeg="+Mean(_angle,Tracking,1)+"/parentValid="+Parent+
                "/parentMeanMm(rawToReference-referenceToPhysical)="+Mean(_parentReference,Parent,1000)+"|"+Mean(_parentResponse,Parent,1000);
        }
        private sealed class Streak
        {
            internal int Started, Completed, Censored, AtLeast2, AtLeast50ms, AtLeast100ms, MaxSamples, CurrentSamples;
            internal float Seconds, MaxSeconds;
            private bool _at50,_at100;
            internal void Reset(){Started=Completed=Censored=AtLeast2=AtLeast50ms=AtLeast100ms=MaxSamples=CurrentSamples=0;Seconds=MaxSeconds=0;_at50=_at100=false;}
            internal void Add(bool passes,bool continuous,float dt)
            {
                if(!continuous)End(true);
                if(!passes){End(false);return;}
                if(CurrentSamples==0){Started++;Seconds=0f;_at50=_at100=false;}else Seconds+=dt;
                CurrentSamples++;if(CurrentSamples==2)AtLeast2++;
                if(!_at50&&Seconds+1e-6f>=.05f){AtLeast50ms++;_at50=true;}
                if(!_at100&&Seconds+1e-6f>=.1f){AtLeast100ms++;_at100=true;}
                MaxSamples=Math.Max(MaxSamples,CurrentSamples);MaxSeconds=Math.Max(MaxSeconds,Seconds);
            }
            internal void End(bool censored){if(CurrentSamples>0){if(censored)Censored++;else Completed++;}CurrentSamples=0;Seconds=0f;}
            internal string Text(string name)=>"\nLANDING_OPPORTUNITY_STREAK_V041420/kind="+name+
                "/started-completed-censored-open="+Started+"|"+Completed+"|"+Censored+"|"+(CurrentSamples>0?1:0)+
                "/atLeast2-atLeast50ms-atLeast100ms="+AtLeast2+"|"+AtLeast50ms+"|"+AtLeast100ms+
                "/maxSamples-maxSeconds="+MaxSamples+"|"+N(MaxSeconds)+"/firstSampleAddsNoTime=True/notStanceSuccessRate=True";
        }
        private readonly LandingWindowOutcomes _links = new LandingWindowOutcomes();
        private readonly Totals _all=new Totals(),_motion=new Totals(),_idle=new Totals(),_unknown=new Totals();
        private readonly Streak _screened=new Streak(),_strong=new Streak();
        private Sample _previous,_positive,_crossBefore,_cross,_worst;
        private PhysicsStepContext _lastClock;
        private bool _hasPrevious,_hasClock,_hasPositive,_hasCross,_hasWorst,_sealed;
        private int _seen,_active,_enables,_invalidClock,_duplicate,_breaks,_faults;
        private int _firstStrong,_strongAbove,_strongShallow,_strongDeep,_strongGeometryMissing,_leftCensoredStrong;
        private int _crossings,_crossNoStrong,_crossNoIdentity;
        internal void Reset()
        {
            _links.Reset();
            _all.Reset();_motion.Reset();_idle.Reset();_unknown.Reset();_screened.Reset();_strong.Reset();
            _previous=_positive=_crossBefore=_cross=_worst=default; _lastClock=default;
            _hasPrevious=_hasClock=_hasPositive=_hasCross=_hasWorst=_sealed=false;
            _seen=_active=_enables=_invalidClock=_duplicate=_breaks=_faults=0;
            _firstStrong=_strongAbove=_strongShallow=_strongDeep=_strongGeometryMissing=_leftCensoredStrong=0;
            _crossings=_crossNoStrong=_crossNoIdentity=0;
        }
        internal void Fault()
        {
            if (_sealed) return;
            _faults++;
            Break();
        }
        private void Break(LandingEventHistory.MissEnd reason = LandingEventHistory.MissEnd.Discontinuity){_links.Cut(reason);_hasPrevious=false;_screened.End(true);_strong.End(true);}
        internal void Seal(){if(_sealed)return;Break(LandingEventHistory.MissEnd.TraceEnded);_sealed=true;}
        internal void ObserveActive(PhysicsStepContext step, SourceStepContext source, int localFixed, Phase gait,
            bool strong, bool nowActive) => Observe(new Sample
        {
            Step = step, Source = source, Fixed = localFixed, Gait = gait, Strong = strong,
            WasActive = true, NowActive = nowActive
        });

        internal void Observe(in Sample s, bool history = false, in RightKneePivot.Frame pivot = default)
        {
            if(_sealed)return;
            if(!s.ClockValid || (int)s.Gait<0 || (int)s.Gait>4 || (int)s.SourceMotion<0 || (int)s.SourceMotion>2)
            {_invalidClock++;Break();return;}
            if(_hasClock&&(s.Step.Epoch<_lastClock.Epoch||(s.Step.Epoch==_lastClock.Epoch&&s.Step.Sequence<=_lastClock.Sequence)))
            {_duplicate++;Break();return;}
            bool consecutive=_hasPrevious&&s.Step.Epoch==_previous.Step.Epoch&&s.Step.Sequence==_previous.Step.Sequence+1&&s.Fixed==_previous.Fixed+1&&
                s.Step.FixedTime>=_previous.Step.FixedTime&&s.Step.UnscaledTime>=_previous.Step.UnscaledTime&&
                s.Source.Sequence>=_previous.Source.Sequence&&s.Source.Time>=_previous.Source.Time&&
                (s.Source.Sequence!=_previous.Source.Sequence||s.Source.Matches(_previous.Source));
            bool sameSurface=consecutive&&s.SupportId!=0&&s.SupportId==_previous.SupportId&&
                (s.FloorCenter-_previous.FloorCenter).sqrMagnitude<=1e-12f&&(s.FloorHalf-_previous.FloorHalf).sqrMagnitude<=1e-12f;
            if(_hasPrevious&&!consecutive)_breaks++;
            _lastClock=s.Step;_hasClock=true;
            _seen++;if(s.NowActive&&!s.WasActive)_enables++;
            bool newSwing = consecutive && LandingWindowOutcomes.StartsNewSwing(s.Gait, _previous.Gait);
            if(s.WasActive){_links.Observe(s, false, sameSurface, newSwing);_active++;_screened.End(true);_strong.End(true);_previous=s;_hasPrevious=true;return;}
            Rejection r=Screen(s);_all.Add(s,r);
            (s.SourceMotion==SourceKind.Motion?_motion:s.SourceMotion==SourceKind.Idle?_idle:_unknown).Add(s,r);
            bool continuous=sameSurface&&!_previous.WasActive;
            // A continuously passing geometric screen can straddle a new gait cycle.
            // The old link closes on that edge; the current step must start its own window.
            if (newSwing) { _screened.End(false); _strong.End(false); }
            _screened.Add(r==Rejection.None,continuous,s.Step.Dt);
            _strong.Add(r==Rejection.None&&s.StrongLanding,continuous,s.Step.Dt);
            _links.Observe(s, r==Rejection.None, sameSurface, newSwing, history, r, pivot);
            if(s.StrongLanding)
            {
                if(!consecutive)_leftCensoredStrong++;
                else if(!_previous.StrongLanding)
                {
                    _firstStrong++;
                    if(!s.HeightValid)_strongGeometryMissing++;
                    else if(s.Min>=0f)_strongAbove++;
                    else if(s.Min>=-FootContactGeometryLimits.OrdinaryPenetrationMeters)_strongShallow++;
                    else _strongDeep++;
                }
            }
            if(continuous&&s.HeightValid&&_previous.HeightValid&&_previous.Min>0f&&s.Min<=0f)
            {
                _crossings++;if(!s.StrongLanding)_crossNoStrong++;if(!s.Identity)_crossNoIdentity++;
                if(!_hasCross||(_cross.SourceMotion!=SourceKind.Motion&&s.SourceMotion==SourceKind.Motion))
                {_hasCross=true;_crossBefore=_previous;_cross=s;}
            }
            if(s.PositiveDescending&&(!_hasPositive||(_positive.SourceMotion!=SourceKind.Motion&&s.SourceMotion==SourceKind.Motion)))
            {_hasPositive=true;_positive=s;}
            if(s.TrackValid&&(!_hasWorst||Math.Abs(s.FootRotationDelta)>Math.Abs(_worst.FootRotationDelta)))
            {_hasWorst=true;_worst=s;}
            _previous=s;_hasPrevious=true;
        }
        internal string Summary()=>"LANDING_OPPORTUNITY_V041420/observed-active-inactive="+_seen+"|"+_active+"|"+_all.Count+
            "/enabledAfterObservation="+_enables+"/invalidClock-duplicateOrBackward-breaks-faults="+_invalidClock+"|"+_duplicate+"|"+_breaks+"|"+_faults+
            "/firstStrong(known-above-shallow-deep-geometryMissing-leftCensored)="+_firstStrong+"|"+_strongAbove+"|"+_strongShallow+"|"+_strongDeep+"|"+_strongGeometryMissing+"|"+_leftCensoredStrong+
            "/crossings-total-withoutStrong-withoutIdentity="+_crossings+"|"+_crossNoStrong+"|"+_crossNoIdentity+
            "/scope=ALL_INACTIVE_PRE_LIFECYCLE_NOT_ONLY_ADOPTED/groups=ALL_PLUS_DISJOINT_SOURCE_KINDS/phaseIndependentScreenIsNotAuthorization=True/diagnosticFeedback=False"+
            "/screen=FULL_BOX_CURRENT_FIVE_POINT_ID_AGE_LE_100MS_GAP_LE_5MM_MIN_1_TO_25MM_CORNER_V_LT_MINUS_0.006_PROJECTION_GE_MINUS_2MM"+
            "/projection=CONSTANT_POINT_V_NOT_SOLVER_OR_SWEEP/parent=HELD_ANIMATION_PROXY_TO_RECEIVER_REFERENCE_TO_PRE_FIXED_PHYSICAL"+
            _all.Text("ALL")+_motion.Text("SOURCE_NON_IDLE")+_idle.Text("SOURCE_IDLE")+_unknown.Text("SOURCE_UNKNOWN")+
            _screened.Text("PHASE_INDEPENDENT")+_strong.Text("STRONG_LANDING")+
            (_hasPositive?Text("FIRST_MOTION_POSITIVE_ELSE_FIRST_ANY",_positive):"\nLANDING_OPPORTUNITY_WITNESS_V041420/positive=NONE")+
            (_hasCross?Text("CROSS_PREVIOUS",_crossBefore)+Text("CROSS_CURRENT_FIRST_MOTION_ELSE_FIRST_ANY",_cross):"\nLANDING_OPPORTUNITY_WITNESS_V041420/crossing=NONE")+
            (_hasWorst?Text("MAX_ABS_FOOT_ROTATION",_worst):"\nLANDING_OPPORTUNITY_WITNESS_V041420/tracking=NONE") + _links.Summary();
        private static string Text(string name,in Sample s)=>"\nLANDING_OPPORTUNITY_WITNESS_V041420/selection="+name+
            "/epoch-fixed-local-source="+s.Step.Epoch+"|"+s.Step.Sequence+"|"+s.Fixed+"|"+s.Source.Sequence+
            "/support="+s.SupportId+"/selectedBoxFallback="+s.SelectedBoxFallback+"/phase-source="+s.Gait+"|"+s.SourceMotion+
            "/strong-identity-fivePoint-adopted-nowActive="+s.Strong+"|"+s.Identity+"|"+s.FivePoint+"|"+s.Adopted+"|"+s.NowActive+
            "/minMm="+(s.HeightValid?N(s.Min*1000):"NA")+"/cornerV-predictedMm="+(s.VelocityValid?N(s.CornerVelocity)+"|"+N(s.PredictedMin*1000):"NA|NA")+
            "/age-gapMm="+N(s.SourceAge)+"|"+N(s.AnchorGap*1000)+"/reject="+Screen(s)+
            "/trackingMm(raw-control-ankleShift-atActualAnkle-actual-rotation)="+(s.TrackValid?N(s.RawMin*1000)+"|"+N(s.ControlMin*1000)+"|"+N(s.AnkleDeltaY*1000)+"|"+N(s.ControlAtActualAnkle*1000)+"|"+N(s.Min*1000)+"|"+N(s.FootRotationDelta*1000):"NA")+
            "/relativeAngleDeg="+(s.TrackValid?N(s.RelativeErrorDegrees):"NA")+
            "/hipHeightMm(raw-reference-physical)="+(s.ParentReady?N(s.RawHipY*1000)+"|"+N(s.ReferenceHipY*1000)+"|"+N(s.ActualHipY*1000):"NA");
        private static string Ints(int[] a)=>string.Join("|",a);
        private static string Mean(double x,int n,double scale)=>n>0?N(x*scale/n):"NA";
        private static string N(double x)=>x.ToString("G9",CultureInfo.InvariantCulture);
        private static bool F(float x)=>PelvisParentFrame.F(x);
        private static bool V(Vector3 x)=>PelvisParentFrame.V(x);
    }

    // Reuses the eight already-read real Box corners. Exact ties use the lowest
    // point velocity; no COM velocity substituted for the rotating lowest point.
    internal struct FootLowestCornerMotion
    {
        private int _count;
        private bool _invalid;
        internal float Minimum, Velocity;
        internal bool Valid=>_count==8&&!_invalid;
        internal void Add(float height,float normalVelocity)
        {
            if(!PelvisParentFrame.F(height)||!PelvisParentFrame.F(normalVelocity))_invalid=true;
            if(_count==0||height<Minimum){Minimum=height;Velocity=normalVelocity;}
            else if(height==Minimum)Velocity=Mathf.Min(Velocity,normalVelocity);
            _count++;
        }
    }
}
