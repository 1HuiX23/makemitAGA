using MakemitAGA.ActiveRagdollLab.Runtime;
using System;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed class PelvisReceiverReport
    {
        internal string Arm = "OFF_V04470_PELVIS_RECEIVER", TraceArm, Issue;
        internal bool Selected, Dynamic, Active, Ready, Quiesced, Finite = true, BaselineCaptured, BaselineRestored = true;
        internal bool TraceRunning, TraceDynamic;
        internal Vector3 LastPdA, LastFeedforwardA;
        internal int PassiveIssued, PassiveResolved, Samples, IdleSamples, MotionSamples, ForcePairWrites, TorqueWrites;
        internal int Resolved, Censored, LinearClamps, AngularClamps, TorqueClamps, FaultEvents, RestoreFailures, CurrentTraceFixed;
        internal float PassiveAdvancePeak, PositionErrorPeak, RotationErrorPeak, ForceSumPeak, TorquePeak, SystemResidualPeak;
        internal float PelvisMass, ChestMass;
        internal float ReferenceSpeedCap, ReferenceAlphaCap, ReferenceInertiaUpper, ReferenceTorqueBudget, LastReferenceAlpha;
        internal string ReferenceBudgetMode, TraceReferenceBudgetMode;
        internal string ReferencePlannerMode, TraceReferencePlannerMode, ReferencePairIssue;
        internal PelvisReferencePairReport.Sample ReferencePair;
        internal readonly PelvisReferencePairReport ReferenceComparison = new PelvisReferencePairReport();
        internal int DirectionalBudgetSamples;
        internal float DirectionalBrakeCapPeak, DirectionalStepCapPeak, DirectionalInertiaTorquePeak, DirectionalRigidGyroPeak;
        internal bool LastReferenceSpeedLimited, LastReferenceAlphaLimited;
        internal float ComReferenceSpeedCap, ComReferenceAccelerationCap, LastComReferenceLag;
        internal bool LastComReferenceSpeedLimited, LastComReferenceAccelerationLimited;
        internal Vector3 LastComReferenceA, LastNativeGoalCom, LastNativeGoalV;
        internal Vector3 PelvisComLocal, ChestComLocal, PelvisInertia, ChestInertia, CommonA, PelvisTorque;
        internal Vector3 NativeChestAnchor, NativePelvisAnchor;
        // V04480 is read-only. These bands describe recovery; they never enable/disable a controller.
        internal const string TrackingRevision = "V04600_RUNTIME_SESSION_CONTEXT";
        internal const float TrackingPositionBand = 0.03f, TrackingRotationBand = 5f, TrackingNativeBand = 0.002f;
        // Timing groups describe raw sampled-animation comparisons only; they never gate physics.
        internal const float RawTimingAgeLimit = 0.1f;
        internal float LastRawCommonA, LastRawPelvisAlpha, LastRawPelvisTorque;
        internal bool LastLinearClamped, LastAngularClamped, LastTorqueClamped;
        internal int TrackingSamples, TrackingInvalid, TrackingNext, TrackingCount, RecoveryNext, RecoveryCount;
        internal int RecoveryStarted, RecoveryWithinBand, RecoveryNewSource, RecoveryStopped, RecoveryLimit, RecoveryGap, SourceOpenAtStop;
        internal double TrackingSeconds;
        internal float LongestPositionOverSeconds, LongestRotationOverSeconds;
        internal string TrackingIssue;
        internal struct AngularReferenceSample
        {
            internal bool Valid, ArrivalBraking;
            internal bool DirectionalBudget;
            internal float ScalarAlphaCap, BrakeAlphaCap, StepAlphaCap, BrakeInertiaNorm, StepInertiaNorm;
            internal float BaseStepAlphaCap, ExtraBrakeAlpha;
            internal Vector3 InertialBudgetTorque, RigidGyroEstimate;
            internal int Sequence;
            internal float Dt, ErrorAngle;
            internal Vector3 BridgeW, CorrectionW, DesiredW, LimitedW, PreviousW, CommandW, GeneratedAlpha;
            internal Vector3 RelativeW, StoppingOffset, ArrivalError;
        }
        internal struct AngularActuationSample
        {
            internal bool Valid, ReferenceFeedforward;
            internal int Fixed;
            internal Vector3 PAlpha, DAlpha, FeedforwardAlpha, AppliedAlpha, InertiaAlphaTorque, GyroTorque, Torque;
            internal float ReducedMass;
            internal Vector3 ParentLever, OrbitalAlphaTorque, OrbitalGyroTorque, RawTorque;
        }
        internal AngularReferenceSample AngularReference;
        internal AngularActuationSample AngularActuation;
        internal int AngularEvidenceSamples, AngularEvidenceInvalid, AngularResponseSamples, JointEvidenceSamples, QuietSourceSamples, ReferenceAwaySamples, BodyAwaySamples;
        internal string AngularEvidenceIssue;
        internal int ParentReferenceSamples, ParentReferenceInvalid;
        internal float ParentHipControlGapPeak, ParentHipBridgeShiftPeak, ParentSourceLocalDriftPeak;
        internal string ParentReferenceIssue;
        internal bool ParentReferenceReady => TrackingComplete && ParentReferenceSamples > 0 && ParentReferenceSamples == TrackingSamples && ParentReferenceInvalid == 0;
        internal TrackingFrame PeakReferenceAwayTracking;
        internal bool AngularEvidenceReady => TrackingComplete && AngularEvidenceSamples == TrackingSamples && AngularEvidenceInvalid == 0;
        internal struct TrackingFrame
        {
            internal bool Enabled, Valid, Dynamic, CommandAlphaAvailable, SourceActive, SourceFresh, Projected;
            internal bool LinearClamped, AngularClamped, TorqueClamped, ChildAngularClamped;
            internal bool ReferenceSpeedLimited, ReferenceAlphaLimited;
            internal bool ComReferenceSpeedLimited, ComReferenceAccelerationLimited, CommandComAAvailable;
            internal int Fixed, Stage, SourceSequence, SourceEventId, RecoveryEventId;
            internal string SourcePhase, Issue;
            internal float Time, Dt, Position, Rotation, NativeGap, TargetNativeGap, VirtualGap, VelocityError;
            internal Vector3 CommandW;
            internal Vector3 CommandCom, CommandComV, CommandComA, ComReferenceA, NativeGoalCom, NativeGoalV;
            internal Vector3 ComError, ComVelocityError, RawRenderComError, BodyChestPosition, PdA, FeedforwardA;
            internal Vector3 SourceV, BodyW, SourceWVector, RotationErrorVector, RawRotationErrorVector, ReferenceRotationErrorVector;
            internal Quaternion BodyPelvisRotation;
            internal AngularReferenceSample AngularReference;
            internal PelvisReferencePairReport.Sample ReferencePair;
            internal AngularActuationSample AngularActuation;
            internal bool ParentReferenceExpected, ParentReferenceValid;
            internal bool RuntimeContextExpected;
            internal PhysicsStepContext RuntimeContext;
            internal PelvisParentFrame ParentFrame;
            internal Vector3 ParentBridgeLeftShift, ParentBridgeRightShift;
            internal string ParentReferenceIssue;
            internal float ParentReferenceTime;
            internal bool AngularBodiesCaptured, AngularComparisonCaptured, AngularContextCaptured, AngularEvidenceValid, AngularResponseAvailable, JointReactionAvailable;
            internal string AngularEvidenceIssue;
            internal Vector3 BridgeSourceW, BridgePoseCorrectionW, BridgeReportedW;
            internal float BridgePredictionAge;
            internal bool BridgeCorrectionClamped;
            internal Vector3 RenderReferenceErrorVector, ChestW, ChestRotationErrorVector, ChildPAlpha, ChildDAlpha, ChildAppliedAlpha;
            internal Vector3 ChildTargetRelativeW, ChildMeasuredRelativeW, JointForce, JointTorque, JointParentLever;
            internal Vector3 PelvisResponseAlpha, ChestResponseAlpha, PreviousPelvisTorque, PreviousChildAlpha;
            internal Vector3 PreviousPelvisCommandAlpha, PreviousPelvisFeedforwardAlpha;
            internal int ResponseFromFixed;
            internal float ResponseDt, BridgeReferenceRate, RenderReferenceRate, BodyControlRate;
            internal float RawRenderPosition, RawRenderRotation, RawBridgePosition, RawChestPosition, RawTargetNativeGap, RawVelocityError, ReferenceShift;
            internal float ReferenceAngularLag, ReferenceAlpha;
            internal float ComReferenceLag;
            internal int RawTimingGroup;
            internal float CommandAlpha, SourceW, SourceAlpha, SourceDt, SourceAge, RawA, RawAlpha, RawTorque, AppliedA, AppliedTorque;
        }
        internal struct TrackingPhase
        {
            internal int Samples, LinearClamps, AngularClamps, TorqueClamps, ChildAngularClamps;
            internal int ReferenceSpeedLimits, ReferenceAlphaLimits;
            internal int ComReferenceSpeedLimits, ComReferenceAccelerationLimits;
            internal double Seconds, PositionIntegral, RotationIntegral, NativeIntegral, TargetNativeIntegral, VirtualIntegral;
            internal double PositionOverSeconds, RotationOverSeconds;
            internal float PositionPeak, RotationPeak, NativePeak, TargetNativePeak, VirtualPeak;
            internal double RawRenderIntegral, RawRenderRotationIntegral, RawBridgeIntegral, RawChestIntegral, RawTargetNativeIntegral, ReferenceShiftIntegral;
            internal float RawRenderPeak, RawRenderRotationPeak, RawBridgePeak, RawChestPeak, RawTargetNativePeak, ReferenceShiftPeak;
            internal double ReferenceLagIntegral;
            internal float ReferenceLagPeak;
            internal double ComReferenceLagIntegral;
            internal float ComReferenceLagPeak, CommandComVPeak, CommandComAPeak;
        }
        internal struct RawTimingStats
        {
            internal int Samples;
            internal double Seconds, PositionIntegral, RotationIntegral, AgeIntegral;
            internal float PositionPeak, RotationPeak, AgePeak;
        }
        internal readonly RawTimingStats[] RawTiming = new RawTimingStats[4];
        internal struct RecoveryFrame
        {
            internal int EventId, StartFixed, EndFixed;
            internal float StartTime, Duration, PositionPeak, RotationPeak, NativePeak, EndPosition, EndRotation, EndNative;
            internal string Result;
        }
        internal readonly TrackingFrame[] TrackingRing = new TrackingFrame[12];
        internal readonly TrackingPhase[] TrackingPhases = new TrackingPhase[4];
        internal readonly RecoveryFrame[] RecoveryRing = new RecoveryFrame[8];
        internal TrackingFrame LastTracking, PeakPositionTracking, PeakRotationTracking, PeakNativeTracking, PeakRawPositionTracking, PeakRawRotationTracking;
        private bool _sourceWasActive, _recoveryActive, _recoveryHolding;
        private int _sourceEventId;
        private float _positionOverRun, _rotationOverRun, _recoveryHold;
        private RecoveryFrame _recovery;
        internal bool TrackingComplete => Samples > 0 && TrackingSamples == Samples && TrackingInvalid == 0;
        // Bounded read-only stop windows reuse existing tracking frames: no native access,
        // per-step logging or additional observer. Thresholds classify evidence, never gate drive.
        internal struct StopFrame
        {
            internal int StartFixed, EndFixed;
            internal float StartTime, Duration, ForwardPeak, LagPeak, RawForwardPeak, RawLagPeak, EndPosition, EndSpeed;
            internal Vector3 Direction;
            internal TrackingFrame Peak, PeakRotation;
            internal string Result;
        }
        internal readonly StopFrame[] StopRing = new StopFrame[8];
        internal int StopNext, StopCount, StopStarted;
        private bool _stopArmed, _stopActive, _stopHolding;
        private float _stopHold;
        private Vector3 _stopDirection;
        private StopFrame _stop;
        private void ObserveStop(TrackingFrame f)
        {
            float speed = f.SourceV.magnitude;
            if (!FiniteValue(speed) || f.RawTimingGroup >= 2 || (LastTracking.Valid && LastTracking.Fixed + 1 != f.Fixed))
            { FinishStop(LastTracking, "CENSORED_SOURCE_CLOCK_OR_GAP"); _stopArmed = false; return; }
            if (speed >= 0.2f)
            {
                FinishStop(LastTracking, "CENSORED_SOURCE_RESUMED");
                _stopArmed = true; _stopDirection = f.SourceV / speed;
            }
            else if (_stopArmed && speed <= 0.05f && f.SourceFresh)
            {
                _stopArmed = false; _stopActive = true; _stopHolding = false; _stopHold = 0f; StopStarted++;
                _stop = new StopFrame { StartFixed = f.Fixed, StartTime = f.Time, Direction = _stopDirection };
            }
            if (!_stopActive) return;
            // Error is target-body: negative projection means body ahead along pre-stop motion.
            float signedControl = Vector3.Dot(f.ComError, _stop.Direction);
            float signedRaw = Vector3.Dot(f.RawRenderComError, _stop.Direction);
            _stop.ForwardPeak = Mathf.Max(_stop.ForwardPeak, -signedControl); _stop.LagPeak = Mathf.Max(_stop.LagPeak, signedControl);
            _stop.RawForwardPeak = Mathf.Max(_stop.RawForwardPeak, -signedRaw); _stop.RawLagPeak = Mathf.Max(_stop.RawLagPeak, signedRaw);
            if (!_stop.Peak.Valid || f.Position > _stop.Peak.Position) _stop.Peak = f;
            if (!_stop.PeakRotation.Valid || f.Rotation > _stop.PeakRotation.Rotation) _stop.PeakRotation = f;
            float bodySpeed = (f.CommandComV - f.ComVelocityError).magnitude;
            bool inBand = speed <= 0.05f && f.CommandComV.magnitude <= 0.05f && bodySpeed <= 0.05f &&
                f.Position <= TrackingPositionBand && f.NativeGap <= TrackingNativeBand;
            _stopHold = inBand && _stopHolding ? _stopHold + f.Dt : 0f; _stopHolding = inBand;
            if (_stopHold >= 0.15f) FinishStop(f, "WITHIN_STOP_BAND");
            else if (f.Time - _stop.StartTime >= 5f) FinishStop(f, "OBSERVATION_LIMIT_5S");
        }
        private void FinishStop(TrackingFrame end, string reason)
        {
            if (!_stopActive) return;
            _stopActive = _stopHolding = false; _stopHold = 0f;
            _stop.EndFixed = end.Fixed; _stop.Duration = Mathf.Max(0f, end.Time - _stop.StartTime);
            _stop.EndPosition = end.Position; _stop.EndSpeed = (end.CommandComV - end.ComVelocityError).magnitude; _stop.Result = reason;
            StopRing[StopNext] = _stop; StopNext = (StopNext + 1) % StopRing.Length; StopCount = Mathf.Min(StopCount + 1, StopRing.Length);
        }
        private void AppendStops(StringBuilder sb)
        {
            sb.Append("\nreceiverStops=SOURCE_PELVIS_SPEED_HYSTERESIS_0.2_TO_0.05/started=").Append(StopStarted)
                .Append("/active=").Append(_stopActive).Append("/retained-capacity=").Append(StopCount).Append("-8")
                .Append("/band=COM30mm_NATIVE2mm_SOURCE_REF_BODY_SPEED0.05_HOLD0.15s/limit=5s/diagnosticOnly=True");
            for (int k = 0; k < StopCount; k++)
            {
                StopFrame f = StopRing[(StopNext - StopCount + k + StopRing.Length) % StopRing.Length];
                sb.Append("\nreceiverStopRow=").Append(f.StartFixed).Append('>').Append(f.EndFixed)
                    .Append("/start-duration=").Append(F(f.StartTime)).Append('-').Append(F(f.Duration)).Append("/result=").Append(f.Result)
                    .Append("/preStopDirection=").Append(V(f.Direction))
                    .Append("/controlForward-lagMm=").Append(F(f.ForwardPeak * 1000f)).Append('-').Append(F(f.LagPeak * 1000f))
                    .Append("/rawRenderForward-lagMm=").Append(F(f.RawForwardPeak * 1000f)).Append('-').Append(F(f.RawLagPeak * 1000f))
                    .Append("/endCOMmm-bodySpeed=").Append(F(f.EndPosition * 1000f)).Append('-').Append(F(f.EndSpeed));
                AppendTrackingFrame(sb, "STOP_PEAK_COM", f.Peak);
                AppendTrackingFrame(sb, "STOP_PEAK_ROTATION", f.PeakRotation);
            }
            sb.Append("\nStop windows begin on a fresh low source pelvis-speed sample after >=0.2m/s motion. Direction is the last >=0.2m/s sample, not a body-facing axis. Forward/lag peaks are separate projections and may occur at different times. Held samples <=100ms remain included; stale/unknown clocks, new movement, trace stop and 5s limit are explicitly censored. Source rotation can continue inside a stop window; inspect its peak frame. A stop-band result is neither a safety verdict nor proof of return to raw animation. Eight latest completed windows are retained; started counts all windows.");
        }

        internal struct Frame
        {
            internal int Fixed, Settled;
            internal bool Resolved, Censored;
            internal float Dt, PelvisMass, ChestMass;
            internal Vector3 PrePelvisV, PreChestV, PostPelvisV, PostChestV, PelvisJ, ChestJ, TorqueJ, SystemDeltaP, SystemResidual;
        }
        internal readonly Frame[] Ring = new Frame[12];
        internal int Next, Count;
        internal bool ExposureReady => Selected && Dynamic && Finite && Ready && Samples >= 50 && IdleSamples >= 50 && MotionSamples >= 30 &&
            ForcePairWrites == Samples && TorqueWrites == Samples && Resolved >= 50 && PassiveResolved == 2;
        internal string GetSummary() => "mode=PELVIS_RUNTIME_SESSION_PREFLIGHT_V04600/arm=" + Arm + "/traceArm=" + (TraceArm ?? "<none>") +
            "/selected-dynamic-active-ready-quiesced-finite=" + Selected + "-" + Dynamic + "-" + Active + "-" + Ready + "-" + Quiesced + "-" + Finite +
            "/issue=" + (Issue ?? "<none>") + "/baselineCaptured-restored=" + BaselineCaptured + "-" + BaselineRestored +
            "/passiveIssued-resolved=" + PassiveIssued + "-" + PassiveResolved + "/passiveAdvancePeakMm=" + F(PassiveAdvancePeak * 1000f) +
            "/samples-idle-motion-forcePairs-torque=" + Samples + "-" + IdleSamples + "-" + MotionSamples + "-" + ForcePairWrites + "-" + TorqueWrites +
            "/resolved-censored=" + Resolved + "-" + Censored + "/clamps(linear-angular-torque)=" + LinearClamps + "-" + AngularClamps + "-" + TorqueClamps +
            "/tracking(valid-invalid-ready)=" + TrackingSamples + "-" + TrackingInvalid + "-" + TrackingComplete +
            "/reference=" + (Dynamic ? "V04520_COMMON_COM_PHASE_ALIGNED_NATIVE_REFERENCE" : "LEGACY_RAW_REFERENCE") +
            "/referenceBounds(Wrad-AlphaRad-InertiaUpperKgM2-budgetNm)=" + (Dynamic ?
                F(ReferenceSpeedCap) + "-" + F(ReferenceAlphaCap) + "-" + F(ReferenceInertiaUpper) + "-" + F(ReferenceTorqueBudget) : "NA_INACTIVE_OR_LEGACY") +
            "/referenceBudgetMode=" + (ReferenceBudgetMode ?? "<none>") + "/traceReferenceBudgetMode=" + (TraceReferenceBudgetMode ?? "<none>") +
            "/referencePlannerMode=" + (ReferencePlannerMode ?? "<none>") + "/traceReferencePlannerMode=" + (TraceReferencePlannerMode ?? "<none>") +
            "/referenceBudgetMeaning=SCALAR_FIELDS_ARE_FALLBACK_ONLY_INERTIAL_BUDGET_GYRO_USES_EXISTING_MARGIN" +
            "/directionalBudget(samples-peakBrakeCap-peakStepCap-peakInertiaTau-peakRigidGyro)=" + DirectionalBudgetSamples + "-" +
                F(DirectionalBrakeCapPeak) + "-" + F(DirectionalStepCapPeak) + "-" + F(DirectionalInertiaTorquePeak) + "-" + F(DirectionalRigidGyroPeak) +
            "/comReferenceBounds(Vmps-Amps2)=" + (Dynamic ? F(ComReferenceSpeedCap) + "-" + F(ComReferenceAccelerationCap) : "NA_INACTIVE_OR_LEGACY") +
            "/faultEvents-restoreFailures=" + FaultEvents + "-" + RestoreFailures + "/exposureReady=" + ExposureReady +
            "/mass(pelvis-chest)=" + F(PelvisMass) + "-" + F(ChestMass) + "/localCOM(pelvis-chest)=" + V(PelvisComLocal) + "|" + V(ChestComLocal) +
            "/inertia(pelvis-chest)=" + V(PelvisInertia) + "|" + V(ChestInertia) +
            "/nativeAnchors(chest-pelvis)=" + V(NativeChestAnchor) + "|" + V(NativePelvisAnchor) +
            "/peaks(positionMm-rotationDeg-forceSumN-torqueNm-systemResidualNs)=" + F(PositionErrorPeak * 1000f) + "-" + F(RotationErrorPeak) + "-" + F(ForceSumPeak) + "-" + F(TorquePeak) + "-" + F(SystemResidualPeak) +
            "/angularReference=" + (Dynamic ? "RELATIVE_W_STOP_OFFSET" : "OFF") +
            "/traceAngularReference=" + (TraceDynamic ? "RELATIVE_W_STOP_OFFSET" : "NO_TRACE") +
            "/pelvisInertia=" + (Dynamic ? "SELF_PLUS_PAIR_ORBITAL" : "OFF") +
            "/tracePelvisInertia=" + (TraceDynamic ? "SELF_PLUS_PAIR_ORBITAL" : "NO_TRACE") +
            "/angularTargetAFF=" + (Dynamic ? "GOVERNED_REFERENCE_ALPHA_GAIN1" : "OFF") +
            "/traceAngularTargetAFF=" + (TraceDynamic ? "GOVERNED_REFERENCE_ALPHA_GAIN1" : "NO_TRACE") +
            "/positionOwner=" + (Dynamic ? "TWO_BODY_COMMON_COM_PD_20_9_PLUS_ARM_AFF_MAXA12" : "LEGACY_KINEMATIC_POSE_BRIDGE") +
            "/rotationOwner=" + (Dynamic ? "PELVIS_INERTIA_GYRO_PD_55_12_MAXA40_MAXTAU2" : "LEGACY_KINEMATIC_POSE_BRIDGE") +
            "/torsoGravity=OFF/HipParentReaction=NOT_ROUTED/commonTargetAFF=" + (Dynamic ? "GOVERNED_COM_A_GAIN1" : "OFF") + "/traceCommonTargetAFF=" + (TraceDynamic ? "GOVERNED_COM_A_GAIN1" : "OFF") + "/ChestRelativeAngular=EXISTING_CHILD_ONLY/wholeBodySupport=NOT_QUALIFIED";
        internal string GetDetails()
        {
            var sb = new StringBuilder(GetSummary());
            for (int k = 0; k < Count; k++)
            {
                Frame f = Ring[(Next - Count + k + Ring.Length) % Ring.Length];
                sb.Append("\nreceiverRow=").Append(f.Fixed).Append('>').Append(f.Settled).Append("/dt=").Append(F(f.Dt))
                    .Append("/knownCommonJ(pelvis-chest)=").Append(V(f.PelvisJ)).Append('|').Append(V(f.ChestJ)).Append("/pelvisTorqueJ=").Append(V(f.TorqueJ))
                    .Append("/preV(pelvis-chest)=").Append(V(f.PrePelvisV)).Append('|').Append(V(f.PreChestV))
                    .Append("/postV(pelvis-chest)=").Append(f.Resolved ? V(f.PostPelvisV) + "|" + V(f.PostChestV) : "NA")
                    .Append("/systemDeltaP-residual=").Append(f.Resolved ? V(f.SystemDeltaP) + "|" + V(f.SystemResidual) : "NA")
                    .Append("/end=").Append(f.Resolved ? "RESOLVED" : f.Censored ? "CENSORED_UNRESOLVED" : "PENDING");
            }
            sb.Append(GetTrackingDetails());
            sb.Append("\nangularEvidence=V04600_RUNTIME_SESSION_CONTEXT/valid-invalid-ready=").Append(AngularEvidenceSamples).Append('-').Append(AngularEvidenceInvalid).Append('-').Append(AngularEvidenceReady)
                .Append("/responseIntervals-jointAvailable=").Append(AngularResponseSamples).Append('-').Append(JointEvidenceSamples)
                .Append("/quiet-referenceAway-bodyAway=").Append(QuietSourceSamples).Append('-').Append(ReferenceAwaySamples).Append('-').Append(BodyAwaySamples)
                .Append("/issue=").Append(AngularEvidenceIssue ?? "<none>");
            AppendTrackingFrame(sb, "peakReferenceAway", PeakReferenceAwayTracking);
            sb.Append('\n').Append(ReferenceComparison.GetDetails(TraceReferencePlannerMode == "DIRECTIONAL_BASELINE_V04670" || TraceReferencePlannerMode == "BRAKING_RESERVE_V04670", TrackingSamples));
            if (ReferencePairIssue != null) sb.Append("\nreferencePairCaptureIssue=").Append(ReferencePairIssue);
            sb.Append("\nlegParentReference=V04590_SAME_FIXED_NATIVE_AND_GOVERNED/valid-invalid-ready=")
                .Append(ParentReferenceSamples).Append('-').Append(ParentReferenceInvalid).Append('-').Append(ParentReferenceReady)
                .Append("/peakMm(controlHipGap-rawBridgeShift-sourceLocalDrift)=").Append(F(ParentHipControlGapPeak * 1000f)).Append('-')
                .Append(F(ParentHipBridgeShiftPeak * 1000f)).Append('-').Append(F(ParentSourceLocalDriftPeak * 1000f))
                .Append("/issue=").Append(ParentReferenceIssue ?? "<none>")
                .Append("/scope=REFERENCE_CONTRACT_ONLY/legBodies-joints-forces=RUNTIME_SCOPE_OWNED/nativeV=COM_V/commandV=ORIGIN_V/UpperTarget=SOURCE_RELATIVE_POSE_ONLY");
            AppendStops(sb);
            sb.Append("\nAngularEvidenceReady qualifies reference/command/body clock coverage; joint availability is counted separately. Angular evidence: sampledInputW is the held/filtered bridge input, sourceW is the raw render derivative; bridge pose correction and reference braking correction are separate. The retained V04580 arrival reference subtracts relativeW*(|relativeW|/(2*sameStepBrakeCap)+dt) from the phase-aligned shortest rotation vector before gain4. Both retained V04670 planners keep that arrival estimate and ordinary alpha4. B may remove additional previous world W up to total alpha8 inside the same full-vector inertial budget1; it cannot spend that reserve accelerating through zero. The paired alternative uses the same actual-run native tensor, not a second physical plant. Its bounded motion cannot predict future source stops. V04590 publishes settled native COM kinematics and final governed command N for the two Hip points. This reference contract does not create bodies or apply leg forces; the runtime scope and combined bundle report physical leg participation. Hip reaction to Pelvis remains OFF. Pelvis inertiaTau/gyroTau remain the self-body terms; pelvisInertiaN adds the applied orbital terms, reduced mass, current native parent COM lever and pre-cap summed torque. The baseline uses mu=mp*mc/(mp+mc), with Chest COM at the joint; Chest self-spin inertia is independent. Angular AFF uses the current FINAL governed world-W derivative with dt_N, gain1 in the fixed baseline; its physical cache is independent of these diagnostics. P/D/AFF are summed before alpha40 and total-torque2 caps. Joint readback is diagnostic only and never fed back as compensation. Final issued commands without a following solve are not given a synthetic cleanup response. Reference and committed P/C commands belong to N; body delta-W/previous commands and native joint readback belong to solve N-1. Delta-W uses the previous interval dt. First/gapped responses and unavailable joint getters are NA. Native joint values are uncalibrated raw readback, not signed Pelvis torque or a closed angular ledger. Child AddTorque is Acceleration; its command remains rad/s^2, not Nm. Positive direction rates mean distance opening under the stated held velocities; source clocks and source/bridge motion remain visible. The quiet-source peak is one retained frame, with no new observer/ring/per-step output. Diagnostic gaps never alter physics or the existing tracking metrics.");
            return sb + "\nThe two-body linear ledger includes the committed common COM forces; native joint forces are internal to the free two-body subsystem. Angular/whole-body closure and natural support are separate questions. This receiver ledger excludes child leg commands; physical leg participation is reported by the runtime scope and combined bundle. Hip reaction to Pelvis is not routed. The legacy freeze verdict is a reference, not receiver certification.";
        }

        internal void ResetTracking()
        {
            ParentReferenceSamples = ParentReferenceInvalid = 0; ParentReferenceIssue = null;
            ParentHipControlGapPeak = ParentHipBridgeShiftPeak = ParentSourceLocalDriftPeak = 0f;
            Array.Clear(RawTiming, 0, RawTiming.Length);
            TrackingSamples = TrackingInvalid = TrackingNext = TrackingCount = RecoveryNext = RecoveryCount = 0;
            RecoveryStarted = RecoveryWithinBand = RecoveryNewSource = RecoveryStopped = RecoveryLimit = RecoveryGap = SourceOpenAtStop = 0;
            TrackingSeconds = 0; LongestPositionOverSeconds = LongestRotationOverSeconds = 0f; TrackingIssue = null;
            LastTracking = PeakPositionTracking = PeakRotationTracking = PeakNativeTracking = new TrackingFrame();
            PeakRawPositionTracking = PeakRawRotationTracking = new TrackingFrame();
            _sourceWasActive = _recoveryActive = _recoveryHolding = false; _sourceEventId = 0;
            _positionOverRun = _rotationOverRun = _recoveryHold = 0f; _recovery = new RecoveryFrame();
            LastRawCommonA = LastRawPelvisAlpha = LastRawPelvisTorque = 0f;
            LastPdA = LastFeedforwardA = Vector3.zero;
            AngularEvidenceSamples = AngularEvidenceInvalid = AngularResponseSamples = JointEvidenceSamples = QuietSourceSamples = ReferenceAwaySamples = BodyAwaySamples = 0;
            DirectionalBudgetSamples = 0;
            DirectionalBrakeCapPeak = DirectionalStepCapPeak = DirectionalInertiaTorquePeak = DirectionalRigidGyroPeak = 0f;
            AngularEvidenceIssue = null; AngularActuation = new AngularActuationSample(); PeakReferenceAwayTracking = new TrackingFrame();
            ReferenceComparison.Reset(); ReferencePairIssue = null;
            StopNext = StopCount = StopStarted = 0; _stopArmed = _stopActive = _stopHolding = false;
            _stopHold = 0f; _stopDirection = Vector3.zero; _stop = new StopFrame();
            Array.Clear(StopRing, 0, StopRing.Length);
            LastLinearClamped = LastAngularClamped = LastTorqueClamped = false;
            Array.Clear(TrackingRing, 0, TrackingRing.Length); Array.Clear(TrackingPhases, 0, TrackingPhases.Length);
            Array.Clear(RecoveryRing, 0, RecoveryRing.Length);
        }

        internal void ObserveTracking(TrackingFrame f, TwoBodyProxyReport source)
        {
            if (!TraceRunning || !f.Enabled) return;
            if (FiniteValue(f.Dt) && f.Dt > 0f) TrackingSeconds += f.Dt;
            if (!f.Valid || f.Fixed != CurrentTraceFixed || source == null ||
                !FiniteValue(source.SourceTurnRawAngularSpeed) || !FiniteValue(source.SourceTurnRawAngularAccelerationMagnitude) ||
                !FiniteValue(source.SourceTurnSampleDeltaSeconds) || !FiniteValue(source.SourceVelocitySampleAgeSeconds))
            {
                f.Issue = f.Issue ?? "TRACKING_CLOCK_OR_SOURCE_INVALID";
                ObserveInvalidTracking(f); return;
            }
            f.SourceV = source.TargetPelvisLinearVelocity; f.SourceWVector = source.SourceTurnRawAngularVelocity;
            if (!FiniteValue(f.SourceWVector.magnitude))
            { f.Valid = false; f.Issue = "TRACKING_SOURCE_ANGULAR_VECTOR_INVALID"; ObserveInvalidTracking(f); return; }
            f.Time = (float)TrackingSeconds; f.SourcePhase = source.MotionPhase;
            f.SourceActive = source.SourceTurnEventActive; f.SourceEventId = source.SourceTurnCurrentEventId;
            f.SourceSequence = source.SourceVelocitySampleSequence; f.SourceFresh = source.SourceVelocitySampleFreshThisFixedStep;
            f.SourceW = source.SourceTurnRawAngularSpeed; f.SourceAlpha = source.SourceTurnRawAngularAccelerationMagnitude;
            f.SourceDt = source.SourceTurnSampleDeltaSeconds; f.SourceAge = source.SourceVelocitySampleAgeSeconds;
            f.RawTimingGroup = f.SourceAge < 0f || f.SourceDt <= 0f || f.SourceSequence <= 0 ? 3 :
                f.SourceAge > RawTimingAgeLimit ? 2 : f.SourceFresh ? 0 : 1;
            f.CommandAlphaAvailable = LastTracking.Valid && LastTracking.Fixed + 1 == f.Fixed;
            f.CommandAlpha = f.CommandAlphaAvailable ? ((f.CommandW - LastTracking.CommandW) / f.Dt).magnitude : 0f;
            f.CommandComAAvailable = f.CommandAlphaAvailable;
            f.CommandComA = f.CommandComAAvailable ? (f.CommandComV - LastTracking.CommandComV) / f.Dt : Vector3.zero;
            if (!FiniteValue(f.CommandAlpha) || !FiniteValue(f.CommandComA.magnitude))
            {
                f.Valid = false; f.Issue = "TRACKING_COMMAND_DERIVATIVE_INVALID";
                ObserveInvalidTracking(f); return;
            }
            f.LinearClamped = f.Dynamic && LastLinearClamped; f.AngularClamped = f.Dynamic && LastAngularClamped;
            f.TorqueClamped = f.Dynamic && LastTorqueClamped; f.ChildAngularClamped = source.AngularAccelerationClamped;
            f.RawA = LastRawCommonA; f.RawAlpha = LastRawPelvisAlpha; f.RawTorque = LastRawPelvisTorque;
            f.PdA = LastPdA; f.FeedforwardA = LastFeedforwardA;
            f.AppliedA = CommonA.magnitude; f.AppliedTorque = PelvisTorque.magnitude;
            f = ObserveAngularEvidence(f, source);
            f = ObserveParentReference(f);
            if (f.SourceActive)
            {
                FinishRecovery(LastTracking, "CENSORED_BY_NEW_SOURCE_EVENT");
                _sourceWasActive = true; _sourceEventId = f.SourceEventId;
            }
            else if (_sourceWasActive)
            {
                _sourceWasActive = false; _recoveryActive = true; _recoveryHolding = false; _recoveryHold = 0f; RecoveryStarted++;
                _recovery = new RecoveryFrame { EventId = _sourceEventId, StartFixed = f.Fixed, StartTime = f.Time };
            }
            // Buckets are exclusive; source/recovery owns its interval, the original source phase is retained on each frame.
            f.Stage = f.SourceActive ? 2 : _recoveryActive ? 3 : f.SourcePhase == "TURN" ? 2 : f.SourcePhase == "IDLE" ? 0 : 1;
            f.RecoveryEventId = _recoveryActive ? _recovery.EventId : 0;
            if (f.AngularEvidenceValid && f.RawTimingGroup <= 1 && f.SourceWVector.magnitude <= 0.05f &&
                f.RenderReferenceErrorVector.magnitude > 0.5f * Mathf.Deg2Rad && f.RenderReferenceRate > 0.01f &&
                (!PeakReferenceAwayTracking.AngularEvidenceValid || f.RenderReferenceRate > PeakReferenceAwayTracking.RenderReferenceRate))
                PeakReferenceAwayTracking = f;
            if (_recoveryActive)
            {
                _recovery.PositionPeak = Mathf.Max(_recovery.PositionPeak, f.Position);
                _recovery.RotationPeak = Mathf.Max(_recovery.RotationPeak, f.Rotation);
                _recovery.NativePeak = Mathf.Max(_recovery.NativePeak, f.NativeGap);
                bool inBand = f.Position <= TrackingPositionBand && f.Rotation <= TrackingRotationBand && f.NativeGap <= TrackingNativeBand;
                // The first qualifying observation establishes a point, not a solved interval before source close.
                _recoveryHold = inBand && _recoveryHolding ? _recoveryHold + f.Dt : 0f;
                _recoveryHolding = inBand;
                if (_recoveryHold >= 0.15f) FinishRecovery(f, "WITHIN_DIAGNOSTIC_BAND");
                else if (f.Time - _recovery.StartTime >= 5f) FinishRecovery(f, "OBSERVATION_LIMIT_5S");
            }
            TrackingPhase p = TrackingPhases[f.Stage]; p.Samples++; p.Seconds += f.Dt;
            p.PositionIntegral += f.Position * (double)f.Dt; p.RotationIntegral += f.Rotation * (double)f.Dt;
            p.NativeIntegral += f.NativeGap * (double)f.Dt; p.VirtualIntegral += f.VirtualGap * (double)f.Dt;
            p.TargetNativeIntegral += f.TargetNativeGap * (double)f.Dt;
            p.RawRenderIntegral += f.RawRenderPosition * (double)f.Dt; p.RawRenderRotationIntegral += f.RawRenderRotation * (double)f.Dt;
            p.RawBridgeIntegral += f.RawBridgePosition * (double)f.Dt; p.RawChestIntegral += f.RawChestPosition * (double)f.Dt;
            p.RawTargetNativeIntegral += f.RawTargetNativeGap * (double)f.Dt; p.ReferenceShiftIntegral += f.ReferenceShift * (double)f.Dt;
            p.ReferenceLagIntegral += f.ReferenceAngularLag * (double)f.Dt;
            p.ComReferenceLagIntegral += f.ComReferenceLag * (double)f.Dt;
            p.PositionPeak = Mathf.Max(p.PositionPeak, f.Position); p.RotationPeak = Mathf.Max(p.RotationPeak, f.Rotation);
            p.NativePeak = Mathf.Max(p.NativePeak, f.NativeGap); p.VirtualPeak = Mathf.Max(p.VirtualPeak, f.VirtualGap);
            p.TargetNativePeak = Mathf.Max(p.TargetNativePeak, f.TargetNativeGap);
            p.RawRenderPeak = Mathf.Max(p.RawRenderPeak, f.RawRenderPosition); p.RawRenderRotationPeak = Mathf.Max(p.RawRenderRotationPeak, f.RawRenderRotation);
            p.RawBridgePeak = Mathf.Max(p.RawBridgePeak, f.RawBridgePosition); p.RawChestPeak = Mathf.Max(p.RawChestPeak, f.RawChestPosition);
            p.RawTargetNativePeak = Mathf.Max(p.RawTargetNativePeak, f.RawTargetNativeGap); p.ReferenceShiftPeak = Mathf.Max(p.ReferenceShiftPeak, f.ReferenceShift);
            p.ReferenceLagPeak = Mathf.Max(p.ReferenceLagPeak, f.ReferenceAngularLag);
            p.ComReferenceLagPeak = Mathf.Max(p.ComReferenceLagPeak, f.ComReferenceLag);
            p.CommandComVPeak = Mathf.Max(p.CommandComVPeak, f.CommandComV.magnitude);
            if (f.CommandComAAvailable) p.CommandComAPeak = Mathf.Max(p.CommandComAPeak, f.CommandComA.magnitude);
            if (f.ReferenceSpeedLimited) p.ReferenceSpeedLimits++; if (f.ReferenceAlphaLimited) p.ReferenceAlphaLimits++;
            if (f.ComReferenceSpeedLimited) p.ComReferenceSpeedLimits++; if (f.ComReferenceAccelerationLimited) p.ComReferenceAccelerationLimits++;
            if (f.Position > TrackingPositionBand) p.PositionOverSeconds += f.Dt;
            if (f.Rotation > TrackingRotationBand) p.RotationOverSeconds += f.Dt;
            if (f.LinearClamped) p.LinearClamps++; if (f.AngularClamped) p.AngularClamps++;
            if (f.TorqueClamped) p.TorqueClamps++; if (f.ChildAngularClamped) p.ChildAngularClamps++;
            TrackingPhases[f.Stage] = p;
            RawTimingStats timing = RawTiming[f.RawTimingGroup]; timing.Samples++; timing.Seconds += f.Dt;
            timing.PositionIntegral += f.RawRenderPosition * (double)f.Dt; timing.RotationIntegral += f.RawRenderRotation * (double)f.Dt;
            timing.PositionPeak = Mathf.Max(timing.PositionPeak, f.RawRenderPosition); timing.RotationPeak = Mathf.Max(timing.RotationPeak, f.RawRenderRotation);
            if (f.RawTimingGroup != 3)
            { timing.AgeIntegral += f.SourceAge * (double)f.Dt; timing.AgePeak = Mathf.Max(timing.AgePeak, f.SourceAge); }
            RawTiming[f.RawTimingGroup] = timing;
            _positionOverRun = f.Position > TrackingPositionBand ? _positionOverRun + f.Dt : 0f;
            _rotationOverRun = f.Rotation > TrackingRotationBand ? _rotationOverRun + f.Dt : 0f;
            LongestPositionOverSeconds = Mathf.Max(LongestPositionOverSeconds, _positionOverRun);
            LongestRotationOverSeconds = Mathf.Max(LongestRotationOverSeconds, _rotationOverRun);
            if (!PeakPositionTracking.Valid || f.Position > PeakPositionTracking.Position) PeakPositionTracking = f;
            if (!PeakRotationTracking.Valid || f.Rotation > PeakRotationTracking.Rotation) PeakRotationTracking = f;
            if (!PeakNativeTracking.Valid || f.NativeGap > PeakNativeTracking.NativeGap) PeakNativeTracking = f;
            if (!PeakRawPositionTracking.Valid || f.RawRenderPosition > PeakRawPositionTracking.RawRenderPosition) PeakRawPositionTracking = f;
            if (!PeakRawRotationTracking.Valid || f.RawRenderRotation > PeakRawRotationTracking.RawRenderRotation) PeakRawRotationTracking = f;
            PositionErrorPeak = Mathf.Max(PositionErrorPeak, f.Position); RotationErrorPeak = Mathf.Max(RotationErrorPeak, f.Rotation);
            ObserveStop(f);
            TrackingRing[TrackingNext] = f; TrackingNext = (TrackingNext + 1) % TrackingRing.Length;
            TrackingCount = Mathf.Min(TrackingCount + 1, TrackingRing.Length); TrackingSamples++; LastTracking = f;
        }

        // These interval/velocity rates are evidence, not a dynamics closure or a control input.
        private TrackingFrame ObserveAngularEvidence(TrackingFrame f, TwoBodyProxyReport source)
        {
            try
            {
                f.BridgeSourceW = source.TargetPelvisAngularVelocity; f.BridgePoseCorrectionW = source.PoseBridgeRotationCorrectionAngularVelocity;
                f.BridgeReportedW = source.PoseBridgeCommandAngularVelocity; f.BridgePredictionAge = source.PoseBridgePredictionAgeSeconds;
                f.BridgeCorrectionClamped = source.PoseBridgeRotationCorrectionClamped;
                AngularReferenceSample r = f.AngularReference; AngularActuationSample a = f.AngularActuation;
                if (!f.AngularBodiesCaptured || !f.AngularComparisonCaptured || !f.AngularContextCaptured || !r.Valid || !a.Valid ||
                    r.Sequence <= 0 || r.Dt != f.Dt || a.Fixed != f.Fixed || !FiniteVector(f.BridgeSourceW) || !FiniteVector(f.BridgePoseCorrectionW) || !FiniteVector(f.BridgeReportedW) || !FiniteValue(f.BridgePredictionAge) || (r.BridgeW - f.BridgeReportedW).magnitude > 0.00001f || (r.CommandW - f.CommandW).magnitude > 0.00001f ||
                    !FiniteVector(r.BridgeW) || !FiniteVector(r.CorrectionW) || !FiniteVector(r.DesiredW) || !FiniteVector(r.LimitedW) ||
                    !FiniteVector(r.PreviousW) || !FiniteVector(r.CommandW) || !FiniteVector(r.GeneratedAlpha) || !FiniteValue(r.ErrorAngle) ||
                    !FiniteValue(r.ScalarAlphaCap) || r.ScalarAlphaCap <= 0f || !FiniteValue(r.BrakeAlphaCap) || r.BrakeAlphaCap <= 0f ||
                    !FiniteValue(r.StepAlphaCap) || r.StepAlphaCap <= 0f || !FiniteValue(r.BrakeInertiaNorm) || !FiniteValue(r.StepInertiaNorm) ||
                    !FiniteValue(r.BaseStepAlphaCap) || r.BaseStepAlphaCap <= 0f || !FiniteValue(r.ExtraBrakeAlpha) || r.ExtraBrakeAlpha < 0f ||
                    !FiniteVector(r.InertialBudgetTorque) || !FiniteVector(r.RigidGyroEstimate) ||
                    !FiniteVector(a.PAlpha) || !FiniteVector(a.DAlpha) || !FiniteVector(a.AppliedAlpha) || !FiniteVector(a.InertiaAlphaTorque) || !FiniteVector(a.GyroTorque) || !FiniteVector(a.Torque) ||
                    !r.ArrivalBraking || !FiniteVector(r.RelativeW) || !FiniteVector(r.StoppingOffset) || !FiniteVector(r.ArrivalError) ||
                    !a.ReferenceFeedforward || !FiniteVector(a.FeedforwardAlpha) ||
                    (a.FeedforwardAlpha - r.GeneratedAlpha).magnitude > 0.00001f ||
                    !FiniteValue(a.ReducedMass) || a.ReducedMass <= 0f || !FiniteVector(a.ParentLever) ||
                    !FiniteVector(a.OrbitalAlphaTorque) || !FiniteVector(a.OrbitalGyroTorque) || !FiniteVector(a.RawTorque) ||
                    !FiniteVector(f.CommandW) || !FiniteVector(f.BodyW) || !FiniteVector(f.RotationErrorVector) || !FiniteVector(f.ReferenceRotationErrorVector) || !FiniteVector(f.ChestW) || !FiniteVector(f.ChestRotationErrorVector) || !FiniteVector(f.RenderReferenceErrorVector) ||
                    !FiniteVector(f.ChildPAlpha) || !FiniteVector(f.ChildDAlpha) || !FiniteVector(f.ChildAppliedAlpha) ||
                    !FiniteVector(f.ChildTargetRelativeW) || !FiniteVector(f.ChildMeasuredRelativeW) ||
                    (f.JointReactionAvailable && (!FiniteVector(f.JointForce) || !FiniteVector(f.JointTorque) || !FiniteVector(f.JointParentLever))))
                    return InvalidAngularEvidence(f, "ANGULAR_CAPTURE_OR_CLOCK_INVALID");
                f.BridgeReferenceRate = DirectionRate(f.ReferenceRotationErrorVector, r.BridgeW - f.CommandW);
                f.RenderReferenceRate = DirectionRate(f.RenderReferenceErrorVector, f.SourceWVector - f.CommandW);
                f.BodyControlRate = DirectionRate(f.RotationErrorVector, f.CommandW - f.BodyW);
                f.AngularResponseAvailable = LastTracking.Valid && LastTracking.AngularEvidenceValid && LastTracking.Fixed + 1 == f.Fixed &&
                    LastTracking.AngularReference.Sequence + 1 == r.Sequence && FiniteValue(LastTracking.Dt) && LastTracking.Dt > 0f;
                if (f.AngularResponseAvailable)
                {
                    // Response at pre-control N belongs to solve/committed commands N-1.
                    f.ResponseFromFixed = LastTracking.Fixed; f.ResponseDt = LastTracking.Dt;
                    f.PelvisResponseAlpha = (f.BodyW - LastTracking.BodyW) / f.ResponseDt;
                    f.ChestResponseAlpha = (f.ChestW - LastTracking.ChestW) / f.ResponseDt;
                    f.PreviousPelvisTorque = LastTracking.AngularActuation.Torque; f.PreviousChildAlpha = LastTracking.ChildAppliedAlpha;
                    f.PreviousPelvisCommandAlpha = LastTracking.AngularActuation.AppliedAlpha;
                    f.PreviousPelvisFeedforwardAlpha = LastTracking.AngularActuation.FeedforwardAlpha;
                    if (!FiniteVector(f.PelvisResponseAlpha) || !FiniteVector(f.ChestResponseAlpha)) return InvalidAngularEvidence(f, "ANGULAR_RESPONSE_NONFINITE");
                }
                if (!FiniteValue(f.BridgeReferenceRate) || !FiniteValue(f.RenderReferenceRate) || !FiniteValue(f.BodyControlRate))
                    return InvalidAngularEvidence(f, "ANGULAR_DIRECTION_RATE_NONFINITE");
                f.AngularEvidenceValid = true; AngularEvidenceSamples++;
                if (r.DirectionalBudget) ReferenceComparison.Observe(f.ReferencePair, r.Sequence, f.Dt, f.SourceWVector, f.RawTimingGroup, r.CommandW);
                if (r.DirectionalBudget)
                {
                    DirectionalBudgetSamples++;
                    DirectionalBrakeCapPeak = Mathf.Max(DirectionalBrakeCapPeak, r.BrakeAlphaCap);
                    DirectionalStepCapPeak = Mathf.Max(DirectionalStepCapPeak, r.StepAlphaCap);
                    DirectionalInertiaTorquePeak = Mathf.Max(DirectionalInertiaTorquePeak, r.InertialBudgetTorque.magnitude);
                    DirectionalRigidGyroPeak = Mathf.Max(DirectionalRigidGyroPeak, r.RigidGyroEstimate.magnitude);
                }
                if (f.AngularResponseAvailable) AngularResponseSamples++;
                if (f.JointReactionAvailable) JointEvidenceSamples++;
                if (f.RawTimingGroup <= 1 && f.SourceWVector.magnitude <= 0.05f)
                {
                    QuietSourceSamples++;
                    if (f.RenderReferenceErrorVector.magnitude > 0.5f * Mathf.Deg2Rad && f.RenderReferenceRate > 0.01f)
                    {
                        ReferenceAwaySamples++;

                    }
                    if (f.RotationErrorVector.magnitude > 0.5f * Mathf.Deg2Rad && f.BodyControlRate > 0.01f) BodyAwaySamples++;
                }
            }
            catch (Exception e) { return InvalidAngularEvidence(f, "ANGULAR_EVIDENCE_EXCEPTION:" + e.GetType().Name); }
            return f;
        }
        private TrackingFrame InvalidAngularEvidence(TrackingFrame f, string issue)
        {
            f.AngularEvidenceValid = f.AngularResponseAvailable = false;
            f.AngularEvidenceIssue = f.AngularEvidenceIssue ?? issue; AngularEvidenceInvalid++; AngularEvidenceIssue = f.AngularEvidenceIssue;
            return f;
        }
        private TrackingFrame ObserveParentReference(TrackingFrame f)
        {
            if (!f.ParentReferenceExpected) return f;
            try
            {
                PelvisParentFrame p = f.ParentFrame;
                if (f.RuntimeContextExpected && (!f.RuntimeContext.Valid || f.RuntimeContext.FixedTime != f.ParentReferenceTime || f.RuntimeContext.Dt != f.Dt))
                    throw new InvalidOperationException("PARENT_RUNTIME_CLOCK_MISMATCH");
                if (!p.Valid || !p.Finite() || p.TraceFixed != f.Fixed || p.FixedTime != f.ParentReferenceTime || p.Dt != f.Dt || p.SourceSequence != f.SourceSequence ||
                    !FiniteVector(f.ParentBridgeLeftShift) || !FiniteVector(f.ParentBridgeRightShift) ||
                    (p.CommandW - f.CommandW).magnitude > 0.00001f || (p.W - f.BodyW).magnitude > 0.00001f)
                    throw new InvalidOperationException("PARENT_REFERENCE_CAPTURE_OR_CLOCK_INVALID");
                f.ParentReferenceValid = true; ParentReferenceSamples++;
                ParentHipControlGapPeak = Mathf.Max(ParentHipControlGapPeak, Mathf.Max((p.Left.CommandPoint - p.Left.SettledPoint).magnitude, (p.Right.CommandPoint - p.Right.SettledPoint).magnitude));
                ParentHipBridgeShiftPeak = Mathf.Max(ParentHipBridgeShiftPeak, Mathf.Max(f.ParentBridgeLeftShift.magnitude, f.ParentBridgeRightShift.magnitude));
                ParentSourceLocalDriftPeak = Mathf.Max(ParentSourceLocalDriftPeak, Mathf.Max(p.Left.SourceLocalDrift.magnitude, p.Right.SourceLocalDrift.magnitude));
            }
            catch (Exception e)
            {
                f.ParentReferenceValid = false; ParentReferenceInvalid++;
                f.ParentReferenceIssue = f.ParentReferenceIssue ?? "PARENT_REFERENCE_EXCEPTION:" + e.GetType().Name + ":" + e.Message;
                ParentReferenceIssue = f.ParentReferenceIssue;
            }
            return f;
        }
        private static bool FiniteVector(Vector3 v) => FiniteValue(v.x) && FiniteValue(v.y) && FiniteValue(v.z) && FiniteValue(v.sqrMagnitude);
        private static float DirectionRate(Vector3 error, Vector3 relativeW) => error.magnitude > 0.00001f ? Vector3.Dot(error / error.magnitude, relativeW) : 0f;
        private static void AppendAngularEvidence(StringBuilder sb, TrackingFrame f)
        {
            if (!f.AngularEvidenceValid) { sb.Append("/angularEvidence=NA/").Append(f.AngularEvidenceIssue ?? "NOT_CAPTURED"); return; }
            AngularReferenceSample r = f.AngularReference; AngularActuationSample a = f.AngularActuation;
            sb.Append("/angularReference(seq-dt-phaseErrorRad)=").Append(r.Sequence).Append('-').Append(F(r.Dt)).Append('-').Append(F(r.ErrorAngle))
                .Append("/referenceArrival(enabled-relativeW-stopOffsetRad-arrivalErrorRad)=").Append(r.ArrivalBraking).Append('|').Append(V(r.RelativeW)).Append('|').Append(V(r.StoppingOffset)).Append('|').Append(V(r.ArrivalError))
                .Append("/referenceBudgetN(directional-scalarCap-brakeCap-stepCap-brakeInertiaNorm-stepInertiaNorm)=").Append(r.DirectionalBudget).Append('|').Append(F(r.ScalarAlphaCap)).Append('|').Append(F(r.BrakeAlphaCap)).Append('|').Append(F(r.StepAlphaCap)).Append('|').Append(F(r.BrakeInertiaNorm)).Append('|').Append(F(r.StepInertiaNorm))
                .Append("/referenceRigidModel(inertiaTau-gyroEstimate)=").Append(r.DirectionalBudget ? V(r.InertialBudgetTorque) + "|" + V(r.RigidGyroEstimate) : "NA_SCALAR_ARM")
                .Append("/referenceReserveN(baseStepCap-extraBrakeAlpha)=").Append(F(r.BaseStepAlphaCap)).Append('|').Append(F(r.ExtraBrakeAlpha))
                .Append("/referencePairN(valid-rawDegA-rawDegB-WA-WB-extraB)=").Append(f.ReferencePair.Valid && f.ReferencePair.RawSourceCaptured ?
                    "True|" + F(f.ReferencePair.RawErrorA.magnitude * Mathf.Rad2Deg) + "|" + F(f.ReferencePair.RawErrorB.magnitude * Mathf.Rad2Deg) + "|" + V(f.ReferencePair.WA) + "|" + V(f.ReferencePair.WB) + "|" + F(f.ReferencePair.ExtraBrakeAlphaB) : "NA_ALTERNATIVE_OR_SOURCE_CAPTURE")
                .Append("/referenceW(bridge-correction-desired-limited-prev-command-alpha)=").Append(V(r.BridgeW)).Append('|').Append(V(r.CorrectionW)).Append('|').Append(V(r.DesiredW)).Append('|').Append(V(r.LimitedW)).Append('|').Append(V(r.PreviousW)).Append('|').Append(V(r.CommandW)).Append('|').Append(V(r.GeneratedAlpha))
                .Append("/bridgeInput(sampledW-poseCorrectionW-outputW-ageSeconds-correctionClamped)=").Append(V(f.BridgeSourceW)).Append('|').Append(V(f.BridgePoseCorrectionW)).Append('|').Append(V(f.BridgeReportedW)).Append('|').Append(F(f.BridgePredictionAge)).Append('|').Append(f.BridgeCorrectionClamped)
                .Append("/angularBodies(chestW-chestErrorRad)=").Append(V(f.ChestW)).Append('|').Append(V(f.ChestRotationErrorVector))
                .Append("/renderToReferenceRad=").Append(V(f.RenderReferenceErrorVector))
                .Append("/directionRatesRad(bridgeToRef-renderToRef-controlToBody)=").Append(f.ReferenceRotationErrorVector.magnitude > 0.00001f ? F(f.BridgeReferenceRate) : "NA_ZERO_ERROR").Append('|').Append(f.RenderReferenceErrorVector.magnitude > 0.00001f ? F(f.RenderReferenceRate) : "NA_ZERO_ERROR").Append('|').Append(f.RotationErrorVector.magnitude > 0.00001f ? F(f.BodyControlRate) : "NA_ZERO_ERROR")
                .Append("/pelvisCommandN(Palpha-Dalpha-appliedAlpha-inertiaTau-gyroTau-tau)=").Append(V(a.PAlpha)).Append('|').Append(V(a.DAlpha)).Append('|').Append(V(a.AppliedAlpha)).Append('|').Append(V(a.InertiaAlphaTorque)).Append('|').Append(V(a.GyroTorque)).Append('|').Append(V(a.Torque))
                .Append("/angularTargetAFFN(enabled-alphaRad)=").Append(a.ReferenceFeedforward).Append('|').Append(V(a.FeedforwardAlpha))
                .Append("/pelvisInertiaN(model-mu-lever-orbitalAlphaTau-orbitalGyroTau-rawTau)=SELF_PLUS_PAIR_ORBITAL|").Append(F(a.ReducedMass)).Append('|').Append(V(a.ParentLever)).Append('|').Append(V(a.OrbitalAlphaTorque)).Append('|').Append(V(a.OrbitalGyroTorque)).Append('|').Append(V(a.RawTorque))
                .Append("/chestCommandN(Palpha-Dalpha-appliedAlpha-targetRelW-measuredRelW)=").Append(V(f.ChildPAlpha)).Append('|').Append(V(f.ChildDAlpha)).Append('|').Append(V(f.ChildAppliedAlpha)).Append('|').Append(V(f.ChildTargetRelativeW)).Append('|').Append(V(f.ChildMeasuredRelativeW))
                .Append("/responseInterval=").Append(f.AngularResponseAvailable ? f.ResponseFromFixed + ">" + f.Fixed + "/dt=" + F(f.ResponseDt) : "NA_FIRST_OR_GAP")
                .Append("/responsePreviousSolve(Palpha-Calpha-previousPtau-previousCalpha)=").Append(f.AngularResponseAvailable ? V(f.PelvisResponseAlpha) + "|" + V(f.ChestResponseAlpha) + "|" + V(f.PreviousPelvisTorque) + "|" + V(f.PreviousChildAlpha) : "NA")
                .Append("/previousPelvisCommand(alphaAfterCap40-affBeforeCaps)=").Append(f.AngularResponseAvailable ? V(f.PreviousPelvisCommandAlpha) + "|" + V(f.PreviousPelvisFeedforwardAlpha) : "NA")
                .Append("/jointReadbackPreviousSolve=").Append(f.JointReactionAvailable ? "AVAILABLE/fromFixed=" + (f.Fixed > 1 ? (f.Fixed - 1) + "" : "PRE_TRACE") + "/rawForce=" + V(f.JointForce) + "|rawTorque=" + V(f.JointTorque) + "|parentLever=" + V(f.JointParentLever) : "NA_UNAVAILABLE");
        }
        private static void AppendParentReference(StringBuilder sb, TrackingFrame f)
        {
            if (!f.ParentReferenceExpected) return;
            if (!f.ParentReferenceValid) { sb.Append("/legParentReference=NA/").Append(f.ParentReferenceIssue ?? "NOT_CAPTURED"); return; }
            PelvisParentFrame p = f.ParentFrame;
            sb.Append("/parentFrame(epoch-seq-traceFixed-body-fixedTime-dt-sourceCapture-sourceSeq)=").Append(p.Epoch).Append('|').Append(p.PhysicsSequence).Append('|').Append(p.TraceFixed).Append('|').Append(p.BodyId).Append('|').Append(F(p.FixedTime)).Append('|').Append(F(p.Dt)).Append('|').Append(p.SourceCapture).Append('|').Append(p.SourceSequence)
                .Append("/hipLocal(L-R)=").Append(V(p.Left.LocalAnchor)).Append('|').Append(V(p.Right.LocalAnchor))
                .Append("/hipPoint(settledL-commandL-settledR-commandR)=").Append(V(p.Left.SettledPoint)).Append('|').Append(V(p.Left.CommandPoint)).Append('|').Append(V(p.Right.SettledPoint)).Append('|').Append(V(p.Right.CommandPoint))
                .Append("/hipV(settledL-commandL-settledR-commandR)=").Append(V(p.Left.SettledVelocity)).Append('|').Append(V(p.Left.CommandVelocity)).Append('|').Append(V(p.Right.SettledVelocity)).Append('|').Append(V(p.Right.CommandVelocity))
                .Append("/hipBridgeShift(L-R)=").Append(V(f.ParentBridgeLeftShift)).Append('|').Append(V(f.ParentBridgeRightShift));
        }

        private void ObserveInvalidTracking(TrackingFrame f)
        {
            FinishStop(LastTracking, "CENSORED_DIAGNOSTIC_GAP"); _stopArmed = false;
            TrackingInvalid++; TrackingIssue = f.Issue;
            FinishRecovery(LastTracking, "CENSORED_BY_DIAGNOSTIC_GAP");
            _sourceWasActive = false; LastTracking.Valid = false; _positionOverRun = _rotationOverRun = 0f;
        }

        internal void EndTracking()
        {
            if (!TraceRunning) return;
            FinishStop(LastTracking, "CENSORED_TRACE_STOP"); _stopArmed = false;
            FinishRecovery(LastTracking, "CENSORED_BY_TRACE_STOP");
            if (_sourceWasActive) SourceOpenAtStop++;
            _sourceWasActive = false;
        }

        private void FinishRecovery(TrackingFrame end, string reason)
        {
            if (!_recoveryActive) return;
            _recoveryActive = _recoveryHolding = false; _recovery.EndFixed = end.Fixed;
            _recovery.Duration = Mathf.Max(0f, end.Time - _recovery.StartTime);
            _recovery.EndPosition = end.Position; _recovery.EndRotation = end.Rotation; _recovery.EndNative = end.NativeGap;
            _recovery.Result = reason;
            if (reason == "WITHIN_DIAGNOSTIC_BAND") RecoveryWithinBand++;
            else if (reason == "CENSORED_BY_NEW_SOURCE_EVENT") RecoveryNewSource++;
            else if (reason == "CENSORED_BY_TRACE_STOP") RecoveryStopped++;
            else if (reason == "OBSERVATION_LIMIT_5S") RecoveryLimit++;
            else RecoveryGap++;
            RecoveryRing[RecoveryNext] = _recovery; RecoveryNext = (RecoveryNext + 1) % RecoveryRing.Length;
            RecoveryCount = Mathf.Min(RecoveryCount + 1, RecoveryRing.Length); _recoveryHold = 0f;
        }

        internal string GetTrackingCompact() => "valid-invalid=" + TrackingSamples + "-" + TrackingInvalid +
            "/t=" + F((float)TrackingSeconds) + "/stage=" + (LastTracking.Valid ? StageName(LastTracking.Stage) : "UNSAMPLED") +
            "/COMmm-Pdeg-nativeMm-virtualMm=" + (LastTracking.Valid ? F(LastTracking.Position * 1000f) + "-" + F(LastTracking.Rotation) + "-" +
                F(LastTracking.NativeGap * 1000f) + "-" + F(LastTracking.VirtualGap * 1000f) : "NA") +
            "/rawRenderCOMmm=" + (LastTracking.Valid ? F(LastTracking.RawRenderPosition * 1000f) : "NA") +
            "/rawTiming=" + (LastTracking.Valid ? RawTimingName(LastTracking.RawTimingGroup) : "UNSAMPLED") + "/issue=" + (TrackingIssue ?? "<none>");

        private string GetTrackingDetails()
        {
            var sb = new StringBuilder("\nreceiverTracking=" + TrackingRevision + "/datasetReady=" + TrackingComplete +
                "/valid-invalid=" + TrackingSamples + "-" + TrackingInvalid + "/seconds=" + F((float)TrackingSeconds) +
                "/sampleDomain=RIGIDBODY_BEFORE_NEXT_CONTROL_WRITE/target=" +
                (TraceDynamic ? "V04520_COMMON_COM_PHASE_ALIGNED_NATIVE_REFERENCE" : "LEGACY_POSE_BRIDGE_PLUS_SOURCE_RELATIVE_POSE") +
                "/longestOver(position30mm-rotation5deg)=" + F(LongestPositionOverSeconds) + "-" + F(LongestRotationOverSeconds) +
                "/issue=" + (TrackingIssue ?? "<none>"));
            for (int i = 0; i < TrackingPhases.Length; i++)
            {
                TrackingPhase p = TrackingPhases[i];
                sb.Append("\nreceiverPhase=").Append(StageName(i)).Append("/samples-seconds=").Append(p.Samples).Append('-').Append(F((float)p.Seconds))
                    .Append("/mean(COMmm-Pdeg-nativeMm-virtualMm)=").Append(p.Samples == 0 ? "NA" :
                        F((float)(p.PositionIntegral / p.Seconds) * 1000f) + "-" + F((float)(p.RotationIntegral / p.Seconds)) + "-" +
                        F((float)(p.NativeIntegral / p.Seconds) * 1000f) + "-" + F((float)(p.VirtualIntegral / p.Seconds) * 1000f))
                    .Append("/peak(COMmm-Pdeg-nativeMm-virtualMm)=").Append(p.Samples == 0 ? "NA" :
                        F(p.PositionPeak * 1000f) + "-" + F(p.RotationPeak) + "-" + F(p.NativePeak * 1000f) + "-" + F(p.VirtualPeak * 1000f))
                    .Append("/overSeconds(position30mm-rotation5deg)=").Append(F((float)p.PositionOverSeconds)).Append('-').Append(F((float)p.RotationOverSeconds))
                    .Append("/targetNativeGapMm(mean-peak)=").Append(p.Samples == 0 ? "NA" : F((float)(p.TargetNativeIntegral / p.Seconds) * 1000f) + "-" + F(p.TargetNativePeak * 1000f))
                    .Append("/rawRender(COMmm-Pdeg mean)=").Append(p.Samples == 0 ? "NA" : F((float)(p.RawRenderIntegral / p.Seconds) * 1000f) + "-" + F((float)(p.RawRenderRotationIntegral / p.Seconds)))
                    .Append("/rawRender(COMmm-Pdeg peak)=").Append(p.Samples == 0 ? "NA" : F(p.RawRenderPeak * 1000f) + "-" + F(p.RawRenderRotationPeak))
                    .Append("/rawBridgeCOMmm(mean-peak)=").Append(p.Samples == 0 ? "NA" : F((float)(p.RawBridgeIntegral / p.Seconds) * 1000f) + "-" + F(p.RawBridgePeak * 1000f))
                    .Append("/rawChestMm(mean-peak)=").Append(p.Samples == 0 ? "NA" : F((float)(p.RawChestIntegral / p.Seconds) * 1000f) + "-" + F(p.RawChestPeak * 1000f))
                    .Append("/rawTargetNativeGapMm(mean-peak)=").Append(p.Samples == 0 ? "NA" : F((float)(p.RawTargetNativeIntegral / p.Seconds) * 1000f) + "-" + F(p.RawTargetNativePeak * 1000f))
                    .Append("/referenceShiftMm(mean-peak)=").Append(p.Samples == 0 ? "NA" : F((float)(p.ReferenceShiftIntegral / p.Seconds) * 1000f) + "-" + F(p.ReferenceShiftPeak * 1000f))
                    .Append("/referenceAngularLagDeg(mean-peak)=").Append(p.Samples == 0 ? "NA" : F((float)(p.ReferenceLagIntegral / p.Seconds)) + "-" + F(p.ReferenceLagPeak))
                    .Append("/referenceLimits(speed-alpha)=").Append(TraceDynamic ? p.ReferenceSpeedLimits + "-" + p.ReferenceAlphaLimits : "NA_INACTIVE_OR_LEGACY")
                    .Append("/comReferenceLagMm(mean-peak)=").Append(p.Samples == 0 ? "NA" : F((float)(p.ComReferenceLagIntegral / p.Seconds) * 1000f) + "-" + F(p.ComReferenceLagPeak * 1000f))
                    .Append("/comReferenceLimits(speed-acceleration)=").Append(TraceDynamic ? p.ComReferenceSpeedLimits + "-" + p.ComReferenceAccelerationLimits : "NA_INACTIVE_OR_LEGACY")
                    .Append("/commonCommandPeak(Vmps-Amps2)=").Append(p.Samples == 0 ? "NA" : F(p.CommandComVPeak) + "-" + F(p.CommandComAPeak))
                    .Append("/receiverClamps(linear-angular-torque)=").Append(TraceDynamic ? p.LinearClamps + "-" + p.AngularClamps + "-" + p.TorqueClamps : "NA_INACTIVE_OR_LEGACY")
                    .Append("/childAngularClamps=").Append(p.ChildAngularClamps);
            }
            for (int i = 0; i < RawTiming.Length; i++)
            {
                RawTimingStats timing = RawTiming[i];
                sb.Append("\nreceiverRawTiming=").Append(RawTimingName(i)).Append("/ageLimitMs=").Append(F(RawTimingAgeLimit * 1000f))
                    .Append("/samples-seconds=").Append(timing.Samples).Append('-').Append(F((float)timing.Seconds))
                    .Append("/rawRender(COMmm-Pdeg mean)=").Append(timing.Samples == 0 ? "NA" : F((float)(timing.PositionIntegral / timing.Seconds) * 1000f) + "-" + F((float)(timing.RotationIntegral / timing.Seconds)))
                    .Append("/rawRender(COMmm-Pdeg peak)=").Append(timing.Samples == 0 ? "NA" : F(timing.PositionPeak * 1000f) + "-" + F(timing.RotationPeak))
                    .Append("/sourceAgeMs(mean-peak)=").Append(timing.Samples == 0 || i == 3 ? "NA" : F((float)(timing.AgeIntegral / timing.Seconds) * 1000f) + "-" + F(timing.AgePeak * 1000f));
            }
            AppendTrackingFrame(sb, "peakCOM", PeakPositionTracking); AppendTrackingFrame(sb, "peakPelvisRotation", PeakRotationTracking);
            AppendTrackingFrame(sb, "peakNativeGap", PeakNativeTracking);
            AppendTrackingFrame(sb, "peakRawRenderCOM", PeakRawPositionTracking); AppendTrackingFrame(sb, "peakRawRenderPelvisRotation", PeakRawRotationTracking);
            for (int k = 0; k < TrackingCount; k++) AppendTrackingFrame(sb, "tail", TrackingRing[(TrackingNext - TrackingCount + k + TrackingRing.Length) % TrackingRing.Length]);
            sb.Append("\nreceiverRecovery=READ_ONLY_FIRST_FIXED_AFTER_SOURCE_CLOSE/started-withinBand-newSource-stop-limit-gap=")
                .Append(RecoveryStarted).Append('-').Append(RecoveryWithinBand).Append('-').Append(RecoveryNewSource).Append('-')
                .Append(RecoveryStopped).Append('-').Append(RecoveryLimit).Append('-').Append(RecoveryGap)
                .Append("/active=").Append(_recoveryActive).Append("/sourceOpenAtStop=").Append(SourceOpenAtStop)
                .Append("/band=COM30mm_P5deg_NATIVE2mm_HOLD0.15s/observationLimit=5s/retained=").Append(RecoveryCount).Append("/capacity=8");
            for (int k = 0; k < RecoveryCount; k++)
            {
                RecoveryFrame f = RecoveryRing[(RecoveryNext - RecoveryCount + k + RecoveryRing.Length) % RecoveryRing.Length];
                sb.Append("\nreceiverRecoveryRow=event").Append(f.EventId).Append("/fixed=").Append(f.StartFixed).Append('>').Append(f.EndFixed)
                    .Append("/start-duration=").Append(F(f.StartTime)).Append('-').Append(F(f.Duration)).Append("/result=").Append(f.Result)
                    .Append("/peak(COMmm-Pdeg-nativeMm)=").Append(F(f.PositionPeak * 1000f)).Append('-').Append(F(f.RotationPeak)).Append('-').Append(F(f.NativePeak * 1000f))
                    .Append("/end(COMmm-Pdeg-nativeMm)=").Append(F(f.EndPosition * 1000f)).Append('-').Append(F(f.EndRotation)).Append('-').Append(F(f.EndNative * 1000f));
            }
            sb.Append("\nTracking groups describe source events/recovery/IDLE/MOVE/TURN, not the operator stopwatch. Raw Render comparisons include held/aged samples; timing groups retain fresh, held, stale and unknown evidence. The bounded reference integrates final bounded W/COM V on the Fixed clock. Inverse native COM geometry and same-step governed angular/COM acceleration AFF are retained. Reference shifts are not physical tracking improvement. The reference alpha budget is a conservative estimate, not native/human certification. Current force/torque commands are N; body response and native joint readback are N-1. Leg-parent preflight separately checks the settled native COM velocity and final command origin velocity, with a live epoch/owner/sequence/time lease and two bound Hip points. The reference contract exposes source-relative Upper poses; it does not certify the separate leg derivative/control path, leg forces or whole-body support. Physical leg readiness belongs to the combined bundle. Raw bridge commands are not a fallback parent trajectory. Stop/recovery bands remain diagnostic; the old virtual-anchor freeze verdict is reference-only.");
            return sb.ToString();
        }

        private static void AppendTrackingFrame(StringBuilder sb, string label, TrackingFrame f)
        {
            sb.Append("\nreceiverTrackingRow=").Append(label);
            if (!f.Valid) { sb.Append("/NA"); return; }
            sb.Append("/fixed-time-dt=").Append(f.Fixed).Append('-').Append(F(f.Time)).Append('-').Append(F(f.Dt))
                .Append("/stage-sourcePhase=").Append(StageName(f.Stage)).Append('-').Append(f.SourcePhase)
                .Append("/COMmm-Pdeg-nativeMm-virtualMm-Verr=").Append(F(f.Position * 1000f)).Append('-').Append(F(f.Rotation)).Append('-')
                .Append(F(f.NativeGap * 1000f)).Append('-').Append(F(f.VirtualGap * 1000f)).Append('-').Append(F(f.VelocityError))
                .Append("/targetNativeGapMm=").Append(F(f.TargetNativeGap * 1000f))
                .Append("/projected=").Append(f.Projected).Append("/rawRender(COMmm-Pdeg)=").Append(F(f.RawRenderPosition * 1000f)).Append('-').Append(F(f.RawRenderRotation))
                .Append("/rawTiming=").Append(RawTimingName(f.RawTimingGroup))
                .Append("/referenceLagDeg-alphaRad-speedLimited-alphaLimited=").Append(F(f.ReferenceAngularLag)).Append('-').Append(F(f.ReferenceAlpha)).Append('-').Append(f.ReferenceSpeedLimited).Append('-').Append(f.ReferenceAlphaLimited)
                .Append("/comReference(lagMm-speedLimited-aLimited-genA)=").Append(F(f.ComReferenceLag * 1000f)).Append('-').Append(f.ComReferenceSpeedLimited).Append('-').Append(f.ComReferenceAccelerationLimited).Append('-').Append(V(f.ComReferenceA))
                .Append("/commonRefCOM(p-v-commandA)=").Append(V(f.CommandCom)).Append('|').Append(V(f.CommandComV)).Append('|').Append(f.CommandComAAvailable ? V(f.CommandComA) : "NA_FIRST_OR_GAP")
                .Append("/nativeGoalCOM(p-v)=").Append(V(f.NativeGoalCom)).Append('|').Append(V(f.NativeGoalV))
                .Append("/COMerror(p-v)=").Append(V(f.ComError)).Append('|').Append(V(f.ComVelocityError))
                .Append("/rawRenderComError-sourceV=").Append(V(f.RawRenderComError)).Append('|').Append(V(f.SourceV))
                .Append("/commonA(pd-ff)=").Append(V(f.PdA)).Append('|').Append(V(f.FeedforwardA))
                .Append("/angularErrorRad(control-rawRender-referenceToBridge)=").Append(V(f.RotationErrorVector)).Append('|').Append(V(f.RawRotationErrorVector)).Append('|').Append(V(f.ReferenceRotationErrorVector))
                .Append("/worldWrad(control-body-source)=").Append(V(f.CommandW)).Append('|').Append(V(f.BodyW)).Append('|').Append(V(f.SourceWVector))
                .Append("/rawBridgeCOMmm-rawChestMm-rawTargetGapMm-shiftMm-rawVerr=").Append(F(f.RawBridgePosition * 1000f)).Append('-')
                .Append(F(f.RawChestPosition * 1000f)).Append('-').Append(F(f.RawTargetNativeGap * 1000f)).Append('-').Append(F(f.ReferenceShift * 1000f)).Append('-').Append(F(f.RawVelocityError))
                .Append("/cmdWrad-cmdAlpharad=").Append(F(f.CommandW.magnitude)).Append('-').Append(f.CommandAlphaAvailable ? F(f.CommandAlpha) : "NA_FIRST_OR_GAP")
                .Append("/source(seq-fresh-event-active-recovery)=").Append(f.SourceSequence).Append('-').Append(f.SourceFresh).Append('-').Append(f.SourceEventId).Append('-').Append(f.SourceActive).Append('-').Append(f.RecoveryEventId)
                .Append("/source(dt-age-Wrad-Alpharad)=").Append(F(f.SourceDt)).Append('-').Append(F(f.SourceAge)).Append('-').Append(F(f.SourceW)).Append('-').Append(F(f.SourceAlpha))
                .Append("/rawReceiver(a-alpha-tau)=").Append(f.Dynamic ? F(f.RawA) + "-" + F(f.RawAlpha) + "-" + F(f.RawTorque) : "NA_INACTIVE_OR_LEGACY")
                .Append("/appliedReceiver(a-tau)=").Append(f.Dynamic ? F(f.AppliedA) + "-" + F(f.AppliedTorque) : "NA_INACTIVE_OR_LEGACY")
                .Append("/receiverClamps(L-A-T)=").Append(f.Dynamic ? f.LinearClamped + "-" + f.AngularClamped + "-" + f.TorqueClamped : "NA_INACTIVE_OR_LEGACY")
                .Append("/childAngularClamp=").Append(f.ChildAngularClamped);
            AppendAngularEvidence(sb, f); AppendParentReference(sb, f);
            if (f.RuntimeContextExpected) sb.Append('/').Append(f.RuntimeContext.Status());
        }
        private static string RawTimingName(int i) => i == 0 ? "FRESH_LE_100MS" : i == 1 ? "HELD_LE_100MS" : i == 2 ? "STALE_GT_100MS" : "UNKNOWN";
        private static string StageName(int i) => i == 0 ? "IDLE" : i == 1 ? "MOVE" : i == 2 ? "TURN" : "RECOVERY";
        private static bool FiniteValue(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static string F(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
        private static string V(Vector3 v) => "(" + F(v.x) + "," + F(v.y) + "," + F(v.z) + ")";
    }
}
