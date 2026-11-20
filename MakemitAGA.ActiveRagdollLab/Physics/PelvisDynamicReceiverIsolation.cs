using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // V04470 rebuilds the receiver as one coherent foundation, following the V0316/20 failure boundaries.
    // The receiver owns common translation and Pelvis rotation, including the
    // bounded bilateral Hip compensation in the typed Hip scope. Torso gravity stays off.
    internal sealed partial class PelvisDynamicReceiverIsolation
    {
        internal const string ArmB = "B_V04600_RUNTIME_SESSION_CONTEXT";
        internal const string DynamicLegArmA = "A_V04830_COHERENT_HIP_NORMAL_AGE_PREDICTED";
        internal const string DynamicLegArmB = "B_V04830_COHERENT_HIP_NORMAL_NO_AGE_PREDICTION";
        internal const string IdleArm = "OFF_V04470_PELVIS_RECEIVER";
        internal readonly PelvisReceiverReport Report = new PelvisReceiverReport();
        private readonly PelvisReceiverControlState _control = new PelvisReceiverControlState();
        internal PelvisReceiverControlStatus ControlStatus => _control.Status;
        private PelvisHipCommandFrame _hipCommand;
        internal PelvisHipCommandFrame HipCommand => _hipCommand;
        private PelvisHipAttitudeCommand _hipAttitudeCommand;
        private bool _expectHipCompletion;
        internal PelvisHipAttitudeCommand HipAttitudeCommand => _hipAttitudeCommand;
        private Rigidbody _pelvis, _chest;
        private BoxCollider _pelvisCollider, _chestCollider;
        private CharacterJoint _joint;
        private State _pelvisBaseline, _chestBaseline;
        private Vector3 _baselineChestAnchor, _baselinePelvisAnchor, _seedPelvis, _seedChest;
        private Vector3 _pelvisSize, _chestSize, _pelvisCenter, _chestCenter;
        private int _pending = -1, _angularReferenceSequence;
        // Reference limits are experiment bounds, not a human-motion certification.
        // The original reference and baseline controller keep their 1/2 Nm budgets.
        // Hip completion has a separate 8 Nm TOTAL bound, not another reference budget.
        private const float ReferenceMaxW = PelvisAngularReferencePlanner.MaxW,
            ReferenceMaxAlpha = PelvisAngularReferencePlanner.MaxAlpha, ReferenceTorqueBudget = PelvisAngularReferencePlanner.TorqueBudget;
        // Bound the pair-COM trajectory directly; keep half the existing 12m/s^2 actuator range as margin.
        private const float ReferenceMaxComV = PelvisComReferencePlanner.MaxV, ReferenceMaxComA = PelvisComReferencePlanner.MaxA;
        private PelvisComReferencePlanner.ComPlan _comReferencePlan;
        private SupportNormalRecovery.Result _lastSupportRecovery; // committed request evidence; never a control input
        private PhysicsStepContext _comReferenceStep, _comFeedbackStep;
        private Quaternion _referenceRotation = Quaternion.identity;
        private Vector3 _referenceW, _referenceCom, _referenceComV;
        private bool _referenceSeeded, _referencePending;
        private Vector3 _referenceA, _referenceAngularA;
        private float _referenceDt, _referenceStepAlphaCap;
        // Only the unselected angular policy is simulated. It never owns a body or COM reference.
        private Quaternion _comparisonRotation;
        private Vector3 _comparisonW;
        private bool _comparisonSeeded;
        private string _comparisonIssue;
        internal Quaternion ControlPelvisRotation => _referenceRotation;
        // Actual controller inputs, independent of optional CaptureTracking/trace state.
        internal bool ControlErrorsValid { get; private set; }
        internal float ControlComErrorMeters { get; private set; }
        internal float ControlPelvisErrorDegrees { get; private set; }
        private ParentHeightProbe.Input _landingHeightInput;
        internal ParentHeightProbe.Input LandingHeightInput => _landingHeightInput;
        internal struct Reference
        {
            internal Vector3 PelvisPosition, PelvisVelocity, PelvisW, ChestPosition, ChestComVelocity;
            internal Quaternion PelvisRotation, ChestRotation;
        }
        // V04500 angular integration and V04510 pair-COM bounds are retained.
        // V04520 compares the incoming end-of-step goal with a prediction at that same step.
        // Native P/C targets are reconstructed from the same COM and angular clocks.
        // No lag clipping, source-event reset or pose snap is allowed during the run.
        internal Reference BuildControlReference(Reference raw, float dt, PhysicsStepContext step,
            SupportReferenceContext support = default, float sampledPelvisY = 0f)
        {
            if (!DynamicSelected || !_control.Active) return raw;
            if (!Near(_control.NativeChestAnchor, _control.ChestComLocal))
                throw new InvalidOperationException("RECEIVER_REFERENCE_REQUIRES_CHEST_COM_PIVOT");
            if (!_referenceSeeded || !F(dt) || dt <= 0f || !Q(raw.PelvisRotation) || !Q(raw.ChestRotation) || !V(raw.PelvisW) || !F(raw.PelvisW.sqrMagnitude) ||
                !V(raw.PelvisPosition) || !V(raw.PelvisVelocity) || !Positive(_control.ReferenceAlphaCap))
                throw new InvalidOperationException("RECEIVER_ANGULAR_REFERENCE_INPUT_INVALID");
            Quaternion relative = Quaternion.Inverse(raw.PelvisRotation) * raw.ChestRotation;
            bool directional = UsesBrakingReserve(_control.Arm);
            PelvisReferenceInertiaBudget budget = default;
            if (directional)
            {
                // A and B share the same settled-native configuration and frozen 1Nm planning budget.
                if (!PelvisReferenceInertiaBudget.TryCreate(_control.PelvisInertia,
                    _pelvis.rotation * _pelvis.inertiaTensorRotation, _control.ChestInertia,
                    _chest.rotation * _chest.inertiaTensorRotation,
                    _pelvis.rotation * (_control.NativePelvisAnchor - _control.PelvisComLocal),
                    _control.PelvisMass, _control.ChestMass, out budget))
                    throw new InvalidOperationException("RECEIVER_DIRECTIONAL_REFERENCE_MODEL_INVALID");
            }
            PelvisAngularReferencePlanner.Plan plan = PelvisAngularReferencePlanner.Build(_referenceRotation, _referenceW,
                raw.PelvisRotation, raw.PelvisW, dt, _control.ReferenceAlphaCap, directional, UsesBrakingReserve(_control.Arm), budget);
            Vector3 nextW = plan.W;
            Quaternion nextRotation = plan.Rotation;
            Vector3 offset = nextRotation * NativeComOffsetLocal();
            Vector3 nativeGoalCom = raw.PelvisPosition + offset;
            Vector3 nativeGoalV = raw.PelvisVelocity + Vector3.Cross(nextW, offset);
            bool comArrival = IsDynamicHipArm(_control.Arm);
            bool feedback = comArrival && _expectHipCompletion;
            // Consume only the preceding actual completion, independently of trace.
            // No receipt exists during passive warm-up / before the bilateral join.
            if (comArrival && (!step.Valid || step.Dt != dt || _referencePending))
                throw new InvalidOperationException("RECEIVER_COM_REFERENCE_CLOCK_OR_DUPLICATE");
            if (feedback && (!_hipAttitudeCommand.Matches(_hipCommand) ||
                step.Epoch != _hipCommand.Step.Epoch || step.Sequence != _hipCommand.Step.Sequence + 1))
                throw new InvalidOperationException("RECEIVER_COM_REFERENCE_PREVIOUS_COMPLETION_INVALID");
            // Move the Hip controller's settled COM velocity read before planning.
            // No solve/body write occurs between this call and Step; both consume
            // this private value under the same complete PhysicsStepContext.
            Vector3 settledV = Vector3.zero;
            if (comArrival)
            {
                float mp = _pelvis.mass, mc = _chest.mass;
                if (!Positive(mp) || !Positive(mc) || !Positive(mp + mc))
                    throw new InvalidOperationException("RECEIVER_COM_REFERENCE_MASS_INVALID");
                settledV = (mp * _pelvis.velocity + mc * _chest.velocity) / (mp + mc);
            }
            PelvisComReferencePlanner.ComPlan comPlan = BuildComReferencePlan(_control.Arm, _referenceCom, _referenceComV,
                nativeGoalCom, nativeGoalV, dt, step, feedback,
                feedback ? _hipCommand.Acceleration : Vector3.zero,
                feedback ? _hipAttitudeCommand.TargetSystemAcceleration : Vector3.zero, settledV, support, sampledPelvisY + offset.y);
            if (LandingObservationTiming.Run.EarlyProbe && _control.Arm == DynamicHipArmB)
            {
                _landingHeightInput = new ParentHeightProbe.Input
                {
                    Step=step, Arm=_control.Arm, Position=_referenceCom, Velocity=_referenceComV,
                    Goal=nativeGoalCom, GoalV=nativeGoalV, Feedback=feedback,
                    PreviousRequest=feedback ? _hipCommand.Acceleration : Vector3.zero,
                    PreviousAllocated=feedback ? _hipAttitudeCommand.TargetSystemAcceleration : Vector3.zero,
                    SettledV=settledV, Support=support, FloorY=sampledPelvisY+offset.y,
                    CommittedPosition=comPlan.Position, CommittedV=comPlan.Velocity
                };
            }
            if (Report.TraceRunning && comArrival) Report.RecordSupportReference(comPlan, step);
            Vector3 nextComV = comPlan.Velocity, nextCom = comPlan.Position;
            Report.LastReferenceSpeedLimited = plan.SpeedLimited;
            Report.LastReferenceAlphaLimited = plan.AlphaLimited;
            Report.LastReferenceAlpha = ((nextW - _referenceW) / dt).magnitude;
            Report.LastComReferenceSpeedLimited = comPlan.SpeedLimited;
            Report.LastComReferenceAccelerationLimited = comPlan.AccelerationLimited;
            _referenceA = comPlan.Acceleration;
            _comReferencePlan = comPlan; _comReferenceStep = step;
            _comFeedbackStep = feedback ? _hipCommand.Step : new PhysicsStepContext();
            // This is the FINAL governed world-W derivative for command N, using dt_N.
            // It is authoritative control state, independent of optional report captures.
            _referenceAngularA = (nextW - _referenceW) / dt;
            _referenceDt = dt; _referenceStepAlphaCap = plan.StepCap; _referencePending = true;
            Report.LastComReferenceA = _referenceA;
            Report.LastNativeGoalCom = nativeGoalCom; Report.LastNativeGoalV = nativeGoalV;
            Report.LastComReferenceLag = (nativeGoalCom - nextCom).magnitude;
            RecordAngularReference(plan, dt, directional, CaptureReferencePair(raw, dt, budget, plan, directional));
            _referenceW = nextW; _referenceRotation = nextRotation;
            _referenceCom = nextCom; _referenceComV = nextComV;
            // Invert COM = P_origin + R_P * mass-weighted native offset.
            // Both origins and COM velocities share the governed COM and angular clocks.
            raw.PelvisPosition = nextCom - offset;
            raw.PelvisVelocity = nextComV - Vector3.Cross(nextW, offset);
            raw.PelvisRotation = nextRotation; raw.PelvisW = nextW;
            // Keep the source Chest-relative pose; its world pose/COM velocity share the governed parent.
            raw.ChestRotation = Normalize(nextRotation * relative);
            raw.ChestPosition = raw.PelvisPosition + raw.PelvisRotation * _control.NativePelvisAnchor - raw.ChestRotation * _control.NativeChestAnchor;
            raw.ChestComVelocity = raw.PelvisVelocity + Vector3.Cross(raw.PelvisW, raw.PelvisRotation * _control.NativePelvisAnchor);
            return raw;
        }
        private static Vector3 RotationVector(Quaternion target, Quaternion current)
        {
            Quaternion error = Normalize(target * Quaternion.Inverse(current));
            if (error.w < 0f) error = new Quaternion(-error.x, -error.y, -error.z, -error.w);
            error.ToAngleAxis(out float angleDeg, out Vector3 axis);
            return angleDeg > 0.0001f ? axis * (angleDeg * Mathf.Deg2Rad) : Vector3.zero;
        }
        private Vector3 NativeComOffsetLocal() => (_control.PelvisMass * _control.PelvisComLocal + _control.ChestMass * _control.NativePelvisAnchor) /
            (_control.PelvisMass + _control.ChestMass);
        private void SeedReference()
        {
            // Upper bound for rigid co-rotation about the pair COM: max(Ip)+max(Ic)+mu*d^2.
            // The articulated/native pair is not certified by this estimate; actual tracking remains measured.
            float mp = _pelvis.mass, mc = _chest.mass;
            Vector3 ip = _pelvis.inertiaTensor, ic = _chest.inertiaTensor;
            float bound = Mathf.Max(ip.x, Mathf.Max(ip.y, ip.z)) + Mathf.Max(ic.x, Mathf.Max(ic.y, ic.z)) +
                (mp * mc / (mp + mc)) * (_control.NativePelvisAnchor - _pelvis.centerOfMass).sqrMagnitude;
            if (!Positive(bound)) throw new InvalidOperationException("RECEIVER_REFERENCE_INERTIA_INVALID");
            Report.ReferenceInertiaUpper = bound; Report.ReferenceSpeedCap = ReferenceMaxW;
            Report.ReferenceAlphaCap = _control.ObserveReferenceAlphaCap(Mathf.Min(ReferenceMaxAlpha, ReferenceTorqueBudget / bound));
            Report.ReferenceTorqueBudget = ReferenceTorqueBudget;
            Report.ComReferenceSpeedCap = ReferenceMaxComV; Report.ComReferenceAccelerationCap = ReferenceMaxComA;
            _angularReferenceSequence = 0; Report.AngularReference = new PelvisReceiverReport.AngularReferenceSample();
            _referenceRotation = Normalize(_pelvis.rotation); _referenceW = Vector3.zero; _referenceSeeded = true;
            _comparisonRotation = _referenceRotation; _comparisonW = _referenceW;
            _comparisonSeeded = UsesBrakingReserve(_control.Arm);
            _comparisonIssue = null;
            _referenceCom = (mp * _pelvis.worldCenterOfMass + mc * _chest.worldCenterOfMass) / (mp + mc);
            _referenceComV = _referenceA = _referenceAngularA = Vector3.zero; _referencePending = false; _referenceDt = _referenceStepAlphaCap = 0f;
            _comReferencePlan = new PelvisComReferencePlanner.ComPlan();
            _lastSupportRecovery = default;
            _comReferenceStep = _comFeedbackStep = new PhysicsStepContext();
        }
        private void ResetReference()
        {
            _lastSupportRecovery = default;
            ControlErrorsValid = false;
            _referencePending = false; _referenceA = _referenceAngularA = Vector3.zero; _referenceDt = _referenceStepAlphaCap = 0f;
            _referenceSeeded = _comparisonSeeded = false; _referenceRotation = _comparisonRotation = Quaternion.identity;
            _referenceW = _referenceCom = _referenceComV = _comparisonW = Vector3.zero;
            _comparisonIssue = null;
            _comReferencePlan = new PelvisComReferencePlanner.ComPlan();
            _comReferenceStep = _comFeedbackStep = new PhysicsStepContext();
        }
        private static Quaternion Normalize(Quaternion q)
        {
            float n = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!Positive(n)) throw new InvalidOperationException("RECEIVER_REFERENCE_QUATERNION_INVALID");
            return new Quaternion(q.x / n, q.y / n, q.z / n, q.w / n);
        }
        internal PelvisReceiverReport.TrackingFrame CaptureAnimationComparison(PelvisReceiverReport.TrackingFrame f,
            Reference control, Reference raw, Vector3 sourceP, Quaternion sourcePQ, Vector3 sourceC, Quaternion sourceCQ)
        {
            if (!f.Enabled || !f.Valid) return f;
            try
            {
                float mp = _control.PelvisMass, mc = _control.ChestMass, m = mp + mc;
                Vector3 controlPCom = control.PelvisPosition + control.PelvisRotation * _control.PelvisComLocal;
                Vector3 controlCCom = control.ChestPosition + control.ChestRotation * _control.ChestComLocal;
                Vector3 sourceComShift = (mp * (sourceP + sourcePQ * _control.PelvisComLocal - controlPCom) +
                    mc * (sourceC + sourceCQ * _control.ChestComLocal - controlCCom)) / m;
                f.RawRenderComError = f.ComError + sourceComShift;
                f.RawRenderPosition = f.RawRenderComError.magnitude;
                f.RawRenderRotation = Quaternion.Angle(sourcePQ, f.BodyPelvisRotation);
                f.RenderReferenceErrorVector = RotationVector(sourcePQ, control.PelvisRotation);
                f.RawRotationErrorVector = RotationVector(sourcePQ, f.BodyPelvisRotation);
                f.ReferencePair = Report.ReferencePair;
                if (f.ReferencePair.Valid)
                {
                    f.ReferencePair.RawErrorA = RotationVector(sourcePQ, f.ReferencePair.RotationA);
                    f.ReferencePair.RawErrorB = RotationVector(sourcePQ, f.ReferencePair.RotationB);
                    f.ReferencePair.RawSourceCaptured = true;
                }
                f.ReferenceRotationErrorVector = RotationVector(raw.PelvisRotation, control.PelvisRotation);
                Vector3 bridgeComShift = (mp * (raw.PelvisPosition + raw.PelvisRotation * _control.PelvisComLocal - controlPCom) +
                    mc * (raw.ChestPosition + raw.ChestRotation * _control.ChestComLocal - controlCCom)) / m;
                f.RawBridgePosition = (f.ComError + bridgeComShift).magnitude;
                f.RawChestPosition = (sourceC - f.BodyChestPosition).magnitude;
                f.ReferenceShift = (control.ChestPosition - raw.ChestPosition).magnitude;
                f.RawTargetNativeGap = ((raw.ChestPosition + raw.ChestRotation * _control.NativeChestAnchor) -
                    (raw.PelvisPosition + raw.PelvisRotation * _control.NativePelvisAnchor)).magnitude;
                Vector3 bridgeVShift = (mp * (raw.PelvisVelocity + Vector3.Cross(raw.PelvisW, raw.PelvisRotation * _control.PelvisComLocal) -
                    control.PelvisVelocity - Vector3.Cross(control.PelvisW, control.PelvisRotation * _control.PelvisComLocal)) +
                    mc * (raw.ChestComVelocity - control.ChestComVelocity)) / m;
                f.RawVelocityError = (f.ComVelocityError + bridgeVShift).magnitude;
                f.ReferenceAngularLag = Quaternion.Angle(raw.PelvisRotation, control.PelvisRotation);
                f.ReferenceAlpha = DynamicSelected ? Report.LastReferenceAlpha : 0f;
                f.ReferenceSpeedLimited = DynamicSelected && Report.LastReferenceSpeedLimited;
                f.ReferenceAlphaLimited = DynamicSelected && Report.LastReferenceAlphaLimited;
                f.ComReferenceLag = DynamicSelected ? Report.LastComReferenceLag : 0f;
                f.ComReferenceA = DynamicSelected ? Report.LastComReferenceA : Vector3.zero;
                f.NativeGoalCom = DynamicSelected ? Report.LastNativeGoalCom : f.CommandCom;
                f.NativeGoalV = DynamicSelected ? Report.LastNativeGoalV : f.CommandComV;
                f.ComReferenceSpeedLimited = DynamicSelected && Report.LastComReferenceSpeedLimited;
                f.ComReferenceAccelerationLimited = DynamicSelected && Report.LastComReferenceAccelerationLimited;
                f.Projected = DynamicSelected; f.AngularComparisonCaptured = true;
                f.Valid = F(f.RawRenderPosition) && F(f.RawRenderRotation) && F(f.RawBridgePosition) && F(f.RawChestPosition) &&
                    F(f.ReferenceShift) && F(f.RawTargetNativeGap) && F(f.RawVelocityError) && F(f.ReferenceAngularLag) && F(f.ReferenceAlpha) &&
                    F(f.ComReferenceLag) && V(f.ComReferenceA) && V(f.NativeGoalCom) && V(f.NativeGoalV) && V(f.RawRotationErrorVector) && V(f.ReferenceRotationErrorVector);
                if (!f.Valid) f.Issue = "TRACKING_RAW_REFERENCE_NONFINITE";
            }
            catch (Exception e) { f.Valid = false; f.Issue = "TRACKING_RAW_REFERENCE_EXCEPTION:" + e.GetType().Name; }
            return f;
        }
        private struct State
        {
            internal int Id;
            internal float Mass, Drag, AngularDrag, MaxW;
            internal Vector3 Com, Inertia;
            internal Quaternion InertiaRotation;
            internal RigidbodyConstraints Constraints;
            internal CollisionDetectionMode CollisionMode;
        }
        internal bool DynamicSelected => _control.DynamicSelected;
        internal bool ReadyForRun => _control.ReadyForRun;
        internal bool SafeOff
        {
            get
            {
                try { return !_control.Selected && !_control.Active && _control.BaselineRestored && ((!_control.BaselineCaptured && _control.Finite) ||
                    (_control.BaselineCaptured && _pelvis != null && _chest != null && _pelvis.isKinematic && _chest.isKinematic && !_pelvis.useGravity && !_chest.useGravity &&
                     BaselineMatches(_pelvis, _pelvisBaseline) && BaselineMatches(_chest, _chestBaseline) && ParametersMatch())); }
                catch { return false; }
            }
        }
        // V04710 read-only Foot attribution uses the retained V04670 braking reference.
        internal const string DynamicContactArm = "B_V04830_DYNAMIC_UNLOADED_STANCE_CONTACT";
        internal const string DynamicFootLoadArm = "B_V04840_DYNAMIC_FOOT_SELF_WEIGHT_CONTACT";
        internal const string DynamicLowerLoadArm = "B_V04860_DYNAMIC_LOWER_SELF_WEIGHT_LEDGER_AUDIT";
        internal const string DynamicKneeCouplingArm = "B_V04870_DYNAMIC_KNEE_NORMAL_COUPLING";
        // v105 OFF arm did not establish net benefit; only its A physical policy is
        // the current baseline. v105 remains the reproducible comparison artifact.
        internal static string ReferencePreparationName(string arm) => IsDynamicHipArm(arm) ? "EARLY_STRONG_TO_STANCE_OR_CONFIRMED" : "OFF_OTHER_SCOPE";
        internal static string NormalBudgetName(string arm) => IsDynamicHipArm(arm) ? "NORMAL_PRIORITY_KEEP_TANGENT_BRAKING" : "OFF_OTHER_SCOPE";
        internal static string NormalFeedbackName(string arm) => IsDynamicHipArm(arm) ? "Y_KP40_KD12.727922" : "BASELINE_OTHER_SCOPE";
        internal static string HipReactionName(string arm) => IsDynamicHipArm(arm) ? "RELATIVE_CONTROL_ONLY" : "OFF";
        internal static string HipNormalParentName(string arm) => IsDynamicHipArm(arm) ? "RECEIVER_TARGET_NORMAL_ONLY" : "PHYSICAL_CONTROL_PARENT";
        internal const string DynamicUpperLoadArm = "B_V04880_DYNAMIC_UPPER_SELF_WEIGHT";
        internal static bool IsDynamicLegArm(string arm) => IsDynamicHipArm(arm) || arm == DynamicUpperLoadArm || arm == DynamicLegArmA || arm == DynamicLegArmB || arm == DynamicContactArm || arm == DynamicFootLoadArm || arm == DynamicLowerLoadArm || arm == DynamicKneeCouplingArm;
        private static bool UsesBrakingReserve(string arm) => IsDynamicLegArm(arm);
        internal static bool IsExperimentArm(string arm) => arm == ArmB || IsDynamicLegArm(arm);
        internal string GetNativeState() { TryGetNativeState(out string state); return state; }
        internal bool TryGetNativeState(out string state)
        {
            try
            {
                state = "nativeReceiver(pelvis-chest)=" + Native(_pelvis, _pelvisCollider) + "|" + Native(_chest, _chestCollider) +
                    "/jointParent=" + (_joint != null && _joint.connectedBody == _pelvis) + "/nativeAnchorGapMm=" +
                    (NativeAnchorGap() * 1000f).ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
                return _pelvis != null && _chest != null && _pelvisCollider != null && _chestCollider != null && _joint != null;
            }
            catch (Exception e) { state = "nativeReceiverReadFailed=" + e.GetType().Name; return false; }
        }
        private static string Native(Rigidbody b, BoxCollider c) => b == null || c == null ? "<none>" :
            "id=" + b.GetInstanceID() + "/mass=" + b.mass + "/kin-g-collider=" + b.isKinematic + "-" + b.useGravity + "-" + c.enabled +
            "/drag-angularDrag-maxW=" + b.drag + "-" + b.angularDrag + "-" + b.maxAngularVelocity +
            "/constraints-ccd=" + b.constraints + "-" + b.collisionDetectionMode + "/tensorRotation=" + b.inertiaTensorRotation;

        internal bool Select(string arm, Rigidbody pelvis, Rigidbody chest, BoxCollider pelvisCollider, BoxCollider chestCollider, CharacterJoint joint)
        {
            if ((!IsExperimentArm(arm) && arm != IdleArm) || _control.Active) return false;
            ResetHipCommands();
            try
            {
                // OFF must restore even after source loss or a failed native-state gate.
                if (arm == IdleArm && _control.BaselineCaptured)
                {
                    EndTrace(); Report.Active = _control.ObserveActive(false);
                    bool restored = RestoreBaseline();
                    Report.Ready = _control.ObserveReady(false); Report.Dynamic = _control.ObserveDynamic(false); Report.Selected = _control.ObserveSelected(false); Report.Arm = _control.ObserveArm(IdleArm);
                    return restored;
                }
                if (pelvis == null || chest == null || pelvis == chest || pelvisCollider == null || chestCollider == null || joint == null ||
                    !pelvis.isKinematic || !chest.isKinematic || pelvis.useGravity || chest.useGravity || pelvisCollider.enabled || chestCollider.enabled ||
                    joint.connectedBody != pelvis || joint.autoConfigureConnectedAnchor)
                    return Invalid("RECEIVER_SELECT_REQUIRES_SAFE_OWNED_KINEMATIC_PAIR");
                if (!_control.BaselineCaptured)
                {
                    _pelvis = pelvis; _chest = chest; _pelvisCollider = pelvisCollider; _chestCollider = chestCollider; _joint = joint;
                    _pelvisBaseline = Capture(pelvis); _chestBaseline = Capture(chest);
                    _baselineChestAnchor = joint.anchor; _baselinePelvisAnchor = joint.connectedAnchor;
                    _pelvisSize = pelvisCollider.size; _chestSize = chestCollider.size; _pelvisCenter = pelvisCollider.center; _chestCenter = chestCollider.center;
                    if (!StateFinite(_pelvisBaseline) || !StateFinite(_chestBaseline) || !V(_baselineChestAnchor) || !V(_baselinePelvisAnchor))
                        return Invalid("RECEIVER_BASELINE_PARAMETERS_INVALID");
                    Report.BaselineCaptured = _control.ObserveBaselineCaptured(true);
                }
                if (pelvis != _pelvis || chest != _chest || joint != _joint || pelvisCollider != _pelvisCollider || chestCollider != _chestCollider)
                    return Invalid("RECEIVER_SELECT_OWNER_CHANGED");
                EndTrace(); if (!RestoreBaseline()) return false;
                if (IsExperimentArm(arm))
                {
                    ApplyBox(_pelvis, _pelvisCollider, _pelvisBaseline.Mass); ApplyBox(_chest, _chestCollider, _chestBaseline.Mass);
                    // Native anchors are computed in Rigidbody pose space. Source/control virtual anchors stay unchanged.
                    _joint.anchor = _chest.centerOfMass;
                    _joint.connectedAnchor = Quaternion.Inverse(_pelvis.rotation) * (_chest.worldCenterOfMass - _pelvis.position);
                    Report.BaselineRestored = _control.ObserveBaselineRestored(false);
                }
                Report.Arm = _control.ObserveArm(arm); Report.Selected = _control.ObserveSelected(IsExperimentArm(arm)); Report.Dynamic = _control.ObserveDynamic(IsExperimentArm(arm));
                Report.ReferenceBudgetMode = arm == ArmB ? "FROZEN_SCALAR_WORST_AXIS_V04660" : "DIRECTIONAL_VECTOR_INERTIA_V04660";
                Report.ReferencePlannerMode = UsesBrakingReserve(arm) ? "BRAKING_RESERVE_V04670" : "LEGACY_SCALAR_BASELINE";
                Report.Finite = _control.ObserveFinite(true); Report.Issue = null; Report.Quiesced = false; Report.Ready = _control.ObserveReady(false);
                Report.PelvisMass = _control.ObservePelvisMass(_pelvis.mass); Report.ChestMass = _control.ObserveChestMass(_chest.mass);
                Report.PelvisComLocal = _control.ObservePelvisComLocal(_pelvis.centerOfMass); Report.ChestComLocal = _control.ObserveChestComLocal(_chest.centerOfMass);
                Report.PelvisInertia = _control.ObservePelvisInertia(_pelvis.inertiaTensor); Report.ChestInertia = _control.ObserveChestInertia(_chest.inertiaTensor);
                Report.NativeChestAnchor = _control.ObserveNativeChestAnchor(_joint.anchor); Report.NativePelvisAnchor = _control.ObserveNativePelvisAnchor(_joint.connectedAnchor);
                if (!ParametersMatch()) throw new InvalidOperationException("RECEIVER_SELECTED_PARAMETER_READBACK_MISMATCH");
                return true;
            }
            catch (Exception e)
            {
                Invalid("RECEIVER_SELECT_EXCEPTION:" + e.GetType().Name);
                if (_control.BaselineCaptured) RestoreBaseline();
                return false;
            }
        }
        internal bool BeginDrive()
        {
            ResetHipCommands();
            HipCompletionIssue = null;
            _lastSupportRecovery = default;
            if (!_control.Selected) return true;
            try
            {
                Report.Active = _control.ObserveActive(true);
                if (_control.Dynamic)
                {
                    _joint.anchor = _chest.centerOfMass;
                    _joint.connectedAnchor = Quaternion.Inverse(_pelvis.rotation) * (_chest.worldCenterOfMass - _pelvis.position);
                    Report.NativeChestAnchor = _control.ObserveNativeChestAnchor(_joint.anchor); Report.NativePelvisAnchor = _control.ObserveNativePelvisAnchor(_joint.connectedAnchor);
                    SeedReference();
                }
                else ResetReference();
                if (!_control.Finite || !ParametersMatch() || !PoseDomainsMatch(_pelvis) || !PoseDomainsMatch(_chest))
                    return Invalid("RECEIVER_PRE_DYNAMIC_POSE_OR_PARAMETER_INVALID");
                Report.Active = _control.ObserveActive(true); Report.BaselineRestored = _control.ObserveBaselineRestored(false); Report.Quiesced = false; Report.PassiveIssued = Report.PassiveResolved = _control.ResetPassiveQualification();
                Report.PassiveAdvancePeak = 0f; Report.Ready = _control.ObserveReady(!_control.Dynamic);
                _seedPelvis = _pelvis.position; _seedChest = _chest.position;
                if (_control.Dynamic)
                {
                    _pelvis.isKinematic = false; _chest.isKinematic = false;
                    _pelvis.velocity = _chest.velocity = Vector3.zero; _pelvis.angularVelocity = _chest.angularVelocity = Vector3.zero;
                }
                if (!BodyStateMatch()) return Invalid("RECEIVER_POST_DYNAMIC_STATE_READBACK_MISMATCH");
                return true;
            }
            catch (Exception e) { return Invalid("RECEIVER_BEGIN_DRIVE_EXCEPTION:" + e.GetType().Name); }
        }
        internal bool Step(bool trace, string phase, Vector3 targetPelvisPosition, Quaternion targetPelvisRotation,
            Vector3 targetPelvisV, Vector3 targetW, Vector3 targetChestPosition, Quaternion targetChestRotation, Vector3 targetChestV,
            float dt, float positionSafetyMeters, PhysicsStepContext step)
        {
            // Preparation can publish a receiver command before the legs join.
            // Once bilateral completion has begun, missing/stale completion is a
            // live contract failure even with trace OFF, never a zero correction.
            if (_expectHipCompletion && _hipCommand.Committed &&
                (!_hipAttitudeCommand.Matches(_hipCommand) || !step.Valid ||
                 step.Epoch != _hipCommand.Step.Epoch || step.Sequence != _hipCommand.Step.Sequence + 1))
                return Invalid("RECEIVER_HIP_ATTITUDE_PREVIOUS_COMPLETION_OR_CLOCK_INVALID");
            _hipCommand = new PelvisHipCommandFrame();
            _hipAttitudeCommand = new PelvisHipAttitudeCommand();
            HipCompletionIssue = null;
            _lastSupportRecovery = default;
            ControlErrorsValid = false;
            if (!_control.Active || !DynamicSelected) return false;
            try
            {
                ResolvePending(step);
                if (!_control.Finite || !BodyStateMatch() || !F(dt) || dt <= 0f || !V(targetPelvisPosition) || !Q(targetPelvisRotation) ||
                    !V(targetPelvisV) || !V(targetW) || !V(targetChestPosition) || !Q(targetChestRotation) || !V(targetChestV))
                    return Invalid("RECEIVER_CURRENT_INPUT_OR_NATIVE_STATE_INVALID");
                bool publishCommon = IsDynamicHipArm(_control.Arm);
                if (publishCommon && (!step.Valid || step.Dt != dt)) return Invalid("RECEIVER_HIP_COMMAND_CONTEXT_INVALID");
                int commonBodyId = publishCommon ? _pelvis.GetInstanceID() : 0;
                if (publishCommon && commonBodyId == 0) return Invalid("RECEIVER_HIP_COMMAND_BODY_INVALID");
                int chestBodyId = publishCommon ? _chest.GetInstanceID() : 0;
                if (publishCommon && (chestBodyId == 0 || chestBodyId == commonBodyId)) return Invalid("RECEIVER_HIP_CHEST_BODY_INVALID");
                // One governed Fixed-step reference per force step; never reuse stale diagnostic data.
                if (!_referencePending || _referenceDt != dt || (IsDynamicHipArm(_control.Arm) && !_comReferenceStep.Matches(step)) ||
                    !V(_referenceA) || !F(_referenceA.sqrMagnitude) ||
                    _referenceA.magnitude > ReferenceMaxComA + 0.001f)
                    return Invalid("RECEIVER_COM_REFERENCE_NOT_CURRENT_OR_INVALID");
                float referenceCeiling = UsesBrakingReserve(_control.Arm) ? PelvisAngularReferencePlanner.MaxBrakeAlpha : ReferenceMaxAlpha;
                if (!Positive(_control.ReferenceAlphaCap) || !Positive(_referenceStepAlphaCap) || _referenceStepAlphaCap > referenceCeiling ||
                    !V(_referenceAngularA) || !F(_referenceAngularA.sqrMagnitude) ||
                    _referenceAngularA.magnitude > _referenceStepAlphaCap + 0.001f)
                    return Invalid("RECEIVER_ANGULAR_REFERENCE_NOT_CURRENT_OR_INVALID");
                _referencePending = false;
                if (!_control.Ready)
                {
                    if (_control.PassiveIssued > _control.PassiveResolved)
                    {
                        float advance = Mathf.Max((_pelvis.position - _seedPelvis).magnitude, (_chest.position - _seedChest).magnitude);
                        Report.PassiveAdvancePeak = Mathf.Max(Report.PassiveAdvancePeak, advance);
                        if (!F(advance) || advance > 0.03f || !PoseDomainsMatch(_pelvis) || !PoseDomainsMatch(_chest) || NativeAnchorGap() > 0.03f)
                            return Invalid("RECEIVER_PASSIVE_SOLVE_POSE_OR_ANCHOR_FAILED");
                        Report.PassiveResolved = _control.ResolvePassiveStep();
                    }
                    if (_control.PassiveResolved < 2) { Report.PassiveIssued = _control.IssuePassiveStep(); return true; }
                    Report.Ready = _control.ObserveReady(true);
                }
                Vector3 pCom = _pelvis.worldCenterOfMass, cCom = _chest.worldCenterOfMass;
                float totalMass = _pelvis.mass + _chest.mass;
                Vector3 actualCom = (_pelvis.mass * pCom + _chest.mass * cCom) / totalMass;
                Vector3 actualV = publishCommon ? _comReferencePlan.SettledVelocity :
                    (_pelvis.mass * _pelvis.velocity + _chest.mass * _chest.velocity) / totalMass;
                Vector3 pTargetCom = targetPelvisPosition + targetPelvisRotation * _pelvis.centerOfMass;
                Vector3 cTargetCom = targetChestPosition + targetChestRotation * _chest.centerOfMass;
                Vector3 targetCom = (_pelvis.mass * pTargetCom + _chest.mass * cTargetCom) / totalMass;
                Vector3 pTargetComV = targetPelvisV + Vector3.Cross(targetW, targetPelvisRotation * _pelvis.centerOfMass);
                Vector3 targetComV = (_pelvis.mass * pTargetComV + _chest.mass * targetChestV) / totalMass;
                Vector3 error = targetCom - actualCom;
                if (!V(error) || error.magnitude > positionSafetyMeters) return Invalid("RECEIVER_COM_POSITION_SAFETY_GATE");
                Vector3 pdA = PelvisComFeedback.Evaluate(error, targetComV - actualV, IsDynamicHipArm(_control.Arm));
                // V04530 B is frozen: same governed COM A before the actuator cap.
                Vector3 feedforwardA = _referenceA;
                Vector3 rawA = pdA + feedforwardA;
                SupportNormalRecovery.Result recovery = BuildCommonAcceleration(_control.Arm, rawA,
                    actualCom, actualV, step, _comReferencePlan);
                Vector3 commonA = recovery.Request;
                Quaternion rotationError = targetPelvisRotation * Quaternion.Inverse(_pelvis.rotation);
                if (rotationError.w < 0f) rotationError = new Quaternion(-rotationError.x, -rotationError.y, -rotationError.z, -rotationError.w);
                rotationError.ToAngleAxis(out float angleDeg, out Vector3 axis);
                if (angleDeg > 180f) angleDeg -= 360f;
                Vector3 rotationVector = Mathf.Abs(angleDeg) < 0.0001f ? Vector3.zero : axis.normalized * (angleDeg * Mathf.Deg2Rad);
                Vector3 rawAlpha = 55f * rotationVector + 12f * (targetW - _pelvis.angularVelocity);
                Vector3 pdAlpha = rawAlpha;
                Vector3 feedforwardAlpha = _referenceAngularA;
                // Sum the bounded reference AFF once before the existing alpha/total-torque caps.
                rawAlpha += feedforwardAlpha;
                Vector3 alpha = Vector3.ClampMagnitude(rawAlpha, 40f);
                Quaternion principalWorld = _pelvis.rotation * _pelvis.inertiaTensorRotation;
                Vector3 inertiaAlpha = MultiplyInertia(principalWorld, _pelvis.inertiaTensor, alpha);
                Vector3 gyro = Vector3.Cross(_pelvis.angularVelocity, MultiplyInertia(principalWorld, _pelvis.inertiaTensor, _pelvis.angularVelocity));
                // Free P/C pair with the native joint at Chest COM: moving that COM adds
                // I_orbit = mu * (|r|^2 Identity - r r^T), mu = mp*mc/(mp+mc).
                // r is the CURRENT native parent anchor minus Pelvis COM, never a target/virtual lever.
                // Chest spin remains independently driven. Its own inertia is not rigidly welded to Pelvis.
                float reducedMass = _pelvis.mass * _chest.mass / totalMass;
                Vector3 parentLever = _pelvis.rotation * (_control.NativePelvisAnchor - _pelvis.centerOfMass);
                if (!Positive(reducedMass) || !V(parentLever) || !F(parentLever.sqrMagnitude))
                    return Invalid("RECEIVER_ANGULAR_GEOMETRY_INVALID");
                // V04560/V04570 B inertia and AFF remain the fixed actuator baseline.
                Vector3 orbitalAlpha = MultiplyOrbitalInertia(parentLever, reducedMass, alpha);
                Vector3 orbitalGyro = Vector3.Cross(_pelvis.angularVelocity,
                    MultiplyOrbitalInertia(parentLever, reducedMass, _pelvis.angularVelocity));
                Vector3 rawTorque = inertiaAlpha + gyro;
                rawTorque += orbitalAlpha + orbitalGyro;
                Vector3 torque = Vector3.ClampMagnitude(rawTorque, 2f);
                Vector3 pForce = _pelvis.mass * commonA, cForce = _chest.mass * commonA;
                var parentResponse = new PelvisChestPointResponse();
                Vector3 parentW = publishCommon ? _pelvis.angularVelocity : Vector3.zero;
                if (publishCommon && (!V(parentW) || !PelvisChestPointResponse.TryCreate(_pelvis.mass, _chest.mass, _pelvis.inertiaTensor,
                    principalWorld, parentLever, out parentResponse))) return Invalid("RECEIVER_HIP_PARENT_RESPONSE_INVALID");
                if (!V(rawA) || !F(rawA.sqrMagnitude) || !V(rawAlpha) || !F(rawAlpha.sqrMagnitude) || !V(rawTorque) || !F(rawTorque.sqrMagnitude) ||
                    !V(commonA) || !V(torque) || !V(pForce) || !V(cForce)) return Invalid("RECEIVER_COMMAND_NONFINITE");
                Vector3 preP = _pelvis.velocity, preC = _chest.velocity;
                int written = 0;
                try
                {
                    _pelvis.AddForce(pForce, ForceMode.Force); written = 1;
                    _chest.AddForce(cForce, ForceMode.Force); written = 2;
                    _pelvis.AddTorque(torque, ForceMode.Force); written = 3;
                }
                catch
                {
                    if (written >= 3) TryRestore(() => _pelvis.AddTorque(-torque, ForceMode.Force));
                    if (written >= 2) TryRestore(() => _chest.AddForce(-cForce, ForceMode.Force));
                    if (written >= 1) TryRestore(() => _pelvis.AddForce(-pForce, ForceMode.Force));
                    throw;
                }
                ControlComErrorMeters = error.magnitude; ControlPelvisErrorDegrees = Mathf.Abs(angleDeg);
                _lastSupportRecovery = recovery;
                ControlErrorsValid = F(ControlComErrorMeters) && F(ControlPelvisErrorDegrees);
                Report.CommonA = commonA; Report.PelvisTorque = torque;
                if (publishCommon) _hipCommand = new PelvisHipCommandFrame
                    { JointTorqueReaction = IsDynamicHipArm(_control.Arm), Committed = true, Step = step, BodyId = commonBodyId, ChestBodyId = chestBodyId,
                        Acceleration = commonA, ReceiverTorque = torque, ParentW = parentW, Response = parentResponse,
                        UpwardRetentionSelected = IsDynamicHipArm(_control.Arm),
                        UpwardRetentionEligible = HipSystemAllocation.Eligible(IsDynamicHipArm(_control.Arm), step,
                            _comReferenceStep, _comReferencePlan, error.y, commonA),
                        BoundedTradeSelected = IsDynamicHipArm(_control.Arm),
                        ReferenceRotation = targetPelvisRotation, ReferenceW = targetW };
                if (publishCommon && LandingObservationTiming.Run.EarlyProbe && _landingHeightInput.Step.Matches(step))
                {
                    _landingHeightInput.ReceiverReady=true; _landingHeightInput.ActualCom=actualCom;
                    _landingHeightInput.PositionError=error; _landingHeightInput.VelocityError=targetComV-actualV;
                    _landingHeightInput.PositionLimit=positionSafetyMeters;
                }
                Report.LastPdA = pdA; Report.LastFeedforwardA = feedforwardA;
                if (trace)
                {
                    if (publishCommon) Report.RecordSupportRecovery(recovery, step);
                    CountStep(phase); Report.ForcePairWrites++; Report.TorqueWrites++;
                    RecordAngularActuation(step, rotationVector * 55f, pdAlpha - rotationVector * 55f, feedforwardAlpha, alpha, inertiaAlpha, gyro,
                        reducedMass, parentLever, orbitalAlpha, orbitalGyro, rawTorque, torque);
                    Report.LastRawCommonA = rawA.magnitude; Report.LastRawPelvisAlpha = rawAlpha.magnitude; Report.LastRawPelvisTorque = rawTorque.magnitude;
                    Report.LastLinearClamped = rawA.magnitude > PelvisHipCommandFrame.MaxMagnitude; Report.LastAngularClamped = rawAlpha.magnitude > 40f; Report.LastTorqueClamped = rawTorque.magnitude > 2f;
                    if (rawA.magnitude > PelvisHipCommandFrame.MaxMagnitude) Report.LinearClamps++;
                    if (rawAlpha.magnitude > 40f) Report.AngularClamps++;
                    if (rawTorque.magnitude > 2f) Report.TorqueClamps++;
                    Report.PositionErrorPeak = Mathf.Max(Report.PositionErrorPeak, error.magnitude);
                    Report.RotationErrorPeak = Mathf.Max(Report.RotationErrorPeak, Mathf.Abs(angleDeg));
                    Report.ForceSumPeak = Mathf.Max(Report.ForceSumPeak, pForce.magnitude + cForce.magnitude);
                    Report.TorquePeak = Mathf.Max(Report.TorquePeak, torque.magnitude);
                    CaptureReceiverTraceInput(step, dt, preP, preC, pForce, cForce, torque);
                }
                return true;
            }
            catch (Exception e) { return Invalid("RECEIVER_STEP_EXCEPTION:" + e.GetType().Name); }
        }
        // Capture the settled native pair before the next controller write.
        // Commit after the existing control step to attach its actual clamp/source labels.
        // A diagnostic failure marks only tracking coverage; it never changes native state or drive eligibility.
        internal PelvisReceiverReport.TrackingFrame CaptureTracking(bool trace, Vector3 pPosition, Quaternion pRotation,
            Vector3 pVelocity, Vector3 pW, Vector3 cPosition, Quaternion cRotation, Vector3 cVelocity,
            Vector3 chestVirtualAnchor, Vector3 pelvisVirtualAnchor, float dt)
        {
            var f = new PelvisReceiverReport.TrackingFrame();
            if (!trace || !Report.TraceRunning || !_control.Active || !_control.Selected || !_control.Ready) return f;
            f.Enabled = true; f.Fixed = Report.CurrentTraceFixed + 1; f.Dt = dt; f.Dynamic = _control.Dynamic;
            try
            {
                float mp = _pelvis.mass, mc = _chest.mass, m = mp + mc;
                Vector3 pc = _pelvis.centerOfMass, cc = _chest.centerOfMass;
                Vector3 pp = _pelvis.position, cp = _chest.position;
                Quaternion pq = _pelvis.rotation, cq = _chest.rotation;
                Vector3 nativeChest = _joint.anchor, nativePelvis = _joint.connectedAnchor;
                Vector3 actualCom = (mp * _pelvis.worldCenterOfMass + mc * _chest.worldCenterOfMass) / m;
                Vector3 targetCom = (mp * (pPosition + pRotation * pc) + mc * (cPosition + cRotation * cc)) / m;
                Vector3 actualV = (mp * _pelvis.velocity + mc * _chest.velocity) / m;
                Vector3 targetV = (mp * (pVelocity + Vector3.Cross(pW, pRotation * pc)) + mc * cVelocity) / m;
                f.CommandCom = targetCom; f.CommandComV = targetV;
                f.ComReference = _comReferencePlan;
                f.ComReferenceStep = _comReferenceStep; f.ComFeedbackStep = _comFeedbackStep;
                Vector3 virtualChest = cp + cq * chestVirtualAnchor, virtualPelvis = pp + pq * pelvisVirtualAnchor;
                f.Position = (targetCom - actualCom).magnitude; f.Rotation = Quaternion.Angle(pRotation, pq);
                f.NativeGap = ((cp + cq * nativeChest) - (pp + pq * nativePelvis)).magnitude;
                f.TargetNativeGap = ((cPosition + cRotation * nativeChest) - (pPosition + pRotation * nativePelvis)).magnitude;
                f.VirtualGap = (virtualChest - virtualPelvis).magnitude;
                f.VelocityError = (targetV - actualV).magnitude; f.CommandW = pW;
                f.ComError = targetCom - actualCom; f.ComVelocityError = targetV - actualV;
                f.BodyChestPosition = cp; f.BodyPelvisRotation = pq;
                f.BodyW = _pelvis.angularVelocity; f.RotationErrorVector = RotationVector(pRotation, pq);
                // Direct helper users default to identical raw/control references; FixedUpdate attaches the real animation comparison.
                f.RawRenderComError = f.ComError;
                f.RawRenderPosition = f.RawBridgePosition = f.Position; f.RawRenderRotation = f.Rotation;
                f.RawRotationErrorVector = f.RotationErrorVector;
                f.RawTargetNativeGap = f.TargetNativeGap; f.RawVelocityError = f.VelocityError;
                f.Valid = F(dt) && dt > 0f && V(pW) && F(f.Position) && F(f.Rotation) && F(f.NativeGap) && F(f.TargetNativeGap) && F(f.VirtualGap) && F(f.VelocityError) && V(f.BodyW) && V(f.RotationErrorVector);
                if (!f.Valid) f.Issue = "TRACKING_READBACK_NONFINITE";
                if (f.Valid) f = CaptureAngularBodies(f, cRotation, cq);
            }
            catch (Exception e) { f.Issue = "TRACKING_READBACK_EXCEPTION:" + e.GetType().Name; }
            return f;
        }
        // All additions below are diagnostic-only; failed evidence never changes a force or gate.
        private PelvisReferencePairReport.Sample CaptureReferencePair(Reference raw, float dt,
            PelvisReferenceInertiaBudget budget, PelvisAngularReferencePlanner.Plan applied, bool directional)
        {
            if (!directional) return default;
            if (!_comparisonSeeded) { Report.ReferencePairIssue = _comparisonIssue; return default; }
            try
            {
                bool reserveApplied = UsesBrakingReserve(_control.Arm);
                var other = PelvisAngularReferencePlanner.Build(_comparisonRotation, _comparisonW, raw.PelvisRotation,
                    raw.PelvisW, dt, _control.ReferenceAlphaCap, true, !reserveApplied, budget);
                var a = reserveApplied ? other : applied;
                var b = reserveApplied ? applied : other;
                _comparisonRotation = other.Rotation; _comparisonW = other.W;
                return new PelvisReferencePairReport.Sample { Valid = true, Dt = dt, AppliedB = reserveApplied,
                    RotationA = a.Rotation, RotationB = b.Rotation, WA = a.W, WB = b.W,
                    AlphaA = a.Alpha.magnitude, AlphaB = b.Alpha.magnitude,
                    ModelTorqueA = a.InertiaTorque.magnitude, ModelTorqueB = b.InertiaTorque.magnitude,
                    BridgeErrorA = RotationVector(raw.PelvisRotation, a.Rotation), BridgeErrorB = RotationVector(raw.PelvisRotation, b.Rotation),
                    ExtraBrakeAlphaB = b.ExtraBrakeAlpha };
            }
            catch (Exception e)
            {
                _comparisonSeeded = false;
                _comparisonIssue = "ALTERNATIVE_PLANNER_EXCEPTION:" + e.GetType().Name;
                Report.ReferencePairIssue = _comparisonIssue;
                return default;
            }
        }
        private void RecordAngularReference(PelvisAngularReferencePlanner.Plan p, float dt, bool directional,
            PelvisReferencePairReport.Sample pair)
        {
            try
            {
                Report.AngularReference = new PelvisReceiverReport.AngularReferenceSample { Valid = true,
                    Sequence = ++_angularReferenceSequence, Dt = dt, ErrorAngle = p.ErrorAngle,
                    BridgeW = p.BridgeW, CorrectionW = p.DesiredW - p.BridgeW, DesiredW = p.DesiredW, LimitedW = p.LimitedW,
                    PreviousW = p.PreviousW, CommandW = p.W, GeneratedAlpha = p.Alpha,
                    ArrivalBraking = true, RelativeW = p.RelativeW, StoppingOffset = p.StoppingOffset, ArrivalError = p.ArrivalError,
                    DirectionalBudget = directional, ScalarAlphaCap = _control.ReferenceAlphaCap, BrakeAlphaCap = p.BrakeCap, StepAlphaCap = p.StepCap,
                    BrakeInertiaNorm = p.BrakeInertia, StepInertiaNorm = p.StepInertia, InertialBudgetTorque = p.InertiaTorque, RigidGyroEstimate = p.RigidGyro,
                    BaseStepAlphaCap = p.BaseStepCap, ExtraBrakeAlpha = p.ExtraBrakeAlpha };
                pair.Sequence = _angularReferenceSequence; Report.ReferencePair = pair;
            }
            catch (Exception e) { Report.AngularReference = new PelvisReceiverReport.AngularReferenceSample();
                Report.ReferencePair = new PelvisReferencePairReport.Sample();
                Report.AngularEvidenceIssue = "ANGULAR_REFERENCE_CAPTURE_EXCEPTION:" + e.GetType().Name; }
        }
        private void RecordAngularActuation(MakemitAGA.ActiveRagdollLab.Runtime.PhysicsStepContext step, Vector3 pAlpha, Vector3 dAlpha, Vector3 feedforwardAlpha, Vector3 alpha, Vector3 inertiaAlpha, Vector3 gyro,
            float reducedMass, Vector3 parentLever, Vector3 orbitalAlpha, Vector3 orbitalGyro, Vector3 rawTorque, Vector3 torque)
        {
            try
            {
                Report.AngularActuation = new PelvisReceiverReport.AngularActuationSample { Valid = true,
                    Step = step, Fixed = Report.CurrentTraceFixed, PAlpha = pAlpha, DAlpha = dAlpha, AppliedAlpha = alpha,
                    InertiaAlphaTorque = inertiaAlpha, GyroTorque = gyro, Torque = torque,
                    ReferenceFeedforward = true, FeedforwardAlpha = feedforwardAlpha,
                    ReducedMass = reducedMass, ParentLever = parentLever,
                    OrbitalAlphaTorque = orbitalAlpha, OrbitalGyroTorque = orbitalGyro, RawTorque = rawTorque };
            }
            catch (Exception e) { Report.AngularActuation = new PelvisReceiverReport.AngularActuationSample();
                Report.AngularEvidenceIssue = "ANGULAR_ACTUATION_CAPTURE_EXCEPTION:" + e.GetType().Name; }
        }
        private PelvisReceiverReport.TrackingFrame CaptureAngularBodies(PelvisReceiverReport.TrackingFrame f, Quaternion targetChest, Quaternion currentChest)
        {
            try
            {
                f.AngularReference = Report.AngularReference;
                f.ChestW = _chest.angularVelocity;
                f.ChestRotationErrorVector = RotationVector(targetChest, currentChest);
                f.AngularBodiesCaptured = V(f.ChestW) && V(f.ChestRotationErrorVector);
            }
            catch (Exception e) { f.AngularBodiesCaptured = false; f.AngularEvidenceIssue = "ANGULAR_BODY_CAPTURE_EXCEPTION:" + e.GetType().Name; }
            return f;
        }
        internal PelvisReceiverReport.TrackingFrame AttachAngularContext(PelvisReceiverReport.TrackingFrame f,
            Vector3 childP, Vector3 childD, Vector3 childAlpha, Vector3 childTargetRelativeW, Vector3 childMeasuredRelativeW,
            bool jointAvailable, Vector3 jointForce, Vector3 jointTorque, Vector3 jointParentLever)
        {
            if (!f.Enabled || !f.Valid) return f;
            try
            {
                f.AngularActuation = Report.AngularActuation;
                if (_lastSupportRecovery.Step.Matches(f.ComReferenceStep)) f.SupportRecovery = _lastSupportRecovery;
                f.ChildPAlpha = childP; f.ChildDAlpha = childD; f.ChildAppliedAlpha = childAlpha;
                f.ChildTargetRelativeW = childTargetRelativeW; f.ChildMeasuredRelativeW = childMeasuredRelativeW;
                f.JointReactionAvailable = jointAvailable; f.JointForce = jointForce; f.JointTorque = jointTorque; f.JointParentLever = jointParentLever;
                f.AngularContextCaptured = true;
            }
            catch (Exception e) { f.AngularContextCaptured = false; f.AngularEvidenceIssue = "ANGULAR_CONTEXT_CAPTURE_EXCEPTION:" + e.GetType().Name; }
            return f;
        }

        internal void CommitTracking(PelvisReceiverReport.TrackingFrame frame, TwoBodyProxyReport source)
        {
            try { Report.ObserveTracking(frame, source); }
            catch (Exception e) { Report.TrackingInvalid++; Report.TrackingIssue = "TRACKING_REPORT_EXCEPTION:" + e.GetType().Name; }
        }
        internal void BeginTrace()
        {
            if (!_control.Selected) return;
            EndTrace(); Report.TraceArm = _control.Arm; Report.TraceDynamic = _control.Dynamic; Report.TraceRunning = true;
            Report.TraceReferenceBudgetMode = Report.ReferenceBudgetMode;
            Report.TraceReferencePlannerMode = Report.ReferencePlannerMode;
            Report.Samples = Report.IdleSamples = Report.MotionSamples = Report.ForcePairWrites = Report.TorqueWrites = 0;
            Report.TraceHipTranslation = IsDynamicHipArm(_control.Arm); Report.HipReactionRecords = 0;
            Report.HipAttitudeRequestedPeak = Report.HipAttitudeAppliedPeak = Report.HipAttitudeResidualPeak = 0f;
            Report.Resolved = Report.Censored = Report.LinearClamps = Report.AngularClamps = Report.TorqueClamps = Report.CurrentTraceFixed = 0;
            Report.PositionErrorPeak = Report.RotationErrorPeak = Report.ForceSumPeak = Report.TorquePeak = Report.SystemResidualPeak = 0f;
            Report.Next = Report.Count = 0; Array.Clear(Report.Ring, 0, Report.Ring.Length);
            Report.ResetTracking();
        }
        internal void EndTrace()
        {
            Report.EndTracking();
            if (_pending >= 0)
            {
                PelvisReceiverReport.Frame f = Report.Ring[_pending]; f.Censored = true; Report.Ring[_pending] = f;
                Report.CensorMotionInput(f);
                Report.Censored++; _pending = -1;
            }
            Report.TraceRunning = false;
            Report.SealMotionEvidence();
        }
        internal bool Quiesce()
        {
            ResetHipCommands();
            EndTrace(); Report.Active = _control.ObserveActive(false); Report.Quiesced = true; Report.CommonA = Report.PelvisTorque = Vector3.zero;
            ResetReference();
            bool ok = true;
            if (_pelvis != null) ok &= TrySafeBody(_pelvis);
            if (_chest != null) ok &= TrySafeBody(_chest);
            return ok;
        }
        internal bool QuiesceOwnedPair(Rigidbody pelvis, Rigidbody chest)
        {
            bool ok = Quiesce();
            // Selection can fail BEFORE a baseline/references are captured. The caller's
            // owned native pair must still be made safe, independently on both sides.
            if (pelvis != null && pelvis != _pelvis) ok &= TrySafeBody(pelvis);
            if (chest != null && chest != _chest) ok &= TrySafeBody(chest);
            return ok;
        }
        internal void MarkFault(string reason) { Invalid(reason); }
        private bool BodyStateMatch() => _pelvis != null && _chest != null && ParametersMatch() && !_pelvis.useGravity && !_chest.useGravity &&
            !_chest.isKinematic && _pelvis.isKinematic == !_control.Dynamic && V(_pelvis.velocity) && V(_chest.velocity) && V(_pelvis.angularVelocity) && V(_chest.angularVelocity);
        private bool ParametersMatch()
        {
            if (_pelvis == null || _chest == null || _joint == null || _joint.connectedBody != _pelvis || _joint.autoConfigureConnectedAnchor ||
                _pelvisCollider == null || _chestCollider == null || _pelvisCollider.enabled || _chestCollider.enabled ||
                !Near(_pelvisCollider.size, _pelvisSize) || !Near(_chestCollider.size, _chestSize) || !Near(_pelvisCollider.center, _pelvisCenter) || !Near(_chestCollider.center, _chestCenter) ||
                _pelvis.GetInstanceID() != _pelvisBaseline.Id || _chest.GetInstanceID() != _chestBaseline.Id) return false;
            if (_control.Dynamic) return BoxMatches(_pelvis, _pelvisCollider, _pelvisBaseline.Mass, _pelvisBaseline.MaxW) && BoxMatches(_chest, _chestCollider, _chestBaseline.Mass, _chestBaseline.MaxW) &&
                Near(_joint.anchor, _chest.centerOfMass) && Near(_joint.anchor, _control.NativeChestAnchor) && Near(_joint.connectedAnchor, _control.NativePelvisAnchor);
            return BaselineMatches(_pelvis, _pelvisBaseline) && BaselineMatches(_chest, _chestBaseline) &&
                Near(_joint.anchor, _baselineChestAnchor) && Near(_joint.connectedAnchor, _baselinePelvisAnchor);
        }
        private bool RestoreBaseline()
        {
            if (!_control.BaselineCaptured) return true;
            int before = _control.RestoreFailures;
            // A returned setter can still fail native readback. Preserve both results,
            // and continue restoring the other owner after either failure.
            bool bodiesSafe = TrySafeBody(_pelvis);
            bodiesSafe &= TrySafeBody(_chest);
            TryRestore(() => _pelvisCollider.enabled = false); TryRestore(() => _chestCollider.enabled = false);
            TryRestore(() => _joint.connectedBody = _pelvis); TryRestore(() => _joint.autoConfigureConnectedAnchor = false);
            RestoreBody(_pelvis, _pelvisBaseline); RestoreBody(_chest, _chestBaseline);
            TryRestore(() => _joint.anchor = _baselineChestAnchor); TryRestore(() => _joint.connectedAnchor = _baselinePelvisAnchor);
            bool ok = false;
            try { ok = bodiesSafe && _control.RestoreFailures == before && BaselineMatches(_pelvis, _pelvisBaseline) && BaselineMatches(_chest, _chestBaseline) &&
                _pelvis.isKinematic && _chest.isKinematic && !_pelvis.useGravity && !_chest.useGravity &&
                !_pelvisCollider.enabled && !_chestCollider.enabled && _joint.connectedBody == _pelvis && !_joint.autoConfigureConnectedAnchor &&
                Near(_joint.anchor, _baselineChestAnchor) && Near(_joint.connectedAnchor, _baselinePelvisAnchor); }
            catch { Report.RestoreFailures = _control.RecordRestoreFailure(); }
            Report.BaselineRestored = _control.ObserveBaselineRestored(ok);
            if (!ok) Invalid("RECEIVER_BASELINE_RESTORE_FAILED");
            return ok;
        }
        private void RestoreBody(Rigidbody b, State s)
        {
            TryRestore(() => b.mass = s.Mass); TryRestore(() => b.centerOfMass = s.Com);
            TryRestore(() => b.inertiaTensor = s.Inertia); TryRestore(() => b.inertiaTensorRotation = s.InertiaRotation);
            TryRestore(() => b.drag = s.Drag); TryRestore(() => b.angularDrag = s.AngularDrag); TryRestore(() => b.maxAngularVelocity = s.MaxW);
            TryRestore(() => b.constraints = s.Constraints); TryRestore(() => b.collisionDetectionMode = s.CollisionMode);
        }
        private void TryRestore(Action action) { try { action(); } catch { Report.RestoreFailures = _control.RecordRestoreFailure(); } }
        private bool TrySafeBody(Rigidbody b)
        {
            int before = _control.RestoreFailures;
            TryRestore(() => b.useGravity = false); TryRestore(() => b.isKinematic = true);
            TryRestore(() => b.velocity = Vector3.zero); TryRestore(() => b.angularVelocity = Vector3.zero);
            bool ok = false;
            try { ok = _control.RestoreFailures == before && b.isKinematic && !b.useGravity; }
            catch { Report.RestoreFailures = _control.RecordRestoreFailure(); }
            if (!ok) Invalid("RECEIVER_SAFE_STATE_READBACK_FAILED");
            return ok;
        }
        private float NativeAnchorGap() => ((_chest.position + _chest.rotation * _joint.anchor) - (_pelvis.position + _pelvis.rotation * _joint.connectedAnchor)).magnitude;
        internal static bool PoseDomainsMatch(Rigidbody b) => b != null && Near(b.position, b.transform.position) && Quaternion.Angle(b.rotation, b.transform.rotation) <= 0.05f;
        internal static void SynchronizeKinematicPose(Rigidbody b, Vector3 position, Quaternion rotation)
        {
            b.useGravity = false; b.isKinematic = true;
            b.transform.position = position; b.transform.rotation = rotation;
            b.position = position; b.rotation = rotation; b.velocity = Vector3.zero; b.angularVelocity = Vector3.zero;
            if (!PoseDomainsMatch(b)) throw new InvalidOperationException("RECEIVER_KINEMATIC_POSE_DOMAINS_MISMATCH");
        }
        private static void ApplyBox(Rigidbody b, BoxCollider c, float mass)
        {
            if (!Positive(mass) || !Positive(c.size.x) || !Positive(c.size.y) || !Positive(c.size.z) || !V(c.center) || !Near(b.transform.lossyScale, Vector3.one))
                throw new InvalidOperationException("RECEIVER_BOX_GEOMETRY_INVALID");
            b.mass = mass; b.centerOfMass = c.center; b.inertiaTensor = BoxInertia(mass, c.size); b.inertiaTensorRotation = Quaternion.identity;
            b.constraints = RigidbodyConstraints.None; b.collisionDetectionMode = CollisionDetectionMode.Discrete; b.drag = 0f; b.angularDrag = 0f;
        }
        private static State Capture(Rigidbody b) => new State { Id = b.GetInstanceID(), Mass = b.mass, Com = b.centerOfMass, Inertia = b.inertiaTensor,
            InertiaRotation = b.inertiaTensorRotation, Drag = b.drag, AngularDrag = b.angularDrag, MaxW = b.maxAngularVelocity, Constraints = b.constraints, CollisionMode = b.collisionDetectionMode };
        private static bool StateFinite(State s) => Positive(s.Mass) && V(s.Com) && Positive(s.Inertia.x) && Positive(s.Inertia.y) && Positive(s.Inertia.z) && Q(s.InertiaRotation) && F(s.Drag) && F(s.AngularDrag) && Positive(s.MaxW);
        private static bool BoxMatches(Rigidbody b, BoxCollider c, float mass, float maxW) => Near(b.transform.lossyScale, Vector3.one) && b.maxAngularVelocity == maxW && Mathf.Abs(b.mass - mass) <= 0.0001f && Near(b.centerOfMass, c.center) && Near(b.inertiaTensor, BoxInertia(mass, c.size)) &&
            Quaternion.Angle(b.inertiaTensorRotation, Quaternion.identity) <= 0.05f && b.constraints == RigidbodyConstraints.None && b.collisionDetectionMode == CollisionDetectionMode.Discrete && b.drag == 0f && b.angularDrag == 0f;
        private static bool BaselineMatches(Rigidbody b, State s) => b != null && b.GetInstanceID() == s.Id && Mathf.Abs(b.mass - s.Mass) <= 0.0001f && Near(b.centerOfMass, s.Com) && Near(b.inertiaTensor, s.Inertia) &&
            Quaternion.Angle(b.inertiaTensorRotation, s.InertiaRotation) <= 0.05f && b.drag == s.Drag && b.angularDrag == s.AngularDrag && b.maxAngularVelocity == s.MaxW && b.constraints == s.Constraints && b.collisionDetectionMode == s.CollisionMode;
        private static Vector3 BoxInertia(float m, Vector3 s) => new Vector3(m * (s.y * s.y + s.z * s.z) / 12f, m * (s.x * s.x + s.z * s.z) / 12f, m * (s.x * s.x + s.y * s.y) / 12f);
        private static Vector3 MultiplyInertia(Quaternion principalWorld, Vector3 inertia, Vector3 v) => principalWorld * Vector3.Scale(inertia, Quaternion.Inverse(principalWorld) * v);
        private static Vector3 MultiplyOrbitalInertia(Vector3 lever, float reducedMass, Vector3 v) =>
            reducedMass * Vector3.Cross(lever, Vector3.Cross(v, lever));
        private void ResetHipCommands()
        {
            _hipCommand = new PelvisHipCommandFrame(); _hipAttitudeCommand = new PelvisHipAttitudeCommand();
            _expectHipCompletion = false;
        }
        private bool Invalid(string reason) { ResetHipCommands(); Report.FaultEvents++; Report.Finite = _control.ObserveFinite(false); Report.Issue = Report.Issue ?? reason; return false; }
        private static bool Near(Vector3 a, Vector3 b) => V(a) && V(b) && (a - b).sqrMagnitude <= 0.00000004f;
        private static bool Positive(float v) => F(v) && v > 0.000001f;
        private static bool F(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z);
        private static bool Q(Quaternion q) => F(q.x) && F(q.y) && F(q.z) && F(q.w) && F(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) && Mathf.Abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1f) <= 0.005f;
    }
}
