using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        private readonly KneeRelativeReference _kneeReference = new KneeRelativeReference();
        private bool _kneePlannedFeedforwardSelected;
        private bool _kneeGroundTargetSelected, _kneeGroundTraceSelected;
        private bool _kneeGroundExtensionSelected, _kneeGroundExtensionTraceSelected, _kneeLandingPreparationSelected, _hipLandingPreparationSelected;
        private KneeGroundTarget.Sample _kneeGroundSample;
        private readonly KneeGroundFeasibility.Trace _kneeGroundFeasibilityTrace = new KneeGroundFeasibility.Trace();
        private int _kneeGroundValid, _kneeGroundCandidates, _kneeGroundApplied, _kneeGroundUnavailable, _kneeGroundPlannedBelow;
        private int _kneeGroundExtensionCandidates, _kneeGroundExtensionApplied;
        private float _kneeGroundPeakCorrection, _kneeGroundPeakGain;
        internal bool KneeGroundTargetSelected => _kneeGroundTargetSelected;
        internal bool KneeGroundExtensionSelected => _kneeGroundExtensionSelected;
        internal bool HipLandingPreparationSelected => _hipLandingPreparationSelected;
        private bool HipLandingPreparationArmSelected => KneeReferenceArmSelected &&
            PelvisDynamicReceiverIsolation.UsesHipLandingPreparation(_torso.PelvisReceiverControl.Arm);
        internal bool KneeLandingPreparationSelected => _kneeLandingPreparationSelected;
        private bool KneeLandingPreparationArmSelected => KneeReferenceArmSelected &&
            PelvisDynamicReceiverIsolation.UsesKneeLandingPreparation(_torso.PelvisReceiverControl.Arm);
        // Both arms retain v134 B, including bounded extension and the original
        // Knee intermediate step. v135 only chooses an input waypoint for Foot.
        private bool KneeGroundTargetArmSelected => KneeReferenceArmSelected;
        private bool KneeGroundExtensionArmSelected => KneeGroundTargetArmSelected;
        internal bool KneeReferenceSelected => _kneeReference.Selected;
        internal bool KneePlannedFeedforwardSelected => _kneePlannedFeedforwardSelected;
        // Seed once at dynamic join and keep R/W/A continuous through unloaded
        // preparation into the Hip run. Contact/load authority stays with its owners.
        private bool KneeReferenceArmSelected => _dynamicParentActive && _torso != null &&
            (_torso.PelvisReceiverControl.Arm == PelvisDynamicReceiverIsolation.DynamicHipArmA ||
             _torso.PelvisReceiverControl.Arm == PelvisDynamicReceiverIsolation.DynamicHipArmB);
        private bool KneePlannedFeedforwardArmSelected => KneeReferenceArmSelected;

        private void ResetKneeReferenceForDrive()
        {
            _kneePlannedFeedforwardSelected = KneePlannedFeedforwardArmSelected;
            _kneeGroundTargetSelected = KneeGroundTargetArmSelected;
            _kneeGroundExtensionSelected = KneeGroundExtensionArmSelected;
            _kneeLandingPreparationSelected = KneeLandingPreparationArmSelected;
            _hipLandingPreparationSelected = HipLandingPreparationArmSelected;
            _kneeGroundSample = default;
            _footFoundation?.ClearKneeFootGroundPacket();
            ResetKneeGroundTrace();
            _kneeReference.Reset(KneeReferenceArmSelected, _controlTargetKneeRelative,
                _controlTargetKneeRelativeAngularVelocityLocal);
        }

        private void DisableKneeReferencePreserveTrace()
        {
            _kneeReference.DisablePreserveTrace();
            _kneePlannedFeedforwardSelected = false;
            _kneeGroundTargetSelected = false; _kneeGroundSample = default;
            _kneeGroundExtensionSelected = _kneeLandingPreparationSelected = _hipLandingPreparationSelected = false;
            _footFoundation?.ClearKneeFootGroundPacket();
        }

        private string KneeReferenceExecutionIssue()
        {
            if (_kneeReference.Selected != KneeReferenceArmSelected ||
                _kneeGroundTargetSelected != KneeGroundTargetArmSelected ||
                _kneeGroundExtensionSelected != KneeGroundExtensionArmSelected ||
                _kneeLandingPreparationSelected != KneeLandingPreparationArmSelected ||
                _hipLandingPreparationSelected != HipLandingPreparationArmSelected ||
                _kneePlannedFeedforwardSelected != KneePlannedFeedforwardArmSelected)
                return "ARM_SELECTION_MISMATCH";
            if (!_kneeReference.Selected) return null; // Legacy raw-reference scopes.
            if (!DynamicParentActive || _dynamicJoinPending || !_joinStep.Valid || !_runtimeStep.Valid ||
                _runtimeStep.Epoch != _joinStep.Epoch || _runtimeStep.Sequence <= _joinStep.Sequence ||
                _runtimeStep.FixedTime < _joinStep.FixedTime || _runtimeStep.SourceSequence < _joinStep.SourceSequence)
                return "DYNAMIC_JOIN_STEP_MISMATCH";
            if (PelvisRunScopes.HasHipCoupling(_dynamicScope)) return null;
            // R2 starts real free-space solves before selecting Hip coupling. This
            // permission is only for the selected Hip reference after a valid join;
            // it cannot enable contact, gravity, load, tracing or a different scope.
            if (_dynamicScope != PelvisRunScope.DynamicLegFreeSpace || _traceEnabled ||
                !FreeSpaceSelectorsOff || !FreeSpaceNativeReady)
                return "UNLOADED_HIP_PREPARATION_REQUIRED";
            return null;
        }

        // One entry point owns the order and the phase contract for both consumers.
        // A preparation/run transition never reseeds the planner or its derivative.
        private bool UpdateKneeReferenceAndFeedforward(out string issue)
        {
            issue = KneeReferenceExecutionIssue();
            if (issue != null) { issue = "KNEE_REFERENCE_INVALID:" + issue; return false; }
            if (!UpdateKneeReference())
            {
                issue = "KNEE_REFERENCE_INVALID:" + (_kneeReference.Issue ?? _hipNormalReference.Issue ?? "REFERENCE_STEP_MISMATCH");
                return false;
            }
            if (!UpdateLegAngularAccelerationFeedforward())
            {
                issue = "LEG_ANGULAR_ACCELERATION_FEEDFORWARD_INVALID";
                return false;
            }
            return true;
        }

        private bool UpdateKneeReference()
        {
            _landingPreparationCommand = default;
            if (!_kneeReference.Selected) return true;
            KneeGroundTarget.Geometry geometry = default;
            KneeFootGroundPacket groundPacket = default;
            _kneeGroundSample = new KneeGroundTarget.Sample { Raw = _sampledTargetKneeRelative,
                Goal = _sampledTargetKneeRelative, AllowExtension = _kneeGroundExtensionSelected, Reason = "STALE_OR_MISSING_INPUT" };
            if (_targetSampleTiming.AgeSeconds <= _profile.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds &&
                _targetSampleTiming.AgeSeconds >= 0f && _physicalParent.Matches(_runtimeStep) && _footFoundation != null)
            {
                if (_footFoundation.TryReadKneeFootGroundPacket(_runtimeStep, _runtimeSource,
                    _physicalParent.BodyId, _upperBody.GetInstanceID(), _lowerBody.GetInstanceID(),
                    out KneeFootGroundPacket packet, out string reason))
                {
                    if (packet.Input.AllowExtension == _kneeGroundExtensionSelected)
                    {
                        geometry = packet.Geometry;
                        groundPacket = packet;
                        _kneeGroundSample = packet.Sample;
                    }
                    else _kneeGroundSample.Reason = "INPUT_DIRECTION_POLICY_MISMATCH";
                }
                else _kneeGroundSample.Reason = reason;
            }
            _kneeGroundSample.Step = _runtimeStep; _kneeGroundSample.Source = _runtimeSource;
            _kneeGroundSample.Applied &= _kneeGroundTargetSelected && _kneeGroundSample.Valid && _kneeGroundSample.Candidate;
            Quaternion input = _kneeGroundSample.Applied ? _kneeGroundSample.Goal : _sampledTargetKneeRelative;
            // Raw W is nominal source feedforward, not a derivative of this geometric
            // waypoint. Only the original planner differentiates its consumed R/W/A.
            if (!_kneeReference.Step(_traceEnabled, _runtimeStep, _runtimeSource,
                input, _sampledTargetKneeRelativeAngularVelocityLocal,
                _targetSampleTiming.AgeSeconds, _profile.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds,
                _profile.LegKneeMaxAngularAcceleration / _profile.LegKneeAngularDamper,
                Mathf.Min(_profile.LegAngularAccelerationFeedforwardMaxAcceleration, _profile.LegKneeMaxAngularAcceleration),
                _profile.LegKneeAngularSpring / _profile.LegKneeAngularDamper, groundPacket,
                _kneeGroundTargetSelected && PelvisRunScopes.HasHipCoupling(_dynamicScope),
                _kneeLandingPreparationSelected && PelvisRunScopes.HasHipCoupling(_dynamicScope),
                _hipNormalReference.Current, _hipNormalReference.Rotation, _hipNormalReference.W)) return false;
            if (!_kneeReference.Current.Matches(_runtimeStep, _runtimeSource)) return false;
            if (!FinalizeHipLandingPreparation(groundPacket, _kneeReference.Current)) return false;
            if (_traceEnabled) CaptureLandingPreparationCommand(groundPacket);
            if (_kneeGroundSample.Valid)
            {
                _kneeGroundSample.PlanMin = KneeGroundTarget.Minimum(geometry, _kneeReference.Current.R, _kneeGroundSample.PlannedFoot);
                _kneeGroundSample.Planned = IsFinite(_kneeGroundSample.PlanMin);
                try { KneeGroundFeasibility.AssessFixed(ref _kneeGroundSample, geometry); }
                catch (System.Exception)
                {
                    _kneeGroundSample.FeasibilityStage = KneeGroundFeasibility.Stage.Unavailable;
                    if (_traceEnabled) _kneeGroundFeasibilityTrace.Fault();
                }
            }
            if (_traceEnabled)
            {
                if (_kneeGroundSample.Valid) _kneeGroundValid++; else _kneeGroundUnavailable++;
                if (_kneeGroundSample.Candidate) _kneeGroundCandidates++;
                if (_kneeGroundSample.Applied) _kneeGroundApplied++;
                if (_kneeGroundSample.Extension) _kneeGroundExtensionCandidates++;
                if (_kneeGroundSample.Extension && _kneeGroundSample.Applied) _kneeGroundExtensionApplied++;
                if (_kneeGroundSample.Planned && _kneeGroundSample.PlanMin < 0f) _kneeGroundPlannedBelow++;
                _kneeGroundPeakCorrection = Mathf.Max(_kneeGroundPeakCorrection, Mathf.Abs(_kneeGroundSample.Correction));
                _kneeGroundPeakGain = Mathf.Max(_kneeGroundPeakGain, _kneeGroundSample.GoalMin - _kneeGroundSample.RawMin);
                try { _kneeGroundFeasibilityTrace.Observe(_kneeGroundSample, groundPacket, _kneeReference.Current); }
                catch (System.Exception) { _kneeGroundFeasibilityTrace.Fault(); }

            }
            _controlTargetKneeRelative = _kneeReference.Current.R;
            _controlTargetKneeRelativeAngularVelocityLocal = _kneeReference.Current.W;
            return true;
        }

        private bool UpdateLegAngularAccelerationFeedforward()
        {
            // Selection and preparation/run scope were checked before updating R/W/A.
            // The exemption still requires the reference produced in this exact step.
            if (_kneeReference.Selected && !_kneeReference.Current.Matches(_runtimeStep, _runtimeSource)) return false;
            Vector3 hipTargetRelativeAWorld = _hipNormalReference.Current.Applied ? _hipNormalReference.Current.A :
                GetHipDerivativeParentRotation() * _sampledTargetHipRelativeAngularAccelerationLocal;
            Vector3 flexionAxisWorld = _lowerBody.transform.TransformDirection(_kneeFlexionAxisLowerLocal);
            if (flexionAxisWorld.sqrMagnitude < 0.0001f)
                return false;
            flexionAxisWorld.Normalize();
            Vector3 kneeTargetRelativeAWorld = GetKneeDerivativeParentRotation() * KneeTargetRelativeA;
            float kneeTargetAxisA = Vector3.Dot(kneeTargetRelativeAWorld, flexionAxisWorld);

            bool ok = _legAngularAccelerationFeedforward.Step(
                hipTargetRelativeAWorld,
                kneeTargetAxisA,
                _torso.ControlPhase,
                _torso.ControlRecoveryActive,
                _targetSampleTiming.AgeSeconds,
                _profile.LegAngularAccelerationFeedforwardGain,
                _profile.LegAngularAccelerationFeedforwardMaxAcceleration,
                _profile.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds,
                _hipAuthority.PhysicalRecoveryPending,
                _softKneeAuthority.PhysicalRecoveryPending,
                (_hipControlFrame.Active && _profile.LegHipControlFrameInertialization),
                HipFeedforwardGoalBridgeActive,
                _dynamicParentActive && _torso.Report != null && _torso.ControlPolicy.HipGoalGateOnlyHip,
                _kneePlannedFeedforwardSelected);
            CaptureLegTurnHipFeedforward(ok, hipTargetRelativeAWorld, kneeTargetAxisA);
            CopyLegAngularAccelerationFeedforwardToReport();
            return ok;
        }

        private void ResetKneeGroundTrace()
        {
            _kneeGroundTraceSelected = _kneeGroundTargetSelected;
            _kneeGroundExtensionTraceSelected = _kneeGroundExtensionSelected;
            _kneeGroundValid = _kneeGroundCandidates = _kneeGroundApplied = _kneeGroundUnavailable = _kneeGroundPlannedBelow = 0;
            _kneeGroundPeakCorrection = _kneeGroundPeakGain = 0f;
            _kneeGroundExtensionCandidates = _kneeGroundExtensionApplied = 0;
            _kneeGroundFeasibilityTrace.Reset();
        }
        private string KneeGroundSummary() => "KNEE_FOOT_GROUND_TARGET_V041260/traceSelected=" + _kneeGroundTraceSelected +
            "/valid-candidate-applied-unavailable-plannedBelow=" + _kneeGroundValid + "|" + _kneeGroundCandidates + "|" +
            _kneeGroundApplied + "|" + _kneeGroundUnavailable + "|" + _kneeGroundPlannedBelow +
            "/candidatePeak(deg-gainMm)=" + F(_kneeGroundPeakCorrection) + "|" + F(_kneeGroundPeakGain * 1000f) +
            "/extensionSelected=" + _kneeGroundExtensionTraceSelected +
            "/extensionCandidate-applied=" + _kneeGroundExtensionCandidates + "|" + _kneeGroundExtensionApplied +
            "/extensionMinBendDeg=" + F(KneeGroundTarget.MinExtensionBendDegrees) +
            "/limits(kneeDeg-footDeg-bendDeg-minGainMm)=5|15|150|0.1/intent=COMPLETED_STRONG_STANCE_OR_APPROACH" +
            "/input=ONE_IMMUTABLE_RENDER_KNEE_FOOT_PAIR/planner=EXISTING_KNEE_FIXED_AND_FOOT_RENDER_RWA" +
            "/counts=FIXED_CONSUMPTIONS_INCLUDING_HELD/nominalKneeW=RAW_SOURCE/noPoseWrite=True/contactAuthority=False\n" +
            _kneeGroundFeasibilityTrace.Summary();

        private Vector3 KneeTargetRelativeA => _kneeReference.Selected
            ? _kneeReference.Current.A : _sampledTargetKneeRelativeAngularAccelerationLocal;

        // Check each lane, not aggregate ActiveThisStep: both Hip arms may select
        // bounded Knee A while raw Hip A is still blocked by source/torso recovery.
        private bool LegFeedforwardGatesSatisfied()
        {
            if (!_legAngularAccelerationFeedforward.ActiveThisStep) return true;
            if (_torso == null || _torso.Report == null ||
                _hipAuthority.PhysicalRecoveryPending || _softKneeAuthority.PhysicalRecoveryPending ||
                (_hipControlFrame.Active && _profile.LegHipControlFrameInertialization)) return false;
            bool sourceRecovery = _torso.ControlPhase.SourceTurnEventActive || _torso.ControlRecoveryActive;
            if (sourceRecovery && _legAngularAccelerationFeedforward.HipActiveThisStep) return false;
            if (_legAngularAccelerationFeedforward.KneeActiveThisStep && sourceRecovery &&
                (!_kneePlannedFeedforwardSelected || !_kneeReference.Current.Matches(_runtimeStep, _runtimeSource) ||
                 _kneeReference.Current.Stale)) return false;
            return true;
        }
    }
}
