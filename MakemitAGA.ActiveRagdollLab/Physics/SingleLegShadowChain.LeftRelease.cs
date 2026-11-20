using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Shin = MakemitAGA.ActiveRagdollLab.Physics.LeftShinForecast;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        // Both legs returned from CompleteHipControllerStep. The provisional Knee
        // tether prepared before Foot.ObserveFixed is now the actual native write.
        internal void CompleteLeftShinForecast(in PhysicsStepContext step)
        {
            var trace = _footFoundation?.LeftRelease;
            var row = trace?.PendingShin(step, _runtimeSource, _fixedUpdateSequence);
            if (row == null) return;
            using var timing = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.LeftShinForecast);
            try { trace.CompleteShin(row, ReadLeftShinCommand(step, row)); }
            catch { trace.CompleteShin(row, new Shin.Result { State = Shin.Status.Fault }); }
        }

        private Shin.Result ReadLeftShinCommand(in PhysicsStepContext step, LeftFootReleaseTrace.Row row)
        {
            var before = row.Before;
            int local = _fixedUpdateSequence;
            var foot = _footFoundation.Report;
            if (!_runtimeStep.Matches(step) || !_legTurnCurrent.Matches(step, local) || !_legTurnCurrent.Source.Matches(row.From.Source) ||
                !foot.RuntimeContext.Matches(step) || !foot.RuntimeSourceContext.Matches(row.From.Source) ||
                _footFoundation.LastCompletedControlFixedSequence != local || _report.FixedUpdateSequence != local || !before.Anchors.Valid)
                return new Shin.Result { State = Shin.Status.MissingFrame };
            if (_report.HipAnchorLinearTransportCurrentFixed != local || !_hipAnchorPair.TryGetCommittedInput(step, out var hip) ||
                !_hipAnchorPair.Report.Current.JointSolved || hip.Seed.Fixed != local || _hipAnchorPair.Report.Current.Fixed != local)
                return new Shin.Result { State = Shin.Status.MissingHipReceipt };
            if (_kneeSupportReaction.Report.CurrentSourceFixed != local || !DynamicKneeCouplingMatches())
                return new Shin.Result { State = Shin.Status.MissingKneeReceipt };

            var applied = _hipAnchorPair.Report.Current;
            var footBody = _footFoundation.DynamicBody;
            if (_upperBody == null || _lowerBody == null || footBody == null || _hipJoint != null || _kneeJoint != null ||
                _upperBody.isKinematic || _lowerBody.isKinematic ||
                footBody.isKinematic || before.UpperId != applied.ChildId || _upperBody.GetInstanceID() != before.UpperId ||
                _lowerBody.GetInstanceID() != before.LowerId || footBody.GetInstanceID() != before.FootId ||
                _upperBody.constraints != RigidbodyConstraints.None || _lowerBody.constraints != RigidbodyConstraints.None ||
                (applied.ChildPoint - before.Anchors.UpperHip).sqrMagnitude > 1e-8f)
                return new Shin.Result { State = Shin.Status.BodyMismatch };

            if (_footFoundation.ExactAuthorizedStanceSupportLoadEligible || footBody.useGravity || foot.StanceFootNativeGravityCurrentApplied ||
                foot.StanceFootNativeGravityCurrentScheduledFixed != 0 || _lowerBody.useGravity || _upperBody.useGravity ||
                _upperCollider.enabled || _lowerCollider.enabled || foot.StanceLowerGroundAuthorityCurrentApplied ||
                foot.StanceLowerGroundAuthorityAppliedAccelerationWorld != Vector3.zero || _report.StanceLowerNativeGravityCurrentScheduledFixed != 0 ||
                _report.StanceLowerSelfWeightTransferCurrentFixed != 0 || !DynamicLowerGravityMatches(true) ||
                !DynamicLowerWeightPairMatches() || !DynamicUpperWeightMatches())
                return new Shin.Result { State = Shin.Status.Loaded };

            var lowerR = _lowerBody.rotation;
            var lower = new Shin.Body {
                Com = _lowerBody.worldCenterOfMass, V = _lowerBody.velocity, W = _lowerBody.angularVelocity,
                Mass = _lowerBody.mass, Inertia = _lowerBody.inertiaTensor, Principal = lowerR * _lowerBody.inertiaTensorRotation,
                Drag = _lowerBody.drag, AngularDrag = _lowerBody.angularDrag, MaxW = _lowerBody.maxAngularVelocity
            };
            var upper = new Shin.Body {
                Com = applied.ChildCom, V = hip.ChildComV, W = hip.ChildW, Inertia = hip.ChildInertia, Principal = hip.ChildPrincipal,
                Mass = applied.ChildMass, Drag = _upperBody.drag, AngularDrag = _upperBody.angularDrag, MaxW = _upperBody.maxAngularVelocity
            };
            var parentAlpha = _kneeParentAngularTransport.Enabled ? _legTurnCurrent.HipFinalA : Vector3.zero;
            if (!RightThighTracking.Same(_legTurnCurrent.KneeTotalA, parentAlpha + _legTurnCurrent.KneeRelativeA) ||
                !RightThighTracking.Same(_legTurnCurrent.HipFinalA, applied.AllocatedHipAlpha))
                return new Shin.Result { State = Shin.Status.AngularMismatch };
            var tether = _kneeCharacterJointResponseIsolation;
            var inputs = new Shin.Commands {
                CommonA = _report.HipAnchorLinearTransportCurrentAppliedAccelerationWorld,
                HipPoint = applied.ChildPoint, HipForce = applied.Force, UpperTorque = applied.UpperActuatorTorque,
                KneeTetherA = tether.CurrentAppliedAccelerationWorld, ParentFeedforwardA = tether.CurrentParentAnchorFeedforwardWorld,
                SpringA = tether.CurrentPositionErrorWorld * KneeCharacterJointResponseIsolation.TetherPositionSpring,
                DamperA = tether.CurrentVelocityErrorWorld * KneeCharacterJointResponseIsolation.TetherVelocityDamper,
                TetherCap = KneeCharacterJointResponseIsolation.TetherMaxAcceleration,
                HipTransportAlpha = parentAlpha, KneeRelativeAlpha = _legTurnCurrent.KneeRelativeA,
                AnkleReactionNm = -row.Motion.Angular.Drive
            };
            return Shin.Predict(step.Dt, before.Anchors, before.Pose.Box.LowerAnkle, upper, lower, inputs);
        }
    }
}
