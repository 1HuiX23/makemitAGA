using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class HipPelvisLoadPathObserver
    {
        private void BeginDynamicTrace(TwoBodyShadowProxy torso)
        {
            try
            {
                if (torso == null || !torso.IsBuilt || !torso.DriveEnabled || !TorsoContract(torso.Report) || !torso.PelvisReceiverReady)
                { Invalid("DYNAMIC_HIP_RECEIVER_NOT_READY"); return; }
                Rigidbody pelvis = torso.PelvisBody, chest = torso.ChestBody;
                if (pelvis == null || chest == null || pelvis == chest || pelvis.isKinematic || chest.isKinematic || pelvis.useGravity || chest.useGravity)
                { Invalid("DYNAMIC_HIP_RECEIVER_NATIVE_BOUNDARY_INVALID"); return; }
                Report.Pelvis = CaptureReceiver(pelvis); Report.Chest = CaptureReceiver(chest);
                if (!DynamicReceiverFinite(Report.Pelvis) || !DynamicReceiverFinite(Report.Chest))
                { Invalid("DYNAMIC_HIP_RECEIVER_PARAMETERS_INVALID"); return; }
                Report.ReceiverCaptured = true;
            }
            catch (Exception e) { Invalid("DYNAMIC_HIP_RECEIVER_READ_EXCEPTION:" + e.GetType().Name); }
        }

        private void ObserveDynamic(bool bothDriving, TwoBodyShadowProxy torso, SingleLegChainReport left, SingleLegChainReport right,
            PhysicsStepContext step, HipCommandWrench l, HipCommandWrench r)
        {
            if (!bothDriving || left == null || right == null || left.DriveAborted || right.DriveAborted)
            { Report.AbortedStepsSkipped++; return; }
            try
            {
                if (!Report.ReceiverCaptured || torso == null || !torso.DriveEnabled || !TorsoContract(torso.Report) || !torso.PelvisReceiverReady ||
                    !DynamicReceiverMatches(torso.PelvisBody, Report.Pelvis) || !DynamicReceiverMatches(torso.ChestBody, Report.Chest) ||
                    !torso.TryGetLegParentCommand(step, out LegParentCommandFrame command) || !command.PhysicalParent.Matches(step) || !command.PhysicalParent.Finite())
                { Invalid("DYNAMIC_HIP_RECEIVER_OR_PARENT_FRAME_CHANGED"); return; }
                PelvisParentFrame parent = command.PhysicalParent;
                if (!string.Equals(left.Side, "LEFT", StringComparison.OrdinalIgnoreCase) || !string.Equals(right.Side, "RIGHT", StringComparison.OrdinalIgnoreCase) ||
                    !DynamicUpperInput(left, step, parent, parent.Left, l) || !DynamicUpperInput(right, step, parent, parent.Right, r) ||
                    l.Fixed != r.Fixed || l.UpperBodyId == r.UpperBodyId || l.UpperBodyId == Report.Chest.BodyId || r.UpperBodyId == Report.Chest.BodyId ||
                    parent.BodyId != Report.Pelvis.BodyId || (torso.PelvisBody.worldCenterOfMass - l.ParentCom).magnitude > 0.0001f)
                { Invalid("DYNAMIC_HIP_COMMITTED_INPUT_OR_GEOMETRY_MISMATCH"); return; }
                if (Report.LastFixed > 0 && (l.Fixed != Report.LastFixed + 1 || !Report.DynamicLastStep.Valid ||
                    step.Epoch != Report.DynamicLastStep.Epoch || step.Sequence != Report.DynamicLastStep.Sequence + 1 ||
                    l.UpperBodyId != Report.DynamicLeftUpperId || r.UpperBodyId != Report.DynamicRightUpperId))
                { Invalid("DYNAMIC_HIP_INPUT_SEQUENCE_OR_OWNER_CHANGED"); return; }
                if (string.IsNullOrEmpty(torso.Report.MotionPhase))
                { Invalid("DYNAMIC_HIP_SOURCE_PHASE_UNAVAILABLE"); return; }
                var f = new HipPelvisLoadPathReport.InputFrame
                {
                    Fixed = l.Fixed, Dt = step.Dt, Idle = torso.Report.MotionPhase == "IDLE" && !torso.Report.SourceTurnEventActive,
                    LeftLoaded = left.UpperSelfWeightSupport.CurrentApplied, RightLoaded = right.UpperSelfWeightSupport.CurrentApplied,
                    PelvisCom = l.ParentCom, LeftUpperCom = l.UpperCom, RightUpperCom = r.UpperCom,
                    LeftHipJ = l.Force * step.Dt, RightHipJ = r.Force * step.Dt,
                    LeftCommonJ = Vector3.zero, RightCommonJ = Vector3.zero,
                    ComMirrorMomentJ = (l.ComForcePairMoment + r.ComForcePairMoment) * step.Dt,
                    LeftWrench = l, RightWrench = r
                };
                Report.Ring[Report.Next] = f; Report.Next = (Report.Next + 1) % Report.Ring.Length;
                Report.Count = Mathf.Min(Report.Count + 1, Report.Ring.Length); Report.LastFixed = f.Fixed;
                Report.DynamicLeftUpperId = l.UpperBodyId; Report.DynamicRightUpperId = r.UpperBodyId;
                Report.Samples++;
                if (f.LeftLoaded) Report.LeftLoaded++; else Report.LeftReleased++;
                if (f.RightLoaded) Report.RightLoaded++; else Report.RightReleased++;
                if (f.LeftLoaded && f.RightLoaded) Report.BothLoaded++;
                if (!f.LeftLoaded && !f.RightLoaded) Report.NeitherLoaded++;
                (f.Idle ? Report.Idle : Report.Motion).Add(f);
                Report.RecordDynamic(f);
            }
            catch (Exception e) { Invalid("DYNAMIC_HIP_INPUT_READ_EXCEPTION:" + e.GetType().Name); }
        }

        private static bool DynamicUpperInput(SingleLegChainReport leg, PhysicsStepContext step, PelvisParentFrame parent,
            PelvisParentFrame.HipReference hip, HipCommandWrench w)
        {
            UpperSelfWeightSupportReport u = leg.UpperSelfWeightSupport;
            if (!w.Finite || !w.Step.Matches(step) || w.Fixed != leg.FixedUpdateSequence || !leg.RuntimeContext.Matches(step) || !leg.RuntimeParentMatched ||
                w.ParentBodyId != parent.BodyId || !leg.HipCharacterJointResponseIsolationEnabled || !leg.HipTranslationTetherUpperOnlyEnabled || leg.HipCharacterJointPresent ||
                !leg.HipAngularActuatorEnabled || leg.HipAngularActuatorArm != HipAngularActuatorInertiaSpaceResponseIsolation.ArmB ||
                leg.HipAnchorLinearTransportCurrentAppliedAccelerationWorld != Vector3.zero ||
                u == null || !u.LedgerFinite || !UpperSelfWeightSupportNormalLoadTransferIsolation.SelectorContract(u, UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB, false) ||
                u.Count <= 0 || u.CurrentFixed != w.Fixed || u.CurrentApplied != u.CurrentEligible) return false;
            UpperSelfWeightSupportReport.UpperFrame actual = u.Ring[(u.Next - 1 + u.Ring.Length) % u.Ring.Length];
            if (!F(actual.Mass) || !F(actual.Dt) || !V(actual.PreCom) || !V(actual.PreV) || !V(actual.HipJ) ||
                actual.Applied != actual.Loaded || actual.Fixed != w.Fixed || actual.BodyId != w.UpperBodyId || actual.Resolved || actual.Censored || actual.Loaded != u.CurrentApplied ||
                actual.Dt != step.Dt || Mathf.Abs(actual.Mass - w.Mass) > 0.0001f ||
                (actual.PreCom - w.UpperCom).magnitude > 0.0001f || (actual.PreV - w.UpperV).magnitude > 0.0001f ||
                (actual.HipJ - w.Force * step.Dt).magnitude > 0.0001f || actual.CommonJ != Vector3.zero ||
                (w.ParentCom - parent.SettledPoint(parent.ComLocal)).magnitude > 0.0001f ||
                (w.ParentAnchor - hip.SettledPoint).magnitude > 0.0001f || (w.ParentPointV - hip.SettledVelocity).magnitude > 0.0001f ||
                (w.ParentV - parent.ComVelocity).magnitude > 0.0001f || (w.ParentW - parent.W).magnitude > 0.0001f ||
                (w.ChildPointV - w.UpperV - Vector3.Cross(w.UpperW, w.ChildAnchor - w.UpperCom)).magnitude > 0.0001f ||
                (w.ParentPointV - w.ParentV - Vector3.Cross(w.ParentW, w.ParentAnchor - w.ParentCom)).magnitude > 0.0001f) return false;
            // Numeric identities only. Passing them does not authorize any candidate write or prove passivity.
            return w.WrenchClosure.magnitude <= 0.0001f * (1f + w.Torque.magnitude + w.ComForcePairMoment.magnitude) &&
                Mathf.Abs(w.CandidateParentComPower - w.CandidateParentAnchorPower) <=
                    0.0001f * (1f + Mathf.Abs(w.CandidateParentComPower) + Mathf.Abs(w.CandidateParentAnchorPower));
        }
        private static bool DynamicReceiverFinite(HipPelvisLoadPathReport.Receiver r) => ReceiverFinite(r) &&
            !r.Kinematic && !r.Gravity && r.Inertia.x > 0.000001f && r.Inertia.y > 0.000001f && r.Inertia.z > 0.000001f &&
            Mathf.Abs(r.InertiaRotation.x * r.InertiaRotation.x + r.InertiaRotation.y * r.InertiaRotation.y +
                r.InertiaRotation.z * r.InertiaRotation.z + r.InertiaRotation.w * r.InertiaRotation.w - 1f) < 0.01f;
        private static bool DynamicReceiverMatches(Rigidbody body, HipPelvisLoadPathReport.Receiver r) => ReceiverMatches(body, r) &&
            (body.centerOfMass - r.ComLocal).sqrMagnitude <= 0.00000001f && (body.inertiaTensor - r.Inertia).sqrMagnitude <= 0.00000001f &&
            Mathf.Abs(body.inertiaTensorRotation.x - r.InertiaRotation.x) <= 0.0001f && Mathf.Abs(body.inertiaTensorRotation.y - r.InertiaRotation.y) <= 0.0001f &&
            Mathf.Abs(body.inertiaTensorRotation.z - r.InertiaRotation.z) <= 0.0001f && Mathf.Abs(body.inertiaTensorRotation.w - r.InertiaRotation.w) <= 0.0001f;
    }
}
