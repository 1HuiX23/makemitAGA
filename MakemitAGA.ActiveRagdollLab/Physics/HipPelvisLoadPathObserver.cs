using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One shared receiver audit, after BOTH leg controllers. It never writes a Rigidbody or controls drive.
    internal sealed partial class HipPelvisLoadPathObserver
    {
        internal readonly HipPelvisLoadPathReport Report = new HipPelvisLoadPathReport();
        internal void BeginTrace(bool selected, TwoBodyShadowProxy torso, bool dynamic = false)
        {
            Report.Selected = Report.Collecting = selected; Report.Finite = true; Report.Issue = null;
            Report.ReceiverCaptured = false; Report.Pelvis = Report.Chest = default;
            Report.Samples = Report.LeftLoaded = Report.RightLoaded = Report.LeftReleased = Report.RightReleased = 0;
            Report.BothLoaded = Report.NeitherLoaded = Report.LastFixed = Report.AbortedStepsSkipped = 0;
            Report.Next = Report.Count = 0; Array.Clear(Report.Ring, 0, Report.Ring.Length);
            Report.Idle.Reset(); Report.Motion.Reset();
            Report.ResetDynamic(); Report.DynamicMode = selected && dynamic;
            if (!selected) return;
            if (Report.DynamicMode) { BeginDynamicTrace(torso); return; }
            try
            {
                if (torso == null || !torso.IsBuilt || !torso.DriveEnabled || !TorsoContract(torso.Report))
                { Invalid("HIP_PELVIS_TORSO_TOPOLOGY_NOT_READY"); return; }
                Rigidbody pelvis = torso.PelvisBody, chest = torso.ChestBody;
                if (pelvis == null || chest == null || pelvis == chest || !pelvis.isKinematic || chest.isKinematic || pelvis.useGravity || chest.useGravity)
                { Invalid("HIP_PELVIS_RECEIVER_STATE_NOT_FROZEN"); return; }
                Report.Pelvis = CaptureReceiver(pelvis); Report.Chest = CaptureReceiver(chest);
                if (!ReceiverFinite(Report.Pelvis) || !ReceiverFinite(Report.Chest))
                { Invalid("HIP_PELVIS_RECEIVER_PARAMETERS_NONFINITE"); return; }
                Report.ReceiverCaptured = true;
            }
            catch (Exception e) { Invalid("HIP_PELVIS_RECEIVER_READ_EXCEPTION:" + e.GetType().Name); }
        }
        internal void EndTrace() { Report.Collecting = false; }
        internal void Observe(bool bothDriving, TwoBodyShadowProxy torso, SingleLegChainReport left, SingleLegChainReport right,
            PhysicsStepContext step = default, HipCommandWrench leftWrench = default, HipCommandWrench rightWrench = default)
        {
            if (!Report.Collecting || !Report.Finite) return;
            if (Report.DynamicMode) { ObserveDynamic(bothDriving, torso, left, right, step, leftWrench, rightWrench); return; }
            // The existing safety owner handles an abort. Do not report a queued/cancelled solve as valid input.
            if (!bothDriving || left == null || right == null || left.DriveAborted || right.DriveAborted)
            { Report.AbortedStepsSkipped++; return; }
            try
            {
                if (!Report.ReceiverCaptured || torso == null || !torso.DriveEnabled || !TorsoContract(torso.Report) ||
                    !ReceiverMatches(torso.PelvisBody, Report.Pelvis) || !ReceiverMatches(torso.ChestBody, Report.Chest))
                { Invalid("HIP_PELVIS_RECEIVER_OWNER_OR_STATE_CHANGED"); return; }
                if (!string.Equals(left.Side, "LEFT", StringComparison.OrdinalIgnoreCase) ||
                    !string.Equals(right.Side, "RIGHT", StringComparison.OrdinalIgnoreCase))
                { Invalid("HIP_PELVIS_BILATERAL_SIDE_IDENTITY_MISMATCH"); return; }
                if (!UpperInput(left, out UpperSelfWeightSupportReport.UpperFrame l) ||
                    !UpperInput(right, out UpperSelfWeightSupportReport.UpperFrame r) ||
                    l.Fixed != r.Fixed || l.Fixed <= Report.LastFixed || l.BodyId == r.BodyId ||
                    l.BodyId == Report.Pelvis.BodyId || r.BodyId == Report.Pelvis.BodyId ||
                    l.BodyId == Report.Chest.BodyId || r.BodyId == Report.Chest.BodyId ||
                    Mathf.Abs(l.Dt - r.Dt) > 0.000001f)
                { Invalid("HIP_PELVIS_BILATERAL_INPUT_CLOCK_OR_OWNER_MISMATCH"); return; }
                if (Report.LastFixed > 0 && l.Fixed != Report.LastFixed + 1)
                { Invalid("HIP_PELVIS_INPUT_SEQUENCE_GAP"); return; }
                if (string.IsNullOrEmpty(torso.Report.MotionPhase))
                { Invalid("HIP_PELVIS_SOURCE_PHASE_UNAVAILABLE"); return; }
                Vector3 pelvisCom = torso.PelvisBody.worldCenterOfMass;
                bool idle = torso.Report.MotionPhase == "IDLE" && !torso.Report.SourceTurnEventActive;
                var f = new HipPelvisLoadPathReport.InputFrame
                {
                    Fixed = l.Fixed, Dt = l.Dt, Idle = idle, LeftLoaded = l.Loaded, RightLoaded = r.Loaded,
                    PelvisCom = pelvisCom, LeftUpperCom = l.PreCom, RightUpperCom = r.PreCom,
                    LeftHipJ = l.HipJ, RightHipJ = r.HipJ, LeftCommonJ = l.CommonJ, RightCommonJ = r.CommonJ,
                    ComMirrorMomentJ = Vector3.Cross(l.PreCom - pelvisCom, l.HipJ) + Vector3.Cross(r.PreCom - pelvisCom, r.HipJ)
                };
                Vector3 hip = f.LeftHipJ + f.RightHipJ, common = f.LeftCommonJ + f.RightCommonJ;
                if (!V(pelvisCom) || !V(hip) || !V(common) || !V(f.ComMirrorMomentJ) ||
                    !F(hip.magnitude / f.Dt) || !F(f.ComMirrorMomentJ.magnitude / f.Dt))
                { Invalid("HIP_PELVIS_INPUT_SUM_OR_GEOMETRY_NONFINITE"); return; }
                Report.Ring[Report.Next] = f; Report.Next = (Report.Next + 1) % Report.Ring.Length;
                Report.Count = Mathf.Min(Report.Count + 1, Report.Ring.Length); Report.LastFixed = f.Fixed;
                Report.Samples++;
                if (f.LeftLoaded) Report.LeftLoaded++; else Report.LeftReleased++;
                if (f.RightLoaded) Report.RightLoaded++; else Report.RightReleased++;
                if (f.LeftLoaded && f.RightLoaded) Report.BothLoaded++;
                if (!f.LeftLoaded && !f.RightLoaded) Report.NeitherLoaded++;
                (idle ? Report.Idle : Report.Motion).Add(f);
            }
            catch (Exception e) { Invalid("HIP_PELVIS_INPUT_READ_EXCEPTION:" + e.GetType().Name); }
        }
        private static bool UpperInput(SingleLegChainReport leg, out UpperSelfWeightSupportReport.UpperFrame f)
        {
            f = default; UpperSelfWeightSupportReport u = leg?.UpperSelfWeightSupport;
            if (u == null || !u.Selected || !u.Enabled || !u.Finite || !u.LedgerFinite || u.Count <= 0 ||
                u.Arm != UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB ||
                !leg.HipCharacterJointResponseIsolationEnabled || !leg.HipTranslationTetherUpperOnlyEnabled || leg.HipCharacterJointPresent ||
                !leg.HipMechanicalParentProxyPresent || !leg.HipMechanicalParentUsesInertializedProxy ||
                leg.HipMechanicalParentMode != HipMechanicalParentProxy.InertializedProxyMode)
                return false;
            f = u.Ring[(u.Next - 1 + u.Ring.Length) % u.Ring.Length];
            return f.Fixed > 0 && f.Fixed == leg.FixedUpdateSequence && f.Fixed == u.CurrentFixed &&
                f.Fixed == leg.HipAnchorLinearTransportCurrentFixed && !f.Resolved && !f.Censored &&
                f.BodyId != 0 && F(f.Mass) && f.Mass > 0.000001f && F(f.Dt) && f.Dt > 0f &&
                Mathf.Abs(f.Mass - u.CurrentUpperMass) <= 0.0001f &&
                f.Applied == f.Loaded && u.CurrentApplied == f.Applied && u.CurrentEligible == f.Loaded &&
                V(f.PreCom) && V(f.HipJ) && V(f.CommonJ) && F(f.HipJ.magnitude / f.Dt);
        }
        private static HipPelvisLoadPathReport.Receiver CaptureReceiver(Rigidbody body) => new HipPelvisLoadPathReport.Receiver
        {
            BodyId = body.GetInstanceID(), Kinematic = body.isKinematic, Gravity = body.useGravity,
            Mass = body.mass, ComLocal = body.centerOfMass, Inertia = body.inertiaTensor, InertiaRotation = body.inertiaTensorRotation
        };
        private static bool ReceiverMatches(Rigidbody body, HipPelvisLoadPathReport.Receiver r) =>
            body != null && body.GetInstanceID() == r.BodyId && body.isKinematic == r.Kinematic && body.useGravity == r.Gravity &&
            F(body.mass) && Mathf.Abs(body.mass - r.Mass) <= 0.0001f;
        private static bool ReceiverFinite(HipPelvisLoadPathReport.Receiver r) => r.BodyId != 0 && F(r.Mass) && r.Mass > 0.000001f &&
            V(r.ComLocal) && V(r.Inertia) && F(r.InertiaRotation.x) && F(r.InertiaRotation.y) && F(r.InertiaRotation.z) && F(r.InertiaRotation.w);
        private static bool TorsoContract(TwoBodyProxyReport r) => r != null && r.Built && r.DriveEnabled && r.FiniteState &&
            !r.GravityEnabled && r.CharacterJointPresent && r.CharacterJointConnectedBodyMatch;
        private void Invalid(string reason) { Report.Finite = false; Report.Issue = Report.Issue ?? reason; }
        private static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool V(Vector3 x) => F(x.x) && F(x.y) && F(x.z);
    }
}
