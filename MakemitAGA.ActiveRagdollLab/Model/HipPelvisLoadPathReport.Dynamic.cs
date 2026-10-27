using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed partial class HipPelvisLoadPathReport
    {
        internal bool DynamicMode;
        internal PhysicsStepContext DynamicLastStep;
        internal int DynamicLeftUpperId, DynamicRightUpperId;
        internal int LeftForceNonzero, RightForceNonzero, LeftTorqueNonzero, RightTorqueNonzero, OffsetMomentNonzero;
        internal float PeakTorque, PeakAnchorMoment, PeakMovedUpperTorque, PeakBothAnchorMoment, PeakEquivalentParentTorque;
        internal float PeakActualUpperPower, PeakCandidatePairPower, PeakBothAnchorPower, PeakClosure, PeakPowerTransportError;
        internal InputFrame ForcePeakFrame, AnchorMomentPeakFrame, PowerPeakFrame;
        internal bool DynamicExposureReady => LeftForceNonzero >= 30 && RightForceNonzero >= 30 &&
            LeftTorqueNonzero >= 30 && RightTorqueNonzero >= 30 && OffsetMomentNonzero >= 30 && Samples == Idle.Samples + Motion.Samples;

        internal void ResetDynamic()
        {
            DynamicMode = false; DynamicLastStep = default; DynamicLeftUpperId = DynamicRightUpperId = 0;
            LeftForceNonzero = RightForceNonzero = LeftTorqueNonzero = RightTorqueNonzero = OffsetMomentNonzero = 0;
            PeakTorque = PeakAnchorMoment = PeakMovedUpperTorque = PeakBothAnchorMoment = PeakEquivalentParentTorque = 0f;
            PeakActualUpperPower = PeakCandidatePairPower = PeakBothAnchorPower = PeakClosure = PeakPowerTransportError = 0f;
            ForcePeakFrame = AnchorMomentPeakFrame = PowerPeakFrame = default;
        }

        internal void RecordDynamic(InputFrame f)
        {
            HipCommandWrench l = f.LeftWrench, r = f.RightWrench;
            if (l.Force.magnitude > 0.000001f) LeftForceNonzero++;
            if (r.Force.magnitude > 0.000001f) RightForceNonzero++;
            if (l.Torque.magnitude > 0.000001f) LeftTorqueNonzero++;
            if (r.Torque.magnitude > 0.000001f) RightTorqueNonzero++;
            float anchor = Mathf.Max(l.AnchorForcePairMoment.magnitude, r.AnchorForcePairMoment.magnitude);
            if (anchor > 0.000001f) OffsetMomentNonzero++;
            float force = Mathf.Max(l.Force.magnitude, r.Force.magnitude);
            if (ForcePeakFrame.Fixed == 0 || force > Mathf.Max(ForcePeakFrame.LeftWrench.Force.magnitude, ForcePeakFrame.RightWrench.Force.magnitude)) ForcePeakFrame = f;
            if (AnchorMomentPeakFrame.Fixed == 0 || anchor > PeakAnchorMoment) AnchorMomentPeakFrame = f;
            float pairPower = Mathf.Max(Mathf.Abs(l.ActualUpperPower + l.CandidateParentAnchorPower), Mathf.Abs(r.ActualUpperPower + r.CandidateParentAnchorPower));
            if (PowerPeakFrame.Fixed == 0 || pairPower > PeakCandidatePairPower) PowerPeakFrame = f;
            PeakTorque = Mathf.Max(PeakTorque, Mathf.Max(l.Torque.magnitude, r.Torque.magnitude));
            PeakAnchorMoment = Mathf.Max(PeakAnchorMoment, anchor);
            PeakMovedUpperTorque = Mathf.Max(PeakMovedUpperTorque, Mathf.Max(l.MoveUpperForceToAnchorTorque.magnitude, r.MoveUpperForceToAnchorTorque.magnitude));
            PeakBothAnchorMoment = Mathf.Max(PeakBothAnchorMoment, Mathf.Max(l.BothAnchorForcePairMoment.magnitude, r.BothAnchorForcePairMoment.magnitude));
            PeakEquivalentParentTorque = Mathf.Max(PeakEquivalentParentTorque, Mathf.Max(l.EquivalentParentAnchorTorque.magnitude, r.EquivalentParentAnchorTorque.magnitude));
            PeakActualUpperPower = Mathf.Max(PeakActualUpperPower, Mathf.Max(Mathf.Abs(l.ActualUpperPower), Mathf.Abs(r.ActualUpperPower)));
            PeakCandidatePairPower = Mathf.Max(PeakCandidatePairPower, pairPower);
            PeakBothAnchorPower = Mathf.Max(PeakBothAnchorPower, Mathf.Max(Mathf.Abs(l.BothAnchorForcePairPower), Mathf.Abs(r.BothAnchorForcePairPower)));
            PeakClosure = Mathf.Max(PeakClosure, Mathf.Max(l.WrenchClosure.magnitude, r.WrenchClosure.magnitude));
            PeakPowerTransportError = Mathf.Max(PeakPowerTransportError, Mathf.Max(Mathf.Abs(l.CandidateParentComPower - l.CandidateParentAnchorPower), Mathf.Abs(r.CandidateParentComPower - r.CandidateParentAnchorPower)));
            DynamicLastStep = l.Step;
        }

        private string GetDynamicSummary() => "mode=READ_ONLY_DYNAMIC_HIP_WRENCH_INPUT_V04890/selected-collecting-finite=" +
            Selected + "-" + Collecting + "-" + Finite + "/issue=" + (Issue ?? "<none>") +
            "/receiverCaptured=" + ReceiverCaptured + "/inputEvidenceReady=" + EvidenceReady +
            "/samples-leftLoaded-rightLoaded-leftReleased-rightReleased-both-neither=" + Samples + "-" + LeftLoaded + "-" + RightLoaded + "-" + LeftReleased + "-" + RightReleased + "-" + BothLoaded + "-" + NeitherLoaded +
            "/lastFixed-abortedStepsSkipped=" + LastFixed + "-" + AbortedStepsSkipped +
            "/" + DynamicLastStep.Status() + "/upperIds(left-right)=" + DynamicLeftUpperId + "-" + DynamicRightUpperId +
            "/nonzero(leftF-rightF-leftTau-rightTau-offsetMoment)=" + LeftForceNonzero + "-" + RightForceNonzero + "-" + LeftTorqueNonzero + "-" + RightTorqueNonzero + "-" + OffsetMomentNonzero +
            "/pelvis={" + ReceiverSummary(Pelvis) + "}/chest={" + ReceiverSummary(Chest) + "}" +
            "/peakSideNm(actualHipTorque-forceMirrorAtAnchor-moveUpperForce-bothAnchorForcePair-equivalentParentTorque)=" + F(PeakTorque) + "|" + F(PeakAnchorMoment) + "|" + F(PeakMovedUpperTorque) + "|" + F(PeakBothAnchorMoment) + "|" + F(PeakEquivalentParentTorque) +
            "/peakAbsSideW(actualUpper-candidatePair-bothAnchorForcePair)=" + F(PeakActualUpperPower) + "|" + F(PeakCandidatePairPower) + "|" + F(PeakBothAnchorPower) +
            "/peakAlgebraError(momentNm-parentPowerW)=" + F(PeakClosure) + "|" + F(PeakPowerTransportError) +
            "/actualUpperForcePoint=COM/actualUpperTorque=COMMITTED_PHYSICAL_HIP_TOTAL/notAllocatedToInternalJoint=True" +
            "/candidateParentWrench=UNEXECUTED_NEGATIVE_UPPER_WRENCH_TRANSPORTED_TO_PARENT_ANCHOR" +
            "/actualHipReactionWritten=False/torsoGravity=OFF/dynamicReceiverQualified=False/wholeBodyBalanceCertified=False" +
            "/idle={" + Idle.Summary() + "}/motion={" + Motion.Summary() + "}";

        private string GetDynamicDetails()
        {
            var sb = new StringBuilder(GetDynamicSummary());
            AppendDynamicFrame(sb, "PEAK_SIDE_FORCE", ForcePeakFrame);
            AppendDynamicFrame(sb, "PEAK_ANCHOR_MOMENT", AnchorMomentPeakFrame);
            AppendDynamicFrame(sb, "PEAK_CANDIDATE_PAIR_POWER", PowerPeakFrame);
            for (int k = 0; k < Count; k++) AppendDynamicFrame(sb, "TAIL", Ring[(Next - Count + k + Ring.Length) % Ring.Length]);
            return sb + "\nV04890 audits committed pre-solve Hip force and physical total torque only. All parent forces, torques, point shifts and paired powers are UNEXECUTED algebraic candidates; algebraic closure is not momentum response or passivity. Knee/weight/contact inputs, drag and torso tracking assistance remain separately owned. No settled Pelvis response, full-body energy balance or internal-joint allocation is inferred. The original Upper force remains at COM; moving it to the child anchor would add the reported Upper torque. The existing 12-row storage is reused; peak rows retain full geometry without a new observer loop.";
        }
        private static void AppendDynamicFrame(StringBuilder sb, string selection, InputFrame f)
        {
            if (f.Fixed <= 0) return;
            sb.Append("\nhipWrenchFrame=").Append(f.Fixed).Append("/selection=").Append(selection).Append("/phase=").Append(f.Idle ? "IDLE" : "MOTION_OR_SOURCE_TURN")
                .Append("/loaded(left-right)=").Append(f.LeftLoaded).Append('-').Append(f.RightLoaded);
            AppendDynamicSide(sb, "LEFT", f.LeftWrench); AppendDynamicSide(sb, "RIGHT", f.RightWrench);
        }
        private static void AppendDynamicSide(StringBuilder sb, string side, HipCommandWrench w)
        {
            sb.Append("\nhipWrenchSide=").Append(side).Append("/fixed=").Append(w.Fixed).Append('/').Append(w.Step.Status())
                .Append("/ids(upper-parent)=").Append(w.UpperBodyId).Append('-').Append(w.ParentBodyId).Append("/mass=").Append(F(w.Mass))
                .Append("/points(upperCOM-parentCOM-childAnchor-parentAnchor)=").Append(V(w.UpperCom)).Append('|').Append(V(w.ParentCom)).Append('|').Append(V(w.ChildAnchor)).Append('|').Append(V(w.ParentAnchor))
                .Append("/v(upperCOM-parentCOM-childAnchor-parentAnchor)=").Append(V(w.UpperV)).Append('|').Append(V(w.ParentV)).Append('|').Append(V(w.ChildPointV)).Append('|').Append(V(w.ParentPointV))
                .Append("/w(upper-parent)=").Append(V(w.UpperW)).Append('|').Append(V(w.ParentW))
                .Append("/actualHip(FN-tauNm)=").Append(V(w.Force)).Append('|').Append(V(w.Torque))
                .Append("/forcePairMomentNm(parentCOM-parentAnchor-bothAnchors)=").Append(V(w.ComForcePairMoment)).Append('|').Append(V(w.AnchorForcePairMoment)).Append('|').Append(V(w.BothAnchorForcePairMoment))
                .Append("/moveUpperForceToAnchorAddsTorqueNm=").Append(V(w.MoveUpperForceToAnchorTorque))
                .Append("/equivalentParentTorqueNm(COM-anchor)=").Append(V(w.EquivalentParentComTorque)).Append('|').Append(V(w.EquivalentParentAnchorTorque))
                .Append("/powerW(actualUpper-parentCOMCandidate-parentAnchorCandidate-bothAnchorForcePair)=").Append(F(w.ActualUpperPower)).Append('|').Append(F(w.CandidateParentComPower)).Append('|').Append(F(w.CandidateParentAnchorPower)).Append('|').Append(F(w.BothAnchorForcePairPower))
                .Append("/algebraMomentClosureNm=").Append(V(w.WrenchClosure)).Append("/candidateWritten=False");
        }
    }
}
