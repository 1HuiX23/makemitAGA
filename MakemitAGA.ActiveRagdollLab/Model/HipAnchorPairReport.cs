using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed class HipAnchorPairReport
    {
        internal bool Selected, Finite = true, CurrentApplied;
        internal string Issue;
        internal int ChildId, ParentId, Samples, Writes, Nonzero, Clamps, RollbackFailures, CancelledCommands;
        internal float PeakForceN, PeakGapM, PeakGeometryMomentNm, PeakUpperTorqueNm, PeakParentTorqueNm;
        internal float PeakFreeResponseError, PeakPositiveDampingPower, RejectedGapM, PeakFeedbackResidualA;
        internal float PeakUpperCorrectionNm, PeakUpperActuatorNm, PeakUpperNetTorqueResidualNm;
        internal int MotionLimitedSteps, FeedbackFallbackSteps;
        internal float MinimumMotionScale = 1f;
        internal int PointBudgetSteps, UpperCorrectionBudgetSteps, UpperActuatorBudgetSteps, ReceiverBudgetSteps;
        internal struct Command
        {
            internal bool JointSolved, FullFeedbackFeasible;
            internal float MotionScale;
            internal Vector3 NetHipTorque, CommonAcceleration, AllocatedHipAlpha;
            internal Vector3 UnboundedForce, PeerForce, PureHipTorque, KnownBiasA, PredictedNetA, FeedbackResidualA;
            internal Vector3 UpperCorrectionTorque, UpperActuatorTorque, UpperCorrectionAlpha;
            internal float UpperActuatorLimitNm;
            internal float PointScale, UpperCorrectionScale, UpperActuatorScale, ReceiverScale, BilateralDampingPower;
            internal PhysicsStepContext Step;
            internal PelvisHipCommandFrame ParentInput;
            internal int Fixed, ChildId, ParentId;
            internal float ChildMass, ParentMass, Scale, DampingPower, PairPower;
            internal Vector3 ChildCom, ParentCom, ChildPoint, ParentPoint, ChildV, ParentV, Error, VelocityError;
            internal Vector3 FeedbackA, DesiredA, AppliedA, Force, ChildTorque, ParentTorque, GeometryMoment;
            internal Vector3 ParentResponseAlpha;
        }
        internal Command Current, ForcePeak, GapPeak;
        internal bool EvidenceReady => Selected && Finite && Samples >= 80 && Writes == Samples && Nonzero >= 30 && RollbackFailures == 0;
        internal void ResetTrace()
        {
            Samples = Writes = Nonzero = Clamps = CancelledCommands = 0;
            MotionLimitedSteps = FeedbackFallbackSteps = 0; MinimumMotionScale = 1f;
            PointBudgetSteps = UpperCorrectionBudgetSteps = UpperActuatorBudgetSteps = ReceiverBudgetSteps = 0; PeakFeedbackResidualA = 0f;
            PeakUpperCorrectionNm = PeakUpperActuatorNm = PeakUpperNetTorqueResidualNm = 0f;
            PeakForceN = PeakGapM = PeakGeometryMomentNm = PeakUpperTorqueNm = PeakParentTorqueNm = 0f;
            PeakFreeResponseError = PeakPositiveDampingPower = RejectedGapM = 0f; ForcePeak = GapPeak = new Command();
        }
        internal void Record(Command c, float responseError)
        {
            if (c.MotionScale < 0.999999f) MotionLimitedSteps++;
            if (!c.FullFeedbackFeasible) FeedbackFallbackSteps++;
            MinimumMotionScale = Mathf.Min(MinimumMotionScale, c.MotionScale);
            Samples++; Writes++; if (c.Force.magnitude > 0.000001f) Nonzero++;
            if (c.Scale < 0.999999f || c.MotionScale < 0.999999f) Clamps++;
            if (c.PointScale < 0.999999f) PointBudgetSteps++;
            if (c.UpperCorrectionScale < 0.999999f) UpperCorrectionBudgetSteps++;
            if (c.UpperActuatorScale < 0.999999f) UpperActuatorBudgetSteps++;
            if (c.ReceiverScale < 0.999999f) ReceiverBudgetSteps++;
            PeakFeedbackResidualA = Mathf.Max(PeakFeedbackResidualA, c.FeedbackResidualA.magnitude);
            PeakUpperCorrectionNm = Mathf.Max(PeakUpperCorrectionNm, c.UpperCorrectionTorque.magnitude);
            PeakUpperActuatorNm = Mathf.Max(PeakUpperActuatorNm, c.UpperActuatorTorque.magnitude);
            PeakUpperNetTorqueResidualNm = Mathf.Max(PeakUpperNetTorqueResidualNm, (c.UpperActuatorTorque + c.ChildTorque - c.NetHipTorque).magnitude);
            if (ForcePeak.Fixed == 0 || c.Force.magnitude > PeakForceN) ForcePeak = c;
            if (GapPeak.Fixed == 0 || c.Error.magnitude > PeakGapM) GapPeak = c;
            PeakForceN = Mathf.Max(PeakForceN, c.Force.magnitude); PeakGapM = Mathf.Max(PeakGapM, c.Error.magnitude);
            PeakGeometryMomentNm = Mathf.Max(PeakGeometryMomentNm, c.GeometryMoment.magnitude);
            PeakUpperTorqueNm = Mathf.Max(PeakUpperTorqueNm, c.ChildTorque.magnitude); PeakParentTorqueNm = Mathf.Max(PeakParentTorqueNm, c.ParentTorque.magnitude);
            PeakFreeResponseError = Mathf.Max(PeakFreeResponseError, responseError); PeakPositiveDampingPower = Mathf.Max(PeakPositiveDampingPower, c.DampingPower);
        }
        internal string GetDetails()
        {
            var sb = new StringBuilder("mode=HIP_ANCHOR_TRANSLATION_PAIR_V041000_MOTION_FEASIBILITY_ALLOCATION/selected-finite-applied=" + Selected + "-" + Finite + "-" + CurrentApplied +
                "/issue=" + (Issue ?? "<none>") + "/rejectedGapMm=" + F(RejectedGapM * 1000f) + "/inputEvidenceReady=" + EvidenceReady + "/ids(child-parent)=" + ChildId + "-" + ParentId +
                "/samples-writes-nonzero-clamps-rollbackFailures-cancelled=" + Samples + "-" + Writes + "-" + Nonzero + "-" + Clamps + "-" + RollbackFailures + "-" + CancelledCommands +
                "/peak(forceN-gapMm-geometryMomentNm-upperTorqueNm-parentTorqueNm)=" + F(PeakForceN) + "|" + F(PeakGapM * 1000f) + "|" + F(PeakGeometryMomentNm) + "|" + F(PeakUpperTorqueNm) + "|" + F(PeakParentTorqueNm) +
                "/peak(modelResponseErrorMps2-positiveDampingPowerW)=" + F(PeakFreeResponseError) + "|" + F(PeakPositiveDampingPower) +
                "/peakUpperTorque(correctionNm-actuatorNm-netResidualNm)=" + F(PeakUpperCorrectionNm) + "|" + F(PeakUpperActuatorNm) + "|" + F(PeakUpperNetTorqueResidualNm) +
                "/basePD=400_40_IMPLICIT_FIXED_DT/compensatedForcePointA_cap=60/upperCorrectionTorqueCapNm=8/upperActuatorCapNm=MAX_8_ORIGINAL_REQUEST/receiverTotalTorqueCapNm=8" +
                "/motionAllocation(limited-fallback-minScale)=" + MotionLimitedSteps + "-" + FeedbackFallbackSteps + "-" + F(MinimumMotionScale) +
                "/fullRequestWouldLimit(point-upperCorrection-upperActuator-receiver)=" + PointBudgetSteps + "-" + UpperCorrectionBudgetSteps + "-" + UpperActuatorBudgetSteps + "-" + ReceiverBudgetSteps +
                "/peakFeedbackResidualA=" + F(PeakFeedbackResidualA) + "/allocation=FEEDBACK_PRIORITY_COMMON_AND_UPPER_MOTION_ALLOCATION/diagonal=I_OVER_UPPER_MASS_PLUS_I_OVER_TOTAL_MASS/offDiagonal=I_OVER_TOTAL_MASS" +
                "/upperCorrection=MINUS_R_CROSS_F/netUpperCommand=SCALED_CONTROLLER_TORQUE/upperActuatorTorque=ALLOCATED_NET_PLUS_POINT_CORRECTION" +
                "/bias=FINAL_COMMON_AND_NET_UPPER_TORQUE/pointModel=FIXED_LEVERS_OTHER_LOADS_AND_DRAG_EXCLUDED" +
                "/forcePointA_excludes=BASELINE_BIAS_OTHER_LOADS_DRAG/netUpperAngularInput=ALLOCATED_BEFORE_KNEE/dampingGate=BILATERAL_SUM_ONLY" +
                "/forcePairClosure=ALGEBRAIC_ZERO/noParentAngularControllerMirror=True/wholeBodyBalanceCertified=False");
            Append(sb, "PEAK_FORCE", ForcePeak); Append(sb, "PEAK_GAP", GapPeak); Append(sb, "CURRENT", Current);
            return sb.ToString();
        }
        private static void Append(StringBuilder sb, string tag, Command c)
        {
            if (c.Fixed <= 0) return;
            sb.Append("\nhipPairCommand=").Append(c.Fixed).Append("/selection=").Append(tag).Append('/').Append(c.Step.Status())
                .Append("/ids(child-parent)=").Append(c.ChildId).Append('-').Append(c.ParentId)
                .Append("/mass(child-parent)=").Append(F(c.ChildMass)).Append('|').Append(F(c.ParentMass))
                .Append("/COM(child-parent)=").Append(V(c.ChildCom)).Append('|').Append(V(c.ParentCom))
                .Append("/point(child-parent)=").Append(V(c.ChildPoint)).Append('|').Append(V(c.ParentPoint))
                .Append("/pointV(child-parent)=").Append(V(c.ChildV)).Append('|').Append(V(c.ParentV))
                .Append("/error-velocityError=").Append(V(c.Error)).Append('|').Append(V(c.VelocityError))
                .Append("/feedbackA=").Append(V(c.FeedbackA)).Append("/commonCommand=").Append(c.ParentInput.Step.Status())
                .Append("/commonBodyId=").Append(c.ParentInput.BodyId).Append("/baselineCommonA=").Append(V(c.ParentInput.Acceleration))
                .Append("/chestBodyId-mass=").Append(c.ParentInput.ChestBodyId).Append('|').Append(F(c.ParentInput.Response.ChestMass))
                .Append("/chestLever=").Append(V(c.ParentInput.Response.ChestLever)).Append("/receiverTorque=").Append(V(c.ParentInput.ReceiverTorque))
                .Append("/parentCoupledAlpha=").Append(V(c.ParentResponseAlpha))
                .Append("/compensatedForcePointA(unbounded-applied)=").Append(V(c.DesiredA)).Append('|').Append(V(c.AppliedA)).Append("/scale=").Append(F(c.Scale))
                .Append("/actualForce(child-parent)=").Append(V(c.Force)).Append('|').Append(V(-c.Force))
                .Append("/actualGeometryTorque(child-parent-net)=").Append(V(c.ChildTorque)).Append('|').Append(V(c.ParentTorque)).Append('|').Append(V(c.GeometryMoment))
                .Append("/powerW(pair-ownDamping-bilateralDamping)=").Append(F(c.PairPower)).Append('|').Append(F(c.DampingPower)).Append('|').Append(F(c.BilateralDampingPower))
                .Append("/jointSolve=").Append(c.JointSolved).Append("/fullRequestForceOnlyScales(point-upperCorrection-upperActuator-receiver)=").Append(F(c.PointScale)).Append('|').Append(F(c.UpperCorrectionScale)).Append('|').Append(F(c.UpperActuatorScale)).Append('|').Append(F(c.ReceiverScale))
                .Append("/jointForce(unbounded-peer)=").Append(V(c.UnboundedForce)).Append('|').Append(V(c.PeerForce))
                .Append("/upperTorque(originalRequest-pointCorrection-actuator-netResidual)=").Append(V(c.PureHipTorque)).Append('|').Append(V(c.UpperCorrectionTorque))
                .Append('|').Append(V(c.UpperActuatorTorque)).Append('|').Append(V(c.UpperActuatorTorque + c.ChildTorque - c.NetHipTorque))
                .Append("/allocation(motionScale-fullFeedbackFeasible-forceScale)=").Append(F(c.MotionScale)).Append('|').Append(c.FullFeedbackFeasible).Append('|').Append(F(c.Scale))
                .Append("/allocatedCommonA-netHipTorque-alpha=").Append(V(c.CommonAcceleration)).Append('|').Append(V(c.NetHipTorque)).Append('|').Append(V(c.AllocatedHipAlpha))
                .Append("/upperActuatorLimitNm=").Append(F(c.UpperActuatorLimitNm))
                .Append("/upperCorrectionAlpha=").Append(V(c.UpperCorrectionAlpha))
                .Append("/jointA(bias-planned-feedbackResidual)=").Append(V(c.KnownBiasA)).Append('|').Append(V(c.PredictedNetA)).Append('|').Append(V(c.FeedbackResidualA));
        }
        private static string F(float v) => v.ToString("G9", CultureInfo.InvariantCulture);
        private static string V(Vector3 v) => "(" + F(v.x) + "," + F(v.y) + "," + F(v.z) + ")";
    }
}
