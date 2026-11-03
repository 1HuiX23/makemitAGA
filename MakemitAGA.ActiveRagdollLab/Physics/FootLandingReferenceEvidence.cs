using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Pure ordered geometric substitutions, stored only in the existing nine-row entry ring.
    // PRE_N bodies + FOR_N selected rotations + held Render source; no advance or feedback.
    internal static class FootLandingReferenceEvidence
    {
        internal struct LandingPath
        {
            internal bool Valid;
            internal string Reason;
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal int Fixed;
            internal float SupportTop;
            internal Vector3 Raw, Goal, UpperTracked, LowerTracked, HipShifted, Actual;
            internal bool UpstreamValid, NormalApplied, ReceiverParentReference;
            internal Vector3 RawChain, ParentPlaced, ParentRotated, SourceFiltered, Governed;
            internal Vector3 Composite, NormalInput, HipSelected;
            internal bool ParentPlacementValid, KneeControlValid;
            internal Vector3 ParentRaw, ParentBoundRaw, ParentReferenceP, ParentReference, ParentResponseP, ParentActual, ParentTotal;
            internal LandingKneeControlInput Knee;
        }

        internal static LandingPath Build(PhysicsStepContext step, SourceStepContext source, int localFixed,
            LegLandingReferenceInput input, Vector3 rawAnkle, Vector3 actualAnkle, Quaternion lower, Vector3 scaledAnkle)
        {
            LandingPath p = new LandingPath { Step = step, Source = source, Fixed = localFixed, Reason = "MISSING_OR_STALE_INPUT" };
            if (!input.Matches(step, source, localFixed) || !PelvisParentFrame.Q(lower) ||
                !PelvisParentFrame.V(rawAnkle) || !PelvisParentFrame.V(actualAnkle) || !PelvisParentFrame.V(scaledAnkle)) return p;
            Vector3 upperSpan = input.UpperHipToKnee, lowerSpan = scaledAnkle - input.LowerKneeAnchor;
            p.Raw = rawAnkle;
            p.Goal = input.ParentHip + input.UpperGoal * upperSpan + (input.UpperGoal * input.KneeRelativeGoal) * lowerSpan;
            p.UpperTracked = input.ParentHip + input.Upper * upperSpan + (input.Upper * input.KneeRelativeGoal) * lowerSpan;
            p.LowerTracked = input.ParentHip + input.Upper * upperSpan + lower * lowerSpan;
            p.HipShifted = input.UpperHip + input.Upper * upperSpan + lower * lowerSpan;
            p.Actual = actualAnkle;
            p.Valid = PelvisParentFrame.V(p.Goal) && PelvisParentFrame.V(p.UpperTracked) &&
                PelvisParentFrame.V(p.LowerTracked) && PelvisParentFrame.V(p.HipShifted);
            p.Reason = p.Valid ? "OK" : "NONFINITE_GEOMETRY";
            if (p.Valid)
            {
                CaptureParentPlacement(ref p, input);
                p.KneeControlValid = input.Knee.Matches(step, source, localFixed);
                if (p.KneeControlValid) p.Knee = input.Knee;
            }
            if (p.Valid && input.UpstreamMatches(step, source, localFixed))
            {
                HipGoalPathEvidence h = input.HipPath;
                Quaternion knee = input.RawKneeRelative;
                p.RawChain = input.RawHip + h.RawUpper * upperSpan + input.RawLower * lowerSpan;
                p.ParentPlaced = input.ParentHip + h.RawUpper * upperSpan + input.RawLower * lowerSpan;
                p.ParentRotated = Endpoint(input.ParentHip, h.RawInControl, knee, upperSpan, lowerSpan);
                p.SourceFiltered = Endpoint(input.ParentHip, h.AfterSource, knee, upperSpan, lowerSpan);
                p.Governed = Endpoint(input.ParentHip, h.AfterGovernor, knee, upperSpan, lowerSpan);
                p.Composite = Endpoint(input.ParentHip, h.AfterComposite, knee, upperSpan, lowerSpan);
                p.NormalInput = Endpoint(input.ParentHip, input.Normal.InputR, knee, upperSpan, lowerSpan);
                p.HipSelected = Endpoint(input.ParentHip, input.UpperGoal, knee, upperSpan, lowerSpan);
                p.NormalApplied = input.Normal.Applied;
                p.ReceiverParentReference = input.Normal.ReceiverParentReference;
                p.UpstreamValid = PelvisParentFrame.V(p.RawChain) && PelvisParentFrame.V(p.ParentPlaced) &&
                    PelvisParentFrame.V(p.ParentRotated) && PelvisParentFrame.V(p.SourceFiltered) &&
                    PelvisParentFrame.V(p.Governed) && PelvisParentFrame.V(p.Composite) &&
                    PelvisParentFrame.V(p.NormalInput) && PelvisParentFrame.V(p.HipSelected);
            }
            return p;
        }

        private static void CaptureParentPlacement(ref LandingPath p, LegLandingReferenceInput input)
        {
            LandingParentPlacementInput f = input.Parent;
            if (!f.Matches(p.Step, p.Source) || !PelvisParentFrame.V(input.RawHip)) return;
            p.ParentRaw = input.RawHip;
            p.ParentBoundRaw = f.RawP + f.RawR * f.LocalAnchor;
            p.ParentReferenceP = f.ReferenceP + f.RawR * f.LocalAnchor;
            p.ParentReference = f.ReferenceP + f.ReferenceR * f.LocalAnchor;
            p.ParentResponseP = f.ActualP + f.ReferenceR * f.LocalAnchor;
            p.ParentActual = f.ActualP + f.ActualR * f.LocalAnchor;
            p.ParentTotal = input.ParentHip - input.RawHip;
            p.ParentPlacementValid = PelvisParentFrame.V(p.ParentBoundRaw) && PelvisParentFrame.V(p.ParentReferenceP) &&
                PelvisParentFrame.V(p.ParentReference) && PelvisParentFrame.V(p.ParentResponseP) &&
                PelvisParentFrame.V(p.ParentActual) && PelvisParentFrame.V(p.ParentTotal);
        }

        private static Vector3 Endpoint(Vector3 hip, Quaternion upper, Quaternion knee, Vector3 u, Vector3 v) =>
            hip + upper * u + (upper * knee) * v;

        internal static string SampleText(LandingPath p)
        {
            if (!p.Valid) return "anklePathV041140=NA/reason=" + (p.Reason ?? "NOT_CAPTURED");
            Vector3 target = p.Goal - p.Raw, hipRotation = p.UpperTracked - p.Goal;
            Vector3 kneeRotation = p.LowerTracked - p.UpperTracked, hipGap = p.HipShifted - p.LowerTracked;
            Vector3 kneeGap = p.Actual - p.HipShifted;
            Vector3 closure = p.Actual - p.Raw - (target + hipRotation + kneeRotation + hipGap + kneeGap);
            return "anklePathV041140=CLOSED_CHAIN_GEOMETRY_NOT_POSITION_COMMAND" +
                "/fixed-physics-source=" + p.Fixed + "|" + p.Step.Sequence + "|" + p.Source.Sequence +
                "/fixedTime-sourceTime=" + S(p.Step.FixedTime) + "|" + S(p.Source.Time) +
                "/worldM(raw-goal-actual)=" + Vec(p.Raw) + "|" + Vec(p.Goal) + "|" + Vec(p.Actual) +
                "/heightMm(raw-goal-upper-lower-hip-actual)=" + S((p.Raw.y - p.SupportTop) * 1000f) + "|" +
                S((p.Goal.y - p.SupportTop) * 1000f) + "|" + S((p.UpperTracked.y - p.SupportTop) * 1000f) + "|" +
                S((p.LowerTracked.y - p.SupportTop) * 1000f) + "|" + S((p.HipShifted.y - p.SupportTop) * 1000f) + "|" + S((p.Actual.y - p.SupportTop) * 1000f) +
                "/deltaMm(target-hipR-kneeR-hipGap-kneeGap)=" + Vec(target * 1000f) + "|" + Vec(hipRotation * 1000f) + "|" +
                Vec(kneeRotation * 1000f) + "|" + Vec(hipGap * 1000f) + "|" + Vec(kneeGap * 1000f) +
                "/closureMm=" + Vec(closure * 1000f) + "/" + UpstreamText(p) +
                "/" + ParentPlacementText(p) + "/" + KneeControlText(p);
        }

        // The parent remap into the normal planner is distinct from its integrated output.
        // Stage differences telescope; they are ordered geometry, not unique force causation.
        private static string UpstreamText(LandingPath p)
        {
            if (!p.UpstreamValid) return "ankleUpstreamV041150=NA_MISSING_STALE_OR_INVALID";
            Vector3 geometry = p.RawChain - p.Raw, parentP = p.ParentPlaced - p.RawChain;
            Vector3 parentR = p.ParentRotated - p.ParentPlaced, source = p.SourceFiltered - p.ParentRotated;
            Vector3 governor = p.Governed - p.SourceFiltered, composite = p.Composite - p.Governed;
            Vector3 normalInput = p.NormalInput - p.Composite, normalPlan = p.HipSelected - p.NormalInput;
            Vector3 knee = p.Goal - p.HipSelected;
            Vector3 closure = p.Goal - p.Raw - (geometry + parentP + parentR + source + governor + composite + normalInput + normalPlan + knee);
            return "ankleUpstreamV041150=ORDERED_GEOMETRY_NOT_CAUSAL_FORCE" +
                "/normal(applied-receiverParent)=" + p.NormalApplied + "-" + p.ReceiverParentReference +
                "/upstreamDeltaMm(geometry-parentP-parentR-source-governor-composite-normalInput-normalPlan-knee)=" +
                Vec(geometry * 1000f) + "|" + Vec(parentP * 1000f) + "|" + Vec(parentR * 1000f) + "|" +
                Vec(source * 1000f) + "|" + Vec(governor * 1000f) + "|" + Vec(composite * 1000f) + "|" +
                Vec(normalInput * 1000f) + "|" + Vec(normalPlan * 1000f) + "|" + Vec(knee * 1000f) +
                "/upstreamClosureMm=" + Vec(closure * 1000f);
        }

        // Preserve order and domains: source Upper origin -> fixed bind anchor in
        // held calibrated source -> FOR_N reference P/R -> PRE_N actual P/R.
        // bindLocal includes source local translation and bind mismatch; it is not
        // automatically an animation fault. responseP is origin, not COM tracking.
        private static string ParentPlacementText(LandingPath p)
        {
            if (!p.ParentPlacementValid) return "parentPlacementV041160=NA_MISSING_STALE_OR_INVALID";
            Vector3 bind = p.ParentBoundRaw - p.ParentRaw, referenceP = p.ParentReferenceP - p.ParentBoundRaw;
            Vector3 referenceR = p.ParentReference - p.ParentReferenceP, responseP = p.ParentResponseP - p.ParentReference;
            Vector3 responseR = p.ParentActual - p.ParentResponseP;
            Vector3 closure = p.ParentTotal - (bind + referenceP + referenceR + responseP + responseR);
            return "parentPlacementV041160=ORDERED_PROXY_FRAME_HIP_POINTS" +
                "/hipHeightMm(raw-bound-referenceP-referencePose-responseP-actual)=" +
                S((p.ParentRaw.y - p.SupportTop) * 1000f) + "|" + S((p.ParentBoundRaw.y - p.SupportTop) * 1000f) + "|" +
                S((p.ParentReferenceP.y - p.SupportTop) * 1000f) + "|" + S((p.ParentReference.y - p.SupportTop) * 1000f) + "|" +
                S((p.ParentResponseP.y - p.SupportTop) * 1000f) + "|" + S((p.ParentActual.y - p.SupportTop) * 1000f) +
                "/parentDeltaMm(bindLocal-referenceP-referenceR-responseP-responseR)=" +
                Vec(bind * 1000f) + "|" + Vec(referenceP * 1000f) + "|" + Vec(referenceR * 1000f) + "|" +
                Vec(responseP * 1000f) + "|" + Vec(responseR * 1000f) + "/parentClosureMm=" + Vec(closure * 1000f);
        }

        private static string KneeControlText(LandingPath p)
        {
            if (!p.KneeControlValid) return "kneeEntryControlV041170=NA_MISSING_STALE_OR_INVALID";
            LandingKneeControlInput k = p.Knee;
            float feedbackResidual = k.Feedback - (k.P + k.D);
            float allocationResidual = k.FinalFlexion - Mathf.Clamp(k.Feedback + k.AllocatedFF, -k.MaxFlexionA, k.MaxFlexionA);
            Vector3 transportResidual = k.TotalA - (k.RelativeA + k.ParentA);
            return "kneeEntryControlV041170=PRE_N_FEEDBACK_FOR_N_COMMAND" +
                "/kneeFFInputDomain=" + (k.ReferenceSelected ? "PLANNED_RELATIVE_A" : "RAW_SOURCE_RELATIVE_A") +
                "/kneeAxis(degAbs-targetW-bodyW-errorW)=" + S(k.ErrorAbsDegrees) + "|" + S(k.TargetW) + "|" + S(k.BodyW) + "|" + S(k.VelocityError) +
                "/kneeFlexionA(P-D-feedback-allocatedFF-final-limit)=" + S(k.P) + "|" + S(k.D) + "|" + S(k.Feedback) + "|" +
                S(k.AllocatedFF) + "|" + S(k.FinalFlexion) + "|" + S(k.MaxFlexionA) +
                "/kneeFF(valid-active-clamped-independent)=" + k.FeedforwardValid + "-" + k.FeedforwardActive + "-" + k.FeedforwardClamped + "-" + k.IndependentGate +
                "/kneeFF(input-selected-reason)=" + (k.FeedforwardValid ? S(k.InputFF) + "|" + S(k.SelectedFF) + "|" + k.FeedforwardReason : "NA") +
                "/kneeFFPolicyV041180(selected-stateBits)=" + (k.FeedforwardValid ? k.PlannedSourceRecoverySelected + "|" + k.FeedforwardStateBits : "NA") +
                "/kneeClamp(flexion-soft)=" + k.FlexionClamped + "-" + k.SoftClamped +
                "/kneeSoftA(raw-applied-limit)=" + S(k.SoftRawA) + "|" + S(k.SoftAppliedA) + "|" + S(k.SoftMaxA) +
                "/kneeSoftPhase=" + (k.SoftPhase ?? "<none>") +
                "/kneeParentTransport=" + k.ParentTransportEnabled +
                "/kneeWorldA(relative-parent-total)=" + Vec(k.RelativeA) + "|" + Vec(k.ParentA) + "|" + Vec(k.TotalA) +
                "/kneeReceiptResidual(feedback-allocation-transport)=" + S(feedbackResidual) + "|" + S(allocationResidual) + "|" + Vec(transportResidual) +
                "/" + KneeReferenceText(p);
        }
        private static string KneeReferenceText(LandingPath p)
        {
            if (!p.Knee.ReferenceSelected) return "kneeReferenceV041170=RAW_BASELINE";
            KneeReferenceState r = p.Knee.Reference;
            if (!r.Matches(p.Step, p.Source) || !PelvisParentFrame.V(p.Knee.RawSourceA))
                return "kneeReferenceV041170=NA_MISSING_STALE_OR_INVALID";
            return "kneeReferenceV041170=UPPER_LOCAL_RELATIVE_RWA" +
                "/kneeReference(stale-alphaLimited-speedLimited)=" + r.Stale + "|" + r.AlphaLimited + "|" + r.SpeedLimited +
                "/kneeReferenceR(previous-raw-final)=" + Quat(r.PreviousR) + "|" + Quat(r.InputR) + "|" + Quat(r.R) +
                "/kneeReferenceW(previous-raw-final)=" + Vec(r.PreviousW) + "|" + Vec(r.InputW) + "|" + Vec(r.W) +
                "/kneeReferenceA(raw-final)=" + Vec(p.Knee.RawSourceA) + "|" + Vec(r.A) +
                "/kneeReferenceLimits(W-A)=" + S(r.MaxW) + "|" + S(r.MaxA);
        }
        private static string S(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
        private static string Vec(Vector3 v) => "(" + S(v.x) + "," + S(v.y) + "," + S(v.z) + ")";
        private static string Quat(Quaternion q) => "(" + S(q.x) + "," + S(q.y) + "," + S(q.z) + "," + S(q.w) + ")";
    }
}
