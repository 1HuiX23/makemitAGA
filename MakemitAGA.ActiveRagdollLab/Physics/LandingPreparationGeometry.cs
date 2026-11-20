using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Ordered, same-command geometry. These substitutions telescope but are not
    // independent force contributions. Candidate deltas keep the observed joint
    // errors and WORLD anchor separations; they never forecast a native solve.
    internal static class LandingPreparationGeometry
    {
        internal struct Sample
        {
            internal bool Valid, ProbeValid, VelocityValid;
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal int Fixed, FootprintMask, ProbeFootprintMask;
            internal Vector3 ParentHip, UpperHip, LowerAnkle, FootAnkle;
            internal Quaternion Upper, Lower, Foot;
            internal float Reference, ParentPlaced, UpperTracked, LowerTracked, HipShifted, KneeClosed, AnkleClosed, Actual;
            internal float HipProbe, KneeProbe, BothProbe, ProbeBaselineResidual, CornerNormalSpeed;
            internal float ParentDelta => ParentPlaced - Reference;
            internal float HipRotationDelta => UpperTracked - ParentPlaced;
            internal float KneeRotationDelta => LowerTracked - UpperTracked;
            internal float HipGapDelta => HipShifted - LowerTracked;
            internal float KneeGapDelta => KneeClosed - HipShifted;
            internal float AnkleGapDelta => AnkleClosed - KneeClosed;
            internal float FootRotationDelta => Actual - AnkleClosed;
            internal float Closure => Actual - Reference - (ParentDelta + HipRotationDelta + KneeRotationDelta +
                HipGapDelta + KneeGapDelta + AnkleGapDelta + FootRotationDelta);
        }

        internal static Sample Build(LandingPreparationCommand c, LegLandingReferenceInput input,
            FootFixedStepEvidence before, Quaternion lower, Quaternion foot, Quaternion consumedFoot)
        {
            var s = new Sample { Step = c.Step, Source = c.Source, Fixed = before.FixedSequence };
            if (!c.Applied || !c.GeometryPrepared || !input.Matches(c.Step, c.Source, before.FixedSequence) ||
                !before.Valid || !before.PreparationPoseValid || c.SupportId == 0 || before.PreparationSupportId != c.SupportId ||
                !V(c.RenderPhysicalHip) || !V(c.UpperSpan) || !V(c.LowerSpan) ||
                c.UpperSpan.sqrMagnitude < .000001f || c.LowerSpan.sqrMagnitude < .000001f ||
                !V(c.Center) || !Positive(c.Half) || !V(c.FloorCenter) || !Positive(c.FloorHalf) ||
                !V(before.LowerAnchorWorld) || !V(before.FootAnchorWorld) ||
                !Q(lower) || !Q(foot) || !Q(consumedFoot) || !Q(input.Upper) ||
                !Q(c.HipSelected) || !Q(c.KneeSelected) || !Q(c.PublishedFoot) || !Q(c.HipBaseline) || !Q(c.KneeBaseline) ||
                !SameRotation(input.UpperGoal, c.HipSelected) || !SameRotation(input.KneeRelativeGoal, c.KneeSelected) ||
                !SameRotation(consumedFoot, c.PublishedFoot) || (input.UpperHipToKnee - c.UpperSpan).magnitude > .000005f) return s;

            Vector3 referenceAnkle = Endpoint(c.RenderPhysicalHip, c.HipSelected, c.KneeSelected, c.UpperSpan, c.LowerSpan);
            // Reuse the entry path's position substitutions, now with the actual
            // Lower ankle copied from the existing pre-lifecycle observation.
            var p = FootLandingReferenceEvidence.Build(c.Step, c.Source, before.FixedSequence, input,
                referenceAnkle, before.LowerAnchorWorld, lower, input.LowerKneeAnchor + c.LowerSpan);
            if (!p.Valid) return s;
            s.ParentHip = input.ParentHip; s.UpperHip = input.UpperHip;
            s.LowerAnkle = before.LowerAnchorWorld; s.FootAnkle = before.FootAnchorWorld;
            s.Upper = input.Upper; s.Lower = lower; s.Foot = foot;
            Quaternion referenceFoot = c.HipSelected * c.KneeSelected * c.PublishedFoot;
            Quaternion upperTrackedFoot = input.Upper * c.KneeSelected * c.PublishedFoot;
            Quaternion lowerTrackedFoot = lower * c.PublishedFoot;
            s.Reference = Height(c, referenceAnkle, referenceFoot, 0, ref s.FootprintMask);
            s.ParentPlaced = Height(c, p.Goal, referenceFoot, 1, ref s.FootprintMask);
            s.UpperTracked = Height(c, p.UpperTracked, upperTrackedFoot, 2, ref s.FootprintMask);
            s.LowerTracked = Height(c, p.LowerTracked, lowerTrackedFoot, 3, ref s.FootprintMask);
            s.HipShifted = Height(c, p.HipShifted, lowerTrackedFoot, 4, ref s.FootprintMask);
            s.KneeClosed = Height(c, p.Actual, lowerTrackedFoot, 5, ref s.FootprintMask);
            s.AnkleClosed = Height(c, s.FootAnkle, lowerTrackedFoot, 6, ref s.FootprintMask);
            s.Actual = Height(c, s.FootAnkle, foot, 7, ref s.FootprintMask);
            // Intermediate points may be outside the finite Box. Keep the explicit
            // mask rather than claiming their plane heights are supported contact.
            s.Valid = (s.FootprintMask & 128) != 0 && F(s.Reference) && F(s.ParentPlaced) && F(s.UpperTracked) &&
                F(s.LowerTracked) && F(s.HipShifted) && F(s.KneeClosed) && F(s.AnkleClosed) && F(s.Actual) &&
                F(s.Closure) && Mathf.Abs(s.Closure) <= .000005f;
            if (!s.Valid) return s;
            s.VelocityValid = V(before.FootAnkleVelocityWorld) && V(before.FootAngularVelocityWorld);
            if (s.VelocityValid)
            {
                s.CornerNormalSpeed = CornerSpeed(c, foot, before.FootAnkleVelocityWorld, before.FootAngularVelocityWorld);
                s.VelocityValid = F(s.CornerNormalSpeed);
            }
            Quaternion actualKnee = Quaternion.Inverse(input.Upper) * lower;
            Quaternion actualFootLocal = Quaternion.Inverse(lower) * foot;
            Quaternion hipCandidate = input.Upper * Quaternion.Inverse(c.HipBaseline) * c.HipSelected;
            Quaternion kneeCandidate = actualKnee * Quaternion.Inverse(c.KneeBaseline) * c.KneeSelected;
            Vector3 heldDebt = s.FootAnkle - Endpoint(s.UpperHip, input.Upper, actualKnee, c.UpperSpan, c.LowerSpan);
            float baseline = Probe(c, s.UpperHip, input.Upper, actualKnee, actualFootLocal, heldDebt, 0, ref s.ProbeFootprintMask);
            s.HipProbe = Probe(c, s.UpperHip, hipCandidate, actualKnee, actualFootLocal, heldDebt, 1, ref s.ProbeFootprintMask);
            s.KneeProbe = Probe(c, s.UpperHip, input.Upper, kneeCandidate, actualFootLocal, heldDebt, 2, ref s.ProbeFootprintMask);
            s.BothProbe = Probe(c, s.UpperHip, hipCandidate, kneeCandidate, actualFootLocal, heldDebt, 3, ref s.ProbeFootprintMask);
            s.ProbeBaselineResidual = baseline - s.Actual;
            s.ProbeValid = F(baseline) && F(s.HipProbe) && F(s.KneeProbe) && F(s.BothProbe) &&
                Mathf.Abs(s.ProbeBaselineResidual) <= .000005f;
            return s;
        }

        private static float Probe(LandingPreparationCommand c, Vector3 hip, Quaternion upper, Quaternion knee,
            Quaternion footLocal, Vector3 heldDebt, int index, ref int mask) =>
            Height(c, Endpoint(hip, upper, knee, c.UpperSpan, c.LowerSpan) + heldDebt, upper * knee * footLocal, index, ref mask);
        private static Vector3 Endpoint(Vector3 hip, Quaternion upper, Quaternion knee, Vector3 u, Vector3 l) =>
            hip + upper * u + (upper * knee) * l;
        private static float Height(LandingPreparationCommand c, Vector3 ankle, Quaternion foot, int index, ref int mask)
        {
            if (FootBoxGeometry.Inside(ankle, foot, c.Center, c.Half, c.FloorCenter, c.FloorHalf)) mask |= 1 << index;
            return FootBoxGeometry.Minimum(ankle, foot, c.Center, c.Half) - c.FloorCenter.y - c.FloorHalf.y;
        }
        private static float CornerSpeed(LandingPreparationCommand c, Quaternion q, Vector3 v, Vector3 w)
        {
            float min = FootBoxGeometry.Minimum(Vector3.zero, q, c.Center, c.Half);
            float speed = float.PositiveInfinity;
            // At tied bottom corners use the minimum normal speed (one-sided
            // derivative of the minimum); rotation about the ANKLE, not the COM.
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
            {
                Vector3 r = q * (c.Center + new Vector3(x * c.Half.x, y * c.Half.y, z * c.Half.z));
                if (r.y <= min + .000001f) speed = Mathf.Min(speed, v.y + Vector3.Cross(w, r).y);
            }
            return speed;
        }
        private static bool SameRotation(Quaternion a, Quaternion b) => Q(a) && Q(b) &&
            FootRelativeReference.Log(a * Quaternion.Inverse(b)).magnitude <= .00001f;
        private static bool F(float f) => PelvisParentFrame.F(f);
        private static bool V(Vector3 v) => PelvisParentFrame.V(v);
        private static bool Q(Quaternion q) => PelvisParentFrame.Q(q) && Mathf.Abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1f) < .01f;
        private static bool Positive(Vector3 v) => V(v) && v.x > 0f && v.y > 0f && v.z > 0f;

        internal static string Text(LandingPreparationCommand c, Sample s)
        {
            if (!s.Valid) return "LANDING_GEOMETRY_V041410=NA_MISSING_STALE_OR_INVALID";
            return "LANDING_GEOMETRY_V041410=ORDERED_HEIGHT_NOT_FORCE_CAUSALITY" +
                "/epoch-fixed-local-source=" + s.Step.Epoch + "|" + s.Step.Sequence + "|" + s.Fixed + "|" + s.Source.Sequence +
                "/heightMm(ref-parent-upper-lower-hip-knee-ankle-actual)=" + N(s.Reference*1000f) + "|" + N(s.ParentPlaced*1000f) + "|" +
                N(s.UpperTracked*1000f) + "|" + N(s.LowerTracked*1000f) + "|" + N(s.HipShifted*1000f) + "|" + N(s.KneeClosed*1000f) + "|" + N(s.AnkleClosed*1000f) + "|" + N(s.Actual*1000f) +
                "/deltaMm(parent-hipR-kneeR-hipGap-kneeGap-ankleGap-footR)=" + Deltas(s) +
                "/closureMm=" + N(s.Closure*1000f) + "/footprintMask=" + s.FootprintMask + "/allInsideMask=255" +
                "/cornerNormalVmps=" + (s.VelocityValid ? N(s.CornerNormalSpeed) : "NA") +
                "/probe=SELECTED_REFERENCE_INCREMENT_WITH_OBSERVED_ERRORS_AND_WORLD_GAPS_NOT_ADMISSIBILITY_OR_PREDICTION" +
                "/probeGainMm(hip-knee-both)=" + (s.ProbeValid ? N((s.HipProbe-s.Actual)*1000f) + "|" + N((s.KneeProbe-s.Actual)*1000f) + "|" + N((s.BothProbe-s.Actual)*1000f) : "NA") +
                "/probeBaselineResidualMm=" + (s.ProbeValid ? N(s.ProbeBaselineResidual*1000f) : "NA") + "/probeFootprintMask=" + s.ProbeFootprintMask + "/probeAllInsideMask=15" +
                "/hipPointsM(renderPhysical-parentPhysical-upper)=" + Vec(c.RenderPhysicalHip) + "|" + Vec(s.ParentHip) + "|" + Vec(s.UpperHip) +
                "/anklesM(lower-foot)=" + Vec(s.LowerAnkle) + "|" + Vec(s.FootAnkle) +
                "/spansM(upper-lower)=" + Vec(c.UpperSpan) + "|" + Vec(c.LowerSpan) +
                "/boxM(center-half-floorCenter-floorHalf)=" + Vec(c.Center) + "|" + Vec(c.Half) + "|" + Vec(c.FloorCenter) + "|" + Vec(c.FloorHalf) +
                "/refR(hipBase-hipSelected-kneeBase-kneeSelected-foot)=" + Rot(c.HipBaseline) + "|" + Rot(c.HipSelected) + "|" + Rot(c.KneeBaseline) + "|" + Rot(c.KneeSelected) + "|" + Rot(c.PublishedFoot) +
                "/actualR(upper-lower-foot)=" + Rot(s.Upper) + "|" + Rot(s.Lower) + "|" + Rot(s.Foot);
        }
        internal static string Deltas(Sample s) => N(s.ParentDelta*1000f) + "|" + N(s.HipRotationDelta*1000f) + "|" + N(s.KneeRotationDelta*1000f) + "|" + N(s.HipGapDelta*1000f) + "|" + N(s.KneeGapDelta*1000f) + "|" + N(s.AnkleGapDelta*1000f) + "|" + N(s.FootRotationDelta*1000f);
        private static string N(float v) => FootGroundTrajectory.N(v);
        private static string Vec(Vector3 v) => "(" + N(v.x) + "," + N(v.y) + "," + N(v.z) + ")";
        private static string Rot(Quaternion q) => "(" + N(q.x) + "," + N(q.y) + "," + N(q.z) + "," + N(q.w) + ")";
    }
}
