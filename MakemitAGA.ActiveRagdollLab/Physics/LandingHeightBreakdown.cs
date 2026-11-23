using MakemitAGA.ActiveRagdollLab.Model;
using System.Text;
using UnityEngine;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal struct LandingHeightBreakdown
    {
        internal bool Parent, Complete, Response, Recovery, RecoverySelected, RecoveryEligible, Ground, GroundSelected, GroundJoint;
        internal int InputFixed, WitnessMask, ApproachMask, WitnessSupportId;
        internal bool Braking, BrakingSelected, GroundGeometry;
        internal float BrakingExtraV;
        internal string GroundReason;
        internal float Reference, ResponseY, Binding, Rotation, Tracking, Aligned, Closure, Q, RecoveryExtra;

        internal static LandingHeightBreakdown Capture(in Trace.Tick t, in LegLandingReferenceInput input, in FootLandingReferenceEvidence.LandingPath path,
            in PelvisNormalResponseEvidence previous, in FootGroundReference.GroundSample ground,
            Quaternion rawFoot, Vector3 center, Vector3 half, float top)
        {
            var h = new LandingHeightBreakdown();
            if (previous.Matches(t.Step))
            {
                h.Response = true; h.InputFixed = previous.Input.Sequence; h.Q = previous.Q;
                h.Braking = previous.BrakingCaptured; h.BrakingSelected = previous.BrakingSelected;
                h.WitnessMask = previous.WitnessMask; h.ApproachMask = previous.ApproachMask;
                h.WitnessSupportId = previous.WitnessSupportId; h.BrakingExtraV = previous.BrakingExtraV;
                h.Recovery = previous.RecoveryCaptured; h.RecoverySelected = previous.RecoverySelected;
                h.RecoveryEligible = previous.RecoveryEligible; h.RecoveryExtra = previous.RecoveryExtraY;
            }
            if (ground.Source.Matches(t.Source.Clock))
            {
                h.Ground = true; h.GroundGeometry = ground.Valid; h.GroundSelected = ground.Selected;
                h.GroundJoint = ground.JointChain; h.GroundReason = ground.Reason;
            }
            if (!t.EntryReferenceValid || t.Floor == 0 || !float.IsFinite(top) ||
                !float.IsFinite(t.SourceMin) || !float.IsFinite(t.ControlMin)) return h;

            // Complete and compact captures describe the same three pelvis socket heights.
            if (input.Fixed != t.Local || !input.Step.Matches(t.Step) || !input.Source.Matches(t.Source.Clock) ||
                !input.Parent.Matches(t.Step, t.Source.Clock) || !PelvisParentFrame.V(input.RawHip)) return h;
            Vector3 reference = input.Parent.ReferencePoint, actual = input.Parent.ActualPoint;
            if (!PelvisParentFrame.V(reference) || !PelvisParentFrame.V(actual)) return h;
            h.Parent = true; h.Reference = reference.y - input.RawHip.y; h.ResponseY = actual.y - reference.y;

            if (!input.Matches(t.Step, t.Source.Clock, t.Local) || !path.Valid || !path.RawSpanValid || !path.ParentPlacementValid ||
                !path.Step.Matches(t.Step) || !path.Source.Matches(t.Source.Clock) || path.Fixed != t.Local ||
                !RightFootTorqueForecast.ValidRotation(rawFoot) || !RightFootTorqueForecast.ValidRotation(path.EntryFootRelative) ||
                !RightFootTorqueForecast.ValidRotation(path.EntryLower) ||
                !PelvisParentFrame.V(center) || !PelvisParentFrame.V(half) || half.x <= 0f || half.y <= 0f || half.z <= 0f)
                return h;
            const float tol = FootBoxGeometry.SupportPlaneToleranceMeters;
            if (Mathf.Abs(path.SupportTop - top) > tol || (input.ParentHip - actual).magnitude > tol ||
                (path.Raw - t.Target).magnitude > tol || (path.Actual - t.Lower).magnitude > tol) return h;
            var selectedFoot = input.UpperGoal * input.KneeRelativeGoal * path.EntryFootRelative;
            if (!RightFootTorqueForecast.ValidRotation(selectedFoot)) return h;
            float rawOffset = FootBoxGeometry.Minimum(Vector3.zero, rawFoot, center, half);
            float selectedOffset = FootBoxGeometry.Minimum(Vector3.zero, selectedFoot, center, half);
            float recordedControl = FootBoxGeometry.Minimum(path.Actual, path.EntryLower * path.EntryFootRelative, center, half) - top;
            if (Mathf.Abs(path.Raw.y + rawOffset - top - t.SourceMin) > tol || Mathf.Abs(recordedControl - t.ControlMin) > tol) return h;

            // Substitute whole-box minima at every stage. Its lowest corner is allowed to change.
            float rawChain = input.RawHip.y + path.RawSpanY + rawOffset - top;
            float selectedChain = path.Goal.y + selectedOffset - top;
            h.Aligned = input.RawHip.y + (path.Goal.y - input.ParentHip.y) + selectedOffset - top;
            h.Binding = rawChain - t.SourceMin;
            h.Rotation = h.Aligned - rawChain;
            // The entry control shoe actually sits on the settled Lower ankle, not the ideal chain endpoint.
            h.Tracking = t.ControlMin - selectedChain;
            h.Closure = (t.ControlMin - t.SourceMin) - (h.Binding + h.Reference + h.ResponseY + h.Rotation + h.Tracking);
            h.Complete = float.IsFinite(h.Closure) && Mathf.Abs(h.Closure) <= tol;
            return h;
        }
    }

    internal sealed partial class BilateralFootTransitionTrace
    {
        private sealed partial class LeftLandingPath
        {
            internal void AppendHeights(StringBuilder b, int start, int first, bool turnTrial)
            {
                b.Append("\nTURN_HEIGHT_V041710/start-first=").Append(start).Append('|').Append(first)
                    .Append("/turnTrial=").Append(turnTrial).Append("/omitted=").Append(Omitted)
                    .Append("/mm=local,source,raw,control,body,bind,parentRef,parentResponse,selectedRot,trackingToControl,bodyMinusControl,alignedToRawHip,closure")
                    .Append("/context=previousSharedFixed,q,witnessMask,approachMask,witnessFloor,brakeSelected,brakeExtraV_mps,recoverySelected,eligible,recoveryExtraY_mps2,groundGeometry,groundSelected,jointChain,groundReason");
                foreach (var p in _points)
                {
                    var h = p.Height;
                    b.Append("\nH171,").Append(p.Local).Append(',').Append(p.Source);
                    HeightValue(b,p.References,p.SourceMin); HeightValue(b,p.References,p.ControlMin); HeightValue(b,true,p.Min);
                    HeightValue(b,h.Complete,h.Binding); HeightValue(b,h.Parent,h.Reference); HeightValue(b,h.Parent,h.ResponseY);
                    HeightValue(b,h.Complete,h.Rotation); HeightValue(b,h.Complete,h.Tracking);
                    HeightValue(b,p.References,p.Min-p.ControlMin); HeightValue(b,h.Complete,h.Aligned); HeightValue(b,h.Complete,h.Closure);
                    b.Append(',').Append(h.Response ? h.InputFixed.ToString(System.Globalization.CultureInfo.InvariantCulture) : "NA")
                        .Append(',').Append(h.Response ? F(h.Q) : "NA").Append(',').Append(Integer(h.Braking,h.WitnessMask))
                        .Append(',').Append(Integer(h.Braking,h.ApproachMask)).Append(',').Append(Integer(h.Braking,h.WitnessSupportId))
                        .Append(',').Append(Bit(h.Braking,h.BrakingSelected)).Append(',').Append(h.Braking ? F(h.BrakingExtraV) : "NA")
                        .Append(',').Append(Bit(h.Recovery,h.RecoverySelected))
                        .Append(',').Append(Bit(h.Recovery,h.RecoveryEligible)).Append(',').Append(h.Recovery ? F(h.RecoveryExtra) : "NA")
                        .Append(',').Append(Bit(h.Ground,h.GroundGeometry)).Append(',').Append(Bit(h.Ground,h.GroundSelected))
                        .Append(',').Append(Bit(h.Ground,h.GroundJoint)).Append(',').Append(h.Ground ? h.GroundReason ?? "NA" : "NA");
                }
            }
            private static void HeightValue(StringBuilder b,bool known,float meters) =>
                b.Append(',').Append(known ? F(meters*1000f) : "NA");
            private static string Integer(bool known,int value) => known ? value.ToString(System.Globalization.CultureInfo.InvariantCulture) : "NA";
            private static string Bit(bool known,bool value) => known ? (value ? "1" : "0") : "NA";
        }
    }
}
