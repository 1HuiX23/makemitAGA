using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using static MakemitAGA.ActiveRagdollLab.Physics.LandingPreparationMath;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Evaluate a copy of the original Knee R/W/A, after the ordinary command is committed.
    // No planner state is advanced. The physical copy preserves BOTH the actual knee position and ankle gap.
    internal static class SwingKneeCandidate
    {
        internal enum Status : byte { None, Unavailable, Protected, Geometry, Budget, NoRoom, NoLegal, Favorable, Fault }
        internal enum HipPlacement : byte { Missing, Unsupported, ReceiverReference, SettledParent }

        internal static bool OrdinaryHip(in HipNormalReferenceState h, PhysicsStepContext step, bool handoff) =>
            h.Matches(step) && h.Selected && h.Applied && h.ReceiverParentReference && !h.SampleAgePrediction &&
            h.PredictionSeconds == 0f && !handoff;

        internal static HipPlacement EarlyHip(in HipGoalPathEvidence path, in HipNormalReferenceState h,
            PhysicsStepContext step, SourceStepContext source, int local, int modes, bool handoff)
        {
            if (!path.Valid || !path.Step.Matches(step) || !path.Source.Matches(source) || path.Fixed != local ||
                path.Modes != modes || !h.Matches(step) || path.NormalApplied != h.Applied) return HipPlacement.Missing;
            if (OrdinaryHip(h,step,handoff)) return HipPlacement.ReceiverReference;
            if (h.Selected && !h.Applied && !h.ReceiverParentReference && !h.SampleAgePrediction &&
                h.PredictionSeconds == 0f && h.Reason == "PROTECTED_BYPASS") return HipPlacement.SettledParent;
            return HipPlacement.Unsupported;
        }

        internal readonly struct Controller
        {
            private readonly KneeRelativeReference _owner;
            internal readonly Vector3 Axis;
            internal readonly bool AllowExtension;
            internal readonly float SoftLimit;
            internal readonly bool FeedbackPriority;
            internal Controller(KneeRelativeReference owner, Vector3 axis, bool allowExtension, float softLimit = 0f, bool feedbackPriority = false)
            { _owner=owner; Axis=axis; AllowExtension=allowExtension; SoftLimit=softLimit; FeedbackPriority=feedbackPriority; }
            internal KneeReferenceState Read() => _owner == null ? default : _owner.Current;
        }
        internal struct Pose
        {
            internal KneeGroundTarget.Geometry Reference;
            internal Vector3 PhysicalKnee, AnkleGap, Axis, HipW;
            internal Quaternion PhysicalUpper, PhysicalLower, PhysicalFootRelative, SelectedFoot;
            internal float ObservedBox, LowerAnkleY;
            internal bool AllowExtension;
        }
        internal struct Result
        {
            internal Status Reason;
            internal int Tested, WithinGeometry, ReferenceImproved, PhysicalImproved, LaneIndex;
            internal GeometryIssue Geometry;
            internal float Angle, ReferenceGain, CoastGain, PhysicalGain, PhysicalCoastGain, AnkleGain;
            internal float CandidateW, LimitW, CandidateA, LimitA;
            internal bool LaneMeasured;
            internal float AxialGap, Room, BaselineW, BaselineA, SpeedRoom, AccelerationRoom;
            internal bool HasDirection;
            internal string HipBasis, HipReason, KneeFeedforward, Gate;
            internal int HipModes, RecoveryBits;
            internal bool Handoff;
            internal string EarlyDetail() => Detail() + "/hipBasis=" + (HipBasis ?? "NA") + "/hipNormal=" + (HipReason ?? "NA") +
                "/hipModes-recoveryBits-handoff=" + HipModes + "|" + RecoveryBits + "|" + Handoff + "/kneeFF=" + (KneeFeedforward ?? "NA") +
                "/guard=" + (Gate ?? "None") + "/baselineW-limitW-baselineA-limitA-axialGapDeg-roomRadPerSec=" +
                (LaneMeasured ? N(BaselineW) + "|" + N(LimitW) + "|" + N(BaselineA) + "|" + N(LimitA) + "|" + N(AxialGap) + "|" + N(Room) : "NA") +
                "/deltaWRoom(speed-accel)=" + (HasDirection ? N(SpeedRoom) + "|" + N(AccelerationRoom) : "NA");
            internal string Detail() => Reason + "/geometry=" + Geometry + "/tested-withinBounds-refImproved-physicalImproved=" +
                Tested + "|" + WithinGeometry + "|" + ReferenceImproved + "|" + PhysicalImproved +
                (LaneIndex == 0 ? "" : (Reason == Status.Favorable ? "/candidate=FAVORABLE" : "/candidate=CLOSEST_REJECTED") + "/laneIndex=" + LaneIndex + "/angleDeg=" + N(Angle) + "/gainMm(ref-refCoast-physical-physicalCoast-lowerAnkle)=" +
                N(1000f*ReferenceGain) + "|" + N(1000f*CoastGain) + "|" + N(1000f*PhysicalGain) + "|" + N(1000f*PhysicalCoastGain) + "|" + N(1000f*AnkleGain) +
                "/radPerSec(W-effectiveLimit)=" + N(CandidateW) + "|" + N(LimitW) + "/radPerSec2(A-limit)=" + N(CandidateA) + "|" + N(LimitA));
            private static string N(float v) => v.ToString("G8", CultureInfo.InvariantCulture);
        }
        internal enum GeometryIssue : byte { None, Inputs, ReferenceFootBounds, PhysicalFootprint, NonFinite, MeasuredClosure }
        private struct Heights { internal float Now, Coast, Actual, ActualCoast, Ankle; }

        internal static Result Evaluate(in Pose p, in KneeReferenceState k, PhysicsStepContext step, SourceStepContext source, bool measureRoom = false)
        {
            var result = new Result { Reason = Status.Unavailable };
            if (!k.Matches(step,source) || !source.Valid || source.Epoch != step.Epoch || source.Sequence != step.SourceSequence) return result;
            result.Reason = Status.Protected;
            if (k.Stale || k.GroundTrajectory.Applied || k.LandingPreparation.Applied)
            {
                result.Gate = k.Stale ? "STALE_KNEE" : k.GroundTrajectory.Applied ? "GROUND_KNEE" : "LANDING_KNEE";
                return result;
            }
            result.Reason = Status.Budget;
            if (!ValidLane(k.PreviousR,k.PreviousW,k.InputR,k.InputW,k.R,k.W,k.A,step.Dt,k.MaxW,k.MaxA) ||
                !PelvisParentFrame.V(p.Axis) || p.Axis.sqrMagnitude < .0001f) return result;
            result.Reason = Status.Geometry; result.Geometry = GeometryIssue.Inputs;
            var g = p.Reference;
            if (!g.Valid || !Unit(g.Upper) || !Unit(g.FootRelative) || !Unit(g.FootBindRelative) ||
                !Unit(p.PhysicalUpper) || !Unit(p.PhysicalLower) || !Unit(p.PhysicalFootRelative) || !Unit(p.SelectedFoot) ||
                !PelvisParentFrame.V(p.PhysicalKnee) || !PelvisParentFrame.V(p.AnkleGap) || !PelvisParentFrame.V(p.HipW) ||
                !PelvisParentFrame.F(p.LowerAnkleY) || !PelvisParentFrame.F(p.ObservedBox)) return result;
            var physical = g;
            physical.Upper = p.PhysicalUpper;
            // Translating the whole physical box by the measured Foot-minus-Lower ankle gap keeps that gap fixed.
            physical.Hip = p.PhysicalKnee - physical.Upper * g.UpperSpan + p.AnkleGap;
            Quaternion relative = Quaternion.Inverse(p.PhysicalUpper) * p.PhysicalLower;
            Quaternion referenceToPhysical = Quaternion.Inverse(k.R) * relative;
            Quaternion coastUpper = FootRelativeReference.Advance(g.Upper, p.HipW, step.Dt);
            if (!Measure(p, physical, referenceToPhysical, coastUpper, k.R, k.W, step.Dt, out var baseH, out result.Geometry)) return result;
            if (Mathf.Abs(baseH.Actual-p.ObservedBox) > FootBoxGeometry.SupportPlaneToleranceMeters ||
                Mathf.Abs(baseH.Ankle-p.LowerAnkleY) > FootBoxGeometry.SupportPlaneToleranceMeters)
            { result.Geometry = GeometryIssue.MeasuredClosure; return result; }
            var lane = MakeLane(k.PreviousR,k.PreviousW,k.InputR,k.InputW,k.R,k.W,step.Dt,k.MaxW,k.MaxA,p.Axis,true);
            if (measureRoom)
            {
                result.LaneMeasured = true; result.BaselineW = k.W.magnitude; result.BaselineA = k.A.magnitude;
                result.LimitW = Mathf.Max(k.MaxW,k.PreviousW.magnitude); result.LimitA = k.MaxA;
                result.AxialGap = Mathf.Abs(Vector3.Dot(FootRelativeReference.Log(k.InputR * Quaternion.Inverse(k.R)),lane.Direction)) * Mathf.Rad2Deg;
                result.Room = lane.Room;
                result.HasDirection = lane.Direction.sqrMagnitude > .5f;
                if (result.HasDirection)
                {
                    result.SpeedRoom = (float)GroundReferenceStepBudget.PositiveBallExit(k.W,lane.Direction,result.LimitW);
                    result.AccelerationRoom = (float)GroundReferenceStepBudget.PositiveBallExit(k.W-k.PreviousW,lane.Direction,k.MaxA*step.Dt);
                }
            }
            if (!PelvisParentFrame.F(lane.Room) || lane.Room <= .000001f) { result.Reason = Status.NoRoom; return result; }
            result.Reason = Status.NoLegal;
            float best = float.NegativeInfinity;
            for (int n=1; n<=SamplesPerLane; n++)
            {
                if (!TryLane(lane,g,n,out var rotation,out var w) || (!p.AllowExtension && Bend(g,rotation)<Bend(g,k.R))) continue;
                result.Tested++;
                if (!Measure(p, physical, referenceToPhysical, coastUpper, rotation, w, step.Dt, out var h, out _)) continue;
                result.WithinGeometry++;
                float reference=h.Now-baseH.Now, actual=h.Actual-baseH.Actual, ankle=h.Ankle-baseH.Ankle;
                if (reference>=KneeGroundTarget.MinGain && h.Coast>=baseH.Coast) result.ReferenceImproved++;
                if (actual>=KneeGroundTarget.MinGain && ankle>=KneeGroundTarget.MinGain && h.ActualCoast>=baseH.ActualCoast) result.PhysicalImproved++;
                float score=Mathf.Min(reference,Mathf.Min(actual,ankle));
                bool favorable = score >= KneeGroundTarget.MinGain && h.Coast >= baseH.Coast && h.ActualCoast >= baseH.ActualCoast;
                // Keep one rejected measurement when the gain or coast bound is just out of reach.
                // Rank rejections by their worst signed margin across the five original constraints.
                float rank = favorable ? score : Mathf.Min(score - KneeGroundTarget.MinGain,
                    Mathf.Min(h.Coast - baseH.Coast, h.ActualCoast - baseH.ActualCoast));
                if (rank <= best) continue;
                best = rank;
                result.Reason = favorable ? Status.Favorable : Status.NoLegal;
                result.LaneIndex = n;
                result.Angle=FootRelativeReference.Log(rotation*Quaternion.Inverse(k.R)).magnitude*Mathf.Rad2Deg;
                result.ReferenceGain=reference; result.CoastGain=h.Coast-baseH.Coast; result.PhysicalGain=actual;
                result.PhysicalCoastGain=h.ActualCoast-baseH.ActualCoast; result.AnkleGain=ankle;
                result.CandidateW=w.magnitude; result.LimitW=Mathf.Max(k.MaxW,k.PreviousW.magnitude);
                result.CandidateA=((w-k.PreviousW)/step.Dt).magnitude; result.LimitA=k.MaxA;
            }
            return result;
        }

        private static bool Measure(in Pose p, KneeGroundTarget.Geometry physical, Quaternion map, Quaternion coastUpper,
            Quaternion rotation, Vector3 w, float dt, out Heights h, out GeometryIssue issue)
        {
            h=default; issue=GeometryIssue.ReferenceFootBounds;
            var g=p.Reference;
            var coast=FootRelativeReference.Advance(rotation,w,dt);
            if (!LegalPose(g,p.SelectedFoot,g.Upper,rotation) || !LegalPose(g,p.SelectedFoot,coastUpper,coast)) return false;
            issue=GeometryIssue.PhysicalFootprint;
            var actual=rotation*map; var actualCoast=coast*map;
            var ankle=KneeGroundTarget.Ankle(physical,actual); var ankleCoast=KneeGroundTarget.Ankle(physical,actualCoast);
            var foot=physical.Upper*actual*p.PhysicalFootRelative; var footCoast=physical.Upper*actualCoast*p.PhysicalFootRelative;
            if (!FootBoxGeometry.Inside(ankle,foot,g.Center,g.Half,g.FloorCenter,g.FloorHalf) ||
                !FootBoxGeometry.Inside(ankleCoast,footCoast,g.Center,g.Half,g.FloorCenter,g.FloorHalf)) return false;
            float top=g.FloorCenter.y+g.FloorHalf.y;
            h.Now=Minimum(g,g.Upper,rotation,p.SelectedFoot); h.Coast=Minimum(g,coastUpper,coast,p.SelectedFoot);
            h.Actual=FootBoxGeometry.Minimum(ankle,foot,g.Center,g.Half)-top;
            h.ActualCoast=FootBoxGeometry.Minimum(ankleCoast,footCoast,g.Center,g.Half)-top;
            h.Ankle=ankle.y-p.AnkleGap.y;
            bool finite=PelvisParentFrame.F(h.Now) && PelvisParentFrame.F(h.Coast) && PelvisParentFrame.F(h.Actual) &&
                PelvisParentFrame.F(h.ActualCoast) && PelvisParentFrame.F(h.Ankle);
            issue=finite ? GeometryIssue.None : GeometryIssue.NonFinite;
            return finite;
        }
        private static bool Unit(Quaternion q) => PelvisParentFrame.Q(q) &&
            Mathf.Abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1f)<.001f;
    }
}
