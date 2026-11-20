using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One alternative COM command from the receiver's original N input. The live controller never
    // consumes this result. Frozen leg rotations and gaps describe geometry, not solver response.
    internal static class ParentHeightProbe
    {
        internal struct Input
        {
            internal PhysicsStepContext Step;
            internal string Arm;
            internal Vector3 Position, Velocity, Goal, GoalV, PreviousRequest, PreviousAllocated, SettledV;
            internal bool Feedback, ReceiverReady;
            internal Vector3 ActualCom, PositionError, VelocityError;
            internal float PositionLimit;
            internal SupportReferenceContext Support;
            internal float FloorY;
            internal Vector3 CommittedPosition, CommittedV;
        }

        internal struct Request
        {
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal int Local, Origin, SupportId;
            internal Vector3 Center, Half;
            internal float Loss, FollowChange, KneeError, AnkleError, LowerY, Box;
            internal bool HeightsValid;
            internal Request(in Sample s, int origin, float loss, float follow)
            {
                Step = s.Step; Source = s.Source; Local = s.Fixed; Origin = origin; SupportId = s.SupportId;
                Center = s.FloorCenter; Half = s.FloorHalf; Loss = loss; FollowChange = follow; Box = s.Min;
                KneeError = s.KneeHeights.z - s.KneeHeights.x; AnkleError = s.AnkleHeights.y - s.AnkleHeights.x;
                LowerY = s.AnkleHeights.y;
                HeightsValid = s.KneeHeightsValid && s.AnkleHeightsValid &&
                    PelvisParentFrame.V(s.KneeHeights) && PelvisParentFrame.V(s.AnkleHeights);
            }
        }

        internal struct FootPose
        {
            internal bool Valid, Active, Authorized;
            internal Vector3 Hip, LowerAnkle, Ankle, Center, Half;
            internal Quaternion Rotation;
            internal float Clearance(in Request t) =>
                FootBoxGeometry.Minimum(Ankle, Rotation, Center, Half) - (t.Center.y + t.Half.y);
            internal bool Inside(in Request t, Vector3 displacement) =>
                FootBoxGeometry.Inside(Ankle + displacement, Rotation, Center, Half, t.Center, t.Half);
        }

        internal enum Status { AwaitingPair, InputMissing, LeftGeometry, Footprint, NoHeightGain, Mixed, LeftImproved, Fault }
        internal enum Peer { Unknown, Inactive, ActiveUnqualified, Authorized }
        internal struct Result
        {
            internal Status Outcome;
            internal Peer Right;
            internal ParentHeightAllocation.Result Allocation;
            internal bool Planned, LeftMeasured, RightMeasured, RightInside, FloorSelected;
            internal Vector3 Delta;
            internal float BaseV, CandidateV, BaseA, CandidateA, KneeGain, AnkleGain, BoxGain;
            internal float RightBox, RightGapGrowth, PinnedReachChange;
        }

        internal static Result Compare(in Request t, in Input original, in FootPose left, in FootPose right,
            out PelvisComReferencePlanner.ComPlan plan)
        {
            plan = default;
            var result = new Result { Outcome = Status.InputMissing };
            if (!original.Step.Matches(t.Step) || !t.Source.Valid || t.Source.Epoch != t.Step.Epoch ||
                t.Source.Sequence != t.Step.SourceSequence || original.Arm != PelvisDynamicReceiverIsolation.DynamicHipArmB ||
                !PelvisParentFrame.F(t.Loss) || t.Loss < FootContactGeometryLimits.OrdinaryPenetrationMeters ||
                !PelvisParentFrame.V(original.CommittedPosition) || !PelvisParentFrame.V(original.CommittedV)) return result;
            bool floor = original.Support.Matches(t.Step);
            if (floor && original.Support.SupportId != t.SupportId) return result;

            // Change only the height of the incoming COM goal by the measured loss since lift.
            // Held target velocity, support floor, braking and the preceding allocation remain original.
            Vector3 goal = original.Goal + Vector3.up * t.Loss;
            plan = PelvisDynamicReceiverIsolation.BuildComReferencePlan(original.Arm, original.Position, original.Velocity,
                goal, original.GoalV, t.Step.Dt, t.Step, original.Feedback, original.PreviousRequest, original.PreviousAllocated,
                original.SettledV, original.Support, original.FloorY);
            result.Planned = true; result.FloorSelected = floor;
            result.Delta = plan.Position - original.CommittedPosition;
            result.BaseV = original.CommittedV.magnitude; result.CandidateV = plan.Velocity.magnitude;
            result.BaseA = ((original.CommittedV - original.Velocity) / t.Step.Dt).magnitude;
            result.CandidateA = plan.Acceleration.magnitude;

            if (right.Valid)
            {
                result.Right = right.Authorized ? Peer.Authorized : right.Active ? Peer.ActiveUnqualified : Peer.Inactive;
                if (right.Inside(t, Vector3.zero))
                {
                    result.RightMeasured = true; result.RightBox = right.Clearance(t);
                    result.RightInside = right.Inside(t, result.Delta);
                    // Free other foot: added clearance above the plane. Pinned other ankle: added reach.
                    result.RightGapGrowth = Mathf.Max(0f, result.RightBox + result.Delta.y) - Mathf.Max(0f, result.RightBox);
                    Vector3 span = right.Hip - right.LowerAnkle;
                    result.PinnedReachChange = (span + result.Delta).magnitude - span.magnitude;
                }
            }
            result.Outcome = Status.LeftGeometry;
            if (!t.HeightsValid || !left.Valid || left.Active ||
                Mathf.Abs(left.Clearance(t) - t.Box) > FootBoxGeometry.SupportPlaneToleranceMeters ||
                Mathf.Abs(left.LowerAnkle.y - t.LowerY) > FootBoxGeometry.SupportPlaneToleranceMeters) return result;
            result.Outcome = Status.Footprint;
            if (!left.Inside(t, Vector3.zero) || !left.Inside(t, result.Delta)) return result;
            float dy = result.Delta.y;
            result.LeftMeasured = true;
            result.KneeGain = AbsoluteGain(t.KneeError, dy); result.AnkleGain = AbsoluteGain(t.AnkleError, dy);
            result.BoxGain = AbsoluteGain(t.Box, dy);
            result.Outcome = dy < KneeGroundTarget.MinGain ? Status.NoHeightGain :
                Mathf.Min(result.BoxGain, Mathf.Min(result.KneeGain, result.AnkleGain)) >= KneeGroundTarget.MinGain ?
                    Status.LeftImproved : Status.Mixed;
            return result;
        }
        private static float AbsoluteGain(float before, float dy) => Mathf.Abs(before) - Mathf.Abs(before + dy);

        // Allocate only on the first B trial. A reservation survives a failed bilateral completion
        // as AwaitingPair; the total and omitted counts keep the denominator when storage is full.
        internal sealed class Trace
        {
            internal const int Capacity = 128;
            private const string Header = "\nPARENT_HEIGHT_V041470/trials=";
            private const string Outcomes = "/outcomes(awaitingPair-inputMissing-leftGeometry-footprint-noHeightGain-mixed-leftImproved-fault)=";
            private const string Model = "/model=ONE_FIXED_FROZEN_ROTATIONS_AND_GAPS/pinnedRight=HIP_TO_LOWER_ANKLE/feedbackToControl=False";
            internal const string EmptySummary = Header + "0" + Outcomes + "0|0|0|0|0|0|0|0/omitted=0" + Model;
            private struct Row { internal Request Trial; internal Result Result; }
            private readonly Row[] _rows = new Row[Capacity];
            private readonly int[] _counts = new int[(int)Status.Fault + 1];
            private int _total, _pendingIndex;
            private Request _pending;
            internal void Begin(in Request trial)
            {
                _pending = trial; _pendingIndex = _total++;
                if (_pendingIndex < Capacity) _rows[_pendingIndex] = new Row { Trial = trial };
                _counts[(int)Status.AwaitingPair]++;
            }
            internal bool TryTake(PhysicsStepContext step, out Request trial)
            {
                trial = _pending;
                if (!_pending.Step.Matches(step)) return false;
                _pending = default; return true;
            }
            internal void Complete(Result result)
            {
                _counts[(int)Status.AwaitingPair]--; _counts[(int)result.Outcome]++;
                if (_pendingIndex < Capacity) _rows[_pendingIndex].Result = result;
            }
            internal string Summary()
            {
                var b = new StringBuilder(Header).Append(_total)
                    .Append(Outcomes)
                    .Append(string.Join("|", _counts)).Append("/omitted=").Append(Math.Max(0, _total - Capacity))
                    .Append(Model);
                for (int i = 0; i < Math.Min(_total, Capacity); i++)
                {
                    ref readonly var t = ref _rows[i].Trial; ref readonly var r = ref _rows[i].Result;
                    b.Append("\nPARENT_HEIGHT_STEP_V041470/epoch-shared-local-source=").Append(t.Step.Epoch).Append('|')
                        .Append(t.Step.Sequence).Append('|').Append(t.Local).Append('|').Append(t.Source.Sequence)
                        .Append("/origin-support=").Append(t.Origin).Append('|').Append(t.SupportId)
                        .Append("/loss-followChangeMm=").Append(M(t.Loss)).Append('|').Append(M(t.FollowChange))
                        .Append("/result=").Append(r.Outcome);
                    if (!r.Planned) continue;
                    b.Append("/supportFloor=").Append(r.FloorSelected)
                        .Append("/deltaCOMmm(x-y-z)=").Append(M(r.Delta.x)).Append('|').Append(M(r.Delta.y)).Append('|').Append(M(r.Delta.z))
                        .Append("/speed(base-trial)=").Append(N(r.BaseV)).Append('|').Append(N(r.CandidateV))
                        .Append("/accel(base-trial)=").Append(N(r.BaseA)).Append('|').Append(N(r.CandidateA));
                    if (r.LeftMeasured)
                        b.Append("/leftBaseMm(kneeMinusRaw-ankleMinusRaw-boxVsFloor)=").Append(M(t.KneeError)).Append('|')
                            .Append(M(t.AnkleError)).Append('|').Append(M(t.Box)).Append("/leftAbsGainMm(knee-ankle-box)=")
                            .Append(M(r.KneeGain)).Append('|').Append(M(r.AnkleGain)).Append('|').Append(M(r.BoxGain));
                    var a = r.Allocation;
                    b.Append("/allocation=").Append(a.State);
                    if (a.State == ParentHeightAllocation.Status.Measured)
                        b.Append("/commonY(baseRequest-trialRequest-baseAllocated-trialAllocated)=")
                            .Append(N(a.BaseRequestY)).Append('|').Append(N(a.TrialRequestY)).Append('|')
                            .Append(N(a.BaseAllocatedY)).Append('|').Append(N(a.TrialAllocatedY))
                            .Append("/rightUpperHipForceDeltaN(y-norm)=").Append(N(a.RightForceDeltaY)).Append('|').Append(N(a.RightForceDeltaNorm));
                    b.Append("/right=").Append(r.Right);
                    if (r.RightMeasured)
                        b.Append("/rightFootprintAfter=").Append(r.RightInside).Append("/rightMm(box-addedGap-pinnedReachChange)=")
                            .Append(M(r.RightBox)).Append('|').Append(M(r.RightGapGrowth)).Append('|').Append(M(r.PinnedReachChange));
                }
                return b.ToString();
            }
            private static string M(float v) => N(v * 1000f);
            private static string N(float v) => v.ToString("G8", CultureInfo.InvariantCulture);
        }
    }
}
