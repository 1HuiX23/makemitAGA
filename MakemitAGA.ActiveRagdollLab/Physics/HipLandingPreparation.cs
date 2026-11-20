using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using static MakemitAGA.ActiveRagdollLab.Physics.LandingPreparationMath;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // V140: finish the original Hip step after the final Knee step, before FF/PD.
    // Hold final Knee and published Foot. A second VIRTUAL chain retains the
    // Render native/reference Upper orientation error E = native * inverse(baseline).
    // E * candidate and E * coast test sensitivity to that existing mismatch;
    // neither this chain nor a one-dt coast predicts a native solve or a swept volume.
    internal static partial class HipLandingPreparation
    {
        internal enum Status { Off, Unavailable, ProtectedOrStale, InvalidTrajectory, InvalidGeometry, BaselineClear, NoHeadroom, NoLegalImprovement, Improved, Faulted }
        internal struct Heights
        {
            internal float ReferenceNow, ReferenceCoast, AlignedNow, AlignedCoast;
            internal float Risk => Mathf.Min(ReferenceNow, ReferenceCoast);
            internal bool Finite => F(ReferenceNow) && F(ReferenceCoast) && F(AlignedNow) && F(AlignedCoast);
        }
        internal struct Result
        {
            internal Status Reason;
            internal bool Evaluated, Approach, Better, Applied;
            internal int Tested, ReferenceGains, AlignedRejected;
            internal Quaternion R, RejectedR;
            // Optional receipt of the unmodified step; never read by selection.
            internal Quaternion BaselineR;
            internal Vector3 W, RejectedW;
            internal Heights Baseline, Best, Rejected;
            internal float Room, DeltaDegrees;
        }

        internal static Result Evaluate(KneeFootGroundPacket packet, HipNormalReferenceState hip,
            Quaternion hipR, Vector3 hipW, KneeReferenceState knee)
        {
            var r = new Result { Reason = Status.Unavailable, BaselineR = hipR, R = hipR, W = hipW };
            var step = knee.Step; var source = knee.Source;
            if (!packet.Matches(step, source, packet.Input.ParentId, packet.Input.UpperId, packet.Input.LowerId, packet.SupportId) ||
                !packet.Input.Apply || !packet.PreparationSupport.EligibleForSource(source, packet.Input.CompletedStep, packet.SupportId)) return r;
            r.Approach = packet.PreparationSupport.StrongLandingApproach;
            r.Reason = Status.ProtectedOrStale;
            // A held source must not extend the age of its completed support witness.
            // EligibleForSource alone qualifies Render, not this later Fixed consumer.
            float supportAge = packet.PreparationSupport.SourceAge + Mathf.Max(step.Dt, Mathf.Max(
                step.FixedTime - packet.Input.CompletedStep.FixedTime, step.UnscaledTime - packet.Input.CompletedStep.UnscaledTime));
            if (!F(supportAge) || supportAge < 0f || supportAge > .1f || !knee.Matches(step, source) || knee.Stale ||
                !hip.Matches(step) || !hip.Selected || !hip.Applied || !hip.ReceiverParentReference ||
                hip.SampleAgePrediction || hip.PredictionSeconds != 0f) return r;
            r.Reason = Status.InvalidTrajectory;
            if (!ValidLane(hip.PreviousR, hip.PreviousW, hip.InputR, hip.InputW, hipR, hipW, hip.A, step.Dt, hip.MaxW, hip.MaxA) ||
                !ValidLane(knee.PreviousR, knee.PreviousW, knee.InputR, knee.InputW, knee.R, knee.W, knee.A, step.Dt, knee.MaxW, knee.MaxA)) return r;
            var g = packet.Geometry; var foot = packet.Sample.PlannedFoot;
            r.Reason = Status.InvalidGeometry;
            Quaternion error = g.Upper * Quaternion.Inverse(hipR);
            r.Baseline = r.Best = Measure(g, foot, error, hipR, hipW, knee.R, knee.W, step.Dt);
            if (!r.Baseline.Finite || !LegalPose(g, foot, hipR, knee.R) || !LegalPose(g, foot, g.Upper, knee.R)) return r;
            r.Evaluated = true;
            if (r.Baseline.Risk >= -KneeGroundTarget.MinGain) { r.Reason = Status.BaselineClear; return r; }
            var lane = MakeLane(hip.PreviousR, hip.PreviousW, hip.InputR, hip.InputW, hipR, hipW,
                step.Dt, hip.MaxW, hip.MaxA, Vector3.zero, false);
            r.Room = lane.Room;
            if (!F(r.Room) || r.Room <= .000001f) { r.Reason = Status.NoHeadroom; return r; }
            r.Reason = Status.NoLegalImprovement;
            Quaternion kneeCoast = FootRelativeReference.Advance(knee.R, knee.W, step.Dt);
            for (int i = 1; i <= SamplesPerLane; i++)
            {
                if (!TryLane(lane, g, i, out Quaternion hr, out Vector3 hw)) continue;
                r.Tested++;
                var next = Measure(g, foot, error, hr, hw, knee.R, knee.W, step.Dt);
                Quaternion hc = FootRelativeReference.Advance(hr, hw, step.Dt);
                if (!next.Finite || !LegalPose(g, foot, hr, knee.R) || !LegalPose(g, foot, hc, kneeCoast) ||
                    next.ReferenceNow < r.Baseline.ReferenceNow || next.ReferenceCoast < r.Baseline.ReferenceCoast ||
                    next.Risk - r.Baseline.Risk < KneeGroundTarget.MinGain) continue;
                r.ReferenceGains++;
                if (!LegalPose(g, foot, error * hr, knee.R) || !LegalPose(g, foot, error * hc, kneeCoast) ||
                    next.AlignedNow < r.Baseline.AlignedNow || next.AlignedCoast < r.Baseline.AlignedCoast)
                {
                    if (r.AlignedRejected == 0 || next.Risk > r.Rejected.Risk)
                    { r.Rejected = next; r.RejectedR = hr; r.RejectedW = hw; }
                    r.AlignedRejected++; continue;
                }
                if (next.Risk <= r.Best.Risk) continue;
                r.Better = true; r.Reason = Status.Improved;
                r.Best = next; r.R = hr; r.W = hw;
                r.DeltaDegrees = FootRelativeReference.Log(hr * Quaternion.Inverse(hipR)).magnitude * Mathf.Rad2Deg;
            }
            return r;
        }

        private static Heights Measure(KneeGroundTarget.Geometry g, Quaternion foot, Quaternion error, Quaternion hip, Vector3 hipW,
            Quaternion knee, Vector3 kneeW, float dt)
        {
            Quaternion kc = FootRelativeReference.Advance(knee, kneeW, dt);
            return new Heights {
                ReferenceNow = Minimum(g, hip, knee, foot),
                ReferenceCoast = Minimum(g, FootRelativeReference.Advance(hip, hipW, dt), kc, foot),
                AlignedNow = Minimum(g, error * hip, knee, foot),
                AlignedCoast = Minimum(g, error * FootRelativeReference.Advance(hip, hipW, dt), kc, foot) };
        }
        private static bool F(float value) => PelvisParentFrame.F(value);
    }
}
