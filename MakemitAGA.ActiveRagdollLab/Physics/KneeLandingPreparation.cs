using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using static MakemitAGA.ActiveRagdollLab.Physics.LandingPreparationMath;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // V139 chooses only a Knee step toward its UNCHANGED input. The caller owns
    // R/W/A history and applies at most one optional ground choice per Fixed step.
    // Both Render Hip point and published Foot are held. Two Upper bases are
    // checked: same-Fixed Hip reference, and the packet's Render native Upper.
    internal static partial class KneeLandingPreparation
    {
        internal enum Status { Off, Unavailable, ProtectedOrStale, ExistingGroundStep,
            InvalidTrajectory, InvalidGeometry, BaselineClear, NoHeadroom, NoLegalImprovement, Improved, Faulted }
        internal struct Heights
        {
            internal float ReferenceNow, ReferenceCoast, NativeNow, NativeCoast;
            internal float Risk => Mathf.Min(ReferenceNow, ReferenceCoast);
            internal bool Finite => F(ReferenceNow) && F(ReferenceCoast) && F(NativeNow) && F(NativeCoast);
        }
        internal struct Result
        {
            internal Status Reason;
            internal bool Evaluated, Approach, Better, Applied;
            internal int Tested, ReferenceGains, NativeRejected;
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
            var r = new Result { Reason = Status.Unavailable, BaselineR = knee.R, R = knee.R, W = knee.W };
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
            // Keep the pre-existing endpoint-clear path intact; never stack two
            // accelerations or reinterpret its diagnostic candidate as an applied step.
            if (knee.GroundTrajectory.Applied) { r.Reason = Status.ExistingGroundStep; return r; }
            r.Reason = Status.InvalidTrajectory;
            if (!ValidLane(hip.PreviousR, hip.PreviousW, hip.InputR, hip.InputW, hipR, hipW, hip.A, step.Dt, hip.MaxW, hip.MaxA) ||
                !ValidLane(knee.PreviousR, knee.PreviousW, knee.InputR, knee.InputW, knee.R, knee.W, knee.A, step.Dt, knee.MaxW, knee.MaxA)) return r;
            var g = packet.Geometry; var foot = packet.Sample.PlannedFoot;
            r.Reason = Status.InvalidGeometry;
            r.Baseline = r.Best = Measure(g, foot, hipR, hipW, knee.R, knee.W, step.Dt);
            if (!r.Baseline.Finite || !LegalPose(g, foot, hipR, knee.R) || !LegalPose(g, foot, g.Upper, knee.R)) return r;
            r.Evaluated = true;
            if (r.Baseline.Risk >= -KneeGroundTarget.MinGain) { r.Reason = Status.BaselineClear; return r; }
            var lane = MakeLane(knee.PreviousR, knee.PreviousW, knee.InputR, knee.InputW, knee.R, knee.W,
                step.Dt, knee.MaxW, knee.MaxA, packet.Input.Axis, true);
            r.Room = lane.Room;
            if (!F(r.Room) || r.Room <= .000001f) { r.Reason = Status.NoHeadroom; return r; }
            r.Reason = Status.NoLegalImprovement;
            Quaternion hipCoast = FootRelativeReference.Advance(hipR, hipW, step.Dt);
            for (int i = 1; i <= SamplesPerLane; i++)
            {
                if (!TryLane(lane, g, i, out Quaternion kr, out Vector3 kw)) continue;
                // Keep the packet's explicit extension policy, even if a caller
                // supplies this helper with a non-current joint-input policy.
                if (!packet.Input.AllowExtension && Bend(g, kr) < Bend(g, knee.R)) continue;
                r.Tested++;
                var next = Measure(g, foot, hipR, hipW, kr, kw, step.Dt);
                Quaternion kneeCoast = FootRelativeReference.Advance(kr, kw, step.Dt);
                if (!next.Finite || !LegalPose(g, foot, hipR, kr) || !LegalPose(g, foot, hipCoast, kneeCoast) ||
                    next.ReferenceNow < r.Baseline.ReferenceNow || next.ReferenceCoast < r.Baseline.ReferenceCoast ||
                    next.Risk - r.Baseline.Risk < KneeGroundTarget.MinGain) continue;
                r.ReferenceGains++;
                // Both bases compare the SAME Knee baseline and candidate. Native
                // Upper is frozen even in its coast check; this is a conservative
                // second geometry test, not a native prediction or a swept volume.
                if (!LegalPose(g, foot, g.Upper, kr) || !LegalPose(g, foot, g.Upper, kneeCoast) ||
                    next.NativeNow < r.Baseline.NativeNow || next.NativeCoast < r.Baseline.NativeCoast)
                {
                    if (r.NativeRejected == 0 || next.Risk > r.Rejected.Risk)
                    { r.Rejected = next; r.RejectedR = kr; r.RejectedW = kw; }
                    r.NativeRejected++; continue;
                }
                if (next.Risk <= r.Best.Risk) continue;
                r.Better = true; r.Reason = Status.Improved;
                r.Best = next; r.R = kr; r.W = kw;
                r.DeltaDegrees = FootRelativeReference.Log(kr * Quaternion.Inverse(knee.R)).magnitude * Mathf.Rad2Deg;
            }
            return r;
        }

        private static Heights Measure(KneeGroundTarget.Geometry g, Quaternion foot, Quaternion hip, Vector3 hipW,
            Quaternion knee, Vector3 kneeW, float dt)
        {
            Quaternion kc = FootRelativeReference.Advance(knee, kneeW, dt);
            return new Heights {
                ReferenceNow = Minimum(g, hip, knee, foot),
                ReferenceCoast = Minimum(g, FootRelativeReference.Advance(hip, hipW, dt), kc, foot),
                NativeNow = Minimum(g, g.Upper, knee, foot), NativeCoast = Minimum(g, g.Upper, kc, foot) };
        }
        private static bool F(float value) => PelvisParentFrame.F(value);
    }
}
