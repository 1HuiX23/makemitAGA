using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal static partial class HipLandingPreparation
    {
        // Existing Hip trace lifecycle; no independent clock, ring, native read,
        // or per-step formatting. Phase-specific counts avoid mixed stance/approach denominators.
        internal sealed class Trace
        {
            private readonly int[] _reasons = new int[(int)Status.Faulted + 1];
            private bool _selected, _hasPeak, _hasApproach, _hasRejected;
            private int _calls, _evaluated, _sources, _held, _approach, _stance, _applied, _appliedApproach, _appliedStance, _alignedRejected;
            private SourceStepContext _lastSource;
            private Witness _peak, _approachPeak, _rejected;
            private struct Witness
            {
                internal KneeFootGroundPacket Packet;
                internal HipNormalReferenceState Hip;
                internal Quaternion HipR;
                internal Vector3 HipW;
                internal KneeReferenceState Knee;
                internal Result Result;
            }
            internal void Reset(bool selected)
            {
                _selected = selected; _hasPeak = _hasApproach = _hasRejected = false;
                _calls = _evaluated = _sources = _held = _approach = _stance = _applied = _appliedApproach = _appliedStance = _alignedRejected = 0;
                _lastSource = default; _peak = _approachPeak = _rejected = default;
                Array.Clear(_reasons, 0, _reasons.Length);
            }
            internal void Observe(bool selected, KneeFootGroundPacket packet, HipNormalReferenceState hip,
                Quaternion hipR, Vector3 hipW, KneeReferenceState baseline, Result result)
            {
                _selected = selected;
                if (!selected) return;
                _calls++; _reasons[(int)result.Reason]++;
                if (!result.Evaluated) return;
                _evaluated++;
                if (_lastSource.Matches(baseline.Source)) _held++; else _sources++;
                _lastSource = baseline.Source;
                if (result.Approach) _approach++; else _stance++;
                var witness = new Witness { Packet = packet, Hip = hip, HipR = hipR, HipW = hipW, Knee = baseline, Result = result };
                if (result.AlignedRejected > 0)
                {
                    _alignedRejected++;
                    if (!_hasRejected || result.Rejected.Risk - result.Baseline.Risk > _rejected.Result.Rejected.Risk - _rejected.Result.Baseline.Risk)
                    { _rejected = witness; _hasRejected = true; }
                }
                if (!result.Applied) return;
                _applied++; if (result.Approach) _appliedApproach++; else _appliedStance++;
                if (!_hasPeak || result.Best.Risk - result.Baseline.Risk > _peak.Result.Best.Risk - _peak.Result.Baseline.Risk)
                { _peak = witness; _hasPeak = true; }
                if (result.Approach && (!_hasApproach || result.Best.Risk - result.Baseline.Risk > _approachPeak.Result.Best.Risk - _approachPeak.Result.Baseline.Risk))
                { _approachPeak = witness; _hasApproach = true; }
            }
            internal string Summary() => "HIP_LANDING_PREPARATION_V041400/traceSelected=" + _selected +
                "/calls-evaluated-sources-held=" + _calls + "|" + _evaluated + "|" + _sources + "|" + _held +
                "/phase(approach-stance)=" + _approach + "|" + _stance +
                "/applied(total-approach-stance)=" + _applied + "|" + _appliedApproach + "|" + _appliedStance +
                "/samplesWithAlignedRejection=" + _alignedRejected +
                "/reason(off-unavailable-protectedStale-trajectory-geometry-clear-noHeadroom-noLegal-improved-faulted)=" + string.Join("|", _reasons) +
                "/faults=" + _reasons[(int)Status.Faulted] +
                "/domain=FINAL_KNEE_PUBLISHED_FOOT_HIP_REFERENCE_AND_ERROR_ALIGNED_VIRTUAL_CHAIN" +
                "/coast=ONE_FIXED_REFERENCE_W_CONSTANT_UPPER_ERROR_NOT_NATIVE_PREDICTION_OR_SWEEP" +
                "/bounds=ORIGINAL_GOAL_W_A_BRAKE_DEBT_FOOT_ROM_FOOTPRINT/contactAuthority=False" +
                (_hasPeak ? Text("PEAK_APPLIED_GAIN", _peak) : "/appliedWitness=NONE") +
                (_hasApproach ? Text("APPROACH_PEAK_APPLIED_GAIN", _approachPeak) : "/approachWitness=NONE") +
                (_hasRejected ? Text("PEAK_ALIGNED_REJECTED_REFERENCE_GAIN", _rejected) : "/rejectedWitness=NONE");
            private static string Text(string selection, Witness w)
            {
                var p = w.Packet; var g = p.Geometry; var h = w.Hip; var k = w.Knee; var r = w.Result;
                return "\nHIP_LANDING_WITNESS_V041400/selection=" + selection +
                    "/fixed=" + Clock(k.Step) + "/render=" + Source(k.Source) + "/completed=" + Clock(p.Input.CompletedStep) +
                    "/owners-support=" + p.Input.ParentId + "|" + p.Input.UpperId + "|" + p.Input.LowerId + "|" + p.SupportId +
                    "/approach=" + r.Approach + "/supportAgeAtCompleted=" + N(p.PreparationSupport.SourceAge) +
                    "/reason=" + r.Reason + "/better-applied=" + r.Better + "|" + r.Applied +
                    "/tested-referenceGain-alignedRejected=" + r.Tested + "|" + r.ReferenceGains + "|" + r.AlignedRejected +
                    "/heightMm(refNow-refCoast-alignedNow-alignedCoast)=" + Ht(r.Baseline) + "/bestHeightMm=" + Ht(r.Best) +
                    "/rejectedHeightMm=" + (r.AlignedRejected > 0 ? Ht(r.Rejected) : "NA") +
                    "/room-deltaDeg=" + N(r.Room) + "|" + N(r.DeltaDegrees) +
                    "/hipPoint-nativeUpper=" + Vt(g.Hip) + "|" + Qt(g.Upper) +
                    "/upperSpan-lowerSpan=" + Vt(g.UpperSpan) + "|" + Vt(g.LowerSpan) +
                    "/boxCenter-half=" + Vt(g.Center) + "|" + Vt(g.Half) + "/floorCenter-half=" + Vt(g.FloorCenter) + "|" + Vt(g.FloorHalf) +
                    "/foot(raw-bind-published)=" + Qt(g.FootRelative) + "|" + Qt(g.FootBindRelative) + "|" + Qt(p.Sample.PlannedFoot) +
                    "/hipR(previous-input-baseline-selected)=" + Qt(h.PreviousR) + "|" + Qt(h.InputR) + "|" + Qt(w.HipR) + "|" + Qt(r.R) +
                    "/hipW(previous-input-baseline-selected-baselineA-selectedA)=" + Vt(h.PreviousW) + "|" + Vt(h.InputW) + "|" + Vt(w.HipW) + "|" + Vt(r.W) + "|" + Vt(h.A) + "|" + Vt((r.W - h.PreviousW) / k.Step.Dt) +
                    "/kneeR(previous-input-final)=" + Qt(k.PreviousR) + "|" + Qt(k.InputR) + "|" + Qt(k.R) +
                    "/kneeW(previous-input-final-A)=" + Vt(k.PreviousW) + "|" + Vt(k.InputW) + "|" + Vt(k.W) + "|" + Vt(k.A) +
                    "/rejectedR-W=" + (r.AlignedRejected > 0 ? Qt(r.RejectedR) + "|" + Vt(r.RejectedW) : "NA") +
                    "/bounds(hipW-hipA-kneeW-kneeA)=" + N(h.MaxW) + "|" + N(h.MaxA) + "|" + N(k.MaxW) + "|" + N(k.MaxA) +
                    "/axis=" + Vt(p.Input.Axis) + "/units=METERS_RADIANS_SECONDS/actualPoseAndContact=SEPARATE_EVIDENCE";
            }
            private static string Clock(PhysicsStepContext s) => s.Epoch + "|" + s.Sequence + "|" + s.SourceSequence + "|" + N(s.FixedTime) + "|" + N(s.UnscaledTime) + "|" + N(s.Dt);
            private static string Source(SourceStepContext s) => s.Epoch + "|" + s.Sequence + "|" + N(s.Time) + "|" + N(s.Dt);
            private static string Ht(Heights h) => N(h.ReferenceNow * 1000f) + "|" + N(h.ReferenceCoast * 1000f) + "|" + N(h.AlignedNow * 1000f) + "|" + N(h.AlignedCoast * 1000f);
            private static string N(float v) => FootGroundTrajectory.N(v);
            private static string Vt(Vector3 v) => N(v.x) + "," + N(v.y) + "," + N(v.z);
            private static string Qt(Quaternion q) => N(q.x) + "," + N(q.y) + "," + N(q.z) + "," + N(q.w);
        }
    }
}
