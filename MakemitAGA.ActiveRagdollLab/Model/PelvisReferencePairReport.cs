using System;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Aggregates existing receiver frames only. No Update, native read, ring, file or physical gate.
    // A/B share bridge input, dt and the ACTUAL run's native tensor. This compares reference
    // planning, not two independently simulated plants or counterfactual feet/forces.
    internal sealed class PelvisReferencePairReport
    {
        internal struct Sample
        {
            internal bool Valid, RawSourceCaptured, AppliedB;
            internal int Sequence;
            internal float Dt, AlphaA, AlphaB, ModelTorqueA, ModelTorqueB, ExtraBrakeAlphaB;
            internal Quaternion RotationA, RotationB;
            internal Vector3 WA, WB, BridgeErrorA, BridgeErrorB, RawErrorA, RawErrorB;
        }
        internal struct Stats
        {
            internal int Samples, QuietSamples, AwaySamples;
            internal double Seconds, RawIntegral, BridgeIntegral, QuietSeconds, QuietRawIntegral, AwaySeconds;
            internal float RawPeak, BridgePeak, QuietRawPeak, QuietWPeak, AlphaPeak, ModelTorquePeak;
        }
        internal int Samples, Invalid, ExtraBrakeSamples;
        internal float ExtraBrakeAlphaPeak;
        internal string Issue;
        internal Stats A, B;
        private int _lastSequence;
        internal void Reset()
        {
            Samples = Invalid = ExtraBrakeSamples = _lastSequence = 0;
            ExtraBrakeAlphaPeak = 0f; Issue = null; A = B = new Stats();
        }
        internal void Observe(Sample s, int sequence, float dt, Vector3 sourceW, int timing, Vector3 appliedW)
        {
            try { ObserveCore(s, sequence, dt, sourceW, timing, appliedW); }
            catch (Exception e) { Invalid++; Issue = Issue ?? "PAIR_AGGREGATION_EXCEPTION:" + e.GetType().Name; }
        }
        private void ObserveCore(Sample s, int sequence, float dt, Vector3 sourceW, int timing, Vector3 appliedW)
        {
            if (!s.Valid || !s.RawSourceCaptured || s.Sequence != sequence || s.Dt != dt || sequence <= 0 ||
                (_lastSequence > 0 && sequence != _lastSequence + 1) || !F(dt) || dt <= 0f || timing < 0 || timing > 3 ||
                !V(sourceW) || !V(s.WA) || !V(s.WB) || !V(s.RawErrorA) || !V(s.RawErrorB) || !V(s.BridgeErrorA) || !V(s.BridgeErrorB) ||
                !Nonnegative(s.AlphaA) || !Nonnegative(s.AlphaB) || !Nonnegative(s.ModelTorqueA) || !Nonnegative(s.ModelTorqueB) ||
                s.AlphaA > 4.001f || s.AlphaB > 8.001f || s.ModelTorqueA > 1.0001f || s.ModelTorqueB > 1.0001f ||
                s.WA.magnitude > 3.00001f || s.WB.magnitude > 3.00001f ||
                !Nonnegative(s.ExtraBrakeAlphaB) || !V(appliedW) || ((s.AppliedB ? s.WB : s.WA) - appliedW).magnitude > 0.00001f)
            {
                Invalid++; Issue = Issue ?? "PAIR_INPUT_CLOCK_OR_APPLIED_REFERENCE_INVALID";
                _lastSequence = sequence; return;
            }
            _lastSequence = sequence; Samples++;
            bool quiet = timing <= 1 && sourceW.magnitude <= 0.05f;
            Accumulate(ref A, s.RawErrorA, s.BridgeErrorA, s.WA, sourceW, s.AlphaA, s.ModelTorqueA, dt, quiet);
            Accumulate(ref B, s.RawErrorB, s.BridgeErrorB, s.WB, sourceW, s.AlphaB, s.ModelTorqueB, dt, quiet);
            if (s.ExtraBrakeAlphaB > 0f) ExtraBrakeSamples++;
            ExtraBrakeAlphaPeak = Mathf.Max(ExtraBrakeAlphaPeak, s.ExtraBrakeAlphaB);
        }
        private static void Accumulate(ref Stats s, Vector3 raw, Vector3 bridge, Vector3 w, Vector3 sourceW,
            float alpha, float torque, float dt, bool quiet)
        {
            float rawDeg = raw.magnitude * Mathf.Rad2Deg, bridgeDeg = bridge.magnitude * Mathf.Rad2Deg;
            s.Samples++; s.Seconds += dt; s.RawIntegral += rawDeg * dt; s.BridgeIntegral += bridgeDeg * dt;
            s.RawPeak = Mathf.Max(s.RawPeak, rawDeg); s.BridgePeak = Mathf.Max(s.BridgePeak, bridgeDeg);
            s.AlphaPeak = Mathf.Max(s.AlphaPeak, alpha); s.ModelTorquePeak = Mathf.Max(s.ModelTorquePeak, torque);
            if (!quiet) return;
            s.QuietSamples++; s.QuietSeconds += dt; s.QuietRawIntegral += rawDeg * dt;
            s.QuietRawPeak = Mathf.Max(s.QuietRawPeak, rawDeg); s.QuietWPeak = Mathf.Max(s.QuietWPeak, w.magnitude);
            if (rawDeg > 0.5f && Vector3.Dot(raw / raw.magnitude, sourceW - w) > 0.01f)
            { s.AwaySamples++; s.AwaySeconds += dt; }
        }
        internal string GetDetails(bool expected, int trackingSamples)
        {
            var sb = new StringBuilder("referencePair=V04670_SAME_INPUT_ANGULAR_PLANNERS/expected=" + expected +
                "/valid-invalid-ready=" + Samples + "-" + Invalid + "-" + (expected && Samples > 0 && Samples == trackingSamples && Invalid == 0) +
                "/issue=" + (Issue ?? "<none>") + "/scope=REFERENCE_ONLY_SHARED_ACTUAL_NATIVE_TENSOR_NOT_PLANT_COUNTERFACTUAL");
            Append(sb, "A_DIRECTIONAL_BASELINE", A); Append(sb, "B_BRAKING_RESERVE", B);
            sb.Append("\nreferenceReserveB=extraAppliedSamples:").Append(ExtraBrakeSamples).Append("/extraAlphaPeakRad=").Append(S(ExtraBrakeAlphaPeak))
                .Append("/bounds(totalBrakeAlpha-inertiaNm-ordinaryAlpha-Wrad)=8-1-4-3/extra=OPPOSE_PREVIOUS_WORLD_W_WITHOUT_CROSSING_ZERO")
                .Append("/quiet=FRESH_OR_HELD_LE100MS_SOURCE_W_LE0.05/away=RAW_ERROR_GT0.5DEG_OPENING_GT0.01RADSEC")
                .Append("/rawMeans=ALL_TIMINGS/diagnosticOnly=True/noPhysicsWrite=True");
            return sb.ToString();
        }
        private static void Append(StringBuilder sb, string arm, Stats s)
        {
            sb.Append("\nreferencePairArm=").Append(arm).Append("/samples-seconds=").Append(s.Samples).Append('-').Append(S((float)s.Seconds))
                .Append("/rawLagDeg(mean-peak)=").Append(s.Samples > 0 ? S((float)(s.RawIntegral / s.Seconds)) + "-" + S(s.RawPeak) : "NA")
                .Append("/bridgeLagDeg(mean-peak)=").Append(s.Samples > 0 ? S((float)(s.BridgeIntegral / s.Seconds)) + "-" + S(s.BridgePeak) : "NA")
                .Append("/quiet(n-seconds-rawMeanDeg-rawPeakDeg-awaySamples-awaySeconds-Wpeak)=").Append(s.QuietSamples).Append('-').Append(S((float)s.QuietSeconds)).Append('-')
                .Append(s.QuietSamples > 0 ? S((float)(s.QuietRawIntegral / s.QuietSeconds)) + "-" + S(s.QuietRawPeak) : "NA-NA")
                .Append('-').Append(s.AwaySamples).Append('-').Append(S((float)s.AwaySeconds)).Append('-').Append(s.QuietSamples > 0 ? S(s.QuietWPeak) : "NA")
                .Append("/alphaPeakRad-modelInertiaTauPeakNm=").Append(s.Samples > 0 ? S(s.AlphaPeak) + "-" + S(s.ModelTorquePeak) : "NA");
        }
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z) && F(v.sqrMagnitude);
        private static bool Nonnegative(float x) => F(x) && x >= 0f;
        private static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static string S(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
    }
}
