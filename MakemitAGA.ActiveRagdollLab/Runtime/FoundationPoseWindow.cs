using System;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    // Startup-only qualification for the current bilateral, flat-floor test fixture.
    // These are pose acquisition limits, not anatomical limits or runtime safety gates.
    // The caller reuses seven slots: Pelvis, L Upper/Lower/Foot, R Upper/Lower/Foot.
    internal sealed class FoundationPoseWindow
    {
        internal const int BoneCount = 7, MinimumSamples = 12;
        internal const float HoldSeconds = 1f, TimeoutSeconds = 15f, MaxSampleGap = .1f;
        internal const float MaxPositionDrift = .004f, MaxRotationDrift = 2f, MaxAnkleHeightDifference = .005f;
        private readonly Vector3[] _positions = new Vector3[BoneCount];
        private readonly Quaternion[] _rotations = new Quaternion[BoneCount];
        private SourceStepContext _last;
        private float _requested, _windowStart;
        private int _samples;
        internal bool Pending { get; private set; }
        internal bool Qualified { get; private set; }
        internal string Reason { get; private set; } = "IDLE";
        internal float StableSeconds { get; private set; }

        internal void Begin(float now)
        {
            if (!Finite(now)) throw new ArgumentException("SETUP_TIME_INVALID");
            _requested = now; _last = default; _samples = 0; StableSeconds = 0f;
            Pending = true; Qualified = false; Reason = "WAIT_STABLE_POSE";
        }
        internal void Cancel(string reason)
        {
            Pending = false; Qualified = false; _samples = 0; StableSeconds = 0f;
            Reason = reason;
        }
        internal void Observe(SourceStepContext source, Vector3[] positions, Quaternion[] rotations)
        {
            if (!Pending) return;
            if (!source.Valid || source.Time < _requested || (_last.Valid &&
                (source.Epoch != _last.Epoch || source.Sequence != _last.Sequence + 1 || source.Time < _last.Time)))
            { Cancel("SOURCE_DISCONTINUITY"); return; }
            if (source.Time - _requested >= TimeoutSeconds) { Cancel("TIMEOUT:" + Reason); return; }
            if (positions == null || rotations == null || positions.Length != BoneCount || rotations.Length != BoneCount)
            { Cancel("SOURCE_PAYLOAD_INVALID"); return; }
            for (int i = 0; i < BoneCount; i++)
            {
                Vector3 p = positions[i]; Quaternion q = rotations[i];
                float norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
                if (!Finite(p.x) || !Finite(p.y) || !Finite(p.z) || !Finite(norm) || norm < .99f || norm > 1.01f)
                { Cancel("SOURCE_POSE_INVALID"); return; }
            }
            float gap = _last.Valid ? source.Time - _last.Time : 0f;
            bool hadLast = _last.Valid;
            _last = source;
            if (Mathf.Abs(positions[3].y - positions[6].y) > MaxAnkleHeightDifference)
            { ResetWindow("WAIT_LEVEL_ANKLES"); return; }
            if (hadLast && (gap <= 0f || gap > MaxSampleGap))
            { ResetWindow("WAIT_CONTIGUOUS_SOURCE"); return; }
            bool moved = false;
            for (int i = 0; i < BoneCount && _samples > 0; i++)
                if ((positions[i] - _positions[i]).magnitude > MaxPositionDrift ||
                    Quaternion.Angle(rotations[i], _rotations[i]) > MaxRotationDrift) moved = true;
            if (moved) ResetWindow("WAIT_STABLE_POSE");
            if (_samples == 0)
            {
                for (int i = 0; i < BoneCount; i++) { _positions[i] = positions[i]; _rotations[i] = rotations[i]; }
                _windowStart = source.Time;
            }
            _samples++; StableSeconds = source.Time - _windowStart;
            if (_samples >= MinimumSamples && StableSeconds >= HoldSeconds)
            { Pending = false; Qualified = true; Reason = "STABLE_NEUTRAL_POSE"; }
        }
        private void ResetWindow(string reason) { _samples = 0; StableSeconds = 0f; Reason = reason; }
        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
