using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// v0.3.12 minimal fixed-ring hard-safety recorder retained after v0.3.10 closure.
    /// No timing correlations, no Rigidbody, no controller output, and no per-FixedUpdate allocation.
    /// </summary>
    internal sealed class KneeSafetyContextRecorder
    {
        internal const string Mode = "KNEE_HARD_SAFETY_MINIMAL_CONTEXT_RING_V03110";
        private const int Capacity = 12;

        private struct Sample
        {
            public int Fixed;
            public string Phase;
            public float PAnchor;
            public float SAnchor;
            public float PRawA;
            public float SRawA;
            public float PAppliedA;
            public float SAppliedA;
            public float PParentFf;
            public float SParentFf;
            public bool PClamp;
            public bool SClamp;
        }

        private readonly Sample[] _ring = new Sample[Capacity];
        private int _write;
        private int _count;
        private bool _traceEnabled;
        private bool _abortLatched;

        internal void Reset(KneeSafetyContextReport report)
        {
            _write = _count = 0;
            _traceEnabled = false;
            _abortLatched = false;
            if (report == null) return;
            report.Mode = Mode;
            report.TraceEnabled = false;
            report.Samples = 0;
            report.PrimaryPeakAnchorMeters = 0f;
            report.SecondaryPeakAnchorMeters = 0f;
            report.AbortCaptured = false;
            report.AbortSide = null;
            report.AbortReason = null;
            report.AbortFixedSequence = 0;
            report.AbortAnchorMeters = 0f;
            report.SafetyLimitMeters = KneeCharacterJointResponseIsolation.TetherAnchorValidityMeters;
            report.RingContext = null;
            report.Key = BuildKey(report);
        }

        internal void BeginTrace(KneeSafetyContextReport report)
        {
            Reset(report);
            _traceEnabled = true;
            if (report != null) report.TraceEnabled = true;
        }

        internal void ObserveFixedStep(KneeSafetyContextReport report, SingleLegShadowChain primary, SingleLegShadowChain secondary)
        {
            if (!_traceEnabled || report == null || primary == null || secondary == null) return;
            SingleLegBilateralReadOnlySnapshot p = primary.GetBilateralReadOnlySnapshot();
            SingleLegBilateralReadOnlySnapshot s = secondary.GetBilateralReadOnlySnapshot();
            if (!p.DriveEnabled && !s.DriveEnabled && !_abortLatched) return;

            float pa = p.KneePositionErrorWorld.magnitude;
            float sa = s.KneePositionErrorWorld.magnitude;
            if (Finite(pa) && pa > report.PrimaryPeakAnchorMeters) report.PrimaryPeakAnchorMeters = pa;
            if (Finite(sa) && sa > report.SecondaryPeakAnchorMeters) report.SecondaryPeakAnchorMeters = sa;
            report.Samples++;

            _ring[_write] = new Sample
            {
                Fixed = Mathf.Max(p.FixedSequence, s.FixedSequence),
                Phase = primary.Report != null ? primary.Report.AttributionPhase : null,
                PAnchor = pa,
                SAnchor = sa,
                PRawA = p.KneeRawAccelerationWorld.magnitude,
                SRawA = s.KneeRawAccelerationWorld.magnitude,
                PAppliedA = p.KneeAppliedAccelerationWorld.magnitude,
                SAppliedA = s.KneeAppliedAccelerationWorld.magnitude,
                PParentFf = p.KneeParentAnchorFeedforwardWorld.magnitude,
                SParentFf = s.KneeParentAnchorFeedforwardWorld.magnitude,
                PClamp = p.KneeTetherClamped,
                SClamp = s.KneeTetherClamped
            };
            _write = (_write + 1) % Capacity;
            if (_count < Capacity) _count++;

            if (!_abortLatched)
            {
                bool pAbort = primary.Report != null && primary.Report.DriveAborted;
                bool sAbort = secondary.Report != null && secondary.Report.DriveAborted;
                if (pAbort || sAbort)
                {
                    _abortLatched = true;
                    bool useP = pAbort && (!sAbort || pa >= sa);
                    SingleLegChainReport lr = useP ? primary.Report : secondary.Report;
                    report.AbortCaptured = true;
                    report.AbortSide = useP ? "PRIMARY" : "SECONDARY";
                    report.AbortReason = lr != null ? lr.DriveAbortReason : "<unknown>";
                    report.AbortFixedSequence = Mathf.Max(p.FixedSequence, s.FixedSequence);
                    report.AbortAnchorMeters = useP ? pa : sa;
                    report.RingContext = BuildRingText();
                }
            }
            report.Key = BuildKey(report);
        }

        internal string Finalize(KneeSafetyContextReport report)
        {
            _traceEnabled = false;
            if (report != null)
            {
                report.TraceEnabled = false;
                if (report.RingContext == null) report.RingContext = BuildRingText();
                report.Key = BuildKey(report);
            }
            return GetStatus(report);
        }

        internal string GetStatus(KneeSafetyContextReport report)
        {
            if (report == null) return "ARL Knee safety context | report=False";
            report.Key = BuildKey(report);
            return report.GetConsoleSummary();
        }

        private string BuildRingText()
        {
            if (_count <= 0) return "<none>";
            StringBuilder sb = new StringBuilder(2048);
            int start = (_write - _count + Capacity) % Capacity;
            for (int i = 0; i < _count; i++)
            {
                Sample x = _ring[(start + i) % Capacity];
                sb.Append("#").Append(x.Fixed)
                  .Append(" phase=").Append(x.Phase ?? "<none>")
                  .Append(" anchorP/S=").Append(F(x.PAnchor)).Append('/').Append(F(x.SAnchor))
                  .Append(" rawA=").Append(F(x.PRawA)).Append('/').Append(F(x.SRawA))
                  .Append(" appliedA=").Append(F(x.PAppliedA)).Append('/').Append(F(x.SAppliedA))
                  .Append(" parentFF=").Append(F(x.PParentFf)).Append('/').Append(F(x.SParentFf))
                  .Append(" clamp=").Append(x.PClamp).Append('/').Append(x.SClamp).AppendLine();
            }
            return sb.ToString();
        }

        private static string BuildKey(KneeSafetyContextReport r) =>
            "MODE=" + (r.Mode ?? "<none>") + "|N=" + r.Samples + "|PEAK=" + F(r.PrimaryPeakAnchorMeters) + "/" + F(r.SecondaryPeakAnchorMeters) + "|ABORT=" + r.AbortCaptured + "/" + (r.AbortSide ?? "<none>") + "|FIXED=" + r.AbortFixedSequence + "|ANCHOR=" + F(r.AbortAnchorMeters);

        private static bool Finite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static string F(float v) => Finite(v) ? v.ToString("0.######", CultureInfo.InvariantCulture) : "nan";
    }
}
