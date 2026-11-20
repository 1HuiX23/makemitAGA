using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class HipNormalReference
    {
        internal HipLandingPreparation.Result LandingPreparation { get; private set; }
        private readonly HipLandingPreparation.Trace _hipLandingTrace = new HipLandingPreparation.Trace();
        private bool _landingPreparationVisited;
        private float _peakBeforeStepGap, _peakBeforeStepW, _peakBeforeStepA;

        // Finalize once, using Step's original Previous R/W. Never integrate twice,
        // replace the input, reseed a derivative or advance either shared clock.
        internal bool PrepareLanding(bool selected, bool trace, KneeFootGroundPacket packet, KneeReferenceState knee)
        {
            if (!selected || !_selected) return true;
            if (_landingPreparationVisited) return Invalid("HIP_LANDING_DUPLICATE_FINALIZATION");
            _landingPreparationVisited = true;
            var frame = Current; var baselineFrame = frame; Quaternion baseline = _rotation; Vector3 baselineW = _w;
            HipLandingPreparation.Result result;
            try { result = HipLandingPreparation.Evaluate(packet, frame, baseline, baselineW, knee); }
            catch { result = new HipLandingPreparation.Result { Reason = HipLandingPreparation.Status.Faulted, R = baseline, W = baselineW }; }
            if (result.Better)
            {
                Vector3 a = (result.W - frame.PreviousW) / frame.Step.Dt;
                if (!LandingPreparationMath.ValidLane(frame.PreviousR, frame.PreviousW, frame.InputR, frame.InputW,
                    result.R, result.W, a, frame.Step.Dt, frame.MaxW, frame.MaxA)) return Invalid("HIP_LANDING_FINAL_BUDGET_INVALID");
                _rotation = result.R; _w = result.W; frame.A = a; result.Applied = true;
                // Replace this step's contribution, preserving earlier trace peaks.
                if (trace)
                {
                    _peakGap = Mathf.Max(_peakBeforeStepGap, Log(frame.InputR * Quaternion.Inverse(_rotation)).magnitude);
                    _peakW = Mathf.Max(_peakBeforeStepW, _w.magnitude); _peakA = Mathf.Max(_peakBeforeStepA, a.magnitude);
                }
            }
            LandingPreparation = result; Current = frame;
            if (trace) _hipLandingTrace.Observe(true, packet, baselineFrame, baseline, baselineW, knee, result);
            return true;
        }
    }
}
