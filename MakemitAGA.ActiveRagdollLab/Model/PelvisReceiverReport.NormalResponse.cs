using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed partial class PelvisReceiverReport
    {
        // One last resolved value, not another ring/observer. Existing entry history
        // copies it while the same PRE_N bodies are sampled. Epoch/sequence joins
        // prevent a trace reset, gap, stop or later run from supplying stale evidence.
        private PelvisNormalResponseEvidence _normalLatest;
        private int _normalSamples, _normalMissing, _normalMotionSamples;
        private double _normalAbsErrorSum, _normalMotionAbsErrorSum;
        private float _normalAbsErrorPeak, _normalMotionAbsErrorPeak, _normalAllocationGapPeak;

        internal PelvisNormalResponseEvidence NormalResponseAt(PhysicsStepContext step) =>
            TraceRunning && TraceHipTranslation && _normalLatest.Matches(step) ? _normalLatest : default;

        private void ResetNormalResponse()
        {
            _normalLatest = default;
            _normalSamples = _normalMissing = _normalMotionSamples = 0;
            _normalAbsErrorSum = _normalMotionAbsErrorSum = 0d;
            _normalAbsErrorPeak = _normalMotionAbsErrorPeak = _normalAllocationGapPeak = 0f;
        }

        private void RecordNormalResponse(Frame f)
        {
            _normalLatest = default;
            try
            {
                MotionInput m = f.Motion;
                if (!m.Valid || !f.Resolved || f.Censored || !f.HipReactionRecorded ||
                    !f.HipAttitude.Matches(f.HipAttitude.Baseline) || !f.HipAttitude.Baseline.Step.Matches(m.Step) ||
                    !m.ComReferenceStep.Matches(m.Step) || !m.ComReference.ArrivalEnabled)
                { _normalMissing++; return; }
                float mass = f.PelvisMass + f.ChestMass;
                bool enhanced = m.ComReference.EnhancedNormalFeedback;
                var s = new PelvisNormalResponseEvidence
                {
                    Valid = true, Input = m.Step, Settled = f.MotionNextStep, Fixed = f.Fixed,
                    PelvisId = f.HipAttitude.Baseline.BodyId, ChestId = f.HipAttitude.Baseline.ChestBodyId,
                    Enhanced = enhanced, SourceFresh = m.SourceFresh, SourcePhase = m.SourcePhase, SourceAge = m.SourceAge,
                    ErrorY = m.Error.y, VelocityErrorY = m.VelocityError.y,
                    P = PelvisComFeedback.PositionY(enhanced) * m.Error.y,
                    D = PelvisComFeedback.VelocityY(enhanced) * m.VelocityError.y,
                    Feedforward = m.FeedforwardA.y, Request = f.HipAttitude.Baseline.Acceleration.y,
                    Actuator = f.HipAttitude.CommonAcceleration.y, Allocated = f.HipAttitude.TargetSystemAcceleration.y,
                    UpwardSelected = f.HipAttitude.Baseline.UpwardRetentionSelected,
                    UpwardEligible = f.HipAttitude.Baseline.UpwardRetentionEligible, UpwardExtra = f.HipAttitude.UpwardExtraAcceleration,
                    Q = f.HipAttitude.MotionScale, Observed = m.ObservedA.y, LinearClamped = m.LinearClamped,
                    PreV = (f.PelvisMass * f.PrePelvisV.y + f.ChestMass * f.PreChestV.y) / mass,
                    PostV = (f.PelvisMass * f.PostPelvisV.y + f.ChestMass * f.PostChestV.y) / mass
                };
                if (!s.Matches(f.MotionNextStep) || !FiniteValue(m.PdA.y) ||
                    Math.Abs(s.P + s.D - m.PdA.y) > 0.0001f * (1f + Math.Abs(m.PdA.y)))
                { _normalMissing++; return; }
                _normalLatest = s;
                float abs = Math.Abs(s.ErrorY);
                _normalSamples++; _normalAbsErrorSum += abs;
                _normalAbsErrorPeak = Math.Max(_normalAbsErrorPeak, abs);
                _normalAllocationGapPeak = Math.Max(_normalAllocationGapPeak, Math.Abs(s.Request - s.Allocated));
                string phase = s.SourcePhase;
                if (phase == "START" || phase == "STOP" || phase == "TURN" || phase == "ACCEL" ||
                    phase == "DECEL" || phase == "STEADY" || phase == "LOW_SPEED")
                {
                    _normalMotionSamples++; _normalMotionAbsErrorSum += abs;
                    _normalMotionAbsErrorPeak = Math.Max(_normalMotionAbsErrorPeak, abs);
                }
            }
            catch { _normalMissing++; } // Evidence cannot affect a force, clock or safety result.
        }

        private void AppendNormalResponse(StringBuilder sb)
        {
            sb.Append("\nreceiverNormalFeedback=V041240/resolved-missing-sourceNonIdle=")
                .Append(_normalSamples).Append('|').Append(_normalMissing).Append('|').Append(_normalMotionSamples)
                .Append("/absErrorYmm(mean-peak)=").Append(F(_normalSamples > 0 ? (float)(_normalAbsErrorSum / _normalSamples) * 1000f : 0f))
                .Append('|').Append(F(_normalAbsErrorPeak * 1000f))
                .Append("/sourceNonIdleAbsErrorYmm(mean-peak)=").Append(F(_normalMotionSamples > 0 ? (float)(_normalMotionAbsErrorSum / _normalMotionSamples) * 1000f : 0f))
                .Append('|').Append(F(_normalMotionAbsErrorPeak * 1000f))
                .Append("/requestToAllocatedYGapPeak=").Append(F(_normalAllocationGapPeak))
                .Append("/domain=INPUT_N_ERROR_AND_SETTLED_N_PLUS_1_SYSTEM_COM_NOT_PELVIS_POINT/notContactCertification=True");
        }
    }
}
