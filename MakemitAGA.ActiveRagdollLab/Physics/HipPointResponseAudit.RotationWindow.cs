using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class HipPointResponseAudit
    {
        // Same owner, same pending command, same next-settled response. No native reads here.
        // The receiver owns peak selection; each side must retain that peak even if its
        // command failed to join. Fixed-size holes are preferable to a different local peak.
        private const int RotationBefore = 8, RotationAfter = 8;
        private readonly ResponseFrame[] _rotationWindow = new ResponseFrame[RotationBefore + 1 + RotationAfter];
        private PelvisReceiverReport.TrackingFrame _rotationPeak;
        private PhysicsStepContext _rotationSessionStep;
        private int _rotationSessionFixed;

        private void ResetRotationWindow()
        {
            _rotationPeak = new PelvisReceiverReport.TrackingFrame();
            _rotationSessionStep = new PhysicsStepContext(); _rotationSessionFixed = 0;
            Array.Clear(_rotationWindow, 0, _rotationWindow.Length);
        }
        private static bool RotationTrackingValid(PelvisReceiverReport.TrackingFrame f) =>
            f.Valid && f.RuntimeContextExpected && f.RuntimeContext.Valid && f.Fixed > 0 &&
            f.Dt == f.RuntimeContext.Dt && F(f.Time) &&
            F(f.Rotation) && f.Rotation >= 0f && F(f.Position) && F(f.RawRenderRotation) &&
            V(f.RotationErrorVector) && V(f.BodyW) && V(f.CommandW);

        private void CaptureRotationTracking(PhysicsStepContext step, int localFixed, PelvisReceiverReport receiver)
        {
            try
            {
                if (receiver == null || !receiver.TraceRunning || !receiver.TraceHipTranslation ||
                    receiver.CurrentTraceFixed != localFixed) return;
                var t = receiver.LastTracking;
                if (_hasPending && _pending.Command.Step.Matches(step) && _pending.Command.Fixed == localFixed &&
                    RotationTrackingValid(t) && t.Fixed == localFixed && t.RuntimeContext.Matches(step))
                    _pending.ReceiverTracking = t;
                SyncRotationPeak(receiver);
                if (_hasPending) RetainRotationFrame(_pending);
            }
            catch { } // Optional evidence cannot reject a successful actuator command.
        }
        private void SyncRotationPeak(PelvisReceiverReport receiver)
        {
            if (receiver == null || !_rotationSessionStep.Valid || _rotationSessionFixed <= 0) return;
            var peak = receiver.PeakRotationTracking;
            if (!RotationTrackingValid(peak) || peak.RuntimeContext.Epoch != _rotationSessionStep.Epoch ||
                peak.Fixed - _rotationSessionFixed != peak.RuntimeContext.Sequence - _rotationSessionStep.Sequence) return;
            if (_rotationPeak.Valid && _rotationPeak.Fixed == peak.Fixed && _rotationPeak.RuntimeContext.Matches(peak.RuntimeContext)) return;
            _rotationPeak = peak;
            Array.Clear(_rotationWindow, 0, _rotationWindow.Length);
            for (int i = 0; i < _count; i++)
                RetainRotationFrame(_ring[(_next - _count + i + _ring.Length) % _ring.Length]);
            if (_hasPending) RetainRotationFrame(_pending);
        }
        private void RetainRotationFrame(ResponseFrame f)
        {
            if (!_rotationPeak.Valid || !f.Command.Step.Valid || f.Command.Fixed <= 0 ||
                f.Command.Step.Epoch != _rotationPeak.RuntimeContext.Epoch) return;
            int offset = f.Command.Fixed - _rotationPeak.Fixed;
            if (offset < -RotationBefore || offset > RotationAfter ||
                f.Command.Step.Sequence - _rotationPeak.RuntimeContext.Sequence != offset ||
                (offset == 0 && !f.Command.Step.Matches(_rotationPeak.RuntimeContext))) return;
            _rotationWindow[offset + RotationBefore] = f;
        }
        private void AppendRotationWindow(StringBuilder sb, PelvisReceiverReport receiver)
        {
            sb.Append("\nhipPelvisRotationWindow=V041090_READ_ONLY/selector=RECEIVER_CONTROL_ROTATION_PEAK/pre-post=")
                .Append(RotationBefore).Append('|').Append(RotationAfter)
                .Append("/phase=ERROR_AND_INPUT_AT_N_RESPONSE_AT_NEXT_SETTLED/noFeedback=True/sharedReceiverFields=DO_NOT_SUM");
            if (!_rotationPeak.Valid)
            { sb.Append("/peak=NA_NO_JOINED_COMMAND_SESSION_OR_VALID_RECEIVER_PEAK"); return; }
            sb.Append("/peakFixed=").Append(_rotationPeak.Fixed).Append('/').Append(_rotationPeak.RuntimeContext.Status())
                .Append("/peakControlDeg=").Append(N(_rotationPeak.Rotation)).Append("/receiverPeakMatch=")
                .Append(receiver == null ? "NA_REPORT_NOT_SUPPLIED" :
                    (RotationTrackingValid(receiver.PeakRotationTracking) && receiver.PeakRotationTracking.Fixed == _rotationPeak.Fixed &&
                     receiver.PeakRotationTracking.RuntimeContext.Matches(_rotationPeak.RuntimeContext)).ToString());
            int issued = 0, resolved = 0, censored = 0, tracking = 0;
            for (int i = 0; i < _rotationWindow.Length; i++)
            {
                var f = _rotationWindow[i];
                if (f.Command.Fixed <= 0) continue;
                issued++; if (f.Resolved) resolved++; if (f.Censored) censored++;
                if (f.ReceiverTracking.Valid) tracking++;
            }
            sb.Append("/coverage(issued-resolved-censored-tracking-missing)=").Append(issued).Append('|').Append(resolved)
                .Append('|').Append(censored).Append('|').Append(tracking).Append('|').Append(_rotationWindow.Length - issued);
            for (int i = 0; i < _rotationWindow.Length; i++)
            {
                int offset = i - RotationBefore;
                var f = _rotationWindow[i];
                if (f.Command.Fixed <= 0)
                {
                    sb.Append("\nhipPelvisRotationMissing=offset:").Append(offset).Append("/expectedLocalGlobal=")
                        .Append(_rotationPeak.Fixed + offset).Append('|').Append(_rotationPeak.RuntimeContext.Sequence + offset)
                        .Append("/response=NA_NO_CAPTURED_COMMAND");
                    continue;
                }
                Append(sb, "PELVIS_ROTATION_PEAK_OFFSET_" + offset, f);
                var t = f.ReceiverTracking;
                if (!t.Valid) { sb.Append("/pelvisTracking=NA_NOT_JOINED"); continue; }
                sb.Append("/pelvisTracking=PRE_SOLVE_SAME_COMMAND/receiverTime-dt=").Append(N(t.Time)).Append('|').Append(N(t.Dt))
                    .Append("/pelvisControl(COMmm-angleDeg)=").Append(N(t.Position * 1000f)).Append('|').Append(N(t.Rotation))
                    .Append("/pelvisRawAngleDeg=").Append(N(t.RawRenderRotation))
                    .Append("/pelvisRotationErrorRad=").Append(S(t.RotationErrorVector))
                    .Append("/pelvisW(body-control-source)=").Append(S(t.BodyW)).Append('|').Append(S(t.CommandW)).Append('|').Append(S(t.SourceWVector))
                    .Append("/pelvisSource(stage-phase-seq-event-recovery-active-fresh-age-dt)=").Append(t.Stage).Append('|')
                    .Append(t.SourcePhase).Append('|').Append(t.SourceSequence).Append('|').Append(t.SourceEventId).Append('|').Append(t.RecoveryEventId)
                    .Append('|').Append(t.SourceActive).Append('|').Append(t.SourceFresh).Append('|').Append(N(t.SourceAge)).Append('|').Append(N(t.SourceDt))
                    .Append("/receiverClamps(L-A-T)=").Append(t.LinearClamped).Append('|').Append(t.AngularClamped).Append('|').Append(t.TorqueClamped);
                var a = t.AngularActuation;
                if (!a.Valid || a.Fixed != f.Command.Fixed || !a.Step.Matches(f.Command.Step))
                { sb.Append("/pelvisBaseline=NA_NOT_JOINED"); continue; }
                sb.Append("/pelvisBaselineAlpha(P-D-FF-applied)=").Append(S(a.PAlpha)).Append('|').Append(S(a.DAlpha))
                    .Append('|').Append(S(a.FeedforwardAlpha)).Append('|').Append(S(a.AppliedAlpha))
                    .Append("/pelvisBaselineTorqueNm(raw-applied)=").Append(S(a.RawTorque)).Append('|').Append(S(a.Torque));
            }
        }
    }
}
