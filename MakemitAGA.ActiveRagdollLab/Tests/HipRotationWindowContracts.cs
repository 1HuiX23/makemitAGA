using System.Text;
using ARL.Contracts;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class HipPointResponseAudit
    {
        internal static void RunRotationWindowContracts()
        {
            Check.Run("rotation and attitude evidence join despite preparation offset", RotationJoins);
            Check.Run("rotation peak keeps prior resolved frames in shared-step order", RotationRetains);
            Check.Run("rotation window excludes stale and different-epoch evidence", RotationRejects);
        }
        private static PelvisReceiverReport.TrackingFrame Tracking(int offset, int receiverFixed)
        {
            var step = PelvisDynamicReceiverIsolation.LoggedStep(offset);
            return new PelvisReceiverReport.TrackingFrame { Valid = true, RuntimeContextExpected = true,
                RuntimeContext = step, Fixed = receiverFixed, Dt = step.Dt, Time = step.FixedTime,
                Position = .001f, Rotation = 2f, RawRenderRotation = 3f, RotationErrorVector = .03f * Vector3.up,
                BodyW = Vector3.zero, CommandW = Vector3.zero, SourceWVector = Vector3.zero,
                SourceActive = false, SourceFresh = true, SourceAge = 0f, SourceDt = 1f / 60f,
                Stage = 1, SourcePhase = "NORMAL", SourceSequence = step.SourceSequence, SourceEventId = 0, RecoveryEventId = 0,
                LinearClamped = false, AngularClamped = false, TorqueClamped = false,
                AngularActuation = new PelvisReceiverReport.AngularActuationSample { Valid = true, Step = step,
                    Fixed = receiverFixed, PAlpha = Vector3.zero, DAlpha = Vector3.zero, FeedforwardAlpha = Vector3.zero,
                    AppliedAlpha = Vector3.zero, RawTorque = .2f * Vector3.up, Torque = .2f * Vector3.up } };
        }
        private void SetPending(int offset, int legFixed, bool resolved = false)
        {
            var step = PelvisDynamicReceiverIsolation.LoggedStep(offset);
            var input = PelvisDynamicReceiverIsolation.Baseline(step);
            var l = PelvisDynamicReceiverIsolation.Pair(input, legFixed, true);
            var r = PelvisDynamicReceiverIsolation.Pair(input, legFixed, false);
            Check.That(PelvisHipAttitudeCommand.TryPlanWithReaction(input, -l.Current.Force-r.Current.Force,
                Vector3.zero, -.2f * Vector3.up, out var attitude), "valid final attitude fixture");
            attitude.CommonAcceleration = l.Current.CommonAcceleration; attitude.Committed = true;
            _pending = new ResponseFrame { Command = l.Current, Attitude = attitude, PeerForce = r.Current.Force,
                PeerParentTorque = Vector3.zero, CoupledInputsJoined = true, Resolved = resolved, Censored = false };
            _hasPending = true; _rotationSessionStep = step;
        }
        private static void RotationJoins()
        {
            foreach (int preparation in new[] { 0, 1, 10, 120 })
            {
                var a = new HipPointResponseAudit(); a.ResetRotationWindow(); a.SetPending(0, preparation + 1);
                var t = Tracking(0, 1);
                var report = new PelvisReceiverReport { CurrentTraceFixed = 1, LastTracking = t, PeakRotationTracking = t, AngularActuation = t.AngularActuation };
                a.CompleteAttitudeBudget(t.RuntimeContext, preparation + 1, report);
                Check.That(a._pending.Budget.Valid && a._pending.ReceiverTracking.Valid, "shared input joins budget and tracking");
                Check.That(a._rotationWindow[RotationBefore].Command.Fixed == preparation + 1, "peak center uses shared sequence");
                var text = new StringBuilder(); a.AppendRotationWindow(text, report);
                Check.That(text.ToString().Contains("/pelvisBaselineAlpha(") && !text.ToString().Contains("pelvisBaseline=NA_NOT_JOINED"),
                    "receiver actuation joins its own trace clock");
            }
        }
        private static void RotationRetains()
        {
            var a = new HipPointResponseAudit(); a.ResetRotationWindow();
            var peak = Tracking(8, 9);
            for (int i = 0; i < 17; i++)
            {
                a.SetPending(i, i + 11, true);
                var t = Tracking(i, i + 1);
                var report = new PelvisReceiverReport { CurrentTraceFixed = i + 1, LastTracking = t,
                    PeakRotationTracking = i < 8 ? t : peak, AngularActuation = t.AngularActuation };
                a.CompleteAttitudeBudget(t.RuntimeContext, i + 11, report);
                a._ring[a._next] = a._pending; a._next = (a._next + 1) % a._ring.Length;
                a._count = System.Math.Min(a._count + 1, a._ring.Length);
            }
            Check.That(a._rotationPeak.Fixed == 9, "older receiver peak remains valid as current shared step advances");
            for (int i = 0; i < 17; i++)
                Check.That(a._rotationWindow[i].Command.Fixed == i + 11 && a._rotationWindow[i].Resolved,
                    "before and after rows retain leg identity at shared offset " + i);
            a.ResetRotationWindow(); Check.That(!a._rotationPeak.Valid && a._rotationWindow[8].Command.Fixed == 0, "new trace clears previous window");
        }
        private static void RotationRejects()
        {
            for (int scenario = 0; scenario < 5; scenario++)
            {
                var a = new HipPointResponseAudit(); a.SetPending(0, 11);
                var t = Tracking(0, 1); var peak = t;
                if (scenario == 0) t.RuntimeContext.SourceSequence++;
                if (scenario == 1) peak.RuntimeContext.Epoch++;
                if (scenario == 2) peak.Fixed++;
                if (scenario == 3) peak.RuntimeContext.Sequence--;
                if (scenario == 4) t.AngularActuation.Step.Sequence--;
                var report = new PelvisReceiverReport { CurrentTraceFixed = 1, LastTracking = t, PeakRotationTracking = peak, AngularActuation = t.AngularActuation };
                a.CompleteAttitudeBudget(PelvisDynamicReceiverIsolation.LoggedStep(), 11, report);
                if (scenario < 4) Check.That(!a._rotationPeak.Valid, "invalid peak cannot be substituted " + scenario);
                else Check.That(!a._pending.Budget.Valid, "stale actuation cannot fabricate budget");
                Check.That(a._hasPending && a._pending.Command.Fixed == 11, "optional evidence never consumes or cancels command");
            }
        }
    }
}
