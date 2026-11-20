using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;
using Shin = MakemitAGA.ActiveRagdollLab.Physics.LeftShinForecast;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class LeftFootReleaseTrace
    {
        private int _shinPredicted, _shinPaired, _revisedTrials, _revisedPaired, _revisedCrossings;
        private double _velocityAbsY, _velocityXZ, _issuedAbsY, _issuedXZ;

        internal Row PendingShin(in PhysicsStepContext step, in SourceStepContext source, int local)
        {
            var row = _pending;
            return row != null && !row.ShinCompleted && row.Command.Predicted && row.From.Local == local &&
                row.From.Step.Matches(step) && row.From.Source.Matches(source) ? row : null;
        }

        internal void CompleteShin(Row row, in Shin.Result result)
        {
            if (row == null || row != _pending || row.ShinCompleted || !row.Command.Predicted) return;
            if (result.State == Shin.Status.Predicted)
            {
                row.Corrected = Foot.WithIssuedLower(row.Before, row.From.Gait, row.Motion, row.Command, result.LowerAnkle);
                if (!row.Corrected.Predicted)
                {
                    row.Shin = new Shin.Result { State = Shin.Status.InvalidInput };
                    row.ShinCompleted = true;
                    EndLead("InvalidFootForecast");
                    return;
                }
                _shinPredicted++;
                if (row.Corrected.State == Foot.Decision.Trial) _revisedTrials++;
            }
            row.Shin = result;
            row.ShinCompleted = true;
            if (result.State != Shin.Status.Predicted) { EndLead(result.State.ToString()); return; }
            try { IssueLead(row); }
            catch { _faults++; EndLead("LeadFault"); }
        }

        private static float XZ(Vector3 v) => Mathf.Sqrt(v.x * v.x + v.z * v.z);
        private static Vector3 VelocityEndpoint(Row r) =>
            r.Before.Pose.Box.LowerAnkle + r.Before.LowerVelocity * r.From.Step.Dt;

        private void SettleShin(Row row, in Foot.Frame next)
        {
            if (row.Shin.State != Shin.Status.Predicted || !next.Anchors.Valid ||
                next.UpperId == 0 || row.Before.UpperId != next.UpperId) return;
            row.ShinPaired = true;
            _shinPaired++;
            var actual = next.Pose.Box.LowerAnkle;
            var velocityError = actual - VelocityEndpoint(row);
            var issuedError = actual - row.Shin.LowerAnkle;
            _velocityAbsY += Mathf.Abs(velocityError.y);
            _velocityXZ += XZ(velocityError);
            _issuedAbsY += Mathf.Abs(issuedError.y);
            _issuedXZ += XZ(issuedError);
            if (row.Corrected.State != Foot.Decision.Trial) return;
            _revisedPaired++;
            row.CorrectedCross = Foot.ReadTrial(row.Corrected, next, out row.CorrectedSole, out row.CorrectedGap);
            if (row.CorrectedCross) _revisedCrossings++;
        }

        private string MeanMm(double sum) => _shinPaired == 0 ? "NA" : N((float)(1000d * sum / _shinPaired));

        private void AppendShinSummary(StringBuilder b)
        {
            b.Append("\nLEFT_SHIN_V041650/issued-predicted-paired=").Append(_issued).Append('|').Append(_shinPredicted)
                .Append('|').Append(_shinPaired).Append("/revisedTrials-paired-conditionedCross=").Append(_revisedTrials).Append('|')
                .Append(_revisedPaired).Append('|').Append(_revisedCrossings)
                .Append("/meanErrorMm(velocityAbsY-XZ-issuedAbsY-XZ)=").Append(MeanMm(_velocityAbsY)).Append('|')
                .Append(MeanMm(_velocityXZ)).Append('|').Append(MeanMm(_issuedAbsY)).Append('|').Append(MeanMm(_issuedXZ))
                .Append("/source=AFTER_BILATERAL_COMMANDS/plantWrites=0");
        }

        private static string Vec(Vector3 v, float scale) =>
            "(" + N(v.x * scale) + "," + N(v.y * scale) + "," + N(v.z * scale) + ")";

        private static void ChainDeltaY(StringBuilder b, in Foot.Frame start, Vector3 hip, Vector3 upperKnee, Vector3 lowerKnee, Vector3 ankle)
        {
            float h = hip.y - start.Anchors.UpperHip.y;
            float uk = upperKnee.y - start.Anchors.UpperKnee.y;
            float lk = lowerKnee.y - start.Anchors.LowerKnee.y;
            float a = ankle.y - start.Pose.Box.LowerAnkle.y;
            b.Append(Mm(h)).Append('|').Append(Mm(uk - h)).Append('|').Append(Mm(lk - uk)).Append('|').Append(Mm(a - lk));
        }

        private static bool SwingEntry(Row row) => row != null && row.Settled &&
            row.From.Gait == Phase.Lift && row.To.Gait == Phase.Swing;

        private static void AppendShinRow(StringBuilder b, Row r, Row next)
        {
            // Retrospective labels only. Both forecasts already existed before the solver ran.
            string role = SwingEntry(r) ? "SwingEntry" :
                SwingEntry(next) && r.Settled && r.To.Local == next.From.Local ? "BeforeSwingEntry" :
                r.From.Released ? "Release" : "Other";
            var f = r.Before;
            var box = f.Pose.Box;
            var s = r.Shin;
            b.Append("\nLEFT_SHIN_STEP_V041650/from-to=").Append(r.From.Local).Append('|').Append(r.Settled ? r.To.Local : 0)
                .Append("/role=").Append(role).Append("/status=").Append(s.State).Append("/paired=").Append(r.ShinPaired);
            if (s.State != Shin.Status.Predicted) return;
            var old = VelocityEndpoint(r);
            var foot = r.Command.FootPoint;
            b.Append("/lowerShiftMm(velocity-issued)=").Append(Vec(old - box.LowerAnkle,1000)).Append('|')
                .Append(Vec(s.LowerAnkle - box.LowerAnkle,1000))
                .Append("/ankleGapVectorMm(velocity-issued)=").Append(Vec(foot - old,1000)).Append('|').Append(Vec(foot - s.LowerAnkle,1000))
                .Append("/chainYmm(hip-upperSpan-kneeGap-lowerSpan)=");
            ChainDeltaY(b, f, s.Hip, s.UpperKnee, s.LowerKnee, s.LowerAnkle);
            b.Append("/lowerShiftPartsMm(coastCOM-common-spring-damper-parentFF-rotation)=")
                .Append(Vec(s.CoastCom,1000)).Append('|').Append(Vec(s.CommonCom,1000)).Append('|').Append(Vec(s.SpringCom,1000))
                .Append('|').Append(Vec(s.DamperCom,1000)).Append('|').Append(Vec(s.FeedforwardCom,1000)).Append('|').Append(Vec(s.Turn,1000))
                .Append("/upperForceN-netMomentNm=").Append(Vec(s.HipForce,1)).Append('|').Append(Vec(s.UpperNetMoment,1))
                .Append("/lowerAlphaRadSec2(hipTransport-kneeRelative-ankleReaction)=").Append(Vec(s.HipTransportAlpha,1))
                .Append('|').Append(Vec(s.KneeRelativeAlpha,1)).Append('|').Append(Vec(s.AnkleReactionAlpha,1))
                .Append("/revisedDecision=").Append(r.Corrected.State)
                .Append("/revisedBaseGap-rise-trialSole-trialGapMm=").Append(Mm(r.Corrected.Gap)).Append('|').Append(Mm(r.Corrected.Rise))
                .Append('|').Append(Mm(r.Corrected.Sole + r.Corrected.Rise)).Append('|').Append(Mm(r.Corrected.TrialGap));
            if (!r.ShinPaired) return;
            var n = r.After; var q = n.Pose.Box;
            b.Append("/actualLowerShift-gapVectorMm=").Append(Vec(q.LowerAnkle - box.LowerAnkle,1000)).Append('|')
                .Append(Vec(q.FootAnkle - q.LowerAnkle,1000)).Append("/actualChainYmm=");
            ChainDeltaY(b, f, n.Anchors.UpperHip, n.Anchors.UpperKnee, n.Anchors.LowerKnee, q.LowerAnkle);
            b.Append("/lowerResidualMm(actualMinusVelocity-actualMinusIssued)=").Append(Vec(q.LowerAnkle - old,1000))
                .Append('|').Append(Vec(q.LowerAnkle - s.LowerAnkle,1000));
            if (r.Corrected.State == Foot.Decision.Trial)
                b.Append("/revisedConditionedSole-gapMm=").Append(Mm(r.CorrectedSole)).Append('|').Append(Mm(r.CorrectedGap))
                    .Append("/revisedConditionedCross=").Append(r.CorrectedCross);
        }
    }
}
