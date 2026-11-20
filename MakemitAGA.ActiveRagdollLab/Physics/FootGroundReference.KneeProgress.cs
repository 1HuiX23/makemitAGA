using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootGroundReference
    {
        private bool _kneeProgressSelected, _traceKneeProgressSelected, _hasProgressWitness;
        private FootKneeProgressTarget.Result _kneeProgress;
        private FootRelativeReference.Sample _progressExpected; // Live publication contract; never cleared by trace reset.
        private readonly int[] _progressReasons = new int[(int)FootKneeProgressTarget.Status.Improved + 1];
        private int _progressSteps, _progressEvaluated, _progressPreviewed, _progressBetter, _progressApplied, _progressFaults;
        private double _progressGainSum;
        private float _progressGainPeak;
        private KneeFootGroundPacket _progressWitness;
        private FootGroundTrajectory.Input _progressWitnessGeometry;
        private FootKneeProgressTarget.Result _progressWitnessResult;
        private FootRelativeReference.Sample _progressPrevious, _progressBaseline, _progressCandidate;
        private Quaternion _progressBaselineGoal, _progressCandidateGoal;
        private Vector3 _progressBaselineGoalW, _progressCandidateGoalW;
        internal bool KneeProgressSelected => _kneeProgressSelected;
        internal FootKneeProgressTarget.Result CurrentKneeProgress => _kneeProgress;

        internal void SelectKneeProgress(bool selected)
        {
            if (selected && (!_enabled || !_apply || !_allowPartial || !_finite))
                throw new InvalidOperationException("KNEE_PROGRESS_REQUIRES_JOINT_GROUND_REFERENCE");
            _kneeProgressSelected = selected; _kneeProgress = default; _progressExpected = default;
            ResetKneeProgressTrace();
        }
        private void ResetKneeProgressTrace()
        {
            _traceKneeProgressSelected = _kneeProgressSelected;
            _progressSteps = _progressEvaluated = _progressPreviewed = _progressBetter = _progressApplied = _progressFaults = 0;
            Array.Clear(_progressReasons, 0, _progressReasons.Length);
            _progressGainSum = 0d; _progressGainPeak = 0f; _hasProgressWitness = false;
            _progressWitness = default; _progressWitnessGeometry = default; _progressWitnessResult = default;
            _progressPrevious = _progressBaseline = _progressCandidate = default;
            _progressBaselineGoal = _progressCandidateGoal = default;
            _progressBaselineGoalW = _progressCandidateGoalW = default;
        }

        // Called once for a fresh Render, before either original owner consumes it.
        // Previews are pure. Only the final candidate enters ObserveRender's history.
        internal void ChooseKneeProgress(bool trace, SourceStepContext source, Quaternion raw, Vector3 rawW, float dt,
            KneeFootGroundPacket packet, FootRelativeReference planner, ref GroundSample candidate, ref FootGroundTrajectory.Input geometry)
        {
            var result = new FootKneeProgressTarget.Result { Source = source, Selected = _kneeProgressSelected };
            var originalCandidate = candidate; var originalGeometry = geometry;
            _progressExpected = default;
            try
            {
                result = FootKneeProgressTarget.Evaluate(packet, geometry, candidate, out GroundSample waypoint);
                result.Source = source; result.Selected = _kneeProgressSelected;
                if (result.Reason == FootKneeProgressTarget.Status.PreviewUnavailable &&
                    packet.Input.PreferKneeProgress == _kneeProgressSelected && planner.GroundTrajectorySelected)
                {
                    var waypointGeometry = geometry;
                    waypointGeometry.KneeGoal = packet.Input.CompletedKneeReference.R;
                    waypointGeometry.CurrentKneeBasis = true;
                    if (TryPreviewInput(source, raw, rawW, dt, candidate, out Quaternion baseGoal, out Vector3 baseW) &&
                        TryPreviewInput(source, raw, rawW, dt, waypoint, out Quaternion nextGoal, out Vector3 nextW) &&
                        planner.TryPreviewRender(source, baseGoal, baseW, dt, geometry, out var baselinePlan) &&
                        planner.TryPreviewRender(source, nextGoal, nextW, dt, waypointGeometry, out var waypointPlan))
                    {
                        FootKneeProgressTarget.AssessPreview(ref result, packet, geometry, baselinePlan, waypointPlan);
                        if (_kneeProgressSelected && result.Better)
                        {
                            candidate = waypoint; geometry = waypointGeometry; result.Applied = true;
                            _progressExpected = waypointPlan;
                        }
                        if (trace && result.Previewed && result.StepBaselineMin < -KneeGroundTarget.MinGain &&
                            (!_hasProgressWitness || result.StepBaselineMin < _progressWitnessResult.StepBaselineMin))
                        {
                            _hasProgressWitness = true; _progressWitness = packet; _progressWitnessGeometry = geometry;
                            _progressWitnessResult = result; _progressPrevious = planner.Current;
                            _progressBaseline = baselinePlan; _progressCandidate = waypointPlan;
                            _progressBaselineGoal = baseGoal; _progressCandidateGoal = nextGoal;
                            _progressBaselineGoalW = baseW; _progressCandidateGoalW = nextW;
                        }
                    }
                }
            }
            catch
            {
                // No candidate mutation precedes a successful preview. Keep original
                // owners responsible for their own native/source/finite contracts.
                candidate = originalCandidate; geometry = originalGeometry; _progressExpected = default;
                result = new FootKneeProgressTarget.Result { Source = source, Selected = _kneeProgressSelected };
                if (trace) _progressFaults++;
            }
            _kneeProgress = result;
            if (!trace) return;
            _traceKneeProgressSelected = _kneeProgressSelected; _progressSteps++; _progressReasons[(int)result.Reason]++;
            if (result.Evaluated) _progressEvaluated++;
            if (result.Previewed) _progressPreviewed++;
            if (result.Better)
            {
                _progressBetter++; float gain = result.StepCandidateMin - result.StepBaselineMin;
                _progressGainSum += gain; _progressGainPeak = Mathf.Max(_progressGainPeak, gain);
            }
            if (result.Applied) _progressApplied++;
        }
        internal void ValidateKneeProgressPublication(SourceStepContext source, FootRelativeReference.Sample actual)
        {
            if (!_kneeProgress.Applied) return;
            if (!_enabled || !_kneeProgressSelected || !_kneeProgress.Source.Matches(source) ||
                !_progressExpected.Source.Matches(source) || !actual.Source.Matches(source) ||
                !Exact(actual.Rotation, _progressExpected.Rotation) || !Exact(actual.W, _progressExpected.W) ||
                !Exact(actual.A, _progressExpected.A) || actual.Dt != _progressExpected.Dt)
                Invalidate("KNEE_PROGRESS_FINAL_PUBLICATION_MISMATCH");
        }

        private string KneeProgressSummary() => "\nFOOT_KNEE_PROGRESS_V041350/selected-traceSelected=" + _kneeProgressSelected + "|" + _traceKneeProgressSelected +
            "/render-evaluated-previewed-better-applied=" + _progressSteps + "|" + _progressEvaluated + "|" + _progressPreviewed + "|" + _progressBetter + "|" + _progressApplied +
            "/reasons(unavailable-invalid-clear-noGoalGain-nativeGoalRegression-previewUnavailable-noStepGain-nativeStepRegression-improved)=" + string.Join("|", _progressReasons) +
            "/stepGainMm(mean-peak)=" + (_progressBetter > 0 ? S((float)(_progressGainSum / _progressBetter) * 1000f) : "NA") + "|" + S(_progressGainPeak * 1000f) +
            "/faults=" + _progressFaults + "/kneeEndpoint=UNCHANGED/footPlanner=ORIGINAL_RENDER_RWA_AND_GROUND_STEP/noNewForce=True" + ProgressWitnessText();

        private string ProgressWitnessText()
        {
            if (!_hasProgressWitness) return "/kneeProgressWitness=NONE";
            var p = _progressWitness; var g = p.Geometry; var k = p.Input.CompletedKneeReference; var native = _progressWitnessGeometry;
            return "\nFOOT_KNEE_PROGRESS_WITNESS_V041350/selection=DEEPEST_PREVIEWED_BASELINE_REFERENCE" +
                "/source(epoch-seq-time-dt)=" + p.Input.Source.Epoch + "|" + p.Input.Source.Sequence + "|" + S(p.Input.Source.Time) + "|" + S(p.Input.Source.Dt) +
                "/completed(fixed-source-fixedTime-unscaledTime-dt)=" + p.Input.CompletedStep.Sequence + "|" + p.Input.CompletedStep.SourceSequence + "|" +
                S(p.Input.CompletedStep.FixedTime) + "|" + S(p.Input.CompletedStep.UnscaledTime) + "|" + S(p.Input.CompletedStep.Dt) +
                "/owners-support=" + p.Input.ParentId + "|" + p.Input.UpperId + "|" + p.Input.LowerId + "|" + p.SupportId +
                "/" + _progressWitnessResult.Text() +
                "/knee(referenceR-W-A-goalR)=" + Qs(k.R) + "|" + Vs(k.W) + "|" + Vs(k.A) + "|" + Qs(p.Sample.Goal) +
                "/kneeSource(seq-time-dt)=" + k.Source.Sequence + "|" + S(k.Source.Time) + "|" + S(k.Source.Dt) +
                "/hip-upper=" + Vs(g.Hip) + "|" + Qs(g.Upper) + "/upperSpan-lowerSpan=" + Vs(g.UpperSpan) + "|" + Vs(g.LowerSpan) +
                "/boxCenter-half=" + Vs(g.Center) + "|" + Vs(g.Half) + "/floorCenter-half=" + Vs(g.FloorCenter) + "|" + Vs(g.FloorHalf) +
                "/nativeAnkle-lower=" + Vs(native.NativeAnkle) + "|" + Qs(native.NativeLower) +
                "/footR(raw-bind-previous-baseGoal-waypoint-basePlan-waypointPlan)=" + Qs(g.FootRelative) + "|" + Qs(g.FootBindRelative) + "|" + Qs(_progressPrevious.Rotation) + "|" +
                Qs(_progressBaselineGoal) + "|" + Qs(_progressCandidateGoal) + "|" + Qs(_progressBaseline.Rotation) + "|" + Qs(_progressCandidate.Rotation) +
                "/footW(previous-baseGoal-waypoint-basePlan-waypointPlan)=" + Vs(_progressPrevious.W) + "|" + Vs(_progressBaselineGoalW) + "|" + Vs(_progressCandidateGoalW) + "|" +
                Vs(_progressBaseline.W) + "|" + Vs(_progressCandidate.W) + "/planDt=" + S(_progressCandidate.Dt) +
                "/units=METERS_RADIANS_SECONDS/heldKneeReference=True/noNativeSolveProof=True";
        }
        private static bool Exact(Quaternion a, Quaternion b) => a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
        private static bool Exact(Vector3 a, Vector3 b) => a.x == b.x && a.y == b.y && a.z == b.z;
        private static string Vs(Vector3 v) => S(v.x) + "," + S(v.y) + "," + S(v.z);
        private static string Qs(Quaternion q) => S(q.x) + "," + S(q.y) + "," + S(q.z) + "," + S(q.w);
    }
}
