using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootRelativeReference
    {
        private bool _groundSelected, _traceGroundSelected, _hasGroundWitness;
        private readonly int[] _groundReasons = new int[8];
        private int _groundSteps, _groundEvaluated, _groundBelow, _groundBetter, _groundApplied;
        private int _groundCurrentKneeEvaluated, _groundCurrentKneeApplied;
        private double _groundGainSum;
        private float _groundGainPeak, _groundDeltaPeak;
        private FootGroundTrajectory.Input _groundWitness;
        private FootGroundTrajectory.Result _groundWitnessResult;
        private Quaternion _witnessPreviousR, _witnessGoal, _witnessBaselineR;
        private Vector3 _witnessPreviousW, _witnessNominalW, _witnessBaselineW;
        private float _witnessDt;
        internal bool GroundTrajectorySelected => _groundSelected;

        // The contact setup owner calls this with trace OFF and colliders quiesced.
        // Changing the experiment selection never reseeds the live pose/velocity.
        internal void SelectGroundTrajectory(bool selected)
        {
            if (selected && (!_selected || !_active || !_finite))
                throw new InvalidOperationException("FOOT_GROUND_TRAJECTORY_REQUIRES_ACTIVE_COHERENT_REFERENCE");
            _groundSelected = selected;
            ResetGroundTrace();
        }
        internal void DisableGroundTrajectoryPreserveTrace() { _groundSelected = false; }
        private void ResetGroundTrace()
        {
            _traceGroundSelected = _groundSelected; _hasGroundWitness = false;
            Array.Clear(_groundReasons, 0, _groundReasons.Length);
            _groundSteps = _groundEvaluated = _groundBelow = _groundBetter = _groundApplied = 0;
            _groundCurrentKneeEvaluated = _groundCurrentKneeApplied = 0;
            _groundGainSum = 0d; _groundGainPeak = _groundDeltaPeak = 0f;
            _groundWitness = default; _groundWitnessResult = default;
            _witnessPreviousR = _witnessGoal = _witnessBaselineR = default;
            _witnessPreviousW = _witnessNominalW = _witnessBaselineW = default; _witnessDt = 0f;
        }

        private void PlanGroundStep(SourceStepContext source, FootGroundTrajectory.Input input,
            Quaternion goal, Vector3 nominalW, bool recovery, ref Sample next)
        {
            var result = new FootGroundTrajectory.Result { Reason = FootGroundTrajectory.Status.Unavailable };
            if (!recovery && input.Matches(source))
                result = FootGroundTrajectory.Evaluate(input, _current.Rotation, _current.W,
                    goal, nominalW, next.Rotation, next.W, next.Dt);
            if (_groundSelected && result.Better)
            {
                Vector3 a = (result.W - _current.W) / next.Dt;
                if (!Q(result.R) || !V(result.W) || !V(a) ||
                    result.W.magnitude > Mathf.Max(MaxW, _current.W.magnitude) + .0001f || a.magnitude > MaxA + .002f)
                    throw new InvalidOperationException("FOOT_GROUND_TRAJECTORY_BUDGET_INVALID");
                result.Applied = true; next.Rotation = result.R; next.W = result.W; next.A = a;
                next.RawGap = Log(goal * Quaternion.Inverse(next.Rotation)).magnitude;
            }
            next.GroundTrajectory = result;
        }

        private void ChooseGroundStep(bool trace, SourceStepContext source, FootGroundTrajectory.Input input,
            Quaternion goal, Vector3 nominalW, bool recovery, ref Sample next)
        {
            Quaternion baselineR = next.Rotation; Vector3 baselineW = next.W;
            PlanGroundStep(source, input, goal, nominalW, recovery, ref next);
            var result = next.GroundTrajectory;
            if (!trace) return;
            _traceGroundSelected = _groundSelected; _groundSteps++; _groundReasons[(int)result.Reason]++;
            if (result.Evaluated)
            {
                _groundEvaluated++;
                if (result.CurrentKneeBasis) _groundCurrentKneeEvaluated++;
                if (result.BaselineMin < -KneeGroundTarget.MinGain) _groundBelow++;
                // One reproducible worst baseline with a clear INPUT endpoint. This
                // includes no-headroom cases; do not only retain successful changes.
                if (result.GoalMin >= 0f && result.BaselineMin < -KneeGroundTarget.MinGain &&
                    (!_hasGroundWitness || result.BaselineMin < _groundWitnessResult.BaselineMin))
                {
                    _hasGroundWitness = true; _groundWitness = input; _groundWitnessResult = result;
                    _witnessPreviousR = _current.Rotation; _witnessPreviousW = _current.W;
                    _witnessGoal = goal; _witnessNominalW = nominalW;
                    _witnessBaselineR = baselineR; _witnessBaselineW = baselineW; _witnessDt = next.Dt;
                }
            }
            if (result.Better)
            {
                _groundBetter++; _groundGainSum += result.CandidateMin - result.BaselineMin;
                _groundGainPeak = Mathf.Max(_groundGainPeak, result.CandidateMin - result.BaselineMin);
                _groundDeltaPeak = Mathf.Max(_groundDeltaPeak, result.DeltaDegrees);
            }
            if (result.Applied) { _groundApplied++; if (result.CurrentKneeBasis) _groundCurrentKneeApplied++; }
        }

        private string GroundSummary() => "\nFOOT_GROUND_TRAJECTORY_V041340/selected-traceSelected=" +
            _groundSelected + "|" + _traceGroundSelected + "/render-evaluated-below-better-applied=" +
            _groundSteps + "|" + _groundEvaluated + "|" + _groundBelow + "|" + _groundBetter + "|" + _groundApplied +
            "/reasons(unavailable-invalid-clear-endpointBelow-noProgress-noHeadroom-noLegal-improved)=" + string.Join("|", _groundReasons) +
            "/betterGainMm(mean-peak)-deltaDegPeak=" + (_groundBetter > 0 ? N((float)(_groundGainSum / _groundBetter) * 1000f) : "NA") +
            "|" + N(_groundGainPeak * 1000f) + "|" + N(_groundDeltaPeak) +
            "/kneeBasisEvaluated(ideal-current)=" + (_groundEvaluated - _groundCurrentKneeEvaluated) + "|" + _groundCurrentKneeEvaluated +
            "/kneeBasisApplied(ideal-current)=" + (_groundApplied - _groundCurrentKneeApplied) + "|" + _groundCurrentKneeApplied +
            "/domain=ONE_RENDER_REPORTED_KNEE_BASIS_PLUS_NATIVE_LOWER_NONREGRESSION_NOT_SOLVED" +
            "/bounds=ORIGINAL_W_A_BRAKING_FOOT_ROM_CORRECTION_UPRIGHT_FOOTPRINT/alphaSpeedLimited=BASELINE_REQUEST" + GroundWitnessText();

        private string GroundWitnessText()
        {
            if (!_hasGroundWitness) return "/footTrajectoryWitness=NONE";
            var p = _groundWitness; var g = p.Geometry;
            return "\nFOOT_TRAJECTORY_WITNESS_V041340/selection=DEEPEST_BASELINE_WITH_CLEAR_INPUT_ENDPOINT" +
                "/epoch-source-geometryFixed-support=" + p.Source.Epoch + "|" + p.Source.Sequence + "|" + p.GeometryStep.Sequence + "|" + p.SupportId +
                "/sourceTime-dt-planDt=" + N(p.Source.Time) + "|" + N(p.Source.Dt) + "|" + N(_witnessDt) +
                "/geometryClock(fixed-source-fixedTime-unscaledTime-dt)=" + p.GeometryStep.Sequence + "|" + p.GeometryStep.SourceSequence + "|" +
                N(p.GeometryStep.FixedTime) + "|" + N(p.GeometryStep.UnscaledTime) + "|" + N(p.GeometryStep.Dt) + "/owners=" + p.ParentId + "|" + p.UpperId + "|" + p.LowerId +
                "/" + _groundWitnessResult.Text() +
                "/hip=" + Vec(g.Hip) + "/upper=" + Quat(g.Upper) + "/upperSpan=" + Vec(g.UpperSpan) + "/lowerSpan=" + Vec(g.LowerSpan) +
                "/boxCenter-half=" + Vec(g.Center) + "|" + Vec(g.Half) + "/floorCenter-half=" + Vec(g.FloorCenter) + "|" + Vec(g.FloorHalf) +
                "/kneeBasis=" + (p.CurrentKneeBasis ? "COMPLETED_KNEE_REFERENCE" : "JOINT_INPUT_GOAL") + "/kneeGoal=" + Quat(p.KneeGoal) + "/nativeAnkle=" + Vec(p.NativeAnkle) + "/nativeLower=" + Quat(p.NativeLower) +
                "/footR(raw-bind-previous-input-baseline-candidate)=" + Quat(g.FootRelative) + "|" + Quat(g.FootBindRelative) + "|" +
                Quat(_witnessPreviousR) + "|" + Quat(_witnessGoal) + "|" + Quat(_witnessBaselineR) + "|" + Quat(_groundWitnessResult.R) +
                "/footW(previous-input-baseline-candidate)=" + Vec(_witnessPreviousW) + "|" + Vec(_witnessNominalW) + "|" +
                Vec(_witnessBaselineW) + "|" + Vec(_groundWitnessResult.W) + "/units=METERS_RADIANS_SECONDS/noNativeResponseProof=True";
        }
        private static string N(float x) => FootGroundTrajectory.N(x);
        private static string Vec(Vector3 v) => N(v.x) + "," + N(v.y) + "," + N(v.z);
        private static string Quat(Quaternion q) => N(q.x) + "," + N(q.y) + "," + N(q.z) + "," + N(q.w);
    }
}
