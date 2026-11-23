using System;
using System.Collections.Generic;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;
using System.Text;
using UnityEngine;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;
using Trial = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Issue at PRE_N only after both command continuations; settle at PRE_N+1.
    // The real solve supplies the nominal geometry. Only the offsets selected at N
    // may be replayed on N+1. These readbacks do not grant a contact opportunity.
    internal sealed partial class TurnHandoffWindow
    {
        private sealed class Row
        {
            internal int From, To, Source, NextSource;
            internal Trial.Plan Plan;
            internal LandingFootTraction.Pair Traction;
            internal CombinedLandingSearch.Step Combined;
            internal OrdinaryFootEntry.Sample Entry;
            internal Trial.Pose Nominal;
            internal Trial.Budget Budget;
            internal Trial.Aim IssuedAim, MeasuredAim;
            internal Trial.Measure Actual, Conditional;
            internal float Residual, Dt;
            internal bool Read, BaselineAgrees, Horizontal, Pass;
            internal bool BasePass => BaselineAgrees && Plan.Base.GeometricPass && Actual.GeometricPass;
        }

        internal enum Policy { RightStop, LeftReactive, LeftEarly, LeftBraked, LeftCombined }
        internal readonly int Start;
        private readonly Policy _policy;
        private readonly List<Row> _rows;
        private readonly int _limit;
        private PhysicsStepContext _visitedStep;
        private SourceStepContext _visitedSource;
        private readonly Vector3 _travel;
        private Trial.Pose _initial;
        private int _upper, _lower, _foot, _floor, _lastVisit;
        private float _hipY;
        private int _peakLocal;
        private string _peakMethod;
        private Trial.Measure _peakBase, _peakTrial;
        private bool _peakActive;
        internal string End { get; private set; }

        internal TurnHandoffWindow(int start, Vector3 travel, Policy policy = Policy.RightStop, bool usePreview = true)
        {
            Start = start; _travel = travel; _policy = policy;
            if (policy == Policy.LeftCombined) _combined = new CombinedLandingSearch(usePreview);
            _limit = policy == Policy.RightStop ? Trial.ComparisonDepth : LandingEventHistory.RingCapacity;
            _rows = new List<Row>(policy == Policy.RightStop ? Trial.ComparisonDepth : 16);
        }
        internal void Stop(string reason)
        {
            End ??= reason;
            _combined?.ReleaseScratch();
        }
        internal bool Pending(int local) => End == null && local >= Start && _lastVisit != local;
        internal bool Missed(int local) => End == null && local > Start && _lastVisit != local - 1;

        private Trial.Aim Aim(in Trace.Tick t) =>
            new Trial.Aim { Target = t.Target, TargetSole = t.SourceSole, Travel = _travel };

        // Read the settled endpoint before a contact selected FOR this solve can end
        // the unloaded window. The two clocks/feet/floor have already been paired.
        internal bool Visit(in Trace.Tick t, in Trial.Pose pose, int upper, int lower, string prohibitIssue)
        {
            if (End != null || _lastVisit == t.Local) return false;
            if (t.Local != Start + _rows.Count || (_lastVisit != 0 && _lastVisit + 1 != t.Local))
            { Stop("MissingCompletion"); return false; }
            if (!t.Step.Valid || !t.Source.Clock.Valid || t.Source.Clock.Epoch != t.Step.Epoch ||
                t.Source.Clock.Sequence != t.Step.SourceSequence || (_visitedStep.Valid &&
                (t.Step.Epoch != _visitedStep.Epoch || t.Step.Sequence != _visitedStep.Sequence + 1)))
            { Stop("ClockMismatch"); return false; }
            _lastVisit = t.Local; _visitedStep = t.Step; _visitedSource = t.Source.Clock;
            var aim = Aim(t);
            if (!t.Full || !aim.Valid || !pose.Valid || t.Body == 0 || t.Floor == 0 || upper == 0 || lower == 0 || upper == lower || upper == t.Body || lower == t.Body)
            { Stop("MissingGeometry"); return false; }
            if (_rows.Count == 0)
            {
                _upper = upper; _lower = lower; _foot = t.Body; _floor = t.Floor; _initial = pose; _hipY = pose.Hip.y;
            }
            else if (_upper != upper || _lower != lower || _foot != t.Body || _floor != t.Floor || !_initial.SameBox(pose))
            { Stop("BodyOrBoxChanged"); return false; }

            var actual = Trial.Evaluate(pose, default, aim);
            if (!MatchesTick(pose, actual, t))
            { Stop("PoseMismatch"); return false; }
            if (_rows.Count > 0)
            {
                var r = _rows[_rows.Count - 1];
                r.MeasuredAim = aim; r.Actual = actual;
                r.To = t.Local; r.NextSource = t.Source.Clock.Sequence; r.Read = true;
                r.Residual = NominalError(r.Nominal, pose, r.Plan.Base, actual);
                r.BaselineAgrees = r.Residual <= FootBoxGeometry.SupportPlaneToleranceMeters;
                if (_combined != null)
                {
                    using var cost = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.CombinedLanding);
                    r.Entry = t.Entry;
                    r.Combined.PhysicalPose = pose;
                    r.Combined.HasPhysicalPose = true;
                    if (!r.BaselineAgrees) { Stop("BaselineMismatch"); return false; }
                    string invalid = CheckNativeEntry(t, actual.Inside, prohibitIssue);
                    if (invalid != null) { Stop(invalid); return false; }
                    r.Combined.VerifiedReadClock = t.Step;
                    _combined.Read(t.Local, r.Combined, pose, aim, actual, t.Entry, t.Miss || t.Ordinary);
                }
                else if (!ReadSelected(r, t, pose, aim, prohibitIssue)) return false;
            }
            // The first native decision is FOR this new solve. Its incoming free endpoint
            // is already settled above, even when the collider has just been enabled.
            if (_policy != Policy.RightStop && (t.Miss || t.Ordinary || t.Recovery))
            {
                if (prohibitIssue != null && prohibitIssue != "LandingResolved")
                { Stop(prohibitIssue); return false; }
                bool first = t.EventFloor == t.Floor && ((t.Miss && !t.Active) || (t.Ordinary && t.Active));
                Stop(first ? "FirstOpportunity" : "OtherEntry"); return false;
            }
            if (_rows.Count == _limit)
            { Stop(_policy == Policy.RightStop ? "ThreeReadbacks" : "Capacity"); return false; }
            if (prohibitIssue != null) { Stop(prohibitIssue); return false; }
            if (t.Active || t.Precontact || t.Miss || !actual.Safe)
            { Stop("ContactOrBounds"); return false; }
            return true;
        }

        private bool ReadSelected(Row r, in Trace.Tick t, in Trial.Pose pose, in Trial.Aim aim, string prohibitIssue)
        {
            r.Conditional = Trial.Evaluate(pose, r.Plan.Offset, aim);
            r.Horizontal = r.Conditional.Tracks(r.Actual);
            r.Pass = r.BaselineAgrees && r.Plan.Trial.GeometricPass && r.Conditional.GeometricPass && r.Horizontal &&
                r.Conditional.MaintainsHeight(r.Actual);
            if (!r.BaselineAgrees) { Stop("BaselineMismatch"); return false; }
            if (!r.Conditional.Safe || !r.Horizontal || !r.Conditional.MaintainsHeight(r.Actual))
            { Stop("ReadbackGeometry"); return false; }
            if (r.Traction != null)
            {
                LandingFootTraction.SettleOnLeg(ref r.Traction.One, pose, r.Conditional, aim, r.Actual, r.Traction.Command.Tether.HardGap);
                LandingFootTraction.SettleOnLeg(ref r.Traction.Two, pose, r.Conditional, aim, r.Actual, r.Traction.Command.Tether.HardGap);
                LandingFootTraction.SettleOnLeg(ref r.Traction.BufferedOne, pose, r.Conditional, aim, r.Actual, r.Traction.Command.Tether.HardGap);
                LandingFootTraction.SettleOnLeg(ref r.Traction.BufferedTwo, pose, r.Conditional, aim, r.Actual, r.Traction.Command.Tether.HardGap);
                ReadContinuous(r, t, pose, aim, prohibitIssue);
            }
            return true;
        }

        private static bool MatchesTick(in Trial.Pose pose, in Trial.Measure actual, in Trace.Tick t) =>
            Mathf.Abs(actual.Sole - t.BoxMin) <= FootBoxGeometry.SupportPlaneToleranceMeters &&
            (pose.Foot - t.Foot).magnitude <= FootBoxGeometry.SupportPlaneToleranceMeters &&
            (pose.Lower - t.Lower).magnitude <= FootBoxGeometry.SupportPlaneToleranceMeters;

        // After solve three, the last selected angular offsets are applied to the
        // measured peak pose only. This sensitivity check extrapolates no motion.
        internal void ReadPeak(in Trace.Tick t, in Trial.Pose pose, int upper, int lower)
        {
            if (_policy != Policy.RightStop || t.Local < Start || _foot != t.Body || _floor != t.Floor || _peakLocal == t.Local || !t.Full || upper != _upper || lower != _lower || !_initial.SameBox(pose) || !pose.Valid) return;
            var aim = Aim(t);
            if (!aim.Valid) return;
            var observed = Trial.Evaluate(pose, default, aim);
            if (!MatchesTick(pose, observed, t)) return;
            Trial.Offset offset = default;
            string method = "AtOrigin";
            if (t.Local != Start)
            {
                int index = Math.Min(t.Local - Start, Trial.ComparisonDepth) - 1;
                if (index < 0 || index >= _rows.Count) return;
                var r = _rows[index];
                if (!r.Read || !r.BaselineAgrees || !r.Horizontal || !r.Conditional.Safe ||
                    (r.To != t.Local && End != "ThreeReadbacks")) return;
                offset = r.Plan.Offset; method = r.To == t.Local ? "PairedStep" : "HeldThirdPose";
            }
            _peakLocal = t.Local; _peakMethod = method; _peakActive = t.Active || t.Precontact;
            _peakBase = observed; _peakTrial = Trial.Evaluate(pose, offset, aim);
        }

        private static float NominalError(in Trial.Pose p, in Trial.Pose q, in Trial.Measure baseline, in Trial.Measure actual)
        {
            // Material basis errors are measured in metres too: an erroneous rotation
            // around the ankle can preserve both ankle and minimum, yet move the sole.
            float soleAngle = FootRelativeReference.Log(p.Rotation * Quaternion.Inverse(q.Rotation)).magnitude;
            float shoe = (q.Center.magnitude + q.Half.magnitude) * soleAngle;
            float axisError = (p.Axis - q.Axis).magnitude * (q.Lower - q.LowerKnee).magnitude;
            float error = Mathf.Max(shoe, axisError);
            error = Mathf.Max(error, Mathf.Abs(baseline.Sole - actual.Sole));
            error = Mathf.Max(error, (p.Foot - q.Foot).magnitude);
            error = Mathf.Max(error, (p.Lower - q.Lower).magnitude);
            error = Mathf.Max(error, (p.UpperKnee - q.UpperKnee).magnitude);
            error = Mathf.Max(error, (p.LowerKnee - q.LowerKnee).magnitude);
            return Mathf.Max(error, (p.Hip - q.Hip).magnitude);
        }

        internal void Issue(in Trace.Tick t, in Trial.Pose nominal, in Trial.Budget budget, in LandingFootTraction.Input traction = default)
        {
            if (End != null || t.Local != _lastVisit || t.Local != Start + _rows.Count || _rows.Count >= _limit) return;
            if (!_visitedStep.Matches(t.Step) || !_visitedSource.Matches(t.Source.Clock) || t.Body != _foot || t.Floor != _floor)
            { Stop("IssueIdentity"); return; }
            if (nominal.Valid && !_initial.SameBox(nominal)) { Stop("ForecastBoxChanged"); return; }
            var previous = _rows.Count == 0 ? default : _rows[_rows.Count - 1].Plan.Offset;
            var r = new Row { From = t.Local, Source = t.Source.Clock.Sequence, Dt = t.Step.Dt, Nominal = nominal, Budget = budget, IssuedAim = Aim(t) };
            if (_combined != null)
            {
                using var cost = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.CombinedLanding);
                if (!nominal.Valid || !budget.Valid) { Stop("MissingForecast"); return; }
                r.Plan.Base = Trial.Evaluate(nominal, default, r.IssuedAim);
                var last = _rows.Count == 0 ? null : _rows[_rows.Count - 1].Combined;
                r.Combined = _combined.Issue(t.Local, r.Dt, _hipY, nominal, r.IssuedAim, budget, traction, t.Foot - t.Lower, last);
                r.Combined.IssueClock = t.Step;
                _rows.Add(r);
                return;
            }
            bool reserve = _policy == Policy.LeftBraked;
            r.Plan = Trial.Select(nominal, r.IssuedAim, budget, t.Step.Dt, _hipY, previous, _policy == Policy.LeftEarly || reserve, reserve);
            if (reserve && r.Plan.Issued)
            {
                var prior = _rows.Count > 0 ? _rows[_rows.Count - 1].Traction : null;
                var pair = r.Traction = new LandingFootTraction.Pair { Command = traction };
                pair.One = LandingFootTraction.Predict(nominal, r.Plan, r.IssuedAim, traction, r.Dt, t.Foot - t.Lower, default, false);
                pair.Two = LandingFootTraction.Predict(nominal, r.Plan, r.IssuedAim, traction, r.Dt, t.Foot - t.Lower,
                    prior?.One ?? default, true);
                pair.BufferedOne = LandingFootTraction.Predict(nominal, r.Plan, r.IssuedAim, traction, r.Dt, t.Foot - t.Lower,
                    default, false, true);
                pair.BufferedTwo = LandingFootTraction.Predict(nominal, r.Plan, r.IssuedAim, traction, r.Dt, t.Foot - t.Lower,
                    prior?.BufferedOne ?? default, true, true);
                IssueContinuous(r, t, prior);
            }
            _rows.Add(r);
            if (!r.Plan.Issued) Stop(r.Plan.Result.ToString());
        }

        internal struct Count
        {
            internal int Issued, Read, Calibrated, OriginalPass, TrialPass, NewPass, LostPass, Improved;
        }

        internal void Accumulate(Count[] totals)
        {
            for (int depth = 0; depth < _rows.Count && depth < totals.Length; depth++)
            {
                var r = _rows[depth];
                ref var c = ref totals[depth];
                if (r.Plan.Issued) c.Issued++;
                if (!r.Read) continue;
                c.Read++;
                if (!r.BaselineAgrees) continue;
                c.Calibrated++;
                bool original = r.BasePass;
                if (original) c.OriginalPass++;
                if (r.Pass) c.TrialPass++;
                if (r.Pass && !original) c.NewPass++;
                if (!r.Pass && original) c.LostPass++;
                if (r.Horizontal && r.Conditional.Safe && r.Conditional.LowerRise >= Trial.Precision &&
                    r.Conditional.Sole - r.Actual.Sole >= Trial.Precision) c.Improved++;
            }
        }

        // A false geometric result and an unavailable trajectory have different denominators.
        internal bool TryReadPass(int local, out bool pass)
        {
            pass = false;
            int index = local - Start - 1;
            if (index < 0 || index >= _rows.Count) return false;
            var r = _rows[index];
            if (!r.Read || !r.BaselineAgrees || r.To != local) return false;
            pass = r.Pass; return true;
        }

        private bool HasReadAim(int from)
        {
            int index = from - Start;
            return index >= 0 && index < _rows.Count && _rows[index].Read;
        }

        private bool HasInputs(int from)
        {
            int index = from - Start;
            return index >= 0 && index < _rows.Count && _rows[index].From == from &&
                _rows[index].Nominal.Valid && _rows[index].Budget.Valid;
        }

        private static string N(float f) => f.ToString("0.#####", CultureInfo.InvariantCulture);
        private static string Mm(float value) => N(value * 1000f);
        private static string Geometry(in Trial.Measure m) =>
            Mm(m.Sole) + "|" + Mm(m.Gap) + "|" + Mm(m.KneeGap) + "|" + Mm(m.LowerRise);
        private static string Planar(in Trial.Measure m) =>
            Mm(m.AnkleAlong) + "|" + Mm(m.SoleAlong) + "|" + Mm(m.AnkleXZ) + "|" + Mm(m.SoleXZ);

        private static string Exact(float f) => f.ToString("R", CultureInfo.InvariantCulture);
        private static string V(Vector3 v) => v.x.ToString("R", CultureInfo.InvariantCulture) + "," +
            v.y.ToString("R", CultureInfo.InvariantCulture) + "," + v.z.ToString("R", CultureInfo.InvariantCulture);
        private static void AppendForecastInputs(StringBuilder b, Row row)
        {
            var p = row.Nominal; var a = row.Budget; var aim = row.IssuedAim;
            b.Append("/nominalM(hip-upperKnee-lowerKnee-lowerAnkle-foot)=").Append(V(p.Hip)).Append('|').Append(V(p.UpperKnee))
                .Append('|').Append(V(p.LowerKnee)).Append('|').Append(V(p.Lower)).Append('|').Append(V(p.Foot))
                .Append("/footR=").Append(V(new Vector3(p.Rotation.x, p.Rotation.y, p.Rotation.z))).Append(',').Append(p.Rotation.w.ToString("R", CultureInfo.InvariantCulture))
                .Append("/boxM(center-half-floorCenter-half)=").Append(V(p.Center)).Append('|').Append(V(p.Half)).Append('|').Append(V(p.FloorCenter)).Append('|').Append(V(p.FloorHalf))
                .Append("/aimM(ankle-sole)=").Append(V(aim.Target)).Append('|').Append(V(aim.TargetSole))
                .Append("/allowExtension=").Append(p.AllowExtension).Append("/axis(issue-endpoint)=").Append(V(a.Axis)).Append('|').Append(V(p.Axis))
                .Append("/baseHipA-kneeA-hipCap-kneeCap=").Append(V(a.HipA)).Append('|').Append(Exact(a.KneeA)).Append('|').Append(Exact(a.HipCap)).Append('|').Append(Exact(a.KneeCap))
                .Append("/baseW(upper-lower-foot)=").Append(V(a.UpperW)).Append('|').Append(V(a.LowerW)).Append('|').Append(V(a.FootW))
                .Append("/speedCaps=").Append(Exact(a.UpperMaxW)).Append('|').Append(Exact(a.LowerMaxW)).Append('|').Append(Exact(a.FootMaxW));
        }

        internal void Append(StringBuilder b, int side, int origin, string landing, int first, int resolved, string control, int peak, float peakLead, TurnHandoffWindow sharedInputs = null, TurnHandoffWindow otherInputs = null)
        {
            b.Append(_policy == Policy.RightStop ? "\nTURN_HANDOFF_V041690/side=" : _policy == Policy.LeftBraked ? "\nTURN_LANDING_V041710/side=" : "\nTURN_LANDING_V041700/side=").Append(side == 0 ? "LEFT" : "RIGHT")
                .Append("/kind=").Append(side == 0 ? "IdleTurnPair" : "QuietRightWithLeftSupport")
                .Append("/parentOrigin-start=").Append(origin).Append('|').Append(Start)
                .Append("/first-outcome-resolved=").Append(first).Append('|').Append(landing).Append('|').Append(resolved)
                .Append("/rightStopLead-peakAt-mm=").Append(control ?? "NA").Append('|').Append(peak).Append('|').Append(peak > 0 ? Mm(peakLead) : "NA")
                .Append("/originHipY=").Append(_upper != 0 ? _hipY.ToString("R", CultureInfo.InvariantCulture) : "NA").Append("/axis=").Append(V(_travel))
                .Append("/end=").Append(End ?? "Open").Append("/model=CONDITIONED_POSE/plantWrites=0");
            if (_policy != Policy.RightStop) b.Append("/policy=").Append(_policy);
            if (side == 1)
            {
                bool matched = peak > 0 && peak == _peakLocal && Mathf.Abs(_peakBase.AnkleAlong - peakLead) <= FootBoxGeometry.SupportPlaneToleranceMeters;
                b.Append("/peakComparison=").Append(matched ? _peakMethod : "Unavailable");
                if (matched) b.Append("/peakUnderContact=").Append(_peakActive).Append("/peakPlanar=").Append(Planar(_peakBase))
                    .Append("/peakConditionedPlanar=").Append(Planar(_peakTrial)).Append("/peakBounds-tracking=")
                    .Append(_peakTrial.Safe).Append('|').Append(_peakTrial.Tracks(_peakBase));
            }
            for (int i = 0; i < _rows.Count; i++)
            {
                var r = _rows[i]; var p = r.Plan;
                b.Append(_policy == Policy.RightStop ? "\nTURN_HANDOFF_STEP_V041690/side=" : _policy == Policy.LeftBraked ? "\nTURN_LANDING_STEP_V041710/side=" : "\nTURN_LANDING_STEP_V041700/side=").Append(side == 0 ? "LEFT" : "RIGHT").Append("/start=").Append(Start)
                    .Append("/depth-from-to=").Append(i + 1).Append('|').Append(r.From).Append('|').Append(r.To)
                    .Append("/dt=").Append(r.Dt.ToString("R", CultureInfo.InvariantCulture)).Append("/source-next=").Append(r.Source).Append('|').Append(r.NextSource)
                    .Append("/choice=").Append(p.Result).Append("/examined-safe-tracks=").Append(p.Tested).Append('|').Append(p.Safe).Append('|').Append(p.Tracking)
                    .Append("/hipDrop-wantedMm=").Append(Mm(p.HipDrop)).Append('|').Append(Mm(p.Wanted));
                if (_policy != Policy.RightStop) b.Append("/policy=").Append(_policy);
                var owner = sharedInputs?.HasInputs(r.From) == true ? sharedInputs : otherInputs?.HasInputs(r.From) == true ? otherInputs : null;
                if (owner != null) b.Append("/inputs=").Append(owner._policy);
                else if (r.Nominal.Valid && r.Budget.Valid) AppendForecastInputs(b, r);
                if (_policy != Policy.RightStop && r.Read && sharedInputs?.HasReadAim(r.From) != true && otherInputs?.HasReadAim(r.From) != true)
                    b.Append("/endpointAimM(ankle-sole)=").Append(V(r.MeasuredAim.Target)).Append('|').Append(V(r.MeasuredAim.TargetSole));
                if (!p.Issued) continue;
                b.Append("/hip-kneeDeg=").Append(N(p.Offset.Hip * Mathf.Rad2Deg)).Append('|').Append(N(p.Offset.Knee * Mathf.Rad2Deg))
                    .Append("/hip-kneeRateRadSec=").Append(N(p.Offset.HipRate)).Append('|').Append(N(p.Offset.KneeRate))
                    .Append("/hip-kneeAddedRadSec2=").Append(N(p.HipDeltaA)).Append('|').Append(N(p.KneeDeltaA))
                    .Append("/forecastBase-min-ankleGap-kneeGap-lowerRiseMm=").Append(Geometry(p.Base))
                    .Append("/forecastTrial=").Append(Geometry(p.Trial)).Append("/forecastAlongAnkle-sole-xzAnkle-soleMm=").Append(Planar(p.Base))
                    .Append("/forecastTrialPlanar=").Append(Planar(p.Trial)).Append("/read=").Append(r.Read);
                if (r.Read) b.Append("/maxNominalErrorMm=").Append(Mm(r.Residual)).Append("/measured=").Append(Geometry(r.Actual))
                    .Append("/conditioned=").Append(Geometry(r.Conditional)).Append("/measuredPlanar=").Append(Planar(r.Actual))
                    .Append("/conditionedPlanar=").Append(Planar(r.Conditional))
                    .Append("/nominalAgrees-horizontal-basePass-trialPass=").Append(r.BaselineAgrees).Append('|')
                    .Append(r.Horizontal).Append('|').Append(r.BasePass).Append('|').Append(r.Pass);
            }
        }
    }
}
