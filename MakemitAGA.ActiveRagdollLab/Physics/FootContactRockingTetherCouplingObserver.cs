using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// V04280 diagnostic-only world-vector accounting. No actuator or safety decision consumes this observer.
    /// Tether components are algebraic shares of one COM command, not independent physical interventions.
    /// The ring includes finite gate candidates even when the production tether never writes a command.
    /// Every captured dynamic write joins its next settled solve; only loaded joins qualify as loaded evidence.
    /// Command normal is immutable. A geometric normal at capture cannot supply a missing command basis.
    /// Full COM residual includes drag, other contacts and solver coupling; it is not a measured contact force.
    /// </summary>
    internal sealed class FootContactRockingTetherCouplingObserver
    {
        internal const string Mode = "READ_ONLY_ROCKING_TETHER_PLANAR_COM_COUPLING_V04280";
        internal const int MinimumResolvedSolves = 50;
        private const int Capacity = 12;
        private const float ClosureTolerance = 0.001f;

        internal const string NormalLoadMode = "READ_ONLY_DYNAMIC_NORMAL_LOAD_PATH_AUDIT_V04330";
        internal const string DragModel = "ORDINARY_PRE_SOLVER_LINEAR_DAMPING_ASSUMPTION_V04350_NOT_NATIVE_FORCE";
        internal const string InputRevision = Plugin.SourceRevision;

        // Normal impulses use the immutable pre-solve support normal and actual command units.
        // Knee COM tracking remains external to this Lower+Foot subsystem; V04430 writes only to Upper.
        // Endpoint residuals include drag/other contacts/solver effects; callback J is not a complete GRF.
        internal struct NormalLoad
        {
            internal bool Captured, Resolved, Loaded, NormalValid, DragModelValid, LowerGravityScheduled, ReciprocalApplied, SelfApplied, PlanarApplied, HipLinearApplied;
            internal int PreFixed, SettledFixed, HipLinearFixed;
            internal bool KneeReactionInputValid, KneeReactionUpperApplied;
            internal int KneeReactionFixed;
            internal Vector3 KneeReactionUpperWorld;
            internal bool UpperLoadApplied;
            internal int UpperLoadFixed;
            internal Vector3 UpperLoadLowerKneeWorld, UpperLoadLowerAnkleWorld, UpperLoadFootWorld;
            internal string SourcePhase;
            internal float LowerMass, FootMass, Dt, LowerDrag, FootDrag;
            internal string DragModelIssue;
            internal Vector3 KneeWorld, HipLinearWorld, PlanarWorld, ReciprocalWorld, SelfWorld, LowerGravityWorld;
            internal Vector3 FootTetherWorld, FootSelfWorld, FootGravityWorld, LowerDragWorld, FootDragWorld;
            internal Vector3 PreNormal, PreLowerV, SettledLowerV, PreFootV, SettledFootV;
            // World-vector copies let V04340 reproject a whole window onto one fixed reference normal.
            internal Vector3 LowerKnownWorld, FootCommandWorld, GroundWorld, LowerDeltaPWorld, FootDeltaPWorld;
            internal float KneeJ, HipLinearJ, PlanarJ, ReciprocalJ, SelfJ, LowerGravityJ;
            internal float FootTetherJ, FootSelfJ, FootGravityJ, GroundJ;
            internal float LowerDeltaP, FootDeltaP, LowerResidual, FootResidual, PairResidual, InternalClosure;
        }

        private struct NormalStat
        {
            internal int Count;
            internal double Sum;
            internal float Min, Max;
            internal void Add(float value)
            {
                if (Count == 0) Min = Max = value;
                else { Min = Mathf.Min(Min, value); Max = Mathf.Max(Max, value); }
                Count++; Sum += value;
            }
            internal string Format() => Count == 0 ? "NA" : S((float)(Sum / Count)) + "," + S(Min) + "," + S(Max);
        }

        private struct NormalTotals
        {
            internal int Count, Fresh, Held, NoGround, DynamicResponses;
            internal NormalStat Inertia, Knee, HipLinear, Ankle, UpperKnee, UpperAnkle, UpperFoot, Ground, LowerResidual, FootResidual, PairResidual, InternalClosure;
            internal void Add(Frame frame)
            {
                NormalLoad d = frame.Normal;
                Count++; if (frame.Fresh) Fresh++; else Held++;
                if (frame.ProjectedCallbacks == 0) NoGround++;
                // Response exposure only, never a source-matching key or controller decision.
                if (Mathf.Abs(d.LowerDeltaP / d.LowerMass) >= 0.02f) DynamicResponses++;
                Inertia.Add(d.LowerDeltaP); Knee.Add(d.KneeJ); Ankle.Add(d.ReciprocalJ + d.SelfJ);
                HipLinear.Add(d.HipLinearJ);
                UpperKnee.Add(Vector3.Dot(d.UpperLoadLowerKneeWorld, d.PreNormal));
                UpperAnkle.Add(Vector3.Dot(d.UpperLoadLowerAnkleWorld, d.PreNormal));
                UpperFoot.Add(Vector3.Dot(d.UpperLoadFootWorld, d.PreNormal));
                Ground.Add(d.GroundJ); LowerResidual.Add(d.LowerResidual); FootResidual.Add(d.FootResidual);
                PairResidual.Add(d.PairResidual); InternalClosure.Add(d.InternalClosure);
            }
            internal string Format() => "n-fresh-held-noGround-dynamicResponse=" + Count + "-" + Fresh + "-" + Held + "-" + NoGround + "-" + DynamicResponses +
                "/Ns(mean,min,max):lowerInertia=" + Inertia.Format() + "/externalKnee=" + Knee.Format() +
                "/externalHipLinear=" + HipLinear.Format() +
                "/upperLoad(lowerKnee-lowerAnkle-foot)=" + UpperKnee.Format() + "|" + UpperAnkle.Format() + "|" + UpperFoot.Format() +
                "/internalAnkle=" + Ankle.Format() + "/callbackGround=" + Ground.Format() +
                "/lowerResidual=" + LowerResidual.Format() + "/footResidual=" + FootResidual.Format() +
                "/pairResidual=" + PairResidual.Format() + "/internalPairClosure=" + InternalClosure.Format();
        }

        private struct Frame
        {
            internal int Fixed, Render, Held, Settled, Callbacks, ProjectedCallbacks, ExitCallbacks;
            internal bool Fresh, Written, Clamped, NormalValid, Loaded, PreCaptured, Resolved, Censored, PlanarIsolation;
            internal bool NativeGravity, GravityScheduled, LoadScheduled, LoadNormalValid;
            internal float Dt, Age, Mass, Drag, Scale, Cap, VelocityClosure, CommandClosure, PointClosure, PositionClosure, RawNormalDampingDelta;
            internal Vector3 N, E, VComError, VRotError, P, DCom, DRot, Transport, Raw, Applied;
            internal Vector3 BaselineRaw, RemovedFootRotD, EffectiveDRot, EffectiveD, EffectiveVelocity;
            internal Vector3 PreFootCom, PreLowerCom, PreFootV, PreLowerV, PreFootW, PreLowerW, PreFootR, PreLowerR, PreFootPointV;
            internal Vector3 LoadN, GravityA, LoadA, UpperLoadA, GroundJ, PostE, DeltaW, DeltaComV, DeltaRotV, DeltaPointV;
            internal Vector3 DeltaECom, DeltaELever, PredictedComDeltaV, ComResidual;
            internal NormalLoad Normal;
        }

        private readonly Frame[] _ring = new Frame[Capacity];
        private int _next, _count, _pending = -1, _evaluations, _written, _resolved, _responseResolved, _censored, _tails, _exitCallbacks;
        private bool _finite = true;
        private string _invalidReason;
        private float _velocityClosureMax, _commandClosureMax, _pointClosureMax, _positionClosureMax;
        private bool _normalFinite = true;
        private string _normalInvalidReason;
        private int _normalCaptured, _normalCensored, _normalSkipped, _normalUnknownPhase;
        private NormalTotals _normalAll, _normalIdle, _normalMotion;
        private Frame _normalPeakLowerResidual, _normalPeakKnee;
        private bool _normalHasPeak;
        private bool _transitionFinite = true;
        private string _transitionInvalidReason;
        private int _transitionCaptured, _transitionResolved, _transitionCensored, _transitionSkipped, _dragModelValid, _dragModelMissing;
        private bool NormalLoadReady => _normalFinite && _normalAll.Count >= MinimumResolvedSolves &&
            _normalIdle.Count >= 30 && _normalMotion.Count >= 30;

        private string NormalLoadReadinessReason => !_normalFinite ? (_normalInvalidReason ?? "NONFINITE") :
            (_normalAll.Count < MinimumResolvedSolves ? "TOTAL_LOADED_SAMPLES_BELOW_50" :
            (_normalIdle.Count < 30 ? "IDLE_LOADED_SAMPLES_BELOW_30" :
            (_normalMotion.Count < 30 ? "NON_IDLE_LOADED_SAMPLES_BELOW_30" : "READY")));

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            Array.Clear(_ring, 0, _ring.Length);
            _next = _count = _evaluations = _written = _resolved = _responseResolved = _censored = _tails = _exitCallbacks = 0;
            _pending = -1;
            _finite = true;
            _invalidReason = null;
            _velocityClosureMax = _commandClosureMax = _pointClosureMax = _positionClosureMax = 0f;
            _normalFinite = true; _normalInvalidReason = null;
            _normalCaptured = _normalCensored = _normalSkipped = _normalUnknownPhase = 0;
            _normalAll = _normalIdle = _normalMotion = default;
            _normalPeakLowerResidual = _normalPeakKnee = default; _normalHasPeak = false;
            _transitionFinite = true; _transitionInvalidReason = null;
            _transitionCaptured = _transitionResolved = _transitionCensored = _transitionSkipped = _dragModelValid = _dragModelMissing = 0;
            CopyToReport(report);
        }

        internal void RecordEvaluation(bool trace, bool dynamic, int fixedSequence, float dt,
            Rigidbody foot, Rigidbody lower, Vector3 footLever, Vector3 lowerLever,
            Vector3 positionError, Vector3 velocityError, Vector3 transport, bool normalValid, Vector3 normal,
            Vector3 effectiveVelocityError, bool planarIsolation, FootAnkleFoundationReport report)
        {
            if (!trace || !dynamic) return;
            try
            {
                int last = Find(fixedSequence);
                if (last >= 0) return;
                if (_pending == _next) ClearPendingPreserveTrace(report);
                Frame f = new Frame
                {
                    Fixed = fixedSequence, Dt = dt, Render = report.TargetRenderSampleSequence,
                    Fresh = report.TargetRenderSampleFreshAtFixedStep, Held = report.FixedStepsSinceTargetRenderSample,
                    Age = report.TargetRenderSampleAgeSeconds, E = positionError,
                    PreFootCom = foot.worldCenterOfMass, PreLowerCom = lower.worldCenterOfMass,
                    PreFootV = foot.velocity, PreLowerV = lower.velocity,
                    PreFootW = foot.angularVelocity, PreLowerW = lower.angularVelocity,
                    PreFootR = footLever, PreLowerR = lowerLever,
                    P = positionError * FootAnkleConstraintTopologyIsolation.TetherPositionSpring,
                    Transport = transport, NormalValid = normalValid && V(normal) && normal.sqrMagnitude > 0.00000001f,
                    N = normalValid && V(normal) && normal.sqrMagnitude > 0.00000001f ? normal.normalized : Vector3.zero
                };
                f.VComError = f.PreLowerV - f.PreFootV;
                f.VRotError = Vector3.Cross(f.PreLowerW, f.PreLowerR) - Vector3.Cross(f.PreFootW, f.PreFootR);
                f.DCom = f.VComError * FootAnkleConstraintTopologyIsolation.TetherVelocityDamper;
                f.DRot = f.VRotError * FootAnkleConstraintTopologyIsolation.TetherVelocityDamper;
                // Baseline is same-state command algebra only, not a simulated A trajectory or A clamp.
                f.BaselineRaw = f.P + velocityError * FootAnkleConstraintTopologyIsolation.TetherVelocityDamper + f.Transport;
                f.PlanarIsolation = planarIsolation;
                f.EffectiveVelocity = effectiveVelocityError;
                f.RemovedFootRotD = planarIsolation && f.NormalValid ?
                    -Planar(Vector3.Cross(f.PreFootW, f.PreFootR), f.N) * FootAnkleConstraintTopologyIsolation.TetherVelocityDamper : Vector3.zero;
                f.EffectiveDRot = f.DRot - f.RemovedFootRotD;
                f.EffectiveD = effectiveVelocityError * FootAnkleConstraintTopologyIsolation.TetherVelocityDamper;
                f.Raw = f.P + f.EffectiveD + f.Transport; // effective candidate until actual write
                f.RawNormalDampingDelta = f.NormalValid ? Vector3.Dot(f.EffectiveD - velocityError * FootAnkleConstraintTopologyIsolation.TetherVelocityDamper, f.N) : 0f;
                f.CommandClosure = Mathf.Max((f.EffectiveD - f.DCom - f.EffectiveDRot).magnitude,
                    (f.Raw - f.BaselineRaw + f.RemovedFootRotD).magnitude);
                f.VelocityClosure = (velocityError - f.VComError - f.VRotError).magnitude;
                Require(V(f.E) && V(f.Raw) && V(f.BaselineRaw) && V(f.EffectiveVelocity) && V(f.RemovedFootRotD) &&
                    V(f.EffectiveDRot) && V(f.EffectiveD) && F(f.RawNormalDampingDelta) && F(f.CommandClosure) && V(f.VComError) && V(f.VRotError) &&
                    V(f.PreFootCom) && V(f.PreLowerCom) && V(f.PreFootV) && V(f.PreLowerV) &&
                    V(f.PreFootW) && V(f.PreLowerW) && V(f.PreFootR) && V(f.PreLowerR) && F(dt) && dt > 0f &&
                    F(f.Age) && F(f.VelocityClosure), "NON_FINITE_EVALUATION");
                _velocityClosureMax = Mathf.Max(_velocityClosureMax, f.VelocityClosure);
                Require(f.VelocityClosure <= ClosureTolerance, "POINT_VELOCITY_DECOMPOSITION_FAILED");
                Vector3 expectedVelocity = velocityError - f.RemovedFootRotD / FootAnkleConstraintTopologyIsolation.TetherVelocityDamper;
                Require((effectiveVelocityError - expectedVelocity).magnitude <= ClosureTolerance &&
                    f.CommandClosure <= ClosureTolerance + f.Raw.magnitude * 0.0001f, "EFFECTIVE_DAMPING_COMPONENT_CLOSURE_FAILED");
                _ring[_next] = f;
                _next = (_next + 1) % Capacity;
                _count = Math.Min(Capacity, _count + 1);
                _evaluations++;
            }
            catch (Exception e) { Invalid("EVALUATION_EXCEPTION:" + e.GetType().Name); }
            CopyToReport(report, false);
        }

        internal void RecordWrittenCommand(bool trace, int fixedSequence, Vector3 raw, Vector3 applied,
            float cap, bool clamped, FootAnkleFoundationReport report)
        {
            if (!trace) return;
            try
            {
                int i = Find(fixedSequence);
                if (i < 0) { Invalid("WRITTEN_COMMAND_WITHOUT_EVALUATION"); return; }
                Frame f = _ring[i];
                if (f.Written) return;
                f.CommandClosure = Mathf.Max(f.CommandClosure, (raw - f.Raw).magnitude);
                f.Scale = raw.magnitude > 0.0000001f ? applied.magnitude / raw.magnitude : 1f;
                f.Raw = raw; f.Applied = applied; f.Cap = cap; f.Clamped = clamped; f.Written = true;
                Vector3 parts = (f.P + f.DCom + f.EffectiveDRot + f.Transport) * f.Scale;
                f.CommandClosure = Mathf.Max(f.CommandClosure, (parts - applied).magnitude);
                Require(V(raw) && V(applied) && F(f.Scale) && F(cap) && F(f.CommandClosure), "NON_FINITE_COMMAND");
                _commandClosureMax = Mathf.Max(_commandClosureMax, f.CommandClosure);
                Require(f.CommandClosure <= ClosureTolerance + raw.magnitude * 0.0001f, "COMMAND_COMPONENT_CLOSURE_FAILED");
                _ring[i] = f; _written++;
            }
            catch (Exception e) { Invalid("COMMAND_EXCEPTION:" + e.GetType().Name); }
            CopyToReport(report, false);
        }

        internal void CapturePreSolve(bool trace, int fixedSequence, float dt, bool exactStance, bool zeroFriction,
            Vector3 normal, Rigidbody foot, Rigidbody lower, Vector3 footAnchorLocal, Vector3 lowerAnchorLocal,
            bool gravityScheduled, bool loadApplied, float signedLoadA, FootAnkleFoundationReport report)
        {
            if (!trace) return;
            try
            {
                if (_pending >= 0 && _ring[_pending].Fixed == fixedSequence) return;
                if (_pending >= 0)
                {
                    Invalid("PENDING_SOLVE_NOT_RESOLVED_BEFORE_NEXT_CAPTURE");
                    ClearPendingPreserveTrace(report);
                }
                int i = Find(fixedSequence);
                if (i < 0 || !_ring[i].Written || _ring[i].PreCaptured || _ring[i].Censored) return;
                Frame f = _ring[i];
                bool solveNormalValid = V(normal) && normal.sqrMagnitude > 0.00000001f;
                f.Loaded = exactStance && zeroFriction && gravityScheduled && loadApplied && solveNormalValid &&
                    foot != null && lower != null && !foot.isKinematic && !lower.isKinematic;
                if (foot == null || lower == null || foot.isKinematic || lower.isKinematic)
                {
                    f.Censored = true; _ring[i] = f; _censored++;
                    CopyToReport(report, false);
                    return; // no dynamic solve can be joined after an aborted/quiesced write
                }
                f.Dt = dt; f.Mass = foot.mass; f.Drag = foot.drag;
                f.PreFootCom = foot.worldCenterOfMass; f.PreLowerCom = lower.worldCenterOfMass;
                f.PreFootV = foot.velocity; f.PreFootW = foot.angularVelocity;
                f.PreLowerV = lower.velocity; f.PreLowerW = lower.angularVelocity;
                f.PreFootR = foot.transform.TransformPoint(footAnchorLocal) - f.PreFootCom;
                f.PreLowerR = lower.transform.TransformPoint(lowerAnchorLocal) - f.PreLowerCom;
                f.PreFootPointV = f.PreFootV + Vector3.Cross(f.PreFootW, f.PreFootR);
                f.NativeGravity = foot.useGravity;
                f.GravityScheduled = gravityScheduled; f.LoadScheduled = loadApplied;
                f.LoadNormalValid = loadApplied && solveNormalValid;
                f.LoadN = f.LoadNormalValid ? normal.normalized : Vector3.zero;
                f.GravityA = f.NativeGravity ? UnityEngine.Physics.gravity : Vector3.zero;
                f.LoadA = f.LoadNormalValid ? f.LoadN * signedLoadA : Vector3.zero; // signed V04210 Foot acceleration
                Require(!loadApplied || solveNormalValid, "SCHEDULED_LOAD_WITHOUT_VALID_NORMAL");
                Require(F(dt) && dt > 0f && F(f.Mass) && f.Mass > 0f && F(f.Drag) && V(f.LoadN) &&
                    V(f.PreFootCom) && V(f.PreLowerCom) && V(f.PreFootV) && V(f.PreFootW) && V(f.PreLowerV) && V(f.PreLowerW) &&
                    V(f.PreFootR) && V(f.PreLowerR) && V(f.PreFootPointV) && V(f.GravityA) && V(f.LoadA), "NON_FINITE_PRE_SOLVE");
                f.PreCaptured = true;
                _ring[i] = f; _pending = i;
            }
            catch (Exception e) { Invalid("PRE_SOLVE_EXCEPTION:" + e.GetType().Name); }
            CopyToReport(report, false);
        }

        internal void CaptureNormalLoadInputs(bool trace, int fixedSequence, SingleLegChainReport leg,
            Rigidbody lower, Vector3 kneeAppliedAcceleration, string sourceMotionPhase, FootAnkleFoundationReport report)
        {
            if (!trace || _pending < 0) return;
            Frame f = _ring[_pending];
            try
            {
                if (f.Fixed != fixedSequence || f.Normal.Captured) return;
                // All selectors have run this step and clear their current fields even on an OFF solve.
                // False is an actual selector result here, not a replacement for an absent command sample.
                UpperSelfWeightSupportReport ul = leg?.UpperSelfWeightSupport;
                // OFF/released samples still need a current evaluated command. A stale STANCE force
                // must never be silently accepted as an unloaded input or an idle selector result.
                bool upperInputCurrent = ul != null && ul.CurrentFixed == fixedSequence &&
                    ul.GravityReadback == ul.CurrentApplied &&
                    (!ul.CurrentApplied || (ul.Selected && ul.Enabled && ul.CurrentEligible && f.Loaded)) &&
                    (!ul.Selected || (ul.Finite && ul.CurrentEligible == f.Loaded &&
                        (!f.Loaded || ul.CurrentApplied == ul.Enabled)));
                bool current = upperInputCurrent && leg != null && leg.FixedUpdateSequence == fixedSequence && lower != null &&
                    leg.KneeCharacterJointResponseIsolationEnabled &&
                    leg.HipCharacterJointResponseIsolationEnabled && leg.HipTranslationTetherUpperOnlyEnabled &&
                    leg.HipAnchorLinearTransportCurrentFixed == fixedSequence &&
                    (!leg.StanceLowerNativeGravityCurrentApplied || leg.StanceLowerNativeGravityCurrentScheduledFixed == fixedSequence) &&
                    (!leg.StanceFootLowerReciprocalCurrentApplied || leg.StanceFootLowerReciprocalCurrentFixed == fixedSequence) &&
                    (!leg.StanceLowerSelfWeightTransferCurrentApplied || leg.StanceLowerSelfWeightTransferCurrentFixed == fixedSequence) &&
                    (!report.StanceLowerGroundAuthorityCurrentApplied || report.StanceLowerGroundAuthorityLastAppliedFixed == fixedSequence) &&
                    (!f.Loaded || (leg.StanceLowerNativeGravityCurrentApplied && leg.StanceFootLowerReciprocalCurrentApplied &&
                        leg.StanceLowerSelfWeightTransferCurrentApplied));
                if (!current) { LedgerInputInvalid(f.Loaded, "COMMAND_CLOCK_OR_LOAD_INPUT_MISSING"); return; }
                // V04190 reads mass/environment on every completed selector step, including non-STANCE.
                float mass = leg.StanceLowerNativeGravityLowerMass;
                Vector3 n = f.Loaded ? f.LoadN : Vector3.zero; // no invented unloaded support normal
                Vector3 gravity = leg.StanceLowerNativeGravityCurrentApplied ? leg.StanceLowerNativeGravityGravityWorld : Vector3.zero;
                Vector3 reaction = leg.StanceFootLowerReciprocalCurrentApplied ? leg.StanceFootLowerReciprocalCurrentReactionForceWorld : Vector3.zero;
                Vector3 self = leg.StanceLowerSelfWeightTransferCurrentApplied ? leg.StanceLowerSelfWeightTransferCurrentForceWorld : Vector3.zero;
                Vector3 upperLowerKnee = ul.CurrentApplied ? ul.CurrentLowerKneeForce : Vector3.zero;
                Vector3 upperLowerAnkle = ul.CurrentApplied ? ul.CurrentLowerAnkleForce : Vector3.zero;
                Vector3 upperFoot = ul.CurrentApplied ? ul.CurrentFootForce : Vector3.zero;
                Vector3 planar = report.StanceLowerGroundAuthorityCurrentApplied ? report.StanceLowerGroundAuthorityAppliedAccelerationWorld : Vector3.zero;
                // The writer's bounded output includes actual zero when evaluated OFF. The frozen Upper-only
                // Hip tether is checked above so its historical Lower force branch is never silently omitted.
                Vector3 hipLinear = leg.HipAnchorLinearTransportCurrentAppliedAccelerationWorld;
                bool massMatch = F(mass) && mass > 0f &&
                    (!leg.StanceFootLowerReciprocalCurrentApplied ||
                        (Mathf.Abs(mass - leg.StanceFootLowerReciprocalCurrentLowerMass) <= 0.0001f &&
                         Mathf.Abs(f.Mass - leg.StanceFootLowerReciprocalCurrentFootMass) <= 0.0001f)) &&
                    (!leg.StanceLowerSelfWeightTransferCurrentApplied ||
                        (Mathf.Abs(mass - leg.StanceLowerSelfWeightTransferCurrentLowerMass) <= 0.0001f &&
                         Mathf.Abs(f.Mass - leg.StanceLowerSelfWeightTransferCurrentFootMass) <= 0.0001f));
                massMatch = massMatch && (!ul.CurrentApplied ||
                    (Mathf.Abs(mass - ul.CurrentLowerMass) <= 0.0001f && Mathf.Abs(f.Mass - ul.CurrentFootMass) <= 0.0001f));
                if (!massMatch || !V(upperLowerKnee) || !V(upperLowerAnkle) || !V(upperFoot) || !V(kneeAppliedAcceleration) || !V(hipLinear) || !V(planar) || !V(gravity) || !V(reaction) || !V(self) ||
                    (!leg.HipAnchorLinearTransportActiveThisStep && hipLinear.sqrMagnitude > 0.00000001f) ||
                    !V(f.GravityA) || (f.Loaded && (!f.LoadNormalValid || !V(n) || (gravity - f.GravityA).magnitude > 0.0001f)))
                { LedgerInputInvalid(f.Loaded, "NONFINITE_OR_INCONSISTENT_LOAD_INPUT"); return; }
                f.UpperLoadA = upperFoot / f.Mass; // Keep the V04210 component separate; no double counting in NormalLoad.
                NormalLoad d = new NormalLoad
                {
                    Captured = true, Loaded = f.Loaded, NormalValid = f.Loaded, SourcePhase = sourceMotionPhase,
                    LowerMass = mass, FootMass = f.Mass, FootDrag = f.Drag,
                    LowerGravityScheduled = leg.StanceLowerNativeGravityCurrentApplied,
                    ReciprocalApplied = leg.StanceFootLowerReciprocalCurrentApplied,
                    SelfApplied = leg.StanceLowerSelfWeightTransferCurrentApplied,
                    PlanarApplied = report.StanceLowerGroundAuthorityCurrentApplied,
                    HipLinearApplied = leg.HipAnchorLinearTransportActiveThisStep,
                    HipLinearFixed = leg.HipAnchorLinearTransportCurrentFixed,
                    PreFixed = f.Fixed, Dt = f.Dt, PreNormal = n, PreLowerV = f.PreLowerV, PreFootV = f.PreFootV,
                    LowerKnownWorld = (mass * (kneeAppliedAcceleration + hipLinear + planar + gravity) + reaction + self + upperLowerKnee + upperLowerAnkle) * f.Dt,
                    FootCommandWorld = (f.Mass * (f.Applied + f.LoadA + f.GravityA) + upperFoot) * f.Dt,
                    UpperLoadApplied = ul.CurrentApplied, UpperLoadFixed = ul.CurrentFixed,
                    UpperLoadLowerKneeWorld = upperLowerKnee * f.Dt, UpperLoadLowerAnkleWorld = upperLowerAnkle * f.Dt, UpperLoadFootWorld = upperFoot * f.Dt,
                    KneeWorld = mass * kneeAppliedAcceleration * f.Dt, PlanarWorld = mass * planar * f.Dt,
                    HipLinearWorld = mass * hipLinear * f.Dt,
                    ReciprocalWorld = reaction * f.Dt, SelfWorld = self * f.Dt, LowerGravityWorld = mass * gravity * f.Dt,
                    FootTetherWorld = f.Mass * f.Applied * f.Dt, FootSelfWorld = f.Mass * f.LoadA * f.Dt,
                    FootGravityWorld = f.Mass * f.GravityA * f.Dt
                };
                // V04430 is an Upper-only command. Keep its diagnostic copy separate: adding it to
                // LowerKnownWorld would fabricate closure for the wrong subsystem. No native getter/buffer.
                KneeSupportNormalReactionReport kr = leg.KneeSupportReaction;
                d.KneeReactionInputValid = kr != null && kr.Finite && kr.CurrentFixed == fixedSequence &&
                    (!kr.CurrentApplied || (kr.CurrentEligible && kr.CurrentSourceFixed == fixedSequence &&
                        Mathf.Abs(kr.CurrentLowerMass - mass) <= 0.0001f)) && V(kr.CurrentUpperForceWorld);
                d.KneeReactionFixed = kr != null ? kr.CurrentFixed : 0;
                d.KneeReactionUpperApplied = d.KneeReactionInputValid && kr.CurrentApplied;
                d.KneeReactionUpperWorld = d.KneeReactionInputValid ? kr.CurrentUpperForceWorld * f.Dt : Vector3.zero;
                d.KneeJ = mass * Vector3.Dot(kneeAppliedAcceleration, n) * f.Dt;
                d.HipLinearJ = mass * Vector3.Dot(hipLinear, n) * f.Dt;
                d.PlanarJ = mass * Vector3.Dot(planar, n) * f.Dt;
                d.ReciprocalJ = Vector3.Dot(reaction, n) * f.Dt;
                d.SelfJ = Vector3.Dot(self, n) * f.Dt;
                d.LowerGravityJ = mass * Vector3.Dot(gravity, n) * f.Dt;
                d.FootTetherJ = f.Mass * Vector3.Dot(f.Applied, n) * f.Dt;
                d.FootSelfJ = f.Mass * Vector3.Dot(f.LoadA, n) * f.Dt;
                d.FootGravityJ = f.Mass * Vector3.Dot(f.GravityA, n) * f.Dt;
                // Only the Lower-Ankle / Foot pair is internal here. The Upper-Knee input crosses this subsystem boundary.
                d.InternalClosure = Vector3.Dot(d.FootTetherWorld + d.ReciprocalWorld + d.FootSelfWorld + d.SelfWorld +
                    d.UpperLoadLowerAnkleWorld + d.UpperLoadFootWorld, n);
                if (!NormalInputsFinite(d)) { LedgerInputInvalid(f.Loaded, "IMPULSE_INPUT_NONFINITE"); return; }
                CaptureDragModel(ref d, lower); // failure leaves the raw ledger intact and the model explicitly NA
                f.Normal = d; _ring[_pending] = f;
                if (f.Loaded) _normalCaptured++; else _transitionCaptured++;
            }
            catch (Exception e) { LedgerInputInvalid(f.Loaded, "CAPTURE_EXCEPTION:" + e.GetType().Name); }
            finally { CopyToReport(report, false); }
        }

        private static Vector3 LinearDragModelImpulse(float mass, float drag, float dt, Vector3 preV, Vector3 commandJ)
            => (Mathf.Max(1f - drag * dt, 0f) - 1f) * (mass * preV + commandJ);

        private static void CaptureDragModel(ref NormalLoad d, Rigidbody lower)
        {
            // Public PhysX ordinary integrator motivates this comparison only. The game's backend/order,
            // CCD passes, sleeping, velocity caps and contact modifications are not certified by this model.
            // Do NOT include callback Ground J in the pre-contact damping input or feed this into any actuator.
            try
            {
                d.LowerDrag = lower.drag; // one added scalar getter; Foot drag/preV/commands already captured
                if (!F(d.LowerDrag) || d.LowerDrag < 0f || !F(d.FootDrag) || d.FootDrag < 0f)
                { d.DragModelIssue = "INVALID_NATIVE_DRAG"; return; }
                d.LowerDragWorld = LinearDragModelImpulse(d.LowerMass, d.LowerDrag, d.Dt, d.PreLowerV, d.LowerKnownWorld);
                d.FootDragWorld = LinearDragModelImpulse(d.FootMass, d.FootDrag, d.Dt, d.PreFootV, d.FootCommandWorld);
                d.DragModelValid = V(d.LowerDragWorld) && V(d.FootDragWorld);
                if (!d.DragModelValid) d.DragModelIssue = "NONFINITE_DRAG_MODEL";
            }
            catch (Exception e) { d.DragModelIssue = "DRAG_READ_EXCEPTION:" + e.GetType().Name; }
        }

        internal void ObserveCollision(bool trace, int preFixed, string phase, bool projectionValid, Vector3 impulse,
            FootAnkleFoundationReport report)
        {
            if (!trace) return;
            if (phase == "EXIT") _exitCallbacks++;
            if (_pending < 0) { CopyToReport(report, false); return; }
            try
            {
                Frame f = _ring[_pending];
                if (f.Fixed != preFixed) return;
                if (phase == "EXIT")
                {
                    f.ExitCallbacks++;
                    _ring[_pending] = f;
                    CopyToReport(report, false);
                    return; // normal lifecycle notification has no reaction-impulse projection
                }
                if (phase != "ENTER" && phase != "STAY")
                {
                    Invalid("UNKNOWN_AUTHORIZED_CALLBACK_PHASE");
                    CopyToReport(report, false);
                    return;
                }
                f.Callbacks++;
                if (projectionValid && V(impulse)) { f.ProjectedCallbacks++; f.GroundJ += impulse; }
                else Invalid("AUTHORIZED_CALLBACK_PROJECTION_INVALID");
                Require(V(f.GroundJ), "NON_FINITE_GROUND_IMPULSE");
                _ring[_pending] = f;
            }
            catch (Exception e) { Invalid("CALLBACK_EXCEPTION:" + e.GetType().Name); }
            CopyToReport(report, false);
        }

        internal void ObserveSettledPreLifecycle(bool trace, int fixedSequence, Rigidbody foot, Rigidbody lower,
            Vector3 footAnchorLocal, Vector3 lowerAnchorLocal, FootAnkleFoundationReport report)
        {
            if (!trace || _pending < 0) return;
            try
            {
                Frame f = _ring[_pending];
                if (fixedSequence != f.Fixed + 1)
                {
                    Invalid("PRE_SETTLED_FIXED_SEQUENCE_MISMATCH");
                    ClearPendingPreserveTrace(report); return;
                }
                Vector3 footCom = foot.worldCenterOfMass, lowerCom = lower.worldCenterOfMass;
                Vector3 footAnchor = foot.transform.TransformPoint(footAnchorLocal), lowerAnchor = lower.transform.TransformPoint(lowerAnchorLocal);
                Vector3 footR = footAnchor - footCom, lowerR = lowerAnchor - lowerCom;
                Vector3 postV = foot.velocity, postW = foot.angularVelocity;
                f.PostE = lowerAnchor - footAnchor;
                f.DeltaW = postW - f.PreFootW;
                f.DeltaComV = postV - f.PreFootV;
                f.DeltaRotV = Vector3.Cross(postW, footR) - Vector3.Cross(f.PreFootW, f.PreFootR);
                f.DeltaPointV = postV + Vector3.Cross(postW, footR) - f.PreFootPointV;
                f.PointClosure = (f.DeltaPointV - f.DeltaComV - f.DeltaRotV).magnitude;
                f.DeltaECom = (lowerCom - f.PreLowerCom) - (footCom - f.PreFootCom);
                f.DeltaELever = (lowerR - f.PreLowerR) - (footR - f.PreFootR);
                f.PositionClosure = (f.PostE - f.E - f.DeltaECom - f.DeltaELever).magnitude;
                f.PredictedComDeltaV = (f.GravityA + f.Applied + f.LoadA + f.UpperLoadA) * f.Dt + f.GroundJ / f.Mass;
                f.ComResidual = f.DeltaComV - f.PredictedComDeltaV;
                Require(V(f.PostE) && V(f.DeltaW) && V(f.DeltaComV) && V(f.DeltaRotV) && V(f.DeltaPointV) &&
                    V(f.DeltaECom) && V(f.DeltaELever) && V(f.PredictedComDeltaV) && V(f.ComResidual) &&
                    F(f.PointClosure) && F(f.PositionClosure), "NON_FINITE_SETTLED_STATE");
                _pointClosureMax = Mathf.Max(_pointClosureMax, f.PointClosure);
                _positionClosureMax = Mathf.Max(_positionClosureMax, f.PositionClosure);
                Require(f.PointClosure <= ClosureTolerance && f.PositionClosure <= ClosureTolerance, "SETTLED_KINEMATIC_CLOSURE_FAILED");
                f.Settled = fixedSequence; f.Resolved = true;
                ResolveNormalLoad(ref f, lower, postV);
                _ring[_pending] = f; _pending = -1; _responseResolved++;
                if (f.Loaded) _resolved++; // unloaded responses never inflate the 50-loaded-solve qualification
                bool tail = f.PostE.magnitude >= 0.020f || f.DeltaW.magnitude >= 5f || f.DeltaRotV.magnitude >= 0.5f ||
                    (f.NormalValid ? Planar(f.ComResidual, f.N).magnitude : f.ComResidual.magnitude) >= 0.5f;
                if (tail)
                {
                    _tails++;
                    // V04430: retained numeric ring/peaks replace the synchronous per-tail text dump.
                }
            }
            catch (Exception e)
            {
                Invalid("SETTLED_EXCEPTION:" + e.GetType().Name);
                ClearPendingPreserveTrace(report);
            }
            CopyToReport(report, false);
        }

        internal void ClearPendingPreserveTrace(FootAnkleFoundationReport report)
        {
            if (_pending >= 0)
            {
                Frame f = _ring[_pending]; f.Censored = true;
                if (f.Normal.Captured && !f.Normal.Resolved)
                { if (f.Loaded) _normalCensored++; else _transitionCensored++; }
                _ring[_pending] = f;
                _censored++; _pending = -1;
            }
            CopyToReport(report, false);
        }

        internal void EndTrace(FootAnkleFoundationReport report)
        {
            ClearPendingPreserveTrace(report);
            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report, bool format = true)
        {
            if (report == null) return;
            report.RockingTetherCouplingMode = Mode;
            report.RockingTetherCouplingFinite = _finite;
            report.RockingTetherCouplingInvalidReason = _invalidReason;
            report.RockingTetherCouplingResolvedSolves = _resolved;
            report.RockingTetherCouplingEvidenceReady = _finite && _resolved >= MinimumResolvedSolves;
            report.DynamicNormalLoadMode = NormalLoadMode;
            report.DynamicNormalLoadFinite = _normalFinite;
            report.DynamicNormalLoadInvalidReason = _normalInvalidReason;
            report.DynamicNormalLoadResolvedSolves = _normalAll.Count;
            report.DynamicNormalLoadEvidenceReady = NormalLoadReady;
            if (!format) return;
            report.DynamicNormalLoadTraceSummary = "captured-resolved-censored-skipped-unknownSource=" + _normalCaptured + "-" + _normalAll.Count + "-" + _normalCensored + "-" + _normalSkipped + "-" + _normalUnknownPhase +
                " | all={" + _normalAll.Format() + "} | sourceIDLE={" + _normalIdle.Format() + "} | sourceNON_IDLE={" + _normalMotion.Format() +
                "} | scope=LOADED_FOOT_LOWER_COM_PRE_NORMAL/readyMin(total-IDLE-NON_IDLE)=50-30-30/readinessReason=" + NormalLoadReadinessReason + "/responseExposureDeltaV=0.02mps" +
                "/knee=EXTERNAL_CHILD_ONLY_COM_COMMAND/hipLinear=EXTERNAL_COMMON_MODE_COM_COMMAND/ankle=INTERNAL_NORMAL_FORCE_PAIR/inputRevision=" + InputRevision +
                "/residual=ENDPOINT_EXPLICIT_IMPULSE_LEDGER_NOT_MEASURED_MISSING_FORCE/callbackJ=NOT_COMPLETE_GRF/observerOnly=True" +
                " | transitionWorldLedger={finite=" + _transitionFinite + "/reason=" + (_transitionInvalidReason ?? "<none>") +
                "/captured-resolved-censored-skipped=" + _transitionCaptured + "-" + _transitionResolved + "-" + _transitionCensored + "-" + _transitionSkipped +
                "/scope=UNLOADED_ACTUAL_INPUTS_NOT_LOADED_QUALIFICATION} | dragModel(valid-missing)=" + _dragModelValid + "-" + _dragModelMissing +
                "/model=" + DragModel;
            report.DynamicNormalLoadPeakFrames = !_normalHasPeak ? "<none>" :
                "PEAK_ABS_LOWER_RESIDUAL | " + NormalRow(_normalPeakLowerResidual) + "\nPEAK_ABS_EXTERNAL_KNEE | " + NormalRow(_normalPeakKnee);
            report.RockingTetherCouplingTraceSummary = "evaluated-written-responseResolved-loadedResolved-censored-tail=" + _evaluations + "-" + _written + "-" + _responseResolved + "-" + _resolved + "-" + _censored + "-" + _tails +
                " | authorizedEXIT=" + _exitCallbacks + " | closureMax(velocity-command-point-position)=" + S(_velocityClosureMax) + "-" + S(_commandClosureMax) + "-" + S(_pointClosureMax) + "-" + S(_positionClosureMax);
            var sb = new StringBuilder();
            for (int k = 0; k < _count; k++)
            {
                int i = (_next - _count + k + Capacity) % Capacity;
                sb.AppendLine(Row(_ring[i]));
            }
            report.RockingTetherCouplingRecentFrames = sb.ToString();
        }

        private int Find(int fixedSequence)
        {
            for (int k = 0; k < _count; k++)
            {
                int i = (_next - 1 - k + Capacity) % Capacity;
                if (_ring[i].Fixed == fixedSequence) return i;
            }
            return -1;
        }

        private static string Row(Frame f)
        {
            string row = "pre-settled=" + f.Fixed + "-" + f.Settled + " | render-fresh-held-age=" + f.Render + "-" + f.Fresh + "-" + f.Held + "-" + S(f.Age) +
                " | commandWritten=" + f.Written + "/loaded=" + f.Loaded + "/resolved=" + f.Resolved + "/censored=" + f.Censored +
                " | dt=" + S(f.Dt) + " | E=" + W(f.E) + " | footCOMV=" + W(f.PreFootV) + "/W=" + W(f.PreFootW) + "/lever=" + W(f.PreFootR) +
                " | lowerCOMV=" + W(f.PreLowerV) + "/W=" + W(f.PreLowerW) + "/lever=" + W(f.PreLowerR) +
                " | velocityErrorCOM=" + W(f.VComError) + "/ROT=" + W(f.VRotError) +
                " | candidateP=" + W(f.P) + "/Dcom=" + W(f.DCom) + "/baselineDrot=" + W(f.DRot) + "/transport=" + W(f.Transport) +
                " | V04300Active=" + f.PlanarIsolation + "/removedSignedFootRotPlanarD=" + W(f.RemovedFootRotD) +
                "/effectiveDrot=" + W(f.EffectiveDRot) + "/effectiveD=" + W(f.EffectiveD) + "/effectiveVelocityError=" + W(f.EffectiveVelocity) +
                "/rawNormalDampingDelta=" + (f.NormalValid ? S(f.RawNormalDampingDelta) : "NA") +
                " | baselineRawSameState_NO_OUTCOME=" + W(f.BaselineRaw) + " | effectiveRaw=" + W(f.Raw);
            row += " | commandNormalValid=" + f.NormalValid + " | preN=" + (f.NormalValid ? W(f.N) : "<none>") +
                "/EtotalMm=" + S(f.E.magnitude * 1000f) + "/E_normal-planarMm=" + ErrorNP(f.E, f);
            if (!f.Written) return row + " | GATE_CANDIDATE_ONLY_NO_TETHER_COMMAND";
            row += " | cap-scale-clamped=" + S(f.Cap) + "-" + S(f.Scale) + "-" + f.Clamped + " | applied=" + W(f.Applied) +
                " | scaledP=" + W(f.P * f.Scale) + "/Dcom=" + W(f.DCom * f.Scale) + "/effectiveDrot=" + W(f.EffectiveDRot * f.Scale) + "/transport=" + W(f.Transport * f.Scale);
            row += " | preCaptured=" + f.PreCaptured + "/loadedEvidenceEligible=" + f.Loaded;
            if (!f.PreCaptured) return row + (f.Censored ? " | RESPONSE_CENSORED_NO_DYNAMIC_PRE_CAPTURE" : " | RESPONSE_NOT_PRE_CAPTURED");
            row += " | mass-drag=" + S(f.Mass) + "-" + S(f.Drag) + " | nativeGravity=" + f.NativeGravity + "/gravityScheduled=" + f.GravityScheduled +
                "/loadScheduled=" + f.LoadScheduled + "/loadNormalValid=" + f.LoadNormalValid + "/loadN=" + (f.LoadNormalValid ? W(f.LoadN) : "<none>") +
                " | gravityA=" + W(f.GravityA) + "/signedLoadA=" + W(f.LoadA) + "/upperLoadA=" + W(f.UpperLoadA) +
                " | callbacks(ENTER_STAY-projected-EXIT)=" + f.Callbacks + "-" + f.ProjectedCallbacks + "-" + f.ExitCallbacks + " | GroundJ=" + W(f.GroundJ) +
                " | appliedP(n-planar)=" + NP(f.P * f.Scale, f) + "/Dcom=" + NP(f.DCom * f.Scale, f) +
                "/effectiveDrot=" + NP(f.EffectiveDRot * f.Scale, f) + "/transport=" + NP(f.Transport * f.Scale, f);
            if (!f.Resolved) return row + (f.Censored ? " | RESPONSE_CENSORED_AT_QUIESCE" : " | RESPONSE_PENDING_NEXT_SETTLED");
            return row + " | normalLoadV04330={" + NormalRow(f) + "} | postE=" + W(f.PostE) + "/totalMm=" + S(f.PostE.magnitude * 1000f) + "/normal-planarMm=" + ErrorNP(f.PostE, f) +
                " | deltaW=" + W(f.DeltaW) + " | deltaCOMV=" + W(f.DeltaComV) + "/n-planar=" + NP(f.DeltaComV, f) +
                " | deltaRotPointV=" + W(f.DeltaRotV) + "/n-planar=" + NP(f.DeltaRotV, f) + " | deltaPointV=" + W(f.DeltaPointV) +
                " | deltaE_COM=" + W(f.DeltaECom) + "/lever=" + W(f.DeltaELever) +
                " | knownCOMDeltaV=" + W(f.PredictedComDeltaV) + "/residual=" + W(f.ComResidual) + "/residualN-planar=" + NP(f.ComResidual, f) +
                " | closure(v-command-point-position)=" + S(f.VelocityClosure) + "-" + S(f.CommandClosure) + "-" + S(f.PointClosure) + "-" + S(f.PositionClosure);
        }

        private void ResolveNormalLoad(ref Frame frame, Rigidbody lower, Vector3 settledFootComVelocity)
        {
            if (!frame.Normal.Captured) return;
            try
            {
                // v0.4.41: own the same pre-lifecycle settled sample after retiring V04230.
                // Both loaded and unloaded solves need this read; no controller write occurs between joins.
                if (lower == null) { LedgerInputInvalid(frame.Loaded, "SETTLED_LOWER_BODY_MISSING"); return; }
                Vector3 settledLowerComVelocity = lower.velocity;
                if (!V(settledLowerComVelocity))
                { LedgerInputInvalid(frame.Loaded, "SETTLED_LOWER_VELOCITY_NONFINITE"); return; }
                NormalLoad d = frame.Normal;
                d.SettledFixed = frame.Settled;
                d.SettledLowerV = settledLowerComVelocity;
                d.SettledFootV = settledFootComVelocity;
                d.LowerDeltaPWorld = d.LowerMass * (settledLowerComVelocity - frame.PreLowerV);
                d.FootDeltaPWorld = frame.Mass * frame.DeltaComV;
                d.GroundWorld = frame.GroundJ;
                // R1: one complete world ledger owns every normal scalar. The former component
                // sum omitted V04450 Foot -F, creating a false -3.5 Ns residual and exhausting windows.
                d.LowerDeltaP = Vector3.Dot(d.LowerDeltaPWorld, d.PreNormal);
                d.FootDeltaP = Vector3.Dot(d.FootDeltaPWorld, d.PreNormal);
                d.GroundJ = Vector3.Dot(d.GroundWorld, d.PreNormal);
                Vector3 lowerResidualWorld = d.LowerDeltaPWorld - d.LowerKnownWorld;
                Vector3 footResidualWorld = d.FootDeltaPWorld - d.FootCommandWorld - d.GroundWorld;
                d.LowerResidual = Vector3.Dot(lowerResidualWorld, d.PreNormal);
                d.FootResidual = Vector3.Dot(footResidualWorld, d.PreNormal);
                d.PairResidual = Vector3.Dot(lowerResidualWorld + footResidualWorld, d.PreNormal);
                if (!F(d.LowerDeltaP) || !F(d.FootDeltaP) || !F(d.GroundJ) || !F(d.LowerResidual) || !F(d.FootResidual) || !F(d.PairResidual) ||
                    !V(d.LowerDeltaPWorld) || !V(d.FootDeltaPWorld) || !V(d.GroundWorld) || !V(d.SettledLowerV) || !V(d.SettledFootV))
                { LedgerInputInvalid(frame.Loaded, "SETTLED_LEDGER_NONFINITE"); return; }
                if (!NormalLedgerConsistent(d))
                { LedgerInputInvalid(frame.Loaded, "NORMAL_COMPONENT_OR_SCALAR_WORLD_LEDGER_MISMATCH"); return; }
                d.Resolved = true; frame.Normal = d;
                if (d.DragModelValid) _dragModelValid++; else _dragModelMissing++;
                if (!frame.Loaded) { _transitionResolved++; return; } // never inflate loaded evidence or peaks
                _normalAll.Add(frame);
                if (string.Equals(d.SourcePhase, "IDLE", StringComparison.Ordinal)) _normalIdle.Add(frame);
                else if (!string.IsNullOrEmpty(d.SourcePhase) && !string.Equals(d.SourcePhase, "UNKNOWN", StringComparison.Ordinal)) _normalMotion.Add(frame);
                else _normalUnknownPhase++;
                if (!_normalHasPeak || Mathf.Abs(d.LowerResidual) > Mathf.Abs(_normalPeakLowerResidual.Normal.LowerResidual)) _normalPeakLowerResidual = frame;
                if (!_normalHasPeak || Mathf.Abs(d.KneeJ) > Mathf.Abs(_normalPeakKnee.Normal.KneeJ)) _normalPeakKnee = frame;
                _normalHasPeak = true;
            }
            catch (Exception e) { LedgerInputInvalid(frame.Loaded, "RESOLVE_EXCEPTION:" + e.GetType().Name); }
        }

        private static bool NormalInputsFinite(NormalLoad d) => F(d.KneeJ) && F(d.HipLinearJ) && F(d.PlanarJ) && F(d.ReciprocalJ) &&
            F(d.SelfJ) && F(d.LowerGravityJ) && F(d.FootTetherJ) && F(d.FootSelfJ) && F(d.FootGravityJ) && F(d.InternalClosure) &&
            V(d.UpperLoadLowerKneeWorld) && V(d.UpperLoadLowerAnkleWorld) && V(d.UpperLoadFootWorld) && V(d.LowerKnownWorld) && V(d.FootCommandWorld) && V(d.KneeWorld) && V(d.HipLinearWorld) && V(d.PlanarWorld) && V(d.ReciprocalWorld) &&
            V(d.SelfWorld) && V(d.LowerGravityWorld) && V(d.FootTetherWorld) && V(d.FootSelfWorld) && V(d.FootGravityWorld);

        // Numeric agreement only; callers separately require the clock, native endpoints and support scope.
        // This diagnostic check never changes forces or safety thresholds and performs no native reads.
        internal static bool NormalLedgerConsistent(NormalLoad d)
        {
            if (!NormalInputsFinite(d) || !V(d.PreNormal) || !V(d.LowerDeltaPWorld) || !V(d.FootDeltaPWorld) || !V(d.GroundWorld))
                return false;
            if (d.Loaded && (!d.NormalValid || Mathf.Abs(d.PreNormal.sqrMagnitude - 1f) > 0.0001f))
                return false;
            Vector3 lower = d.LowerDeltaPWorld - d.LowerKnownWorld;
            Vector3 foot = d.FootDeltaPWorld - d.FootCommandWorld - d.GroundWorld;
            return WorldComponentsMatch(d.LowerKnownWorld, d.KneeWorld + d.HipLinearWorld + d.PlanarWorld + d.ReciprocalWorld + d.SelfWorld +
                    d.LowerGravityWorld + d.UpperLoadLowerKneeWorld + d.UpperLoadLowerAnkleWorld) &&
                WorldComponentsMatch(d.FootCommandWorld, d.FootTetherWorld + d.FootSelfWorld + d.FootGravityWorld + d.UpperLoadFootWorld) &&
                ScalarProjectionMatches(d.LowerDeltaP, Vector3.Dot(d.LowerDeltaPWorld, d.PreNormal)) &&
                ScalarProjectionMatches(d.FootDeltaP, Vector3.Dot(d.FootDeltaPWorld, d.PreNormal)) &&
                ScalarProjectionMatches(d.GroundJ, Vector3.Dot(d.GroundWorld, d.PreNormal)) &&
                ScalarProjectionMatches(d.LowerResidual, Vector3.Dot(lower, d.PreNormal)) &&
                ScalarProjectionMatches(d.FootResidual, Vector3.Dot(foot, d.PreNormal)) &&
                ScalarProjectionMatches(d.PairResidual, Vector3.Dot(lower + foot, d.PreNormal)) &&
                ScalarProjectionMatches(d.InternalClosure, Vector3.Dot(d.FootTetherWorld + d.ReciprocalWorld + d.FootSelfWorld + d.SelfWorld +
                    d.UpperLoadLowerAnkleWorld + d.UpperLoadFootWorld, d.PreNormal));
        }

        private static bool ScalarProjectionMatches(float actual, float projected) => F(actual) && F(projected) &&
            Mathf.Abs(actual - projected) <= 0.00001f * Mathf.Max(1f, Mathf.Max(Mathf.Abs(actual), Mathf.Abs(projected)));

        private static bool WorldComponentsMatch(Vector3 actual, Vector3 expected) =>
            ScalarProjectionMatches(actual.x, expected.x) && ScalarProjectionMatches(actual.y, expected.y) && ScalarProjectionMatches(actual.z, expected.z);

        internal NormalLoad GetSettledNormalLoad(int settledFixed)
        {
            int i = Find(settledFixed - 1);
            return i >= 0 && _ring[i].Resolved && _ring[i].Settled == settledFixed ? _ring[i].Normal : default;
        }

        private static string NormalRow(Frame frame)
        {
            NormalLoad d = frame.Normal;
            if (!d.Captured) return "pre-settled=" + frame.Fixed + "-" + frame.Settled + "/INPUT_NOT_CAPTURED";
            return FormatNormalLoad(d, frame.Censored) + (d.Resolved ?
                "/callbacks-projected-EXIT=" + frame.Callbacks + "-" + frame.ProjectedCallbacks + "-" + frame.ExitCallbacks : string.Empty);
        }

        internal static string FormatNormalLoad(NormalLoad d, bool censored)
        {
            string row = "pre-settled=" + d.PreFixed + "-" + d.SettledFixed + "/source=" + (d.SourcePhase ?? "<none>") +
                "/massLowerFoot=" + S(d.LowerMass) + "-" + S(d.FootMass) + "/dt=" + S(d.Dt) +
                "/scope=" + (d.Loaded ? "LOADED" : "UNLOADED_ACTUAL_COMMANDS") + "/inputRevision=" + InputRevision +
                "/hipLinear(active-fixed)=" + d.HipLinearApplied + "-" + d.HipLinearFixed +
                "/actualSelectors(lowerGravity-reciprocal-self-planar)=" + d.LowerGravityScheduled + "-" + d.ReciprocalApplied + "-" + d.SelfApplied + "-" + d.PlanarApplied;
            if (d.NormalValid)
                row += "/preN=" + W(d.PreNormal) + "/Ns:externalKnee=" + S(d.KneeJ) + "/externalLowerPlanar=" + S(d.PlanarJ) +
                    "/externalHipLinear=" + S(d.HipLinearJ) +
                    "/internalReciprocal=" + S(d.ReciprocalJ) + "/internalLowerSelf=" + S(d.SelfJ) +
                    "/lowerGravity=" + S(d.LowerGravityJ) + "/footTether=" + S(d.FootTetherJ) +
                    "/footSelf=" + S(d.FootSelfJ) + "/footGravity=" + S(d.FootGravityJ) + "/internalPairClosure=" + S(d.InternalClosure);
            else row += "/preN=NA_NO_LOADED_NORMAL/Ns:preNormalProjection=NA_USE_WINDOW_REFERENCE";
            row += "/worldComponentJ(knee-planar-reciprocal-self-lowerG)=" + W(d.KneeWorld) + "-" + W(d.PlanarWorld) + "-" +
                W(d.ReciprocalWorld) + "-" + W(d.SelfWorld) + "-" + W(d.LowerGravityWorld) +
                "/worldHipLinearJ=" + W(d.HipLinearWorld) +
                "/V04450(applied-fixed)=" + d.UpperLoadApplied + "-" + d.UpperLoadFixed +
                "/worldUpperLoadJ(lowerKnee-lowerAnkle-foot)=" + W(d.UpperLoadLowerKneeWorld) + "|" + W(d.UpperLoadLowerAnkleWorld) + "|" + W(d.UpperLoadFootWorld) +
                "/V04430UpperOnly(valid-applied-fixed)=" + d.KneeReactionInputValid + "-" + d.KneeReactionUpperApplied + "-" + d.KneeReactionFixed +
                "/V04430UpperCommandJ=" + (d.KneeReactionInputValid ? W(d.KneeReactionUpperWorld) : "NA") +
                "/worldComponentJ(footTether-self-g)=" + W(d.FootTetherWorld) + "-" + W(d.FootSelfWorld) + "-" + W(d.FootGravityWorld);
            if (!d.Resolved) return row + (censored ? "/CENSORED_UNRESOLVED" : "/SETTLED_NOT_AUDITED");
            if (d.NormalValid)
                row += "/groundCallback=" + S(d.GroundJ) + "/lowerDeltaP=" + S(d.LowerDeltaP) + "/footDeltaP=" + S(d.FootDeltaP) +
                    "/lowerResidual=" + S(d.LowerResidual) + "/footResidual=" + S(d.FootResidual) + "/pairResidual=" + S(d.PairResidual);
            row += "/lowerV(pre-post)=" + W(d.PreLowerV) + "-" + W(d.SettledLowerV) +
                "/footV(pre-post)=" + W(d.PreFootV) + "-" + W(d.SettledFootV) +
                "/worldDeltaP(lower-foot)=" + W(d.LowerDeltaPWorld) + "-" + W(d.FootDeltaPWorld) +
                "/worldKnown(lower-footCommand-ground)=" + W(d.LowerKnownWorld) + "-" + W(d.FootCommandWorld) + "-" + W(d.GroundWorld);
            if (d.NormalValid)
                row += "/preNormalResidualAfterDragModelNs(lower-foot-pair)=" + (d.DragModelValid ?
                    S(d.LowerResidual - Vector3.Dot(d.LowerDragWorld, d.PreNormal)) + "-" +
                    S(d.FootResidual - Vector3.Dot(d.FootDragWorld, d.PreNormal)) + "-" +
                    S(d.PairResidual - Vector3.Dot(d.LowerDragWorld + d.FootDragWorld, d.PreNormal)) : "NA");
            return row + "/dragModelValid=" + d.DragModelValid + "/dragLowerFoot=" + (d.DragModelValid ? S(d.LowerDrag) + "-" + S(d.FootDrag) : "NA") +
                "/worldDragModelJ(lower-foot)=" + (d.DragModelValid ? W(d.LowerDragWorld) + "-" + W(d.FootDragWorld) : "NA:" + (d.DragModelIssue ?? "<none>")) +
                "/dragModel=" + DragModel;
        }

        private void LedgerInputInvalid(bool loaded, string reason)
        {
            if (loaded) { NormalInvalid(reason); _normalSkipped++; }
            else { _transitionFinite = false; if (_transitionInvalidReason == null) _transitionInvalidReason = reason; _transitionSkipped++; }
        }

        private void NormalInvalid(string reason) { _normalFinite = false; if (_normalInvalidReason == null) _normalInvalidReason = reason; }
        private void Require(bool valid, string reason) { if (!valid) Invalid(reason); }
        private void Invalid(string reason) { _finite = false; if (_invalidReason == null) _invalidReason = reason; }
        private static Vector3 Planar(Vector3 value, Vector3 n) => value - n * Vector3.Dot(value, n);
        private static string NP(Vector3 value, Frame f) => f.NormalValid ? S(Vector3.Dot(value, f.N)) + ":" + W(Planar(value, f.N)) : "NA";
        private static string ErrorNP(Vector3 value, Frame f) => f.NormalValid ? S(Vector3.Dot(value, f.N) * 1000f) + "-" + S(Planar(value, f.N).magnitude * 1000f) : "NA";
        private static bool F(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool V(Vector3 value) => F(value.x) && F(value.y) && F(value.z);
        private static string S(float value) => value.ToString("G9", CultureInfo.InvariantCulture);
        private static string W(Vector3 value) => "(" + S(value.x) + "," + S(value.y) + "," + S(value.z) + ")";
    }
}
