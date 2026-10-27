using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // V04450: common V04430 B Knee PD. B adds Upper native gravity and an explicit static
    // COM load path: Upper +F / Lower -F, then Lower +F / Foot -F. No tracking gain changes.
    // This idealized axial feedforward is not a natural joint, an angular closure, or a GRF verdict.
    internal sealed class UpperSelfWeightSupportNormalLoadTransferIsolation
    {
        internal const string Mode = "UPPER_SELF_WEIGHT_SERIAL_LOAD_AB_V04450";
        internal const string ArmA = "A_V04450_UPPER_WEIGHT_OFF";
        internal const string ArmB = "B_V04450_UPPER_SELF_WEIGHT_SERIAL_TRANSFER";
        internal const string IdleArm = "OFF_V04450_UPPER_WEIGHT";
        internal const int MinimumEligibleSamples = 50, MinimumNonStanceSamples = 30;
        internal readonly UpperSelfWeightSupportReport Report = new UpperSelfWeightSupportReport { Arm = IdleArm };
        private int _pending = -1, _lastScheduledFixed;
        internal bool Off => !Report.Selected && !Report.Enabled && !Report.CurrentApplied && !Report.GravityReadback;
        internal static bool IsExperimentArm(string arm) => arm == ArmA || arm == ArmB;

        internal bool Select(string arm, Rigidbody upper)
        {
            if (!IsExperimentArm(arm) && arm != IdleArm) return false;
            if (!Quiesce(upper)) return false;
            Report.Arm = arm; Report.Selected = IsExperimentArm(arm); Report.Enabled = arm == ArmB;
            Report.Finite = true; Report.InvalidReason = null; _lastScheduledFixed = 0;
            return true;
        }

        internal void ResetTrace()
        {
            EndTrace(); Array.Clear(Report.Ring, 0, Report.Ring.Length);
            Report.Next = Report.Count = 0;
            Report.Samples = Report.Eligible = Report.NonStance = Report.Writes = 0;
            Report.ReadbackFailures = Report.RollbackFailures = Report.FirstWrittenFixed = Report.LastWrittenFixed = 0;
            Report.CandidateImpulseSum = Report.AppliedImpulseSum = 0d; Report.PeakForce = 0f;
            Report.LedgerCaptured = Report.LedgerResolved = Report.LedgerLoaded = Report.LedgerCensored = Report.LedgerDragValid = 0;
            Report.HipNormalSum = Report.CommonNormalSum = Report.KneeNormalSum = Report.WeightSupportNormalSum = Report.GravityNormalSum = Report.DeltaNormalSum = Report.ResidualNormalSum = 0d;
            Report.PeakAbsHipNormal = Report.PeakAbsResidual = 0f; Report.LedgerFinite = true; Report.LedgerIssue = null;
            // Statistics reset must not erase a gravity command already queued for the next solve.
            _lastScheduledFixed = 0;
        }

        // Resolve the last solve BEFORE any controller, gravity clear, kinematic preview or stop zeroes velocity.
        internal bool PrepareControllerStep(Rigidbody upper, int settledFixed)
        {
            if (!Report.Selected) return true; // Leave the pre-existing general-gravity path untouched.
            if (!Report.Finite) return false;
            try
            {
                if (upper == null || upper.useGravity != Report.CurrentApplied)
                { Report.ReadbackFailures++; return Invalid("UPPER_GRAVITY_PRESTEP_READBACK_MISMATCH"); }
                ResolveLedger(upper, settledFixed);
                if (!GravityOff(upper)) return false;
                ClearCurrent();
                return true;
            }
            catch (Exception e) { return Invalid("UPPER_PRESTEP_EXCEPTION:" + e.GetType().Name); }
        }

        internal bool Schedule(bool trace, int fixedSequence, bool stance, bool zeroFriction,
            Rigidbody upper, Rigidbody lower, Rigidbody foot, Vector3 normal, float dt,
            SingleLegChainReport leg, FootAnkleFoundationReport footReport, Vector3 hipAppliedAcceleration,
            PelvisRunScope scope = PelvisRunScope.TorsoPairOnly)
        {
            ClearCurrent(); Report.CurrentFixed = fixedSequence;
            bool dynamicUpper = PelvisRunScopes.HasUpperWeight(scope);
            if (dynamicUpper && (!SelectorContract(Report, ArmB, false) || !DynamicReciprocalOff(leg) ||
                footReport == null || !leg.RuntimeContext.Valid || !footReport.RuntimeContext.Matches(leg.RuntimeContext) ||
                leg.RuntimeContext.Dt != dt || leg.FixedUpdateSequence != fixedSequence ||
                footReport.FootControllerLastCompletedFixedSequence != fixedSequence))
                return Invalid("DYNAMIC_UPPER_SELECTOR_OR_RECIPROCAL_BOUNDARY_INVALID");
            if (!Report.Selected) return true;
            if (PelvisRunScopes.IsContact(scope) && !dynamicUpper)
                return Invalid("UPPER_LOAD_NOT_AUTHORIZED_IN_CONTACT_SCOPE");
            if (!Report.Finite) return false;
            if (trace) { Report.Samples++; if (!stance) Report.NonStance++; }
            if (upper == null || lower == null || foot == null || upper == lower || upper == foot || lower == foot ||
                upper.isKinematic || lower.isKinematic || foot.isKinematic || upper.useGravity || !F(dt) || dt <= 0f ||
                fixedSequence <= 0 || fixedSequence <= _lastScheduledFixed)
                return Invalid("UPPER_LOAD_BODY_CLOCK_OR_GRAVITY_BOUNDARY_INVALID");
            _lastScheduledFixed = fixedSequence;
            Report.CurrentUpperMass = upper.mass; Report.CurrentLowerMass = lower.mass; Report.CurrentFootMass = foot.mass;
            if (!Mass(upper.mass) || !Mass(lower.mass) || !Mass(foot.mass)) return Invalid("UPPER_LOAD_MASS_INVALID");
            if (stance)
            {
                KneeSupportNormalReactionReport kr = leg?.KneeSupportReaction;
                if (!zeroFriction || kr == null || !kr.Finite || kr.Arm != KneeSupportNormalReactionIsolation.ArmB ||
                    !kr.CurrentApplied || !kr.CurrentNormalConstraint || kr.CurrentFixed != fixedSequence ||
                    leg.FixedUpdateSequence != fixedSequence ||
                    !leg.StanceLowerNativeGravityCurrentApplied || leg.StanceLowerNativeGravityCurrentScheduledFixed != fixedSequence ||
                    !leg.StanceLowerSelfWeightTransferCurrentApplied || leg.StanceLowerSelfWeightTransferCurrentFixed != fixedSequence ||
                    (!dynamicUpper && (!leg.StanceFootLowerReciprocalCurrentApplied || leg.StanceFootLowerReciprocalCurrentFixed != fixedSequence)) ||
                    footReport == null || !footReport.StanceFootNativeGravityCurrentApplied || footReport.StanceFootNativeGravityCurrentScheduledFixed != fixedSequence ||
                    !lower.useGravity || !foot.useGravity ||
                    !V(normal) || !F(normal.sqrMagnitude) || normal.sqrMagnitude <= 0.000001f)
                    return Invalid("UPPER_LOAD_EXACT_STANCE_BASELINE_OR_COMMAND_CLOCK_INVALID");
                Vector3 n = normal.normalized, g = UnityEngine.Physics.gravity;
                float a = -Vector3.Dot(g, n);
                // A scalar axial transfer must not silently discard gravity parallel to a sloped support.
                if (!V(g) || !V(n) || !F(a) || a <= 0f || (g + a * n).magnitude > 0.001f ||
                    (leg.StanceLowerNativeGravityGravityWorld - g).magnitude > 0.0001f ||
                    (footReport.StanceFootNativeGravityGravityWorld - g).magnitude > 0.0001f ||
                    Mathf.Abs(footReport.StanceFootNativeGravityFootMass - foot.mass) > 0.0001f ||
                    Mathf.Abs(kr.CurrentUpperMass - upper.mass) > 0.0001f || Mathf.Abs(kr.CurrentLowerMass - lower.mass) > 0.0001f)
                    return Invalid("UPPER_LOAD_GRAVITY_NORMAL_OR_MASS_INCONSISTENT");
                Vector3 force = upper.mass * a * n;
                float impulse = force.magnitude * dt;
                if (!V(force) || !F(impulse)) return Invalid("UPPER_LOAD_COMMAND_NONFINITE");
                Report.CurrentEligible = true; Report.CurrentNormal = n; Report.CurrentExpectedImpulse = impulse;
                if (trace) { Report.Eligible++; Report.CandidateImpulseSum += impulse; }
                if (Report.Enabled)
                {
                    int written = 0;
                    try
                    {
                        upper.AddForce(force, ForceMode.Force); written = 1;
                        lower.AddForce(-force, ForceMode.Force); written = 2;
                        lower.AddForce(force, ForceMode.Force); written = 3;
                        foot.AddForce(-force, ForceMode.Force); written = 4;
                        upper.useGravity = true;
                        if (!upper.useGravity) { Report.ReadbackFailures++; throw new InvalidOperationException("GRAVITY_READBACK_FALSE"); }
                    }
                    catch (Exception e)
                    {
                        Invalid("UPPER_SERIAL_LOAD_WRITE_EXCEPTION:" + e.GetType().Name);
                        // Undo every successfully returned one-solve command; chain fail-close also makes all bodies kinematic.
                        if (written >= 4) Undo(foot, force);
                        if (written >= 3) Undo(lower, -force);
                        if (written >= 2) Undo(lower, force);
                        if (written >= 1) Undo(upper, -force);
                        GravityOff(upper);
                        return false;
                    }
                    Report.GravityReadback = true; Report.CurrentApplied = true; Report.CurrentGravity = g;
                    Report.CurrentUpperForce = force; Report.CurrentLowerKneeForce = -force;
                    Report.CurrentLowerAnkleForce = force; Report.CurrentFootForce = -force;
                    if (trace)
                    {
                        Report.Writes++; Report.AppliedImpulseSum += impulse; Report.PeakForce = Mathf.Max(Report.PeakForce, force.magnitude);
                        if (Report.FirstWrittenFixed == 0) Report.FirstWrittenFixed = fixedSequence;
                        Report.LastWrittenFixed = fixedSequence;
                    }
                }
            }
            if (trace) CaptureLedger(upper, fixedSequence, stance, dt, leg, hipAppliedAcceleration);
            return true;
        }

        internal bool Quiesce(Rigidbody upper)
        {
            EndTrace(); bool ok = GravityOff(upper); ClearCurrent(); return ok;
        }
        internal void EndTrace()
        {
            if (_pending < 0) return;
            UpperSelfWeightSupportReport.UpperFrame f = Report.Ring[_pending]; f.Censored = true;
            Report.Ring[_pending] = f; _pending = -1; Report.LedgerCensored++;
        }
        private void CaptureLedger(Rigidbody upper, int fixedSequence, bool stance, float dt, SingleLegChainReport leg, Vector3 hipA)
        {
            EndTrace();
            try
            {
                KneeSupportNormalReactionReport kr = leg?.KneeSupportReaction;
                if (leg == null || leg.FixedUpdateSequence != fixedSequence || !leg.HipTranslationTetherUpperOnlyEnabled ||
                    !leg.HipCharacterJointResponseIsolationEnabled || leg.HipAnchorLinearTransportCurrentFixed != fixedSequence ||
                    kr == null || !kr.Finite || kr.CurrentFixed != fixedSequence || !V(hipA) ||
                    !V(leg.HipAnchorLinearTransportCurrentAppliedAccelerationWorld) || !V(kr.CurrentUpperForceWorld))
                { LedgerInvalid("UPPER_ACTUAL_INPUT_CLOCK_OR_TOPOLOGY_INVALID"); return; }
                var f = new UpperSelfWeightSupportReport.UpperFrame
                {
                    Fixed = fixedSequence, BodyId = upper.GetInstanceID(), Mass = upper.mass, Dt = dt,
                    Loaded = stance, Applied = Report.CurrentApplied, Normal = Report.CurrentNormal, PreV = upper.velocity,
                    HipJ = upper.mass * hipA * dt, CommonJ = upper.mass * leg.HipAnchorLinearTransportCurrentAppliedAccelerationWorld * dt,
                    KneeJ = kr.CurrentUpperForceWorld * dt, WeightJ = Report.CurrentUpperForce * dt,
                    GravityJ = upper.mass * Report.CurrentGravity * dt
                };
                f.KnownJ = f.HipJ + f.CommonJ + f.KneeJ + f.WeightJ + f.GravityJ;
                if (!V(f.PreV) || !V(f.KnownJ)) { LedgerInvalid("UPPER_LEDGER_NONFINITE"); return; }
                try
                {
                    float drag = upper.drag;
                    if (F(drag) && drag >= 0f)
                    { f.DragJ = (Mathf.Max(1f - drag * dt, 0f) - 1f) * (f.Mass * f.PreV + f.KnownJ); f.DragValid = V(f.DragJ); }
                }
                catch { } // Raw ledger remains valid; the optional comparison is explicitly NA.
                // V04460 optional geometry: a read failure must not invalidate the existing COM momentum ledger.
                try { f.PreCom = upper.worldCenterOfMass; }
                catch { f.PreCom = new Vector3(float.NaN, float.NaN, float.NaN); }
                Report.Ring[Report.Next] = f; _pending = Report.Next;
                Report.Next = (Report.Next + 1) % Report.Ring.Length; Report.Count = Mathf.Min(Report.Count + 1, Report.Ring.Length);
                Report.LedgerCaptured++;
            }
            catch (Exception e) { LedgerInvalid("UPPER_CAPTURE_EXCEPTION:" + e.GetType().Name); }
        }
        private void ResolveLedger(Rigidbody upper, int settledFixed)
        {
            if (_pending < 0) return;
            try
            {
                UpperSelfWeightSupportReport.UpperFrame f = Report.Ring[_pending];
                if (settledFixed != f.Fixed + 1 || upper.GetInstanceID() != f.BodyId || upper.isKinematic ||
                    !Mass(upper.mass) || Mathf.Abs(upper.mass - f.Mass) > 0.0001f || !V(upper.velocity))
                { LedgerInvalid("UPPER_SETTLED_CLOCK_BODY_OR_MASS_MISMATCH"); EndTrace(); return; }
                f.Settled = settledFixed; f.PostV = upper.velocity; f.DeltaP = f.Mass * (f.PostV - f.PreV);
                f.Residual = f.DeltaP - f.KnownJ;
                if (!V(f.DeltaP) || !V(f.Residual)) { LedgerInvalid("UPPER_SETTLED_NONFINITE"); EndTrace(); return; }
                f.Resolved = true; Report.Ring[_pending] = f; _pending = -1; Report.LedgerResolved++;
                if (f.DragValid) Report.LedgerDragValid++;
                if (!f.Loaded) return;
                Report.LedgerLoaded++;
                Report.HipNormalSum += Vector3.Dot(f.HipJ, f.Normal); Report.CommonNormalSum += Vector3.Dot(f.CommonJ, f.Normal);
                Report.KneeNormalSum += Vector3.Dot(f.KneeJ, f.Normal); Report.WeightSupportNormalSum += Vector3.Dot(f.WeightJ, f.Normal);
                Report.GravityNormalSum += Vector3.Dot(f.GravityJ, f.Normal); Report.DeltaNormalSum += Vector3.Dot(f.DeltaP, f.Normal);
                Report.ResidualNormalSum += Vector3.Dot(f.Residual, f.Normal);
                Report.PeakAbsHipNormal = Mathf.Max(Report.PeakAbsHipNormal, Mathf.Abs(Vector3.Dot(f.HipJ, f.Normal)));
                Report.PeakAbsResidual = Mathf.Max(Report.PeakAbsResidual, f.Residual.magnitude);
            }
            catch (Exception e) { LedgerInvalid("UPPER_RESOLVE_EXCEPTION:" + e.GetType().Name); EndTrace(); }
        }
        private bool GravityOff(Rigidbody upper)
        {
            try
            {
                if (upper != null) upper.useGravity = false;
                Report.GravityReadback = upper != null && upper.useGravity;
                if (!Report.GravityReadback) return true;
            }
            catch { Report.GravityReadback = true; }
            Report.ReadbackFailures++; return Invalid("UPPER_GRAVITY_OFF_READBACK_FAILED");
        }
        private void ClearCurrent()
        {
            Report.CurrentFixed = 0; Report.CurrentEligible = Report.CurrentApplied = false;
            Report.CurrentUpperMass = Report.CurrentLowerMass = Report.CurrentFootMass = Report.CurrentExpectedImpulse = 0f;
            Report.CurrentNormal = Report.CurrentUpperForce = Report.CurrentLowerKneeForce = Report.CurrentLowerAnkleForce = Report.CurrentFootForce = Report.CurrentGravity = Vector3.zero;
        }
        private void Undo(Rigidbody body, Vector3 force)
        { try { body.AddForce(force, ForceMode.Force); } catch { Report.RollbackFailures++; } }
        private bool Invalid(string reason) { Report.Finite = false; Report.InvalidReason = Report.InvalidReason ?? reason; return false; }
        private void LedgerInvalid(string reason) { Report.LedgerFinite = false; Report.LedgerIssue = Report.LedgerIssue ?? reason; }
        internal static bool DynamicReciprocalOff(SingleLegChainReport leg) => leg != null &&
            leg.StanceFootLowerReciprocalArm == FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmA &&
            leg.StanceFootLowerReciprocalFinite && !leg.StanceFootLowerReciprocalEnabled &&
            !leg.StanceFootLowerReciprocalCurrentApplied && leg.StanceFootLowerReciprocalCurrentFixed == 0;

        internal static bool DynamicCommandMatches(UpperSelfWeightSupportReport r, SingleLegChainReport leg,
            FootAnkleFoundationReport foot, int fixedSequence, bool loaded)
        {
            if (!SelectorContract(r, ArmB, false) || !DynamicReciprocalOff(leg) || foot == null ||
                r.CurrentFixed != fixedSequence || r.CurrentEligible != loaded || r.CurrentApplied != loaded ||
                r.GravityReadback != loaded || !V(r.CurrentUpperForce) || !V(r.CurrentLowerKneeForce) ||
                !V(r.CurrentLowerAnkleForce) || !V(r.CurrentFootForce) || !V(r.CurrentGravity)) return false;
            if (!loaded) return r.CurrentUpperForce == Vector3.zero && r.CurrentLowerKneeForce == Vector3.zero &&
                r.CurrentLowerAnkleForce == Vector3.zero && r.CurrentFootForce == Vector3.zero && r.CurrentGravity == Vector3.zero;
            KneeSupportNormalReactionReport knee = leg.KneeSupportReaction;
            if (knee == null || !Mass(knee.CurrentUpperMass) || !Mass(leg.StanceLowerNativeGravityLowerMass) ||
                !Mass(foot.StanceFootNativeGravityFootMass) || !V(knee.CurrentNormalWorld) ||
                !V(leg.StanceLowerNativeGravityGravityWorld) || !V(foot.StanceFootNativeGravityGravityWorld) || !Mass(r.CurrentUpperMass) || !Mass(r.CurrentLowerMass) || !Mass(r.CurrentFootMass) ||
                !V(r.CurrentNormal) || Mathf.Abs(r.CurrentNormal.sqrMagnitude - 1f) > 0.0001f ||
                !F(r.CurrentExpectedImpulse) || r.CurrentExpectedImpulse <= 0f ||
                Mathf.Abs(r.CurrentUpperMass - knee.CurrentUpperMass) > 0.0001f ||
                Mathf.Abs(r.CurrentLowerMass - leg.StanceLowerNativeGravityLowerMass) > 0.0001f ||
                Mathf.Abs(r.CurrentFootMass - foot.StanceFootNativeGravityFootMass) > 0.0001f ||
                (r.CurrentNormal - knee.CurrentNormalWorld).magnitude > 0.0001f ||
                (r.CurrentGravity - leg.StanceLowerNativeGravityGravityWorld).magnitude > 0.0001f ||
                (r.CurrentGravity - foot.StanceFootNativeGravityGravityWorld).magnitude > 0.0001f) return false;
            Vector3 force = -r.CurrentUpperMass * r.CurrentGravity;
            return (r.CurrentUpperForce - force).magnitude <= 0.0001f &&
                (r.CurrentLowerKneeForce + force).magnitude <= 0.0001f &&
                (r.CurrentLowerAnkleForce - force).magnitude <= 0.0001f &&
                (r.CurrentFootForce + force).magnitude <= 0.0001f;
        }

        internal static bool SelectorContract(UpperSelfWeightSupportReport r, string arm, bool completed) =>
            r != null && r.Finite && r.Selected && IsExperimentArm(arm) && r.Arm == arm && r.Enabled == (arm == ArmB) &&
            r.ReadbackFailures == 0 && r.RollbackFailures == 0 && r.GravityReadback == r.CurrentApplied &&
            (!r.CurrentApplied || (r.Enabled && r.CurrentEligible && r.CurrentFixed > 0)) &&
            (!completed || (!r.GravityReadback && !r.CurrentApplied));
        internal static bool ExposureReady(UpperSelfWeightSupportReport r, string arm) =>
            r != null && r.Eligible >= MinimumEligibleSamples && r.NonStance >= MinimumNonStanceSamples &&
            (arm == ArmA ? r.Writes == 0 : r.Writes == r.Eligible) && r.LedgerFinite && r.LedgerLoaded >= MinimumEligibleSamples;
        private static bool Mass(float x) => F(x) && x > 0.000001f;
        private static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool V(Vector3 x) => F(x.x) && F(x.y) && F(x.z);
    }
}
