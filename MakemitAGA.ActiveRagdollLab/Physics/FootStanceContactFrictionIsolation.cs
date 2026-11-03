using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.8 stance-contact tangential/yaw constraint isolation.
    ///
    /// Both experimental arms reuse the exact same v0.4.7 per-foot STANCE contact lifecycle. The only
    /// causal variable is the PhysicMaterial assigned to the Foot BoxCollider while that lifecycle runs:
    /// A restores the collider's original/default material, while B assigns an ARL-owned zero-friction
    /// material (static/dynamic friction 0, Minimum combine). Gravity, gait thresholds, contact identity,
    /// contact timing, Foot/Knee actuators and all hard gates are unchanged.
    /// </summary>
    internal sealed class FootStanceContactFrictionIsolation
    {
        internal const string Mode = "STANCE_CONTACT_TANGENTIAL_YAW_CONSTRAINT_ISOLATION_V0480";
        internal const string ArmA = "A_V0480_DEFAULT_FRICTION_STANCE_CONTACT";
        internal const string ArmB = "B_V0480_ZERO_FRICTION_STANCE_CONTACT";

        private PhysicMaterial _baselineMaterial;
        private bool _baselineCaptured;
        private PhysicMaterial _zeroFrictionMaterial;
        private string _arm = ArmA;
        private bool _finite = true;
        private string _invalidReason;

        internal string Arm => _arm;
        internal bool Finite => _finite;
        internal bool ZeroFriction => string.Equals(_arm, ArmB, StringComparison.Ordinal);

        internal bool SetArm(string arm, BoxCollider collider, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (collider == null)
            {
                error = "FOOT_COLLIDER_MISSING";
                Invalidate(error, report);
                return false;
            }
            if (!string.Equals(arm, ArmA, StringComparison.Ordinal) && !string.Equals(arm, ArmB, StringComparison.Ordinal))
            {
                error = "UNKNOWN_FRICTION_ISOLATION_ARM:" + (arm ?? "<null>");
                Invalidate(error, report);
                return false;
            }

            try
            {
                CaptureBaseline(collider);
                if (string.Equals(arm, ArmB, StringComparison.Ordinal))
                {
                    EnsureZeroFrictionMaterial(collider);
                    collider.sharedMaterial = _zeroFrictionMaterial;
                }
                else
                {
                    collider.sharedMaterial = _baselineMaterial;
                }

                _arm = arm;
                if (report != null)
                {
                    report.StanceContactFrictionIsolationGroundMaterialAssigned = false;
                    report.StanceContactFrictionIsolationGroundStaticFriction = 0f;
                    report.StanceContactFrictionIsolationGroundDynamicFriction = 0f;
                    report.StanceContactFrictionIsolationGroundFrictionCombine = "<not-checked>";
                    report.StanceContactFrictionIsolationEffectiveZeroGuaranteed = !string.Equals(arm, ArmB, StringComparison.Ordinal);
                }
                _finite = ValidateReadback(collider.sharedMaterial, arm, out error);
                _invalidReason = _finite ? null : error;
                CopyToReport(collider, report);
                return _finite;
            }
            catch (Exception e)
            {
                error = "FRICTION_ISOLATION_EXCEPTION:" + e.GetType().Name + ":" + e.Message;
                Invalidate(error, report);
                return false;
            }
        }

        internal bool ValidateGroundCompatibility(Collider groundCollider, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (!ZeroFriction)
            {
                CopyGroundToReport(groundCollider, true, report);
                return true;
            }
            if (groundCollider == null)
            {
                CopyGroundToReport(null, false, report);
                error = "ZERO_FRICTION_AUTHORIZED_GROUND_MISSING";
                Invalidate(error, report);
                return false;
            }

            PhysicMaterial ground = groundCollider.sharedMaterial;
            bool finite = ground == null || (IsFinite(ground.staticFriction) && IsFinite(ground.dynamicFriction));
            bool guaranteed = finite && (ground == null || ground.frictionCombine != PhysicMaterialCombine.Maximum ||
                              (Mathf.Abs(ground.staticFriction) <= 0.0001f && Mathf.Abs(ground.dynamicFriction) <= 0.0001f));
            CopyGroundToReport(groundCollider, guaranteed, report);
            if (!guaranteed)
            {
                error = finite ? "GROUND_MAXIMUM_FRICTION_COMBINE_PREVENTS_ZERO_EFFECTIVE_FRICTION" : "GROUND_FRICTION_READBACK_NONFINITE";
                Invalidate(error, report);
                return false;
            }
            return true;
        }

        internal int RestoreAndDispose(BoxCollider collider, FootAnkleFoundationReport report)
        {
            int failures = 0;
            try
            {
                if (collider != null && _baselineCaptured)
                {
                    collider.sharedMaterial = _baselineMaterial;
                    if (collider.sharedMaterial != _baselineMaterial)
                        throw new InvalidOperationException("DEFAULT_MATERIAL_RESTORE_READBACK_MISMATCH");
                }
                _arm = ArmA;
            }
            catch { failures++; }

            try
            {
                if (_zeroFrictionMaterial != null)
                    Object.Destroy(_zeroFrictionMaterial);
                _zeroFrictionMaterial = null;
            }
            catch { failures++; } // Preserve the owned handle for a later rollback attempt.
            if (failures > 0)
            {
                _finite = false;
                if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = "FRICTION_RESTORE_OR_DISPOSE_FAILED";
            }
            try { CopyToReport(collider, report); }
            catch { failures++; }
            return failures;
        }

        private void CaptureBaseline(BoxCollider collider)
        {
            if (_baselineCaptured) return;
            _baselineMaterial = collider.sharedMaterial;
            _baselineCaptured = true;
        }

        private void EnsureZeroFrictionMaterial(BoxCollider collider)
        {
            if (_zeroFrictionMaterial == null)
            {
                // Unity automatically instantiates a private runtime PhysicMaterial when Collider.material
                // is accessed.  This is IL2CPP-safe and avoids the managed PhysicMaterial(string)
                // constructor, which is not exposed by MiSide's generated interop wrapper.
                _zeroFrictionMaterial = collider.material;
                if (_zeroFrictionMaterial == null)
                    throw new InvalidOperationException("COLLIDER_MATERIAL_RUNTIME_INSTANCE_UNAVAILABLE");
                _zeroFrictionMaterial.name = "ARL_v0.4.8_ZeroFrictionFoot";
            }

            _zeroFrictionMaterial.staticFriction = 0f;
            _zeroFrictionMaterial.dynamicFriction = 0f;
            _zeroFrictionMaterial.bounciness = 0f;
            _zeroFrictionMaterial.frictionCombine = PhysicMaterialCombine.Minimum;
            _zeroFrictionMaterial.bounceCombine = PhysicMaterialCombine.Minimum;
        }

        private bool ValidateReadback(PhysicMaterial current, string arm, out string error)
        {
            error = null;
            if (string.Equals(arm, ArmA, StringComparison.Ordinal))
            {
                if (current != _baselineMaterial)
                {
                    error = "DEFAULT_MATERIAL_READBACK_MISMATCH";
                    return false;
                }
                return true;
            }

            if (current == null)
            {
                error = "ZERO_FRICTION_MATERIAL_READBACK_NULL";
                return false;
            }
            if (!IsFinite(current.staticFriction) || !IsFinite(current.dynamicFriction))
            {
                error = "ZERO_FRICTION_READBACK_NONFINITE";
                return false;
            }
            if (Mathf.Abs(current.staticFriction) > 0.0001f || Mathf.Abs(current.dynamicFriction) > 0.0001f)
            {
                error = "ZERO_FRICTION_READBACK_NONZERO:static=" + F(current.staticFriction) + ":dynamic=" + F(current.dynamicFriction);
                return false;
            }
            if (current.frictionCombine != PhysicMaterialCombine.Minimum)
            {
                error = "ZERO_FRICTION_COMBINE_READBACK_MISMATCH:" + current.frictionCombine;
                return false;
            }
            return true;
        }

        private static void CopyGroundToReport(Collider groundCollider, bool guaranteed, FootAnkleFoundationReport report)
        {
            if (report == null) return;
            PhysicMaterial ground = groundCollider != null ? groundCollider.sharedMaterial : null;
            report.StanceContactFrictionIsolationGroundMaterialAssigned = ground != null;
            report.StanceContactFrictionIsolationGroundStaticFriction = ground != null ? ground.staticFriction : 0f;
            report.StanceContactFrictionIsolationGroundDynamicFriction = ground != null ? ground.dynamicFriction : 0f;
            report.StanceContactFrictionIsolationGroundFrictionCombine = ground != null ? ground.frictionCombine.ToString() : "DEFAULT_NULL";
            report.StanceContactFrictionIsolationEffectiveZeroGuaranteed = guaranteed;
        }

        private void Invalidate(string reason, FootAnkleFoundationReport report)
        {
            _finite = false;
            if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = reason;
            CopyToReport(null, report);
        }

        internal void CopyToReport(BoxCollider collider, FootAnkleFoundationReport report)
        {
            if (report == null) return;
            PhysicMaterial current = collider != null ? collider.sharedMaterial : null;
            // Arm selection is only the request. Keep validating the native material that
            // contact will actually use, just like the collision-mode/offset owners do.
            if (collider != null && _baselineCaptured && !ValidateReadback(current, _arm, out string readbackError))
            {
                _finite = false;
                if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = readbackError;
            }
            report.StanceContactFrictionIsolationMode = Mode;
            report.StanceContactFrictionIsolationArm = _arm;
            report.StanceContactFrictionIsolationFinite = _finite;
            report.StanceContactFrictionIsolationInvalidReason = _invalidReason;
            report.StanceContactFrictionIsolationZeroFriction = ZeroFriction;
            report.StanceContactFrictionIsolationMaterialAssigned = current != null;
            report.StanceContactFrictionIsolationStaticFriction = current != null ? current.staticFriction : 0f;
            report.StanceContactFrictionIsolationDynamicFriction = current != null ? current.dynamicFriction : 0f;
            report.StanceContactFrictionIsolationFrictionCombine = current != null ? current.frictionCombine.ToString() : "DEFAULT_NULL";
            report.FrictionSummaryGroundAssigned = report.StanceContactFrictionIsolationGroundMaterialAssigned;
            report.FrictionSummaryGroundStatic = report.StanceContactFrictionIsolationGroundStaticFriction;
            report.FrictionSummaryGroundDynamic = report.StanceContactFrictionIsolationGroundDynamicFriction;
            report.FrictionSummaryGroundCombine = report.StanceContactFrictionIsolationGroundFrictionCombine;
            report.FrictionSummaryEffectiveZero = report.StanceContactFrictionIsolationEffectiveZeroGuaranteed;
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
