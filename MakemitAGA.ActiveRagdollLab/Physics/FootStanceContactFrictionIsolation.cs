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
                _finite = ValidateReadback(collider, arm, out error);
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
            bool guaranteed = ground == null || ground.frictionCombine != PhysicMaterialCombine.Maximum ||
                              (Mathf.Abs(ground.staticFriction) <= 0.0001f && Mathf.Abs(ground.dynamicFriction) <= 0.0001f);
            CopyGroundToReport(groundCollider, guaranteed, report);
            if (!guaranteed)
            {
                error = "GROUND_MAXIMUM_FRICTION_COMBINE_PREVENTS_ZERO_EFFECTIVE_FRICTION";
                Invalidate(error, report);
                return false;
            }
            return true;
        }

        internal void RestoreAndDispose(BoxCollider collider, FootAnkleFoundationReport report)
        {
            try
            {
                if (collider != null && _baselineCaptured)
                    collider.sharedMaterial = _baselineMaterial;
            }
            catch { }

            try
            {
                if (_zeroFrictionMaterial != null)
                    Object.Destroy(_zeroFrictionMaterial);
            }
            catch { }
            _zeroFrictionMaterial = null;
            _arm = ArmA;
            CopyToReport(collider, report);
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

        private bool ValidateReadback(BoxCollider collider, string arm, out string error)
        {
            error = null;
            PhysicMaterial current = collider.sharedMaterial;
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
            report.StanceContactFrictionIsolationMode = Mode;
            report.StanceContactFrictionIsolationArm = _arm;
            report.StanceContactFrictionIsolationFinite = _finite;
            report.StanceContactFrictionIsolationInvalidReason = _invalidReason;
            report.StanceContactFrictionIsolationZeroFriction = ZeroFriction;
            report.StanceContactFrictionIsolationMaterialAssigned = current != null;
            report.StanceContactFrictionIsolationStaticFriction = current != null ? current.staticFriction : 0f;
            report.StanceContactFrictionIsolationDynamicFriction = current != null ? current.dynamicFriction : 0f;
            report.StanceContactFrictionIsolationFrictionCombine = current != null ? current.frictionCombine.ToString() : "DEFAULT_NULL";
            report.StanceContactFrictionIsolationSummary =
                "mode=" + Mode + "/arm=" + _arm + "/finite=" + _finite +
                "/zero=" + ZeroFriction + "/assigned=" + (current != null) +
                "/static=" + F(report.StanceContactFrictionIsolationStaticFriction) +
                "/dynamic=" + F(report.StanceContactFrictionIsolationDynamicFriction) +
                "/combine=" + report.StanceContactFrictionIsolationFrictionCombine +
                "/groundAssigned=" + report.StanceContactFrictionIsolationGroundMaterialAssigned +
                "/groundStatic=" + F(report.StanceContactFrictionIsolationGroundStaticFriction) +
                "/groundDynamic=" + F(report.StanceContactFrictionIsolationGroundDynamicFriction) +
                "/groundCombine=" + (report.StanceContactFrictionIsolationGroundFrictionCombine ?? "<none>") +
                "/effectiveZeroGuaranteed=" + report.StanceContactFrictionIsolationEffectiveZeroGuaranteed +
                "/reason=" + (string.IsNullOrEmpty(_invalidReason) ? "<none>" : _invalidReason);
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
