using System;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.17 production Single-Leg mass-property foundation.
    ///
    /// v0.2.12-v0.2.16 已完成 Unit/Box、COM/inertia、Upper/Lower 的归因实验。
    /// 结论已经冻结：运行时创建的腿部 Rigidbody 默认 inertiaTensor=(1,1,1) 不是可接受的正式物理基础；
    /// 正式主线必须使用与 ARL BoxCollider 几何一致的显式 uniform-box COM / inertia tensor。
    ///
    /// 历史 A-F build arm、Unity ResetInertiaTensor probe、运行中 mutation 都已从活动代码删除。
    /// 如未来需要复核，使用 v0.2.12-v0.2.16 归档源码，而不是把实验分支重新塞回正式控制路径。
    /// </summary>
    internal sealed class SingleLegRigidbodyMassProperties
    {
        public const string FoundationMode = "EXPLICIT_UNIFORM_BOX_MASS_PROPERTY_FOUNDATION_VALIDATED";
        public const string ValidationBasis = "ANALYTICAL_UNIFORM_BOX_COM_AND_TENSOR";

        private const float TensorEpsilon = 0.000001f;
        private const float RelativeTolerance = 0.0025f;
        private const float AbsoluteTolerance = 0.00005f;
        private const float QuaternionToleranceDegrees = 0.05f;

        internal sealed class Snapshot
        {
            public string Label;
            public float Mass;
            public bool ColliderEnabled;
            public Vector3 ColliderCenterLocal;
            public Vector3 ColliderSizeLocal;
            public Vector3 LocalScale;
            public Vector3 LossyScale;
            public Vector3 PhysicalBoxSize;
            public Vector3 RuntimeCenterOfMassLocal;
            public Vector3 RuntimeWorldCenterOfMass;
            public Vector3 RuntimeInertiaTensor;
            public Quaternion RuntimeInertiaTensorRotation;
            public Vector3 ExpectedCenterOfMassLocal;
            public Vector3 ExpectedUniformBoxInertiaTensor;
            public Quaternion ExpectedInertiaTensorRotation;
            public float RuntimeAxisEffectiveInertia;
            public float ExpectedAxisEffectiveInertia;
            public bool RuntimeUnitTensorSuspect;
            public bool CenterMatchesExpected;
            public bool TensorMatchesExpected;
            public bool TensorRotationMatchesExpected;
            public bool FinitePositive;

            public bool MatchesFoundation =>
                FinitePositive &&
                !RuntimeUnitTensorSuspect &&
                CenterMatchesExpected &&
                TensorMatchesExpected &&
                TensorRotationMatchesExpected;
        }

        private Snapshot _upper;
        private Snapshot _lower;
        private bool _auditCompleted;
        private bool _buildFoundationValidated;
        private string _lastAction = "UNINITIALIZED";
        private string _lastError;
        private float _runtimeLowerSplitWeight;
        private float _runtimeUpperSplitWeight;
        private float _expectedLowerSplitWeight;
        private float _expectedUpperSplitWeight;

        public bool AuditCompleted => _auditCompleted;
        public bool BuildFoundationValidated => _buildFoundationValidated;
        public string LastAction => _lastAction;
        public string LastError => _lastError;
        public Snapshot Upper => _upper;
        public Snapshot Lower => _lower;

        public bool ApplyValidatedBuildFoundation(
            Rigidbody upperBody,
            BoxCollider upperCollider,
            Rigidbody lowerBody,
            BoxCollider lowerCollider,
            Vector3 kneeAxisWorld)
        {
            _buildFoundationValidated = false;
            _auditCompleted = false;
            _lastAction = "BUILD_APPLY_EXPLICIT_UNIFORM_BOX_MASS_PROPERTIES";
            _lastError = null;

            if (!ValidateInputs(upperBody, upperCollider, lowerBody, lowerCollider, kneeAxisWorld, out string error))
            {
                _lastError = error;
                return false;
            }

            try
            {
                ApplyUniformBox(upperBody, upperCollider);
                ApplyUniformBox(lowerBody, lowerCollider);
            }
            catch (Exception e)
            {
                _lastError = "APPLY_EXCEPTION:" + e.GetType().Name + "/" + e.Message;
                return false;
            }

            if (!RefreshReadOnlyStatus(upperBody, upperCollider, lowerBody, lowerCollider, kneeAxisWorld))
                return false;

            if (_upper == null || _lower == null || !_upper.MatchesFoundation || !_lower.MatchesFoundation)
            {
                _lastError = "READBACK_MISMATCH";
                return false;
            }

            _buildFoundationValidated = true;
            return true;
        }

        public bool RefreshReadOnlyStatus(
            Rigidbody upperBody,
            BoxCollider upperCollider,
            Rigidbody lowerBody,
            BoxCollider lowerCollider,
            Vector3 kneeAxisWorld)
        {
            _auditCompleted = false;
            _lastError = null;

            if (!ValidateInputs(upperBody, upperCollider, lowerBody, lowerCollider, kneeAxisWorld, out string error))
            {
                _lastError = error;
                return false;
            }

            _upper = Capture("Upper", upperBody, upperCollider, kneeAxisWorld);
            _lower = Capture("Lower", lowerBody, lowerCollider, kneeAxisWorld);
            _auditCompleted = _upper != null && _lower != null;
            if (!_auditCompleted)
            {
                _lastError = "SNAPSHOT_FAILED";
                return false;
            }

            ComputeSplits();
            return true;
        }

        public string GetConsoleSummary()
        {
            return
                "mode=" + FoundationMode +
                " | audit=" + _auditCompleted +
                " | buildValidated=" + _buildFoundationValidated +
                " | basis=" + ValidationBasis +
                " | action=" + _lastAction +
                " | upper[" + Compact(_upper) + "]" +
                " | lower[" + Compact(_lower) + "]" +
                " | splitRuntime(L/U)=" + F(_runtimeLowerSplitWeight) + "/" + F(_runtimeUpperSplitWeight) +
                " | splitExpected(L/U)=" + F(_expectedLowerSplitWeight) + "/" + F(_expectedUpperSplitWeight) +
                " | error=" + (_lastError ?? "<none>");
        }

        public string GetDetailedReport()
        {
            StringBuilder sb = new StringBuilder(2048);
            sb.AppendLine("mode=" + FoundationMode);
            sb.AppendLine("validationBasis=" + ValidationBasis);
            sb.AppendLine("auditCompleted=" + _auditCompleted);
            sb.AppendLine("buildFoundationValidated=" + _buildFoundationValidated);
            sb.AppendLine("lastAction=" + _lastAction);
            sb.AppendLine("lastError=" + (_lastError ?? "<none>"));
            AppendSnapshot(sb, _upper);
            AppendSnapshot(sb, _lower);
            sb.AppendLine("splitRuntimeLower=" + F(_runtimeLowerSplitWeight));
            sb.AppendLine("splitRuntimeUpper=" + F(_runtimeUpperSplitWeight));
            sb.AppendLine("splitExpectedLower=" + F(_expectedLowerSplitWeight));
            sb.AppendLine("splitExpectedUpper=" + F(_expectedUpperSplitWeight));
            return sb.ToString();
        }

        private static void ApplyUniformBox(Rigidbody body, BoxCollider collider)
        {
            Vector3 physical = GetPhysicalBoxSize(collider);
            Vector3 tensor = CalculateUniformBoxTensor(body.mass, physical);
            if (!IsFinite(tensor) || tensor.x <= TensorEpsilon || tensor.y <= TensorEpsilon || tensor.z <= TensorEpsilon)
                throw new InvalidOperationException("INVALID_ANALYTICAL_BOX_TENSOR");

            body.centerOfMass = collider.center;
            body.inertiaTensorRotation = Quaternion.identity;
            body.inertiaTensor = tensor;
        }

        private static Snapshot Capture(string label, Rigidbody body, BoxCollider collider, Vector3 axisWorld)
        {
            Vector3 physical = GetPhysicalBoxSize(collider);
            Vector3 expectedTensor = CalculateUniformBoxTensor(body.mass, physical);
            Vector3 runtimeTensor = body.inertiaTensor;
            Quaternion runtimeTensorRotation = body.inertiaTensorRotation;
            Vector3 expectedCom = collider.center;

            Snapshot s = new Snapshot
            {
                Label = label,
                Mass = body.mass,
                ColliderEnabled = collider.enabled,
                ColliderCenterLocal = collider.center,
                ColliderSizeLocal = collider.size,
                LocalScale = collider.transform.localScale,
                LossyScale = collider.transform.lossyScale,
                PhysicalBoxSize = physical,
                RuntimeCenterOfMassLocal = body.centerOfMass,
                RuntimeWorldCenterOfMass = body.worldCenterOfMass,
                RuntimeInertiaTensor = runtimeTensor,
                RuntimeInertiaTensorRotation = runtimeTensorRotation,
                ExpectedCenterOfMassLocal = expectedCom,
                ExpectedUniformBoxInertiaTensor = expectedTensor,
                ExpectedInertiaTensorRotation = Quaternion.identity,
                RuntimeAxisEffectiveInertia = CalculateAxisEffectiveInertia(body.rotation, runtimeTensorRotation, runtimeTensor, axisWorld),
                ExpectedAxisEffectiveInertia = CalculateAxisEffectiveInertia(body.rotation, Quaternion.identity, expectedTensor, axisWorld),
                RuntimeUnitTensorSuspect = Approximately(runtimeTensor.x, 1f) && Approximately(runtimeTensor.y, 1f) && Approximately(runtimeTensor.z, 1f),
                CenterMatchesExpected = Approximately(body.centerOfMass, expectedCom),
                TensorMatchesExpected = Approximately(runtimeTensor, expectedTensor),
                TensorRotationMatchesExpected = Quaternion.Angle(runtimeTensorRotation, Quaternion.identity) <= QuaternionToleranceDegrees
            };
            s.FinitePositive =
                IsFinite(s.Mass) && s.Mass > 0f &&
                IsFinite(s.RuntimeCenterOfMassLocal) &&
                IsFinite(s.RuntimeInertiaTensor) &&
                s.RuntimeInertiaTensor.x > TensorEpsilon && s.RuntimeInertiaTensor.y > TensorEpsilon && s.RuntimeInertiaTensor.z > TensorEpsilon &&
                IsFinite(s.RuntimeAxisEffectiveInertia) && s.RuntimeAxisEffectiveInertia > TensorEpsilon;
            return s;
        }

        private void ComputeSplits()
        {
            _runtimeLowerSplitWeight = 0f;
            _runtimeUpperSplitWeight = 0f;
            _expectedLowerSplitWeight = 0f;
            _expectedUpperSplitWeight = 0f;
            if (_upper == null || _lower == null)
                return;

            float runtimeSum = _upper.RuntimeAxisEffectiveInertia + _lower.RuntimeAxisEffectiveInertia;
            if (IsFinite(runtimeSum) && runtimeSum > TensorEpsilon)
            {
                _runtimeLowerSplitWeight = _upper.RuntimeAxisEffectiveInertia / runtimeSum;
                _runtimeUpperSplitWeight = _lower.RuntimeAxisEffectiveInertia / runtimeSum;
            }

            float expectedSum = _upper.ExpectedAxisEffectiveInertia + _lower.ExpectedAxisEffectiveInertia;
            if (IsFinite(expectedSum) && expectedSum > TensorEpsilon)
            {
                _expectedLowerSplitWeight = _upper.ExpectedAxisEffectiveInertia / expectedSum;
                _expectedUpperSplitWeight = _lower.ExpectedAxisEffectiveInertia / expectedSum;
            }
        }

        private static bool ValidateInputs(
            Rigidbody upperBody,
            BoxCollider upperCollider,
            Rigidbody lowerBody,
            BoxCollider lowerCollider,
            Vector3 kneeAxisWorld,
            out string error)
        {
            error = null;
            if (upperBody == null || lowerBody == null || upperCollider == null || lowerCollider == null)
            {
                error = "MISSING_RIGIDBODY_OR_BOXCOLLIDER";
                return false;
            }
            if (!IsFinite(kneeAxisWorld) || kneeAxisWorld.sqrMagnitude < 0.0001f)
            {
                error = "INVALID_KNEE_AXIS";
                return false;
            }
            if (!IsFinite(upperBody.mass) || !IsFinite(lowerBody.mass) || upperBody.mass <= 0f || lowerBody.mass <= 0f)
            {
                error = "INVALID_MASS";
                return false;
            }
            return true;
        }

        private static Vector3 GetPhysicalBoxSize(BoxCollider collider)
        {
            Vector3 size = collider.size;
            Vector3 scale = collider.transform.lossyScale;
            return new Vector3(
                Mathf.Abs(size.x * scale.x),
                Mathf.Abs(size.y * scale.y),
                Mathf.Abs(size.z * scale.z));
        }

        private static Vector3 CalculateUniformBoxTensor(float mass, Vector3 size)
        {
            float x2 = size.x * size.x;
            float y2 = size.y * size.y;
            float z2 = size.z * size.z;
            float k = mass / 12f;
            return new Vector3(
                k * (y2 + z2),
                k * (x2 + z2),
                k * (x2 + y2));
        }

        private static float CalculateAxisEffectiveInertia(
            Quaternion bodyRotation,
            Quaternion tensorRotation,
            Vector3 tensor,
            Vector3 axisWorld)
        {
            if (!IsFinite(axisWorld) || axisWorld.sqrMagnitude < 0.0001f ||
                !IsFinite(tensor) || tensor.x <= TensorEpsilon || tensor.y <= TensorEpsilon || tensor.z <= TensorEpsilon)
                return float.NaN;

            Quaternion principalWorld = bodyRotation * tensorRotation;
            Vector3 n = Quaternion.Inverse(principalWorld) * axisWorld.normalized;
            float inv = n.x * n.x / tensor.x + n.y * n.y / tensor.y + n.z * n.z / tensor.z;
            if (!IsFinite(inv) || inv <= TensorEpsilon)
                return float.NaN;
            return 1f / inv;
        }

        private static string Compact(Snapshot s)
        {
            if (s == null) return "<none>";
            return
                "m=" + F(s.Mass) +
                ",collider=" + V(s.ColliderSizeLocal) +
                ",physical=" + V(s.PhysicalBoxSize) +
                ",com=" + V(s.RuntimeCenterOfMassLocal) +
                ",I=" + V(s.RuntimeInertiaTensor) +
                ",Ieff=" + F(s.RuntimeAxisEffectiveInertia) +
                ",unit=" + s.RuntimeUnitTensorSuspect +
                ",comMatch=" + s.CenterMatchesExpected +
                ",tensorMatch=" + s.TensorMatchesExpected +
                ",rotMatch=" + s.TensorRotationMatchesExpected +
                ",foundationMatch=" + s.MatchesFoundation;
        }

        private static void AppendSnapshot(StringBuilder sb, Snapshot s)
        {
            if (s == null)
            {
                sb.AppendLine("snapshot=<none>");
                return;
            }
            string p = s.Label ?? "Body";
            sb.AppendLine(p + ".mass=" + F(s.Mass));
            sb.AppendLine(p + ".colliderEnabled=" + s.ColliderEnabled);
            sb.AppendLine(p + ".colliderCenter=" + V(s.ColliderCenterLocal));
            sb.AppendLine(p + ".colliderSize=" + V(s.ColliderSizeLocal));
            sb.AppendLine(p + ".physicalBoxSize=" + V(s.PhysicalBoxSize));
            sb.AppendLine(p + ".runtimeCOM=" + V(s.RuntimeCenterOfMassLocal));
            sb.AppendLine(p + ".expectedCOM=" + V(s.ExpectedCenterOfMassLocal));
            sb.AppendLine(p + ".runtimeTensor=" + V(s.RuntimeInertiaTensor));
            sb.AppendLine(p + ".expectedTensor=" + V(s.ExpectedUniformBoxInertiaTensor));
            sb.AppendLine(p + ".runtimeIeff=" + F(s.RuntimeAxisEffectiveInertia));
            sb.AppendLine(p + ".expectedIeff=" + F(s.ExpectedAxisEffectiveInertia));
            sb.AppendLine(p + ".unitTensorSuspect=" + s.RuntimeUnitTensorSuspect);
            sb.AppendLine(p + ".centerMatches=" + s.CenterMatchesExpected);
            sb.AppendLine(p + ".tensorMatches=" + s.TensorMatchesExpected);
            sb.AppendLine(p + ".tensorRotationMatches=" + s.TensorRotationMatchesExpected);
            sb.AppendLine(p + ".foundationMatches=" + s.MatchesFoundation);
        }

        private static bool Approximately(float a, float b)
        {
            float tolerance = Math.Max(AbsoluteTolerance, Math.Max(Math.Abs(a), Math.Abs(b)) * RelativeTolerance);
            return Math.Abs(a - b) <= tolerance;
        }

        private static bool Approximately(Vector3 a, Vector3 b)
        {
            return Approximately(a.x, b.x) && Approximately(a.y, b.y) && Approximately(a.z, b.z);
        }

        private static bool IsFinite(float value) => !(float.IsNaN(value) || float.IsInfinity(value));
        private static bool IsFinite(Vector3 value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        private static bool IsFinite(Quaternion value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);
        private static string F(float value) => value.ToString("G6", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
