using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.2：只从源动画学习 Knee 主旋转轴。
    ///
    /// 关键约束：
    /// 1) 只在 NORMAL_STEADY、非 SourceTurn、非 Recovery 的真实 Render/LateUpdate 样本中收集；
    /// 2) 只读取 source knee relative angular velocity，不读取 proxy torque/error，因此不会把当前错误 flexion axis 反馈进标定；
    /// 3) 在 source LowerLeg local frame 中累计 Σ(w w^T)，用最大主特征向量得到符号无关的主旋转轴；
    /// 4) 标定期间 Single-Leg 必须保持 kinematic preview，禁止动态改 Knee CharacterJoint flexion axis；
    /// 5) 只有显式 apply 且 Leg Drive=OFF 时才把候选轴写入 ARL 自有 Knee CharacterJoint flexion axis；源角色永远只读。
    /// </summary>
    internal sealed class KneeFlexionAxisCalibrator
    {
        public const string ModeName = "DATA_DRIVEN_KNEE_FLEXION_AXIS_PCA";
        public const int MinAcceptedSamples = 250;
        public const float MinRelativeAngularSpeed = 0.25f;
        public const float MinPrincipalEnergyRatio = 0.80f;

        private bool _collecting;
        private int _accepted;
        private int _rejectedSourceWindow;
        private int _rejectedNonSteady;
        private int _rejectedLowSpeed;
        private int _rejectedInvalid;

        // Symmetric scatter matrix Σ(w w^T), lower-leg local coordinates.
        private double _xx;
        private double _xy;
        private double _xz;
        private double _yy;
        private double _yz;
        private double _zz;

        private Vector3 _initialAxisLowerLocal = Vector3.right;
        private Vector3 _candidateAxisLowerLocal = Vector3.right;
        private float _principalEnergyRatio;
        private float _principalRmsProjectionRatio;
        private float _angleFromInitialDegrees;
        private bool _ready;
        private bool _applied;
        private string _reason = "NOT_STARTED";
        private float _sourceRejectUntilTime;

        public bool Collecting => _collecting;
        public bool Ready => _ready;
        public bool Applied => _applied;
        public int AcceptedSamples => _accepted;
        public Vector3 CandidateAxisLowerLocal => _candidateAxisLowerLocal;
        public float PrincipalEnergyRatio => _principalEnergyRatio;
        public float AngleFromInitialDegrees => _angleFromInitialDegrees;
        public string Reason => _reason;

        public void MarkApplied()
        {
            if (_ready && !_collecting)
            {
                _applied = true;
                _reason = "APPLIED";
            }
        }

        public void Begin(Vector3 initialAxisLowerLocal)
        {
            ResetAccumulators(initialAxisLowerLocal);
            _collecting = true;
            _reason = "COLLECTING";
        }

        public void Stop()
        {
            _collecting = false;
            RecomputeCandidate();
            UpdateReadiness();
        }

        public void Reset(Vector3 initialAxisLowerLocal)
        {
            ResetAccumulators(initialAxisLowerLocal);
            _collecting = false;
            _reason = "RESET";
        }

        public void Observe(
            Vector3 kneeRelativeAngularVelocityLowerLocal,
            TwoBodyProxyReport torso,
            bool sourceTurnLikeTrigger,
            float sourceTurnPostSeconds)
        {
            if (!_collecting || torso == null)
                return;

            float now = Time.unscaledTime;
            if (sourceTurnLikeTrigger)
                _sourceRejectUntilTime = Mathf.Max(_sourceRejectUntilTime, now + Mathf.Max(0.05f, sourceTurnPostSeconds));

            // Works both with and without arl_trace: use the torso event when available, plus a local raw-trigger quiet window.
            if (torso.SourceTurnEventActive || torso.ControlRecoveryActive || now <= _sourceRejectUntilTime)
            {
                _rejectedSourceWindow++;
                return;
            }

            if (torso.MotionTransition || !string.Equals(torso.MotionPhase, "STEADY", StringComparison.OrdinalIgnoreCase))
            {
                _rejectedNonSteady++;
                return;
            }

            if (!IsFinite(kneeRelativeAngularVelocityLowerLocal))
            {
                _rejectedInvalid++;
                return;
            }

            float speed = kneeRelativeAngularVelocityLowerLocal.magnitude;
            if (!IsFinite(speed) || speed < MinRelativeAngularSpeed)
            {
                _rejectedLowSpeed++;
                return;
            }

            double x = kneeRelativeAngularVelocityLowerLocal.x;
            double y = kneeRelativeAngularVelocityLowerLocal.y;
            double z = kneeRelativeAngularVelocityLowerLocal.z;
            _xx += x * x;
            _xy += x * y;
            _xz += x * z;
            _yy += y * y;
            _yz += y * z;
            _zz += z * z;
            _accepted++;

            // 只周期性重算，避免每个 Render sample 做不必要的迭代；Stop/Status 前还会再算一次。
            if ((_accepted & 31) == 0)
            {
                RecomputeCandidate();
                UpdateReadiness();
            }
        }

        public string GetStatusText()
        {
            RecomputeCandidate();
            UpdateReadiness();
            return
                "mode=" + ModeName +
                " | collecting=" + _collecting +
                " | ready=" + _ready +
                " | reason=" + (_reason ?? "<none>") +
                " | samples=" + _accepted + "/" + MinAcceptedSamples +
                " | candidateLowerLocal=" + V(_candidateAxisLowerLocal) +
                " | principalEnergy=" + F(_principalEnergyRatio) +
                " | principalRmsProjection=" + F(_principalRmsProjectionRatio) +
                " | angleFromInitialDeg=" + F(_angleFromInitialDegrees) +
                " | rejected(source/nonSteady/lowSpeed/invalid)=" +
                _rejectedSourceWindow + "/" + _rejectedNonSteady + "/" + _rejectedLowSpeed + "/" + _rejectedInvalid +
                " | thresholds(samples/speed/energy)=" + MinAcceptedSamples + "/" + F(MinRelativeAngularSpeed) + "/" + F(MinPrincipalEnergyRatio);
        }

        public void CopyToReport(SingleLegChainReport report)
        {
            if (report == null)
                return;
            // Hot path copy only. Candidate is recomputed every 32 accepted samples and on Stop/Status.
            report.KneeAxisCalibrationCollecting = _collecting;
            report.KneeAxisCalibrationReady = _ready;
            report.KneeAxisCalibrationReason = _reason;
            report.KneeAxisCalibrationSampleCount = _accepted;
            report.KneeAxisCalibrationPrincipalEnergyRatio = _principalEnergyRatio;
            report.KneeAxisCalibrationAngleFromInitialDegrees = _angleFromInitialDegrees;
        }

        private void ResetAccumulators(Vector3 initialAxisLowerLocal)
        {
            _accepted = 0;
            _rejectedSourceWindow = 0;
            _rejectedNonSteady = 0;
            _rejectedLowSpeed = 0;
            _rejectedInvalid = 0;
            _xx = _xy = _xz = _yy = _yz = _zz = 0.0;

            _initialAxisLowerLocal = NormalizeOr(initialAxisLowerLocal, Vector3.right);
            _candidateAxisLowerLocal = _initialAxisLowerLocal;
            _principalEnergyRatio = 0f;
            _principalRmsProjectionRatio = 0f;
            _angleFromInitialDegrees = 0f;
            _ready = false;
            _applied = false;
            _sourceRejectUntilTime = 0f;
        }

        private void RecomputeCandidate()
        {
            double trace = _xx + _yy + _zz;
            if (_accepted <= 0 || !(trace > 1e-12) || double.IsNaN(trace) || double.IsInfinity(trace))
            {
                _candidateAxisLowerLocal = _initialAxisLowerLocal;
                _principalEnergyRatio = 0f;
                _principalRmsProjectionRatio = 0f;
                _angleFromInitialDegrees = 0f;
                return;
            }

            Vector3 seed = _candidateAxisLowerLocal + new Vector3(0.17320508f, 0.38971143f, 0.5270463f) * 0.01f;
            Vector3 v = NormalizeOr(seed, _initialAxisLowerLocal);
            for (int i = 0; i < 32; i++)
            {
                Vector3 mv = Multiply(v);
                if (!IsFinite(mv) || mv.sqrMagnitude < 1e-12f)
                    break;
                v = mv.normalized;
            }

            // Eigenvector sign is arbitrary. Align it with the historical CHARACTER_RIGHT axis for stable reports.
            if (Vector3.Dot(v, _initialAxisLowerLocal) < 0f)
                v = -v;
            v = NormalizeOr(v, _initialAxisLowerLocal);

            Vector3 av = Multiply(v);
            double lambda = Vector3.Dot(v, av);
            double ratio = lambda / trace;
            if (double.IsNaN(ratio) || double.IsInfinity(ratio)) ratio = 0.0;
            ratio = Math.Max(0.0, Math.Min(1.0, ratio));

            _candidateAxisLowerLocal = v;
            _principalEnergyRatio = (float)ratio;
            _principalRmsProjectionRatio = Mathf.Sqrt(Mathf.Clamp01(_principalEnergyRatio));
            _angleFromInitialDegrees = Vector3.Angle(_initialAxisLowerLocal, _candidateAxisLowerLocal);
        }

        private Vector3 Multiply(Vector3 v)
        {
            return new Vector3(
                (float)(_xx * v.x + _xy * v.y + _xz * v.z),
                (float)(_xy * v.x + _yy * v.y + _yz * v.z),
                (float)(_xz * v.x + _yz * v.y + _zz * v.z));
        }

        private void UpdateReadiness()
        {
            if (_collecting)
            {
                _ready = false;
                _reason = _accepted >= MinAcceptedSamples && _principalEnergyRatio >= MinPrincipalEnergyRatio
                    ? "COLLECTING_QUALITY_READY_STOP_TO_FINALIZE"
                    : "COLLECTING";
                return;
            }
            if (_accepted < MinAcceptedSamples)
            {
                _ready = false;
                _reason = "INSUFFICIENT_SAMPLES";
                return;
            }
            if (_principalEnergyRatio < MinPrincipalEnergyRatio)
            {
                _ready = false;
                _reason = "PRINCIPAL_AXIS_NOT_DOMINANT";
                return;
            }
            if (!IsFinite(_candidateAxisLowerLocal) || _candidateAxisLowerLocal.sqrMagnitude < 0.5f)
            {
                _ready = false;
                _reason = "INVALID_CANDIDATE_AXIS";
                return;
            }
            _ready = true;
            _reason = _applied ? "APPLIED" : "READY_TO_APPLY";
        }

        private static Vector3 NormalizeOr(Vector3 value, Vector3 fallback)
        {
            if (!IsFinite(value) || value.sqrMagnitude < 1e-8f)
                value = fallback;
            if (!IsFinite(value) || value.sqrMagnitude < 1e-8f)
                value = Vector3.right;
            return value.normalized;
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        private static string F(float value) => value.ToString("G6", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
