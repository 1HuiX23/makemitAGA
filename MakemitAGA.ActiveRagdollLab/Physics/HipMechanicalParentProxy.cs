using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.56 — Hip mechanical-parent carrier with explicit STRICT command-closure audit.
    /// v0.2.56 keeps the kinematic MoveRotation law unchanged, but the upstream strict branch now starts each active
    /// FixedUpdate from this proxy's actual settled pose/W before producing the next bounded command. This class then
    /// compares its pose-derived kinematic W/A against that same strict W/A so a second silent command truth cannot recur.
    /// v0.2.51 additionally exposes the PhysX-settled proxy rotation so RAW→STRICT activation can begin from the
    /// actual connectedBody state instead of the one-step-newer raw mathematical state. No proxy motion law changes.
    ///
    /// REAL_PELVIS arm:
    ///   Hip CharacterJoint.connectedBody remains the frozen torso Pelvis Rigidbody exactly as v0.2.45.
    ///
    /// INERTIALIZED_PROXY arm:
    ///   A dedicated kinematic Rigidbody is placed directly at the raw v0.2.45 Pelvis-connected Hip anchor world path. Its world position follows
    ///   the raw/source Hip anchor, while its rotation follows the v0.2.48 STRICT mechanical branch. The reference-side
    ///   25-degree lag guard may rebase Hip P/governor pose, but that rebase never reaches this Rigidbody. The proxy has no Collider.
    ///
    /// The proxy deliberately does not create a second spring/controller. It only changes the mechanical parent that
    /// PhysX sees at the Hip CharacterJoint, allowing a clean A/B test of raw kinematic parent angular shock versus the
    /// strict 6/60 -> 8/80 inertialized parent trajectory without reference-side guard injection.
    /// </summary>
    internal sealed class HipMechanicalParentProxy
    {
        public const string RealPelvisMode = "REAL_PELVIS";
        public const string InertializedProxyMode = "INERTIALIZED_MECHANICAL_PARENT";
        public const string ExperimentName = "REAL_PELVIS_VS_INERTIALIZED_MECHANICAL_PARENT_AB";

        private GameObject _object;
        private Rigidbody _body;
        private bool _useInertialized = true;

        private bool _hasPreviousProxyRotation;
        private Quaternion _previousProxyRotation = Quaternion.identity;
        private Vector3 _previousProxyAngularVelocity;
        private bool _hasPreviousProxyPosition;
        private Vector3 _previousProxyPosition;
        private Vector3 _previousProxyLinearVelocity;
        private bool _hasPreviousRawPelvisAngularVelocity;
        private Vector3 _previousRawPelvisAngularVelocity;

        private Vector3 _measuredProxyAngularVelocity;
        private Vector3 _measuredProxyAngularAcceleration;
        private Vector3 _measuredProxyLinearVelocity;
        private Vector3 _measuredProxyLinearAcceleration;
        private Vector3 _measuredRawPelvisAngularVelocity;
        private Vector3 _measuredRawPelvisAngularAcceleration;
        private Vector3 _commandedProxyAngularVelocity;
        private Vector3 _commandedProxyAngularAcceleration;
        private Vector3 _commandedProxyLinearVelocity;
        private Vector3 _commandedProxyLinearAcceleration;
        private float _peakCommandedProxyAngularSpeed;
        private float _peakCommandedProxyAngularAcceleration;
        private float _peakCommandedProxyLinearSpeed;
        private float _peakCommandedProxyLinearAcceleration;
        private float _positionError;
        private float _rawPelvisRotationLag;
        private float _controlFrameRotationError;
        private int _observedFixedSteps;
        private int _commandedFixedSteps;
        private int _controlFrameCommandedFixedSteps;
        private float _peakProxyAngularSpeed;
        private float _peakProxyAngularAcceleration;
        private float _peakRawPelvisAngularAcceleration;
        private float _peakPositionError;
        private float _peakRawPelvisRotationLag;
        private float _peakControlFrameRotationError;
        private float _commandClosureAngularVelocityError;
        private float _commandClosureAngularAccelerationError;
        private float _commandClosureMaxAccelerationRatio;
        private bool _commandClosureAccelerationContractSatisfied = true;
        private bool _poseDerivedAuditWithinTolerance = true;
        private float _poseDerivedAuditTolerance;
        private float _poseDerivedCommandOverMaxA;
        private float _peakCommandClosureAngularVelocityError;
        private float _peakCommandClosureAngularAccelerationError;
        private float _peakCommandClosureMaxAccelerationRatio;

        internal bool Built => _object != null && _body != null;
        internal Rigidbody Body => _body;
        internal bool UseInertialized => _useInertialized;
        internal string Mode => _useInertialized ? InertializedProxyMode : RealPelvisMode;
        // v0.2.51 read-only handoff state. ObserveSettled() runs before the control-frame Step each FixedUpdate,
        // therefore this is the actual kinematic connectedBody rotation that PhysX settled on the previous step.
        internal Quaternion SettledRotation => Built ? _body.rotation : Quaternion.identity;
        internal Vector3 MeasuredProxyAngularVelocity => _measuredProxyAngularVelocity;
        internal Vector3 CommandedProxyAngularAcceleration => _commandedProxyAngularAcceleration;
        internal Vector3 CommandedProxyLinearAcceleration => _commandedProxyLinearAcceleration;
        internal float CommandClosureAngularVelocityError => _commandClosureAngularVelocityError;
        internal float CommandClosureAngularAccelerationError => _commandClosureAngularAccelerationError;
        internal float CommandClosureMaxAccelerationRatio => _commandClosureMaxAccelerationRatio;
        internal bool CommandClosureAccelerationContractSatisfied => _commandClosureAccelerationContractSatisfied;
        internal bool PoseDerivedAuditWithinTolerance => _poseDerivedAuditWithinTolerance;
        internal float PoseDerivedAuditTolerance => _poseDerivedAuditTolerance;
        internal float PoseDerivedCommandOverMaxA => _poseDerivedCommandOverMaxA;

        internal void Build(Transform ownerRoot, int layer, Vector3 hipWorldPosition, Quaternion initialRotation, float maxAngularVelocity)
        {
            if (Built)
                return;

            _object = new GameObject("ARL_HipMechanicalParent");
            _object.hideFlags = HideFlags.DontSave;
            _object.layer = layer;
            _object.transform.position = hipWorldPosition;
            _object.transform.rotation = initialRotation;
            if (ownerRoot != null)
                _object.transform.SetParent(ownerRoot, true);

            _body = Il2CppComponentFactory.Add<Rigidbody>(_object, "LEG_ADD_HIP_MECHANICAL_PARENT_RIGIDBODY");
            _body.isKinematic = true;
            _body.useGravity = false;
            _body.detectCollisions = false;
            _body.drag = 0f;
            _body.angularDrag = 0f;
            _body.maxAngularVelocity = maxAngularVelocity;
            _body.interpolation = RigidbodyInterpolation.None;
            _body.collisionDetectionMode = CollisionDetectionMode.Discrete;
            _body.velocity = Vector3.zero;
            _body.angularVelocity = Vector3.zero;

            ResetPose(hipWorldPosition, initialRotation);
            ResetStatistics();
        }

        internal bool TrySetMode(bool useInertialized)
        {
            if (_useInertialized == useInertialized)
                return false;
            _useInertialized = useInertialized;
            return true;
        }

        internal void ResetPose(Vector3 hipWorldPosition, Quaternion rotation)
        {
            if (!Built)
                return;
            _body.isKinematic = true;
            _body.useGravity = false;
            _body.detectCollisions = false;
            _body.position = hipWorldPosition;
            _body.rotation = rotation;
            _body.velocity = Vector3.zero;
            _body.angularVelocity = Vector3.zero;
            _previousProxyRotation = rotation;
            _previousProxyAngularVelocity = Vector3.zero;
            _measuredProxyAngularVelocity = Vector3.zero;
            _measuredProxyAngularAcceleration = Vector3.zero;
            _previousProxyPosition = hipWorldPosition;
            _previousProxyLinearVelocity = Vector3.zero;
            _measuredProxyLinearVelocity = Vector3.zero;
            _measuredProxyLinearAcceleration = Vector3.zero;
            _hasPreviousProxyRotation = true;
            _hasPreviousProxyPosition = true;
            _hasPreviousRawPelvisAngularVelocity = false;
            _previousRawPelvisAngularVelocity = Vector3.zero;
            _measuredRawPelvisAngularVelocity = Vector3.zero;
            _measuredRawPelvisAngularAcceleration = Vector3.zero;
            _commandedProxyAngularVelocity = Vector3.zero;
            _commandedProxyAngularAcceleration = Vector3.zero;
            _commandedProxyLinearVelocity = Vector3.zero;
            _commandedProxyLinearAcceleration = Vector3.zero;
            _commandClosureAngularVelocityError = 0f;
            _commandClosureAngularAccelerationError = 0f;
            _commandClosureMaxAccelerationRatio = 0f;
            _commandClosureAccelerationContractSatisfied = true;
            _poseDerivedAuditWithinTolerance = true;
            _poseDerivedAuditTolerance = 0f;
            _poseDerivedCommandOverMaxA = 0f;
        }

        internal void ResetStatistics()
        {
            _observedFixedSteps = 0;
            _commandedFixedSteps = 0;
            _controlFrameCommandedFixedSteps = 0;
            _peakProxyAngularSpeed = 0f;
            _peakProxyAngularAcceleration = 0f;
            _peakRawPelvisAngularAcceleration = 0f;
            _peakPositionError = 0f;
            _peakRawPelvisRotationLag = 0f;
            _peakControlFrameRotationError = 0f;
            _peakCommandedProxyAngularSpeed = 0f;
            _peakCommandedProxyAngularAcceleration = 0f;
            _peakCommandedProxyLinearSpeed = 0f;
            _peakCommandedProxyLinearAcceleration = 0f;
            _positionError = 0f;
            _rawPelvisRotationLag = 0f;
            _controlFrameRotationError = 0f;
            _commandClosureAngularVelocityError = 0f;
            _commandClosureAngularAccelerationError = 0f;
            _commandClosureMaxAccelerationRatio = 0f;
            _commandClosureAccelerationContractSatisfied = true;
            _poseDerivedAuditWithinTolerance = true;
            _poseDerivedAuditTolerance = 0f;
            _poseDerivedCommandOverMaxA = 0f;
            _peakCommandClosureAngularVelocityError = 0f;
            _peakCommandClosureAngularAccelerationError = 0f;
            _peakCommandClosureMaxAccelerationRatio = 0f;
        }

        /// <summary>
        /// Samples the *settled previous physics step* before issuing this FixedUpdate's next kinematic command.
        /// This is the mechanical-parent quantity that must be compared with lag-1 Hip command/response attribution.
        /// </summary>
        internal void ObserveSettled(
            Vector3 rawPelvisAngularVelocityWorld,
            Quaternion rawPelvisRotation,
            Quaternion desiredControlParentRotation,
            Vector3 sourceHipWorldPosition,
            float fixedDeltaTime,
            SingleLegChainReport report)
        {
            if (!Built || !Finite(rawPelvisAngularVelocityWorld) || !Finite(rawPelvisRotation) ||
                !Finite(desiredControlParentRotation) || !Finite(sourceHipWorldPosition) ||
                !Finite(fixedDeltaTime) || fixedDeltaTime <= 0.0001f)
            {
                CopyToReport(report, null);
                return;
            }

            Quaternion currentRotation = _body.rotation;
            if (_hasPreviousProxyRotation)
            {
                Vector3 currentW = AngularVelocity(_previousProxyRotation, currentRotation, fixedDeltaTime);
                _measuredProxyAngularAcceleration = (currentW - _previousProxyAngularVelocity) / fixedDeltaTime;
                _measuredProxyAngularVelocity = currentW;
            }
            else
            {
                _measuredProxyAngularVelocity = Vector3.zero;
                _measuredProxyAngularAcceleration = Vector3.zero;
            }
            _previousProxyRotation = currentRotation;
            _previousProxyAngularVelocity = _measuredProxyAngularVelocity;
            _hasPreviousProxyRotation = true;

            Vector3 currentPosition = _body.position;
            if (_hasPreviousProxyPosition)
            {
                Vector3 currentV = (currentPosition - _previousProxyPosition) / fixedDeltaTime;
                _measuredProxyLinearAcceleration = (currentV - _previousProxyLinearVelocity) / fixedDeltaTime;
                _measuredProxyLinearVelocity = currentV;
            }
            else
            {
                _measuredProxyLinearVelocity = Vector3.zero;
                _measuredProxyLinearAcceleration = Vector3.zero;
            }
            _previousProxyPosition = currentPosition;
            _previousProxyLinearVelocity = _measuredProxyLinearVelocity;
            _hasPreviousProxyPosition = true;

            _measuredRawPelvisAngularVelocity = rawPelvisAngularVelocityWorld;
            if (_hasPreviousRawPelvisAngularVelocity)
                _measuredRawPelvisAngularAcceleration = (rawPelvisAngularVelocityWorld - _previousRawPelvisAngularVelocity) / fixedDeltaTime;
            else
                _measuredRawPelvisAngularAcceleration = Vector3.zero;
            _commandedProxyAngularVelocity = Vector3.zero;
            _commandedProxyAngularAcceleration = Vector3.zero;
            _commandedProxyLinearVelocity = Vector3.zero;
            _commandedProxyLinearAcceleration = Vector3.zero;
            _commandClosureAngularVelocityError = 0f;
            _commandClosureAngularAccelerationError = 0f;
            _commandClosureMaxAccelerationRatio = 0f;
            _commandClosureAccelerationContractSatisfied = true;
            _poseDerivedAuditWithinTolerance = true;
            _poseDerivedAuditTolerance = 0f;
            _poseDerivedCommandOverMaxA = 0f;
            _previousRawPelvisAngularVelocity = rawPelvisAngularVelocityWorld;
            _hasPreviousRawPelvisAngularVelocity = true;

            _positionError = Vector3.Distance(_body.position, sourceHipWorldPosition);
            _rawPelvisRotationLag = Quaternion.Angle(_body.rotation, rawPelvisRotation);
            _controlFrameRotationError = Quaternion.Angle(_body.rotation, desiredControlParentRotation);
            _observedFixedSteps++;

            Vector3 effectiveW = _useInertialized ? _measuredProxyAngularVelocity : _measuredRawPelvisAngularVelocity;
            Vector3 effectiveA = _useInertialized ? _measuredProxyAngularAcceleration : _measuredRawPelvisAngularAcceleration;
            _peakProxyAngularSpeed = Mathf.Max(_peakProxyAngularSpeed, effectiveW.magnitude);
            _peakProxyAngularAcceleration = Mathf.Max(_peakProxyAngularAcceleration, effectiveA.magnitude);
            _peakRawPelvisAngularAcceleration = Mathf.Max(_peakRawPelvisAngularAcceleration, _measuredRawPelvisAngularAcceleration.magnitude);
            _peakPositionError = Mathf.Max(_peakPositionError, _positionError);
            _peakRawPelvisRotationLag = Mathf.Max(_peakRawPelvisRotationLag, _rawPelvisRotationLag);
            _peakControlFrameRotationError = Mathf.Max(_peakControlFrameRotationError, _controlFrameRotationError);
            CopyToReport(report, null);
        }

        /// <summary>
        /// Schedules the kinematic mechanical parent for the upcoming physics integration.
        /// Position is always the raw/source Hip anchor. Rotation is either the validated inertialized parent trajectory
        /// or the raw frozen-torso Pelvis, depending on A/B arm.
        /// </summary>
        internal void CommandForNextPhysics(
            Vector3 sourceHipWorldPosition,
            Quaternion rawPelvisRotation,
            Quaternion inertializedControlRotation,
            Vector3 strictMechanicalAngularVelocityWorld,
            Vector3 strictMechanicalAngularAccelerationWorld,
            float strictMechanicalMaxAngularAcceleration,
            bool controlFrameActive,
            float fixedDeltaTime,
            SingleLegChainReport report)
        {
            if (!Built)
                return;

            Quaternion desiredRotation = _useInertialized && controlFrameActive
                ? inertializedControlRotation
                : rawPelvisRotation;
            if (!Finite(sourceHipWorldPosition) || !Finite(desiredRotation) || !Finite(fixedDeltaTime) || fixedDeltaTime <= 0.0001f)
                return;
            if (_useInertialized && controlFrameActive &&
                (!Finite(strictMechanicalAngularVelocityWorld) || !Finite(strictMechanicalAngularAccelerationWorld) ||
                 !Finite(strictMechanicalMaxAngularAcceleration) || strictMechanicalMaxAngularAcceleration <= 0f))
                return;

            // v0.2.48/v0.2.84: calculate the kinematic parent's upcoming W/A before MoveRotation is issued.
            // During strict Source/Recovery, desiredRotation is the guard-decoupled STRICT mechanical branch.
            // During Healthy Normal, desiredRotation is the raw Pelvis command rotation. v0.2.84 exposes that already-
            // scheduled MoveRotation-derived Normal A as a causal transport input, without changing the proxy motion itself.
            if (_useInertialized)
            {
                _commandedProxyAngularVelocity = AngularVelocity(_body.rotation, desiredRotation, fixedDeltaTime);
                _commandedProxyAngularAcceleration = (_commandedProxyAngularVelocity - _measuredProxyAngularVelocity) / fixedDeltaTime;
                _peakCommandedProxyAngularSpeed = Mathf.Max(_peakCommandedProxyAngularSpeed, _commandedProxyAngularVelocity.magnitude);
                _peakCommandedProxyAngularAcceleration = Mathf.Max(_peakCommandedProxyAngularAcceleration, _commandedProxyAngularAcceleration.magnitude);
            }
            else
            {
                _commandedProxyAngularVelocity = Vector3.zero;
                _commandedProxyAngularAcceleration = Vector3.zero;
            }

            if (_useInertialized && controlFrameActive)
            {
                // v0.2.56 command closure audit. STRICT has already re-anchored to this same settled body before
                // integrating, so the pose-derived kinematic W/A and strict W/A are now required to describe one
                // mechanical trajectory. No clamp is performed here; mismatch remains fail-closed evidence.
                _commandClosureAngularVelocityError = (_commandedProxyAngularVelocity - strictMechanicalAngularVelocityWorld).magnitude;
                _commandClosureAngularAccelerationError = (_commandedProxyAngularAcceleration - strictMechanicalAngularAccelerationWorld).magnitude;
                _commandClosureMaxAccelerationRatio = strictMechanicalMaxAngularAcceleration > 0.0001f
                    ? _commandedProxyAngularAcceleration.magnitude / strictMechanicalMaxAngularAcceleration
                    : 0f;
                // v0.2.60: the strict mathematical W/A state is authoritative for the MaxA fail-fast contract.
                // Pose-derived MoveRotation W/A is a finite-step audit and may differ slightly because quaternion endpoint
                // reconstruction is not the same quantity as the strict tangent-space integrator state.
                _commandClosureAccelerationContractSatisfied = strictMechanicalAngularAccelerationWorld.magnitude <= strictMechanicalMaxAngularAcceleration + 0.05f;
                _poseDerivedAuditTolerance = Mathf.Max(0.5f, strictMechanicalMaxAngularAcceleration * 0.01f);
                _poseDerivedCommandOverMaxA = Mathf.Max(0f, _commandedProxyAngularAcceleration.magnitude - strictMechanicalMaxAngularAcceleration);
                _poseDerivedAuditWithinTolerance = _poseDerivedCommandOverMaxA <= _poseDerivedAuditTolerance;
                _peakCommandClosureAngularVelocityError = Mathf.Max(_peakCommandClosureAngularVelocityError, _commandClosureAngularVelocityError);
                _peakCommandClosureAngularAccelerationError = Mathf.Max(_peakCommandClosureAngularAccelerationError, _commandClosureAngularAccelerationError);
                _peakCommandClosureMaxAccelerationRatio = Mathf.Max(_peakCommandClosureMaxAccelerationRatio, _commandClosureMaxAccelerationRatio);

                // v0.2.50 same-FixedUpdate linear command. Position still follows the exact raw Hip anchor path from v0.2.46;
                // this derivative is consumed only by the bounded whole-leg COM transport experiment, never by the proxy itself.
                _commandedProxyLinearVelocity = (sourceHipWorldPosition - _body.position) / fixedDeltaTime;
                _commandedProxyLinearAcceleration = (_commandedProxyLinearVelocity - _measuredProxyLinearVelocity) / fixedDeltaTime;
                _peakCommandedProxyLinearSpeed = Mathf.Max(_peakCommandedProxyLinearSpeed, _commandedProxyLinearVelocity.magnitude);
                _peakCommandedProxyLinearAcceleration = Mathf.Max(_peakCommandedProxyLinearAcceleration, _commandedProxyLinearAcceleration.magnitude);
            }
            else
            {
                // Normal keeps the exact historical raw-Pelvis MoveRotation behavior. Only its upcoming angular W/A is
                // now retained for v0.2.84 diagnostics/transport. Strict-only closure contracts stay inactive here.
                _commandedProxyLinearVelocity = Vector3.zero;
                _commandedProxyLinearAcceleration = Vector3.zero;
                _commandClosureAngularVelocityError = 0f;
                _commandClosureAngularAccelerationError = 0f;
                _commandClosureMaxAccelerationRatio = 0f;
                _commandClosureAccelerationContractSatisfied = true;
                _poseDerivedAuditWithinTolerance = true;
                _poseDerivedAuditTolerance = 0f;
                _poseDerivedCommandOverMaxA = 0f;
            }

            if (_useInertialized)
            {
                _body.MovePosition(sourceHipWorldPosition);
                _body.MoveRotation(desiredRotation);
                _commandedFixedSteps++;
                if (controlFrameActive)
                    _controlFrameCommandedFixedSteps++;
            }
            CopyToReport(report, null);
        }

        internal Rigidbody GetConnectedBody(Rigidbody rawPelvisBody)
        {
            return _useInertialized ? _body : rawPelvisBody;
        }

        internal void CopyToReport(SingleLegChainReport report, CharacterJoint hipJoint)
        {
            if (report == null)
                return;
            report.HipMechanicalParentExperiment = ExperimentName;
            report.HipMechanicalParentMode = Mode;
            report.HipMechanicalParentProxyPresent = Built;
            report.HipMechanicalParentUsesInertializedProxy = _useInertialized;
            report.HipMechanicalParentObservedFixedSteps = _observedFixedSteps;
            report.HipMechanicalParentCommandedFixedSteps = _commandedFixedSteps;
            report.HipMechanicalParentControlFrameCommandedFixedSteps = _controlFrameCommandedFixedSteps;
            report.HipMechanicalParentCurrentPositionErrorMeters = _positionError;
            report.HipMechanicalParentCurrentRawPelvisRotationLagDegrees = _rawPelvisRotationLag;
            report.HipMechanicalParentCurrentControlFrameRotationErrorDegrees = _controlFrameRotationError;
            Vector3 effectiveW = _useInertialized ? _measuredProxyAngularVelocity : _measuredRawPelvisAngularVelocity;
            Vector3 effectiveA = _useInertialized ? _measuredProxyAngularAcceleration : _measuredRawPelvisAngularAcceleration;
            report.HipMechanicalParentCurrentAngularSpeed = effectiveW.magnitude;
            report.HipMechanicalParentCurrentAngularAcceleration = effectiveA.magnitude;
            report.HipMechanicalParentCurrentRawPelvisAngularAcceleration = _measuredRawPelvisAngularAcceleration.magnitude;
            report.HipMechanicalParentCurrentCommandedAngularSpeed = _commandedProxyAngularVelocity.magnitude;
            report.HipMechanicalParentCurrentCommandedAngularAcceleration = _commandedProxyAngularAcceleration.magnitude;
            report.HipMechanicalParentPeakPositionErrorMeters = _peakPositionError;
            report.HipMechanicalParentPeakRawPelvisRotationLagDegrees = _peakRawPelvisRotationLag;
            report.HipMechanicalParentPeakControlFrameRotationErrorDegrees = _peakControlFrameRotationError;
            report.HipMechanicalParentPeakAngularSpeed = _peakProxyAngularSpeed;
            report.HipMechanicalParentPeakAngularAcceleration = _peakProxyAngularAcceleration;
            report.HipMechanicalParentPeakRawPelvisAngularAcceleration = _peakRawPelvisAngularAcceleration;
            report.HipMechanicalParentPeakCommandedAngularSpeed = _peakCommandedProxyAngularSpeed;
            report.HipMechanicalParentPeakCommandedAngularAcceleration = _peakCommandedProxyAngularAcceleration;
            report.HipMechanicalParentCommandClosureAngularVelocityError = _commandClosureAngularVelocityError;
            report.HipMechanicalParentCommandClosureAngularAccelerationError = _commandClosureAngularAccelerationError;
            report.HipMechanicalParentCommandClosureMaxAccelerationRatio = _commandClosureMaxAccelerationRatio;
            report.HipMechanicalParentCommandClosureAccelerationContractSatisfied = _commandClosureAccelerationContractSatisfied;
            report.HipMechanicalParentPoseDerivedAuditWithinTolerance = _poseDerivedAuditWithinTolerance;
            report.HipMechanicalParentPoseDerivedAuditTolerance = _poseDerivedAuditTolerance;
            report.HipMechanicalParentPoseDerivedCommandOverMaxA = _poseDerivedCommandOverMaxA;
            report.HipMechanicalParentPeakCommandClosureAngularVelocityError = _peakCommandClosureAngularVelocityError;
            report.HipMechanicalParentPeakCommandClosureAngularAccelerationError = _peakCommandClosureAngularAccelerationError;
            report.HipMechanicalParentPeakCommandClosureMaxAccelerationRatio = _peakCommandClosureMaxAccelerationRatio;
            report.HipMechanicalParentConnectedToProxy = Built && hipJoint != null && hipJoint.connectedBody == _body;
            report.HipMechanicalParentKey = GetKey(hipJoint);
        }

        internal string GetKey(CharacterJoint hipJoint)
        {
            bool connectedProxy = Built && hipJoint != null && hipJoint.connectedBody == _body;
            float aRatio = _peakRawPelvisAngularAcceleration > 0.0001f
                ? _peakProxyAngularAcceleration / _peakRawPelvisAngularAcceleration
                : 0f;
            return
                "mode=" + Mode +
                " | experiment=" + ExperimentName +
                " | proxyBuilt=" + Built +
                " | connectedProxy=" + connectedProxy +
                " | steps(observed/commanded/controlFrame)=" + _observedFixedSteps + "/" + _commandedFixedSteps + "/" + _controlFrameCommandedFixedSteps +
                " | current(posErr/rawLag/controlErr/W/A/rawA/cmdW/cmdA/cmdV/cmdLinearA)=" + F(_positionError) + "/" + F(_rawPelvisRotationLag) + "/" + F(_controlFrameRotationError) + "/" + F((_useInertialized ? _measuredProxyAngularVelocity : _measuredRawPelvisAngularVelocity).magnitude) + "/" + F((_useInertialized ? _measuredProxyAngularAcceleration : _measuredRawPelvisAngularAcceleration).magnitude) + "/" + F(_measuredRawPelvisAngularAcceleration.magnitude) + "/" + F(_commandedProxyAngularVelocity.magnitude) + "/" + F(_commandedProxyAngularAcceleration.magnitude) + "/" + F(_commandedProxyLinearVelocity.magnitude) + "/" + F(_commandedProxyLinearAcceleration.magnitude) +
                " | peak(posErr/rawLag/controlErr/W/A/rawA/cmdW/cmdA/cmdV/cmdLinearA/AmechanicalToRaw)=" + F(_peakPositionError) + "/" + F(_peakRawPelvisRotationLag) + "/" + F(_peakControlFrameRotationError) + "/" + F(_peakProxyAngularSpeed) + "/" + F(_peakProxyAngularAcceleration) + "/" + F(_peakRawPelvisAngularAcceleration) + "/" + F(_peakCommandedProxyAngularSpeed) + "/" + F(_peakCommandedProxyAngularAcceleration) + "/" + F(_peakCommandedProxyLinearSpeed) + "/" + F(_peakCommandedProxyLinearAcceleration) + "/" + F(aRatio) +
                " | commandClosure(currentWErr/currentAErr/maxARatio/strictContract/poseAuditOK/tolerance/overMaxA/peakWErr/peakAErr/peakMaxARatio)=" + F(_commandClosureAngularVelocityError) + "/" + F(_commandClosureAngularAccelerationError) + "/" + F(_commandClosureMaxAccelerationRatio) + "/" + _commandClosureAccelerationContractSatisfied + "/" + _poseDerivedAuditWithinTolerance + "/" + F(_poseDerivedAuditTolerance) + "/" + F(_poseDerivedCommandOverMaxA) + "/" + F(_peakCommandClosureAngularVelocityError) + "/" + F(_peakCommandClosureAngularAccelerationError) + "/" + F(_peakCommandClosureMaxAccelerationRatio) +
                " | invariant=POSITION_RAW_HIP_ANCHOR_ROTATION_SETTLED_ORIGIN_STRICT_WA_GUARD_DECOUPLED";
        }

        internal void Destroy()
        {
            // Do not hide ownership failures here.  The Single-Leg rollback caller aggregates and reports them,
            // so an exceptional Destroy failure cannot be misreported as cleanup success.
            if (_object != null) Object.Destroy(_object);
            _object = null;
            _body = null;
        }

        private static Vector3 AngularVelocity(Quaternion previous, Quaternion current, float dt)
        {
            if (dt <= 0.000001f)
                return Vector3.zero;
            return QuaternionToAngularVector(current * Quaternion.Inverse(previous)) / dt;
        }

        private static Vector3 QuaternionToAngularVector(Quaternion q)
        {
            if (q.w < 0f)
            {
                q.x = -q.x;
                q.y = -q.y;
                q.z = -q.z;
                q.w = -q.w;
            }
            float w = Mathf.Clamp(q.w, -1f, 1f);
            float angle = 2f * Mathf.Acos(w);
            if (angle > Mathf.PI)
                angle -= 2f * Mathf.PI;
            float sinHalf = Mathf.Sqrt(Mathf.Max(0f, 1f - w * w));
            if (sinHalf <= 0.0001f || Mathf.Abs(angle) <= 0.000001f)
                return Vector3.zero;
            return new Vector3(q.x / sinHalf, q.y / sinHalf, q.z / sinHalf) * angle;
        }

        private static bool Finite(Vector3 v) => Finite(v.x) && Finite(v.y) && Finite(v.z);
        private static bool Finite(Quaternion q) => Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w);
        private static bool Finite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static string F(float v) => v.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
