// Test-only boundary for the linked SingleLegShadowChain.KneeReference.cs.
// Only surrounding observations/identities and reporting sinks are substituted.
// Reference planning, ground correction, phase checks and FF gates are production code.
// These ports expose no forces, colliders or simulated native solve.
using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal readonly struct TorsoControlPhase
    {
        internal bool SourceTurnEventActive { get; }
        internal TorsoControlPhase(bool sourceTurn) { SourceTurnEventActive = sourceTurn; }
    }

    internal sealed partial class SingleLegShadowChain
    {
        private bool _dynamicParentActive = false, _dynamicJoinPending = false, _driveEnabled = false, _traceEnabled = false;
        private PhysicsStepContext _joinStep, _runtimeStep;
        private SourceStepContext _runtimeSource;
        private PelvisRunScope _dynamicScope = PelvisRunScope.DynamicLegFreeSpace;
        private PelvisParentFrame _physicalParent;
        private readonly HipCompositeReferenceShockAbsorberIsolation _hipCompositeReferenceShockAbsorber = new HipCompositeReferenceShockAbsorberIsolation();
        private Vector3 _controlTargetHipRelativeAngularVelocityLocal;
        private Quaternion _controlTargetHipRelative = Quaternion.identity;
        private Quaternion _controlTargetKneeRelative = Quaternion.identity, _sampledTargetKneeRelative = Quaternion.identity;
        private Vector3 _controlTargetKneeRelativeAngularVelocityLocal = Vector3.zero;
        private Vector3 _sampledTargetKneeRelativeAngularVelocityLocal = Vector3.zero;
        private Vector3 _sampledTargetKneeRelativeAngularAccelerationLocal = Vector3.zero;
        private Vector3 _sampledTargetHipRelativeAngularAccelerationLocal = Vector3.zero;
        private Vector3 _kneeFlexionAxisLowerLocal = Vector3.right;
        private Vector3 _rawHipConnectedAnchorLocal = Vector3.zero, _kneeChildAnchorLowerLocal = Vector3.zero;
        private Vector3 _kneeParentAnchorUpperLocal = new Vector3(0f, -.45f, 0f);
        private readonly SingleLegAngularAccelerationFeedforward _legAngularAccelerationFeedforward = new SingleLegAngularAccelerationFeedforward();
        private readonly TorsoPort _torso = new TorsoPort();
        private readonly ProfilePort _profile = new ProfilePort();
        private readonly TimingPort _targetSampleTiming = new TimingPort();
        private readonly BodyPort _upperBody = new BodyPort(12), _lowerBody = new BodyPort(13);
        private readonly BodyPort _pelvisBody = new BodyPort(11);
        private readonly GroundPort _footFoundation = new GroundPort();
        private readonly RecoveryPort _hipAuthority = new RecoveryPort(), _softKneeAuthority = new RecoveryPort();
        private readonly FramePort _hipControlFrame = new FramePort();
        private readonly HipNormalReference _hipNormalReference = new HipNormalReference();
        private bool _testSelectorsOff = true, _testNativeOff = true, _testGoalBridge = false;
        private int _testFeedforwardCalls;
        private bool DynamicParentActive => _dynamicParentActive && _driveEnabled;
        private bool FreeSpaceSelectorsOff => _testSelectorsOff;
        private bool FreeSpaceNativeReady => _testNativeOff;
        private bool HipFeedforwardGoalBridgeActive => _testGoalBridge;
        private Quaternion GetHipDerivativeParentRotation() => _physicalParent.Rotation;
        private Quaternion GetKneeDerivativeParentRotation() => _upperBody.transform.rotation;
        private void CaptureLegTurnHipFeedforward(bool ok, Vector3 hip, float knee) { _testFeedforwardCalls++; }
        private void CopyLegAngularAccelerationFeedforwardToReport() { }
        private static bool IsFinite(float v) => float.IsFinite(v);
        private static bool IsFinite(Vector3 v) => PelvisParentFrame.V(v);
        private static string F(float v) => v.ToString("G9", CultureInfo.InvariantCulture);

        private sealed class TorsoPort
        {
            internal readonly SelectionPort PelvisReceiverControl = new SelectionPort();
            internal readonly PolicyPort ControlPolicy = new PolicyPort();
            internal readonly object Report = new object();
            internal TorsoControlPhase ControlPhase = new TorsoControlPhase(false);
            internal bool ControlRecoveryActive = false;
        }
        private sealed class SelectionPort { internal string Arm = "LEGACY"; }
        private sealed class PolicyPort { internal bool HipGoalGateOnlyHip = true; }
        private sealed class ProfilePort
        {
            internal float LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds = .1f;
            internal float LegAngularAccelerationFeedforwardMaxAcceleration = 20f;
            internal float LegAngularAccelerationFeedforwardGain = 1f;
            internal float LegKneeMaxAngularAcceleration = 40f, LegKneeAngularDamper = 9f, LegKneeAngularSpring = 45f;
            internal bool LegHipControlFrameInertialization = true;
        }
        private sealed class TimingPort { internal float AgeSeconds = 0f; }
        private sealed class RecoveryPort { internal bool PhysicalRecoveryPending = false; }
        private sealed class FramePort { internal bool Active = false; }
        private sealed class BodyPort
        {
            internal readonly PosePort transform = new PosePort();
            internal Vector3 position = Vector3.zero;
            internal Quaternion rotation { get => transform.rotation; set => transform.rotation = value; }
            private readonly int _id;
            internal BodyPort(int id) { _id = id; }
            internal int GetInstanceID() => _id;
        }
        private sealed class PosePort
        {
            internal Vector3 lossyScale = new Vector3(1f, 1f, 1f);
            internal Quaternion rotation = Quaternion.identity;
            internal Vector3 TransformDirection(Vector3 local) => rotation * local;
        }
        private sealed class GroundPort
        {
            internal bool Available = false;
            internal KneeFootGroundPacket Packet;
            internal void ClearKneeFootGroundPacket() { Packet = default; Available = false; }
            internal bool TryReadKneeFootGroundPacket(PhysicsStepContext step, SourceStepContext source,
                int parentId, int upperId, int lowerId, out KneeFootGroundPacket packet, out string reason)
            {
                packet = Packet; reason = "TEST_JOINT_PACKET_UNAVAILABLE";
                return Available && Packet.Matches(step, source, parentId, upperId, lowerId, 21);
            }
        }
    }
}
