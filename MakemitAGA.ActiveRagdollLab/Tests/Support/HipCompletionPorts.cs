// Test-only observations and native-call boundary. The linked completion, input
// capture, next-step resolution and rotation-window implementations are production
// code. No native dynamics are simulated; faults occur before a port call returns.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class PelvisDynamicReceiverIsolation
    {
        private sealed class SelectionPort { internal string Arm = DynamicHipArmB; }
        private readonly SelectionPort _control = new SelectionPort();
        private readonly NativeBodyPort _pelvis = new NativeBodyPort(101, 4f);
        private readonly NativeBodyPort _chest = new NativeBodyPort(102, 6f);
        private readonly PelvisReceiverReport Report = new PelvisReceiverReport();
        private int _pending = -1;
        private PelvisHipCommandFrame _hipCommand;
        private PelvisHipAttitudeCommand _hipAttitudeCommand;
        private bool _expectHipCompletion = false, _bodyMatches = true;
        private bool ReadyForRun => _bodyMatches && Report.Finite;
        private bool BodyStateMatch() => _bodyMatches;
        private static bool F(float v) => float.IsFinite(v);
        private static bool V(Vector3 v) => PelvisParentFrame.V(v);
        private bool Invalid(string reason)
        {
            _hipCommand = default; _hipAttitudeCommand = default; _expectHipCompletion = false;
            Report.FaultEvents++; Report.Finite = false; Report.Issue ??= reason; return false;
        }
        private void EndTrace()
        {
            if (_pending >= 0)
            {
                var f = Report.Ring[_pending]; f.Censored = true;
                Report.Ring[_pending] = f; _pending = -1; Report.Censored++;
            }
            Report.TraceRunning = false;
        }
        private sealed class NativeBodyPort
        {
            private readonly int _id;
            internal readonly float mass;
            internal Vector3 velocity = Vector3.zero;
            internal Vector3 ForceSum = Vector3.zero, TorqueSum = Vector3.zero;
            internal int ForceCalls = 0, TorqueCalls = 0;
            internal readonly HashSet<int> FailForceCalls = new HashSet<int>();
            internal bool FailTorque = false;
            internal NativeBodyPort(int id, float bodyMass) { _id = id; mass = bodyMass; }
            internal int GetInstanceID() => _id;
            internal void AddForce(Vector3 force, ForceMode mode)
            {
                ForceCalls++;
                if (FailForceCalls.Contains(ForceCalls)) throw new InvalidOperationException("native-force-port");
                ForceSum += force;
            }
            internal void AddTorque(Vector3 torque, ForceMode mode)
            {
                TorqueCalls++;
                if (FailTorque) throw new InvalidOperationException("native-torque-port");
                TorqueSum += torque;
            }
        }
    }

    internal sealed partial class HipPointResponseAudit
    {
        private struct ResponseFrame
        {
            internal HipAnchorPairReport.Command Command;
            internal HipAttitudeBudgetSample Budget;
            internal PelvisHipAttitudeCommand Attitude;
            internal PelvisReceiverReport.TrackingFrame ReceiverTracking;
            internal bool Resolved, Censored, CoupledInputsJoined;
            internal Vector3 PeerForce, PeerParentTorque;
        }
        private readonly ResponseFrame[] _ring = new ResponseFrame[16];
        private int _next = 0, _count = 0;
        private ResponseFrame _pending;
        private bool _hasPending = false;
        private static bool F(float v) => float.IsFinite(v);
        private static bool V(Vector3 v) => PelvisParentFrame.V(v);
        private static string N(float v) => v.ToString("G9", CultureInfo.InvariantCulture);
        private static string S(Vector3 v) => "(" + N(v.x) + "," + N(v.y) + "," + N(v.z) + ")";
        private static void Append(StringBuilder sb, string tag, ResponseFrame f) =>
            sb.Append("\nselection=").Append(tag).Append("/legFixed=").Append(f.Command.Fixed);
    }
}

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Only the report surface consumed by the linked production methods. These
    // are observation sinks, not substitutes for any eligibility or solver math.
    internal sealed partial class PelvisReceiverReport
    {
        internal bool TraceRunning = true, TraceHipTranslation = true, Finite = true;
        internal string Issue = null;
        internal int CurrentTraceFixed = 0, Samples = 0, IdleSamples = 0, MotionSamples = 0;
        internal int Next = 0, Count = 0, FaultEvents = 0, HipReactionRecords = 0;
        internal int Resolved = 0, Censored = 0;
        internal float HipAttitudeRequestedPeak = 0f, HipAttitudeAppliedPeak = 0f, HipAttitudeResidualPeak = 0f, SystemResidualPeak = 0f;
        internal Vector3 CommonA = Vector3.zero, PelvisTorque = Vector3.zero;
        internal readonly Frame[] Ring = new Frame[8];
        internal AngularActuationSample AngularActuation = default;
        internal TrackingFrame LastTracking = default, PeakRotationTracking = default;
        internal void CaptureMotionInput(ref Frame frame, PhysicsStepContext step) { }
        internal void ResolveMotionInput(ref Frame frame, PhysicsStepContext step) { }
        internal struct Frame
        {
            internal PhysicsStepContext InputStep;
            internal int Fixed, Settled;
            internal float Dt, PelvisMass, ChestMass;
            internal bool HipReactionRecorded, Resolved, Censored;
            internal Vector3 PrePelvisV, PreChestV, PostPelvisV, PostChestV, PelvisJ, ChestJ, TorqueJ;
            internal Vector3 HipLeftJ, HipRightJ, HipMomentJ, SystemDeltaP, SystemResidual;
            internal PelvisHipAttitudeCommand HipAttitude;
        }
        internal struct AngularActuationSample
        {
            internal bool Valid;
            internal PhysicsStepContext Step;
            internal int Fixed;
            internal Vector3 PAlpha, DAlpha, FeedforwardAlpha, AppliedAlpha, RawTorque, Torque;
        }
        internal struct TrackingFrame
        {
            internal bool Valid, RuntimeContextExpected, SourceActive, SourceFresh, LinearClamped, AngularClamped, TorqueClamped;
            internal PhysicsStepContext RuntimeContext;
            internal int Fixed, Stage, SourceSequence, SourceEventId, RecoveryEventId;
            internal float Dt, Time, Rotation, Position, RawRenderRotation, SourceAge, SourceDt;
            internal Vector3 RotationErrorVector, BodyW, CommandW, SourceWVector;
            internal string SourcePhase;
            internal AngularActuationSample AngularActuation;
        }
    }
}
