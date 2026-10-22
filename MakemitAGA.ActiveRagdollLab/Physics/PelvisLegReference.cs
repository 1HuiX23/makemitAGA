using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Reusable physical-parent interface; V04590 exercises it without creating leg
    // bodies/joints or running the old kinematic mechanical-parent controller.
    // Source upper-leg poses are sampled beside the torso source pose, never in Fixed.
    internal sealed class PelvisLegReference
    {
        private Rigidbody _owner;
        private Transform _sourcePelvis, _left, _right;
        private Vector3 _leftAnchor, _rightAnchor, _leftDrift, _rightDrift;
        private Quaternion _leftRelative = Quaternion.identity, _rightRelative = Quaternion.identity;
        private int _epoch, _bodyId, _physicsSequence, _capture, _sourceSequence;
        private bool _sampleValid, _preValid;
        private PelvisParentFrame _pre, _current;
        internal bool Bound { get; private set; }
        internal string Issue { get; private set; }

        internal bool Bind(Rigidbody owner, Transform sourcePelvis, Transform left, Transform right,
            Vector3 sourcePosition, Quaternion sourceRotation, int sourceSequence)
        {
            Clear();
            try
            {
                if (owner == null || !owner.isKinematic || !owner.gameObject.activeInHierarchy || sourcePelvis == null || left == null || right == null || left == right ||
                    !sourcePelvis.gameObject.activeInHierarchy || !left.gameObject.activeInHierarchy || !right.gameObject.activeInHierarchy ||
                    !left.IsChildOf(sourcePelvis) || !right.IsChildOf(sourcePelvis) ||
                    !PelvisParentFrame.V(sourcePosition) || !PelvisParentFrame.Q(sourceRotation))
                    return Fail("PARENT_BIND_OWNER_OR_SOURCE_INVALID");
                Vector3 p = owner.position; Quaternion r = owner.rotation;
                if (!PelvisParentFrame.V(p) || !PelvisParentFrame.Q(r) || (p - sourcePosition).magnitude > 0.0001f || Quaternion.Angle(r, sourceRotation) > 0.05f)
                    return Fail("PARENT_BIND_REQUIRES_SYNCHRONIZED_NATIVE_POSE");
                _owner = owner; _sourcePelvis = sourcePelvis; _left = left; _right = right;
                _bodyId = owner.GetInstanceID(); _epoch++; _physicsSequence = _capture = 0;
                _leftAnchor = Quaternion.Inverse(r) * (left.position - p);
                _rightAnchor = Quaternion.Inverse(r) * (right.position - p);
                if (!PelvisParentFrame.V(_leftAnchor) || !PelvisParentFrame.V(_rightAnchor)) return Fail("PARENT_BIND_ANCHOR_NONFINITE");
                Bound = true;
                Sample(sourcePosition, sourceRotation, sourceSequence);
                if (!_sampleValid) Bound = false;
                return _sampleValid;
            }
            catch (Exception e) { return Fail("PARENT_BIND_EXCEPTION:" + e.GetType().Name); }
        }
        internal void Sample(Vector3 sourcePosition, Quaternion sourceRotation, int sourceSequence)
        {
            Invalidate(); _sampleValid = false;
            if (!Bound) return;
            try
            {
                if (_sourcePelvis == null || _left == null || _right == null || !_sourcePelvis.gameObject.activeInHierarchy ||
                    !_left.gameObject.activeInHierarchy || !_right.gameObject.activeInHierarchy || !_left.IsChildOf(_sourcePelvis) || !_right.IsChildOf(_sourcePelvis) || !PelvisParentFrame.V(sourcePosition) ||
                    !PelvisParentFrame.Q(sourceRotation) || sourceSequence < 0) { Fail("PARENT_SOURCE_INVALID"); return; }
                Quaternion inverse = Quaternion.Inverse(sourceRotation);
                _leftDrift = inverse * (_left.position - sourcePosition) - _leftAnchor;
                _rightDrift = inverse * (_right.position - sourcePosition) - _rightAnchor;
                _leftRelative = inverse * _left.rotation; _rightRelative = inverse * _right.rotation;
                if (!PelvisParentFrame.V(_leftDrift) || !PelvisParentFrame.V(_rightDrift) ||
                    !PelvisParentFrame.Q(_leftRelative) || !PelvisParentFrame.Q(_rightRelative)) { Fail("PARENT_SOURCE_NONFINITE"); return; }
                _sourceSequence = sourceSequence; _capture++; _sampleValid = true; Issue = null;
            }
            catch (Exception e) { Fail("PARENT_SOURCE_EXCEPTION:" + e.GetType().Name); }
        }
        internal void BeginStep(PhysicsStepContext step)
        {
            Invalidate(); _preValid = false;
            if (!Bound) return;
            _physicsSequence++;
            try
            {
                if (!step.Valid || _owner == null || _owner.GetInstanceID() != _bodyId || _owner.isKinematic || !_owner.gameObject.activeInHierarchy || !_sampleValid)
                { Fail("PARENT_STEP_OWNER_OR_SOURCE_INVALID"); return; }
                _pre = new PelvisParentFrame { Step = step, Epoch = _epoch, BodyId = _bodyId, PhysicsSequence = _physicsSequence,
                    FixedTime = step.FixedTime, Dt = step.Dt, SourceCapture = _capture, SourceSequence = _sourceSequence,
                    Position = _owner.position, Rotation = _owner.rotation, ComLocal = _owner.centerOfMass,
                    ComVelocity = _owner.velocity, W = _owner.angularVelocity };
                _preValid = true;
            }
            catch (Exception e) { Fail("PARENT_NATIVE_CAPTURE_EXCEPTION:" + e.GetType().Name); }
        }
        internal void Publish(PelvisDynamicReceiverIsolation.Reference command, int sourceSequence, int traceFixed,
            bool ready, float fixedTime, float dt)
        {
            _current.Valid = false;
            if (!Bound) return;
            try
            {
                if (!ready) { Issue = "PARENT_PENDING_PASSIVE_SOLVES"; return; }
                if (!_preValid || !_sampleValid || _pre.FixedTime != fixedTime || _pre.Dt != dt || _sourceSequence != sourceSequence || _pre.SourceCapture != _capture)
                { Fail("PARENT_COMMAND_CLOCK_OR_SOURCE_MISMATCH"); return; }
                PelvisParentFrame f = _pre;
                f.TraceFixed = traceFixed; f.CommandPosition = command.PelvisPosition; f.CommandRotation = command.PelvisRotation;
                f.CommandOriginVelocity = command.PelvisVelocity; f.CommandW = command.PelvisW;
                f.Left = f.MapHip(_leftAnchor, _leftDrift, _leftRelative);
                f.Right = f.MapHip(_rightAnchor, _rightDrift, _rightRelative);
                if (!f.Finite()) { Fail("PARENT_FRAME_NONFINITE"); return; }
                f.Valid = true; _current = f; Issue = null;
            }
            catch (Exception e) { Fail("PARENT_PUBLISH_EXCEPTION:" + e.GetType().Name); }
            finally { _preValid = false; }
        }
        internal bool TryGet(float fixedTime, float dt, out PelvisParentFrame frame)
        {
            frame = new PelvisParentFrame();
            if (!Bound || !_current.Matches(_epoch, _bodyId, _physicsSequence, fixedTime, dt)) return false;
            frame = _current; return true;
        }
        internal void Invalidate() { _current.Valid = false; _preValid = false; }
        internal void Clear()
        {
            Invalidate(); Bound = _sampleValid = _preValid = false;
            _owner = null; _sourcePelvis = _left = _right = null; Issue = null;
        }
        private bool Fail(string issue) { Invalidate(); _preValid = false; Issue = issue; return false; }
    }
}
