using UnityEngine;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // One immutable-by-value handoff: settled native state BEFORE command N and the
    // final governed command N. Rigidbody.velocity is COM velocity, not origin velocity.
    // No force, target acceleration or whole-leg certification is implied by this frame.
    internal struct PelvisParentFrame
    {
        internal bool Valid;
        internal PhysicsStepContext Step;
        internal int Epoch, PhysicsSequence, TraceFixed, BodyId, SourceCapture, SourceSequence;
        internal float FixedTime, Dt;
        internal Vector3 Position, ComLocal, ComVelocity, W;
        internal Quaternion Rotation;
        internal Vector3 CommandPosition, CommandOriginVelocity, CommandW;
        internal Quaternion CommandRotation;
        internal HipReference Left, Right;

        internal struct HipReference
        {
            internal Vector3 LocalAnchor, SourceLocalDrift;
            internal Quaternion SourceRelativeRotation;
            internal Vector3 SettledPoint, SettledVelocity, CommandPoint, CommandVelocity;
            internal Quaternion InitialUpperRotation, CommandUpperRotation;
        }

        internal Vector3 SettledPoint(Vector3 local) => Position + Rotation * local;
        internal Vector3 SettledPointVelocity(Vector3 local) => ComVelocity + Vector3.Cross(W, Rotation * (local - ComLocal));
        internal Vector3 TargetPoint(Vector3 local) => CommandPosition + CommandRotation * local;
        internal Vector3 TargetPointVelocity(Vector3 local) => CommandOriginVelocity + Vector3.Cross(CommandW, CommandRotation * local);
        internal HipReference MapHip(Vector3 local, Vector3 drift, Quaternion sourceRelative) => new HipReference
        {
            LocalAnchor = local, SourceLocalDrift = drift, SourceRelativeRotation = sourceRelative,
            SettledPoint = SettledPoint(local), SettledVelocity = SettledPointVelocity(local),
            CommandPoint = TargetPoint(local), CommandVelocity = TargetPointVelocity(local),
            InitialUpperRotation = Rotation * sourceRelative, CommandUpperRotation = CommandRotation * sourceRelative
        };
        internal bool Matches(int epoch, int bodyId, int sequence, float fixedTime, float dt) =>
            Valid && Epoch == epoch && BodyId == bodyId && PhysicsSequence == sequence && FixedTime == fixedTime && Dt == dt;
        internal bool Matches(PhysicsStepContext step) => Valid && Step.Matches(step) && FixedTime == step.FixedTime && Dt == step.Dt;
        internal bool Finite() => Epoch > 0 && PhysicsSequence > 0 && BodyId != 0 && SourceCapture > 0 && SourceSequence >= 0 &&
            F(FixedTime) && F(Dt) && Dt > 0f && V(Position) && V(ComLocal) && V(ComVelocity) && V(W) && Q(Rotation) &&
            V(CommandPosition) && V(CommandOriginVelocity) && V(CommandW) && Q(CommandRotation) && H(Left) && H(Right);
        private static bool H(HipReference h) => V(h.LocalAnchor) && V(h.SourceLocalDrift) && Q(h.SourceRelativeRotation) &&
            V(h.SettledPoint) && V(h.SettledVelocity) && V(h.CommandPoint) && V(h.CommandVelocity) && Q(h.InitialUpperRotation) && Q(h.CommandUpperRotation);
        internal static bool F(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        internal static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z) && F(v.sqrMagnitude);
        internal static bool Q(Quaternion q) => F(q.x) && F(q.y) && F(q.z) && F(q.w) &&
            Mathf.Abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1f) < 0.01f;
    }
}
