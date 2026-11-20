using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Same unloaded solve for both bodies. Hip force is applied at its point on
    // Upper. Lower receives COM acceleration, direct angular acceleration and
    // the opposite ankle torque. These three native force modes stay distinct.
    internal static class LeftShinForecast
    {
        internal enum Status { NotSampled, MissingReceipt, MissingFrame, MissingHipReceipt, MissingKneeReceipt,
            AngularMismatch, Loaded, BodyMismatch, InvalidInput, Predicted, Fault }

        internal struct Body
        {
            internal Vector3 Com, V, W, Inertia;
            internal Quaternion Principal;
            internal float Mass, Drag, AngularDrag, MaxW;

            internal bool Valid => HipPointPairMobility.ValidBody(Mass, Inertia, Principal) &&
                RightFootTorqueForecast.ValidRotation(Principal) && HipPointPairMobility.V(Com) &&
                HipPointPairMobility.V(V) && HipPointPairMobility.V(W) && float.IsFinite(Drag) && Drag >= 0f &&
                float.IsFinite(AngularDrag) && AngularDrag >= 0f && float.IsFinite(MaxW) && MaxW > 0f;
        }

        internal struct Commands
        {
            internal Vector3 HipForce, HipPoint, UpperTorque, CommonA;
            internal Vector3 KneeTetherA, SpringA, DamperA, ParentFeedforwardA;
            internal Vector3 HipTransportAlpha, KneeRelativeAlpha, AnkleReactionNm;
            internal float TetherCap;
        }

        internal struct Result
        {
            internal Status State;
            internal Vector3 Hip, UpperKnee, LowerKnee, LowerAnkle;
            // Each displacement below is in world metres. Translation and rotation
            // sum to the full lower-side ankle displacement (up to floating-point rounding).
            internal Vector3 CoastCom, CommonCom, SpringCom, DamperCom, FeedforwardCom, Turn;
            internal Vector3 HipForce, UpperNetMoment, HipTransportAlpha, KneeRelativeAlpha, AnkleReactionAlpha;
        }

        private struct Advance
        {
            internal Vector3 Before, After;
            internal Quaternion Rotation;
            internal Vector3 Point(Vector3 point) => After + Rotation * (point - Before);
        }

        private static Advance Integrate(in Body body, Vector3 a, Vector3 alpha, float dt)
        {
            float linearDamping = Mathf.Max(0f, 1f - body.Drag * dt);
            float angularDamping = Mathf.Max(0f, 1f - body.AngularDrag * dt);
            var w = Vector3.ClampMagnitude((body.W + alpha * dt) * angularDamping, body.MaxW);
            return new Advance {
                Before = body.Com, After = body.Com + (body.V + a * dt) * (linearDamping * dt),
                Rotation = FootRelativeReference.Advance(Quaternion.identity, w, dt)
            };
        }

        internal static Result Predict(float dt, in LegMeasuredAnchors anchors, Vector3 ankle,
            in Body upper, in Body lower, in Commands cmd)
        {
            var invalid = new Result { State = Status.InvalidInput };
            if (!float.IsFinite(dt) || dt <= 0f || dt > .1f || !anchors.Valid || !upper.Valid || !lower.Valid ||
                !HipPointPairMobility.V(ankle) || !HipPointPairMobility.V(cmd.HipPoint) ||
                !HipPointPairMobility.V(cmd.HipForce) || !HipPointPairMobility.V(cmd.UpperTorque) ||
                !HipPointPairMobility.V(cmd.CommonA) || !HipPointPairMobility.V(cmd.HipTransportAlpha) ||
                !HipPointPairMobility.V(cmd.KneeRelativeAlpha) || !HipPointPairMobility.V(cmd.AnkleReactionNm) ||
                !HipPointPairMobility.V(cmd.SpringA) || !HipPointPairMobility.V(cmd.DamperA) ||
                !HipPointPairMobility.V(cmd.ParentFeedforwardA) || !HipPointPairMobility.V(cmd.KneeTetherA) ||
                !float.IsFinite(cmd.TetherCap) || cmd.TetherCap <= 0f) return invalid;

            var raw = cmd.ParentFeedforwardA + cmd.SpringA + cmd.DamperA;
            if (!HipPointPairMobility.V(raw) || !RightThighTracking.Same(Vector3.ClampMagnitude(raw, cmd.TetherCap), cmd.KneeTetherA))
                return invalid;
            // The committed, possibly capped COM vector drives the forecast. Its
            // uniformly scaled P/D/FF components explain the same native write.
            float scale = raw.magnitude > cmd.TetherCap ? cmd.TetherCap / raw.magnitude : 1f;
            var upperMoment = Vector3.Cross(cmd.HipPoint - upper.Com, cmd.HipForce) + cmd.UpperTorque;
            var upperAlpha = HipPointPairMobility.InverseTensor(upper.Inertia, upper.Principal, upperMoment);
            var reactionAlpha = HipPointPairMobility.InverseTensor(lower.Inertia, lower.Principal, cmd.AnkleReactionNm);
            var lowerAlpha = cmd.HipTransportAlpha + cmd.KneeRelativeAlpha + reactionAlpha;
            var upperA = cmd.CommonA + cmd.HipForce / upper.Mass;
            var lowerA = cmd.CommonA + cmd.KneeTetherA;
            if (!HipPointPairMobility.V(upperAlpha) || !HipPointPairMobility.V(lowerAlpha) ||
                !HipPointPairMobility.V(upperA) || !HipPointPairMobility.V(lowerA)) return invalid;

            var up = Integrate(upper, upperA, upperAlpha, dt);
            var low = Integrate(lower, lowerA, lowerAlpha, dt);
            float velocityTime = dt * Mathf.Max(0f, 1f - lower.Drag * dt);
            float accelerationTime = dt * velocityTime;
            var lever = ankle - lower.Com;
            var r = new Result {
                State = Status.Predicted, Hip = up.Point(anchors.UpperHip), UpperKnee = up.Point(anchors.UpperKnee),
                LowerKnee = low.Point(anchors.LowerKnee), LowerAnkle = low.Point(ankle),
                CoastCom = lower.V * velocityTime, CommonCom = cmd.CommonA * accelerationTime,
                SpringCom = cmd.SpringA * (scale * accelerationTime), DamperCom = cmd.DamperA * (scale * accelerationTime),
                FeedforwardCom = cmd.ParentFeedforwardA * (scale * accelerationTime),
                Turn = low.Rotation * lever - lever, HipForce = cmd.HipForce, UpperNetMoment = upperMoment,
                HipTransportAlpha = cmd.HipTransportAlpha,
                KneeRelativeAlpha = cmd.KneeRelativeAlpha, AnkleReactionAlpha = reactionAlpha
            };
            return HipPointPairMobility.V(r.Hip) && HipPointPairMobility.V(r.UpperKnee) &&
                HipPointPairMobility.V(r.LowerKnee) && HipPointPairMobility.V(r.LowerAnkle) ? r : invalid;
        }
    }
}
