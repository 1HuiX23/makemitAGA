using UnityEngine;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;
using Motion = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast.Motion;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;
using Shin = MakemitAGA.ActiveRagdollLab.Physics.LeftShinForecast;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Free Lower and Foot each have a child-only COM tether. A translation changes
    // neither body's rotation. The recorded hip, both angular commands and support
    // plane are consequently shared. The foot DOES see the changed lower velocity,
    // including its native one-step finite-difference parent acceleration.
    internal static partial class LeftLegHeightPreparation
    {
        internal const int Length = 3;
        private const float AnkleLimit = FootContactGeometryLimits.RecoveryMaxAnchorGapMeters;
        private const float KneePrecision = .000001f;

        internal enum Group { None, Descent, RisingControl }
        internal enum Action { Unavailable, Budget, Floor, HardGap, BelowResolution, NoJointRoom, Prepare, Feedback, Control }

        internal struct KneeMotion
        {
            internal Vector3 Raw, Applied;
            internal float Kp, Kd, Cap, Drag, HardGap;
        }

        internal struct Translation
        {
            internal Vector3 Position, Velocity;
            internal Translation Advance(Vector3 deltaA, float dt, float damping)
            {
                var nextV = (Velocity + deltaA * dt) * damping;
                return new Translation { Position = Position + nextV * dt, Velocity = nextV };
            }
        }

        internal sealed class Step
        {
            internal int Local, Depth;
            internal Group OriginGroup;
            internal Action Decision;
            internal bool Issued => Decision == Action.Prepare || Decision == Action.Feedback || Decision == Action.Control;
            internal bool Read, Inside, ActualPass, TrialPass, ReadHard;
            internal Translation Lower, FootBody;
            internal CombinedFoot Combined;
            internal Vector3 LowerDeltaV, KneeA, FootA, ParentDeltaA;
            internal float AvailableRise, SeedRise;
            internal float PredSole, PredAnkle, PredKnee, BaseKnee;
            internal float ActualSole, ActualAnkle, ActualKnee, TrialSole, TrialAnkle, TrialKnee, BestFixedPose;
        }

        internal static Group Classify(Phase phase, in Foot.Frame f, in Foot.Result foot, in Shin.Result shin)
        {
            if (phase != Phase.Lift || !f.FivePoint || !f.Valid || !f.Anchors.Valid || !foot.Predicted ||
                shin.State != Shin.Status.Predicted || foot.State == Foot.Decision.Outside) return Group.None;
            if (LeftFootLeadSequence.EarlyDescent(f,foot,shin)) return Group.Descent;
            var b = f.Pose.Box;
            return shin.LowerAnkle.y - b.LowerAnkle.y >= KneeGroundTarget.MinGain &&
                foot.FootPoint.y - b.FootAnkle.y >= KneeGroundTarget.MinGain ? Group.RisingControl : Group.None;
        }

        private static bool Finite(Vector3 a) => HipPointPairMobility.V(a) && float.IsFinite(a.sqrMagnitude);
        private static bool ValidLaw(Vector3 raw, Vector3 applied, float selected, float cap, float kp, float kd, float hard)
        {
            return Finite(raw) && Finite(applied) && float.IsFinite(selected) && float.IsFinite(cap) &&
                cap > 0f && selected >= cap && float.IsFinite(kp) && kp > 0f && float.IsFinite(kd) && kd >= 0f &&
                float.IsFinite(hard) && hard > AnkleLimit && applied.magnitude <= cap + .00001f &&
                (Vector3.ClampMagnitude(raw,selected)-applied).sqrMagnitude <= 1e-8f;
        }

        internal static Step Predict(int local, Group group, Phase phase, in Foot.Frame frame, in Foot.Result foot,
            in Shin.Result shin, in Motion motion, Step previous)
        {
            var s = new Step { Local = local, Depth = (previous?.Depth ?? 0) + 1, OriginGroup = group };
            if (local <= 0 || group == Group.None || !frame.Valid || !frame.Anchors.Valid || !foot.Predicted ||
                shin.State != Shin.Status.Predicted || !RightFootTorqueForecast.ValidRotation(foot.FootRotation) ||
                (previous == null ? Classify(phase,frame,foot,shin) != group :
                    previous.OriginGroup != group || !previous.Read || !previous.Inside || previous.ReadHard || previous.Local+1 != local || s.Depth > Length))
                return s;

            var knee = shin.Knee;
            var tether = motion.Tether;
            float cap = Mathf.Min(tether.BaseCap,tether.SelectedCap);
            var box = frame.Pose.Box;
            float dt = box.Dt;
            float lowerDamping = Mathf.Max(0f,1f-knee.Drag*dt);
            float footDamping = Mathf.Max(0f,1f-motion.LinearDrag*dt);
            if (!motion.MeasuredLowerTransport || !float.IsFinite(tether.BaseCap) ||
                !float.IsFinite(knee.Drag) || knee.Drag < 0f || !float.IsFinite(motion.LinearDrag) || motion.LinearDrag < 0f ||
                dt > .1f || lowerDamping <= 0f || footDamping <= 0f ||
                !ValidLaw(knee.Raw,knee.Applied,knee.Cap,knee.Cap,knee.Kp,knee.Kd,knee.HardGap) ||
                !ValidLaw(tether.Raw,motion.AppliedLinearA,tether.SelectedCap,cap,tether.Kp,tether.Kd,tether.HardGap))
            { s.Decision = Action.Budget; return s; }

            var lower = previous?.Lower ?? default;
            var child = previous?.FootBody ?? default;
            if (!frame.FivePoint || foot.State == Foot.Decision.Outside || !Inside(box,box.FootAnkle+child.Position,box.FootR))
            { s.Decision = Action.Floor; return s; }
            if ((frame.Anchors.LowerKnee-frame.Anchors.UpperKnee+lower.Position).magnitude > knee.HardGap ||
                (box.FootAnkle-box.LowerAnkle+child.Position-lower.Position).magnitude >= tether.HardGap)
            { s.Decision = Action.HardGap; return s; }

            var kneeGap = shin.LowerKnee-shin.UpperKnee;
            var ankleGap = foot.FootPoint-shin.LowerAnkle;
            s.BaseKnee = kneeGap.magnitude;
            s.KneeA = knee.Applied;
            s.FootA = motion.AppliedLinearA;
            if (group == Group.RisingControl) s.Decision = Action.Control;
            else if (previous == null)
            {
                float response = dt*dt*lowerDamping;
                float lowerDY = shin.LowerAnkle.y-box.LowerAnkle.y;
                float relativeDY = foot.FootPoint.y-box.FootAnkle.y-lowerDY;
                s.AvailableRise = Mathf.Max(0f,Mathf.Min(Mathf.Min(-shin.FeedforwardCom.y,-knee.Applied.y*response),
                    Mathf.Min(-lowerDY,relativeDY)));
                s.Decision = ChooseSeed(knee,kneeGap,ankleGap,response,s);
                if (!s.Issued) return s;
            }
            else
            {
                // Upper follows the committed hip trajectory. Re-evaluate both springs
                // at their own point positions and velocities before the native 3D clamp.
                var kneeRaw = knee.Raw - lower.Position*knee.Kp - lower.Velocity*knee.Kd;
                s.ParentDeltaA = previous.LowerDeltaV / dt;
                var footRaw = FootRaw(tether,lower,child,s.ParentDeltaA);
                if (!Finite(kneeRaw) || !Finite(footRaw)) return s;
                s.KneeA = Vector3.ClampMagnitude(kneeRaw,knee.Cap);
                s.FootA = Vector3.ClampMagnitude(footRaw,cap);
                s.Decision = Action.Feedback;
            }

            s.Lower = lower.Advance(s.KneeA-knee.Applied,dt,lowerDamping);
            s.LowerDeltaV = s.Lower.Velocity-lower.Velocity;
            s.FootBody = child.Advance(s.FootA-motion.AppliedLinearA,dt,footDamping);
            s.PredSole = foot.Sole+s.FootBody.Position.y;
            s.PredAnkle = (ankleGap+s.FootBody.Position-s.Lower.Position).magnitude;
            s.PredKnee = (kneeGap+s.Lower.Position).magnitude;
            if (!Finite(s.Lower.Position) || !Finite(s.FootBody.Position) || !Finite(s.Lower.Velocity) ||
                !Finite(s.FootBody.Velocity) || !float.IsFinite(s.PredKnee) || !float.IsFinite(s.PredAnkle) ||
                !float.IsFinite(s.PredSole)) s.Decision = Action.Unavailable;
            else if (!Inside(box,foot.FootPoint+s.FootBody.Position,foot.FootRotation)) s.Decision = Action.Floor;
            else if (s.PredKnee > knee.HardGap || s.PredAnkle >= tether.HardGap) s.Decision = Action.HardGap;
            if (s.Issued && group == Group.Descent)
                PredictCombined(s,previous,phase,frame,foot,shin,motion,lower,footDamping,cap);
            return s;
        }

        private static Vector3 FootRaw(in RightAnkleLandingSequence.Tether tether, Translation lower,
            Translation foot, Vector3 deltaParentA) => tether.Raw + (lower.Position-foot.Position)*tether.Kp +
                (lower.Velocity-foot.Velocity)*tether.Kd + deltaParentA;

        private static Action ChooseSeed(in KneeMotion knee, Vector3 joint, Vector3 ankle, float response, Step s)
        {
            // Reduce at most the actual downward parent feedforward. A lift of Lower
            // may open the knee: keep the predicted knee gap inside its original sphere
            // with the minimum necessary XZ closure toward the upper-side knee point.
            float radius = joint.magnitude;
            float rise = Mathf.Min(s.AvailableRise,radius-joint.y);
            if (s.AvailableRise < KneeGroundTarget.MinGain) return Action.BelowResolution;
            var flat = new Vector3(joint.x,0f,joint.z);
            for (int i=2; i>=0; i--)
            {
                float y = rise*RightAnkleLandingSequence.Fraction(i);
                if (y < KneeGroundTarget.MinGain) break;
                float signedY = joint.y+y;
                float slice = Mathf.Sqrt(Mathf.Max(0f,(radius-signedY)*(radius+signedY)));
                var delta = Vector3.up*y;
                if (flat.magnitude > slice) delta -= flat*((flat.magnitude-slice)/flat.magnitude);
                var candidate = knee.Applied+delta/response;
                if (!Finite(candidate) || candidate.magnitude > knee.Cap || candidate.y > .00001f ||
                    (joint+delta).magnitude > radius+KneePrecision ||
                    (ankle-delta).magnitude > Mathf.Max(AnkleLimit,ankle.magnitude)) continue;
                s.SeedRise = y; s.KneeA = candidate;
                return Action.Prepare;
            }
            return Action.NoJointRoom;
        }

        private static bool Inside(in RightKneePivot.Frame box, Vector3 ankle, Quaternion rotation) =>
            FootBoxGeometry.Inside(ankle,rotation,box.Center,box.Half,box.FloorCenter,box.FloorHalf);

        private static bool Pass(float sole, float ankle) =>
            RightFootTorqueForecast.Band(sole)==RightFootTorqueForecast.HeightBand.InBand && ankle <= AnkleLimit;

        internal static void Readback(Step s, in Foot.Frame next, float kneeHard, float footHard)
        {
            if (s==null || !s.Issued || s.Read || !next.Valid || !next.FivePoint || !next.Anchors.Valid) return;
            var b = next.Pose.Box;
            var ankle = b.FootAnkle-b.LowerAnkle;
            var knee = next.Anchors.LowerKnee-next.Anchors.UpperKnee;
            s.ActualSole = b.Box; s.ActualAnkle = ankle.magnitude; s.ActualKnee = knee.magnitude;
            s.TrialSole = b.Box+s.FootBody.Position.y;
            s.TrialAnkle = (ankle+s.FootBody.Position-s.Lower.Position).magnitude;
            s.TrialKnee = (knee+s.Lower.Position).magnitude;
            s.BestFixedPose = b.Box+AnkleLimit-ankle.y;
            s.Read = true;
            bool actualInside = Inside(b,b.FootAnkle,b.FootR);
            s.Inside = actualInside && Inside(b,b.FootAnkle+s.FootBody.Position,b.FootR);
            s.ReadHard = s.TrialAnkle>=footHard || s.TrialKnee>kneeHard;
            s.ActualPass = actualInside && Pass(s.ActualSole,s.ActualAnkle);
            s.TrialPass = s.Inside && !s.ReadHard && Pass(s.PredSole,s.PredAnkle) &&
                Pass(s.TrialSole,s.TrialAnkle) && s.PredKnee<=s.BaseKnee+KneePrecision &&
                s.TrialKnee<=s.ActualKnee+KneePrecision;
            ReadCombined(s,next,kneeHard,footHard,actualInside);
        }
    }
}
