using UnityEngine;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;
using Motion = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast.Motion;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // An unloaded Foot COM force has no linear reaction on Lower. Changing only
    // translation preserves the recorded orientations, shin trajectory and angular
    // commands. Recompute the foot PD at its own offset position AND velocity.
    internal static class LeftFootLeadSequence
    {
        internal const int Length = 3;
        private const float Radius = FootContactGeometryLimits.RecoveryMaxAnchorGapMeters;
        private const float PrecisionInset = .000001f; // Aim 1 micrometre inside; scoring uses the original 5 mm.
        internal enum Action { Invalid, Floor, HardGap, Budget, VerticalGap, NoRoom, Horizontal, Limited, Feedback }

        internal sealed class Step
        {
            internal int Origin, Local, Depth;
            internal Action Decision;
            internal Foot.Decision? Vertical;
            internal Vector3 Offset, Velocity, Applied;
            internal float Cap, Needed, Added, Rise;
            internal float BaseSole, BaseGap, TrialSole, TrialGap, KneeGap;
            internal bool Read, Inside, ActualPass, TrialPass;
            internal float ActualSole, ActualGap, ActualKnee, ConditionedSole, ConditionedGap;
            internal bool Issued => Decision == Action.Horizontal || Decision == Action.Limited || Decision == Action.Feedback;
        }

        internal static bool Requests(in Foot.Frame f, in Foot.Result issued, in LeftShinForecast.Result shin) =>
            EarlyDescent(f,issued,shin) && issued.Gap > Radius;

        internal static bool EarlyDescent(in Foot.Frame f, in Foot.Result issued, in LeftShinForecast.Result shin)
        {
            // Clear implies a five-point Lift, valid footprint and a sole still inside the lower bound.
            // This test sees only PRE_N and the commands issued for N, never the successor gait.
            var b = f.Pose.Box;
            float shinY = shin.LowerAnkle.y - b.LowerAnkle.y;
            float footY = issued.FootPoint.y - b.FootAnkle.y;
            return shin.State == LeftShinForecast.Status.Predicted && issued.State == Foot.Decision.Clear &&
                RightFootTorqueForecast.Band(issued.Sole) == RightFootTorqueForecast.HeightBand.InBand &&
                f.Pose.Gap <= Radius && shinY < 0f && footY - shinY >= KneeGroundTarget.MinGain;
        }

        internal static Step Predict(int local, Phase gait, in Foot.Frame frame, in Foot.Result original,
            in LeftShinForecast.Result shin, in Motion motion, Step previous)
        {
            var s = new Step { Origin = previous?.Origin ?? local, Depth = (previous?.Depth ?? 0) + 1,
                Local = local, Decision = Action.Invalid };
            if (local <= 0 || (previous == null && gait != Phase.Lift) || (previous != null && (!previous.Read || !previous.Inside ||
                previous.Local + 1 != local || s.Depth > Length)) || !frame.Valid ||
                !original.Predicted || shin.State != LeftShinForecast.Status.Predicted || !frame.Anchors.Valid ||
                !RightFootTorqueForecast.ValidRotation(original.FootRotation)) return s;
            var t = motion.Tether;
            float dt = frame.Pose.Box.Dt;
            float damping = Mathf.Max(0f, 1f - motion.LinearDrag * dt);
            float response = dt * dt * damping;
            if (!float.IsFinite(response) || response <= 0f || !float.IsFinite(motion.LinearDrag) || motion.LinearDrag < 0f) return s;
            s.Cap = Mathf.Min(t.BaseCap, t.SelectedCap);
            if (!float.IsFinite(t.BaseCap) || !float.IsFinite(t.SelectedCap) || s.Cap <= 0f ||
                !float.IsFinite(t.Kp) || t.Kp <= 0f || !float.IsFinite(t.Kd) || t.Kd < 0f ||
                !float.IsFinite(t.HardGap) || t.HardGap <= Radius || !HipPointPairMobility.V(t.Raw) ||
                !float.IsFinite(t.Raw.sqrMagnitude) || !HipPointPairMobility.V(motion.AppliedLinearA) ||
                (Vector3.ClampMagnitude(t.Raw,t.SelectedCap) - motion.AppliedLinearA).sqrMagnitude > 1e-8f ||
                motion.AppliedLinearA.magnitude > s.Cap + .00001f)
            { s.Decision = Action.Budget; return s; }

            var b = frame.Pose.Box;
            var oldOffset = previous?.Offset ?? Vector3.zero;
            var oldVelocity = previous?.Velocity ?? Vector3.zero;
            float startGap = (b.FootAnkle - b.LowerAnkle + oldOffset).magnitude;
            if (startGap >= t.HardGap) { s.Decision = Action.HardGap; return s; }
            if (!frame.FivePoint || !Inside(b,b.FootAnkle + oldOffset,b.FootR) || original.State == Foot.Decision.Outside)
            { s.Decision = Action.Floor; return s; }

            var baseDelta = original.FootPoint - shin.LowerAnkle;
            s.BaseGap = baseDelta.magnitude;
            s.BaseSole = original.Sole;
            s.KneeGap = (shin.UpperKnee - shin.LowerKnee).magnitude;
            var raw = t.Raw - t.Kp * oldOffset - t.Kd * oldVelocity;
            if (!HipPointPairMobility.V(raw) || !float.IsFinite(raw.sqrMagnitude)) return s;
            s.Applied = previous == null ? motion.AppliedLinearA : Vector3.ClampMagnitude(raw,s.Cap);
            if (previous == null)
            {
                if (!Requests(frame,original,shin)) return s;
                s.Decision = FitHorizontal(baseDelta,response,s.Cap,ref s.Applied,out s.Needed,out s.Added);
                if (!s.Issued) return s;
            }
            else s.Decision = Action.Feedback;

            Advance(s,oldOffset,oldVelocity,motion.AppliedLinearA,dt,damping);
            if (s.Depth == 2)
            {
                // The recorded parent transport and angular inputs are shared. At
                // this different foot state the native child-only spring/damper is not.
                var shifted = frame;
                shifted.Pose.Box.FootAnkle += oldOffset;
                shifted.Pose.Box.Box += oldOffset.y;
                shifted.Pose.Com += oldOffset;
                shifted.Pose.Velocity += oldVelocity;
                shifted.Pose.PresentGoal += oldOffset.y;
                shifted.Pose.Gap = startGap;
                var alt = motion;
                alt.AppliedLinearA = s.Applied;
                alt.Tether.Raw = raw;
                alt.Tether.SelectedCap = s.Cap;
                var point = original.FootPoint + s.Offset;
                bool outside = !Inside(b,point,original.FootRotation);
                var vertical = Foot.CompareEndpoint(shifted,gait,alt,shin.LowerAnkle,point,original.FootRotation,
                    original.Sole + s.Offset.y,outside);
                s.Vertical = vertical.State;
                if (vertical.State == Foot.Decision.Trial)
                {
                    s.Applied.y = vertical.TrialY;
                    s.Rise = vertical.Rise;
                    Advance(s,oldOffset,oldVelocity,motion.AppliedLinearA,dt,damping);
                }
            }
            s.TrialSole = original.Sole + s.Offset.y;
            s.TrialGap = (baseDelta + s.Offset).magnitude;
            if (!HipPointPairMobility.V(s.Velocity) || !HipPointPairMobility.V(s.Offset) ||
                !float.IsFinite(s.TrialSole) || !float.IsFinite(s.TrialGap) || !float.IsFinite(s.KneeGap) ||
                !float.IsFinite(s.Applied.magnitude) || s.Applied.magnitude > s.Cap + .00001f) s.Decision = Action.Invalid;
            else if (!Inside(b,original.FootPoint + s.Offset,original.FootRotation)) s.Decision = Action.Floor;
            else if (s.TrialGap >= t.HardGap) s.Decision = Action.HardGap;
            return s;
        }

        internal static Action FitHorizontal(Vector3 gap, float response, float cap, ref Vector3 applied,
            out float needed, out float added)
        {
            needed = added = 0f;
            float y = Mathf.Abs(gap.y);
            if (y > Radius) return Action.VerticalGap;
            var planar = new Vector3(gap.x,0f,gap.z);
            float distance = planar.magnitude;
            if (distance <= 0f) return Action.NoRoom;
            var direction = -planar / distance;
            needed = Mathf.Max(0f,distance - Slice(Radius,y));
            float target = Mathf.Max(y,Radius - PrecisionInset);
            float request = Mathf.Max(0f,distance - Slice(target,y)) / response;
            // Positive root of |issued + k*direction|² = cap². Keeping Y
            // fixed also preserves the original acceleration perpendicular to this ray.
            float along = Vector3.Dot(applied,direction);
            float headroom = Mathf.Max(0f,cap * cap - applied.sqrMagnitude);
            float root = Mathf.Sqrt(along * along + headroom);
            float available = along > 0f ? headroom / (root + along) : root - along;
            float extra = Mathf.Min(request,available);
            if (extra <= 0f || !float.IsFinite(extra)) return Action.NoRoom;
            applied += direction * extra;
            added = extra * response;
            return extra < request ? Action.Limited : Action.Horizontal;
        }

        private static float Slice(float radius, float y) => Mathf.Sqrt(Mathf.Max(0f,(radius - y) * (radius + y)));
        private static void Advance(Step s, Vector3 offset, Vector3 velocity, Vector3 baselineA, float dt, float damping)
        {
            s.Velocity = (velocity + (s.Applied - baselineA) * dt) * damping;
            s.Offset = offset + s.Velocity * dt;
        }

        private static bool Inside(in RightKneePivot.Frame b, Vector3 point, Quaternion r) =>
            FootBoxGeometry.Inside(point,r,b.Center,b.Half,b.FloorCenter,b.FloorHalf);

        private static bool Pass(float sole, float gap) => RightFootTorqueForecast.Band(sole) ==
            RightFootTorqueForecast.HeightBand.InBand && gap <= Radius;

        internal static void Readback(Step s, in Foot.Frame next)
        {
            if (s == null || !s.Issued || s.Read || !next.Valid || !next.Anchors.Valid || !next.FivePoint) return;
            var b = next.Pose.Box;
            var gap = b.FootAnkle - b.LowerAnkle;
            s.ActualSole = b.Box;
            s.ActualGap = gap.magnitude;
            s.ActualKnee = (next.Anchors.UpperKnee - next.Anchors.LowerKnee).magnitude;
            s.ConditionedSole = b.Box + s.Offset.y;
            s.ConditionedGap = (gap + s.Offset).magnitude;
            s.Read = true;
            bool actualInside = Inside(b,b.FootAnkle,b.FootR);
            s.Inside = actualInside && Inside(b,b.FootAnkle + s.Offset,b.FootR);
            s.ActualPass = actualInside && Pass(s.ActualSole,s.ActualGap);
            s.TrialPass = s.Inside && Pass(s.TrialSole,s.TrialGap) && Pass(s.ConditionedSole,s.ConditionedGap);
        }
    }
}
