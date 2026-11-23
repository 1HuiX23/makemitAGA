using UnityEngine;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;
using Lead = MakemitAGA.ActiveRagdollLab.Physics.LeftFootLeadSequence;
using Motion = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast.Motion;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;
using Shin = MakemitAGA.ActiveRagdollLab.Physics.LeftShinForecast;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal static partial class LeftLegHeightPreparation
    {
        // Same upper and lower path as the knee-only alternative. The paired
        // trial additionally uses the already bounded 166 horizontal/vertical
        // foot law, evaluated at this alternative's own ankle and velocity.
        internal sealed class CombinedFoot
        {
            internal Translation Body;
            internal Vector3 Applied;
            internal Lead.Action? Horizontal;
            internal Foot.Decision? Vertical;
            internal float AddedXZ, Rise, Sole, Ankle;
            internal float PredSole = float.NaN, PredAnkle = float.NaN;
            internal bool Read, Pass;
            internal string End;
        }

        private static void PredictCombined(Step s, Step previous, Phase phase, in Foot.Frame frame,
            in Foot.Result foot, in Shin.Result shin, in Motion motion, Translation oldLower, float damping, float cap)
        {
            if (previous != null && (previous.Combined == null || !previous.Combined.Read || previous.Combined.End != null)) return;
            var combined = s.Combined = new CombinedFoot();
            var oldFoot = previous?.Combined.Body ?? default;
            var box = frame.Pose.Box;
            float dt = box.Dt;
            if (!Inside(box,box.FootAnkle+oldFoot.Position,box.FootR))
            { combined.End="StartFootprint"; return; }
            var shifted = frame;
            shifted.Pose.Box.FootAnkle += oldFoot.Position;
            shifted.Pose.Box.LowerAnkle += oldLower.Position;
            shifted.Pose.Gap = (shifted.Pose.Box.FootAnkle-shifted.Pose.Box.LowerAnkle).magnitude;
            if (shifted.Pose.Gap >= motion.Tether.HardGap) { combined.End="StartHardGap"; return; }
            shifted.Pose.Com += oldFoot.Position;
            shifted.Pose.Velocity += oldFoot.Velocity;
            shifted.Pose.Box.Box += oldFoot.Position.y;
            shifted.Pose.PresentGoal += oldFoot.Position.y;

            Vector3 lowerEnd = shin.LowerAnkle+s.Lower.Position;
            Vector3 raw = FootRaw(motion.Tether,oldLower,oldFoot,s.ParentDeltaA);
            if (!Finite(raw)) { combined.End="InvalidTether"; return; }
            combined.Applied = previous == null ? motion.AppliedLinearA : Vector3.ClampMagnitude(raw,cap);
            var gap = foot.FootPoint-lowerEnd;
            if (s.Depth == 1 && gap.magnitude > AnkleLimit)
                combined.Horizontal = Lead.FitHorizontal(gap,dt*dt*damping,cap,ref combined.Applied,out _,out combined.AddedXZ);
            combined.Body = oldFoot.Advance(combined.Applied-motion.AppliedLinearA,dt,damping);
            if (s.Depth == 2)
            {
                var alt = motion;
                alt.AppliedLinearA = combined.Applied;
                alt.Tether.Raw = raw;
                alt.Tether.SelectedCap = cap;
                var trialEnd = foot.FootPoint+combined.Body.Position;
                var vertical = Foot.CompareEndpoint(shifted,phase,alt,lowerEnd,trialEnd,foot.FootRotation,
                    foot.Sole+combined.Body.Position.y,!Inside(box,trialEnd,foot.FootRotation));
                combined.Vertical = vertical.State;
                if (vertical.State == Foot.Decision.Trial)
                {
                    combined.Applied.y = vertical.TrialY;
                    combined.Rise = vertical.Rise;
                    combined.Body = oldFoot.Advance(combined.Applied-motion.AppliedLinearA,dt,damping);
                }
            }
            combined.PredSole = foot.Sole+combined.Body.Position.y;
            combined.PredAnkle = (gap+combined.Body.Position).magnitude;
            if (!Finite(combined.Body.Velocity) || !Finite(combined.Body.Position) || !Finite(combined.Applied) ||
                !float.IsFinite(combined.PredSole) || !float.IsFinite(combined.PredAnkle) ||
                combined.Applied.magnitude > cap+.00001f) combined.End="InvalidCommand";
            else if (!Inside(box,foot.FootPoint+combined.Body.Position,foot.FootRotation)) combined.End="Footprint";
            else if (combined.PredAnkle >= motion.Tether.HardGap) combined.End="HardGap";
        }

        private static void ReadCombined(Step s, in Foot.Frame next, float kneeHard, float footHard, bool actualInside)
        {
            var c = s.Combined;
            if (c == null || c.End != null) return;
            var b = next.Pose.Box;
            c.Sole = b.Box+c.Body.Position.y;
            c.Ankle = (b.FootAnkle-b.LowerAnkle+c.Body.Position-s.Lower.Position).magnitude;
            c.Read = true;
            if (!actualInside || !Inside(b,b.FootAnkle+c.Body.Position,b.FootR)) c.End="ReadFootprint";
            else if (c.Ankle>=footHard || s.TrialKnee>kneeHard) c.End="ReadHardGap";
            else c.Pass = Pass(c.PredSole,c.PredAnkle) && Pass(c.Sole,c.Ankle) &&
                s.PredKnee<=s.BaseKnee+KneePrecision && s.TrialKnee<=s.ActualKnee+KneePrecision;
        }
    }
}
