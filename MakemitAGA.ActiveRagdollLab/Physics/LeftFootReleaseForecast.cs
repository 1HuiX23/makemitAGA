using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Forecast = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One unloaded solve. The issued hip/knee/foot torques and the horizontal tether
    // are common to both forecasts. A smaller downward Foot COM acceleration has no
    // opposite Lower force in the production no-joint, child-only tether.
    internal static class LeftFootReleaseForecast
    {
        internal enum Decision { MissingPose, MissingCommand, Loaded, Outside, NoFloorWitness, OtherPhase,
            OutsideStartBand, Clear, StartGap, ProtectedBudget, NoDownwardTether, CommonDescent, Gap, Subthreshold, Trial }

        internal struct Frame
        {
            internal Forecast.Pose Pose;
            internal Vector3 LowerVelocity;
            internal int FootId, LowerId, UpperId;
            internal LegMeasuredAnchors Anchors;
            internal float SourceAnkleY, HipY, KneeY;
            internal bool FivePoint, KneeExamined;
            internal SwingKneeCandidate.Result Knee;
            internal bool Valid => FootId != 0 && LowerId != 0 && FootId != LowerId &&
                float.IsFinite(Pose.Box.Dt) && Pose.Box.Dt > 0f && Forecast.Valid(Pose) && PelvisParentFrame.V(LowerVelocity) && float.IsFinite(SourceAnkleY);
        }

        internal struct Result
        {
            internal Decision State;
            internal bool Predicted;
            internal Vector3 FootPoint;
            internal Quaternion FootRotation;
            internal float LowerY, FootY, Sole, Gap, AppliedY, TrialY, Cap, AvailableRise, Rise, TrialGap;
        }

        internal static bool UnloadedFoot(bool contact, bool collider, bool nativeGravity, bool scheduledGravity, int scheduledFixed, bool loadEligible) =>
            !contact && !collider && !nativeGravity && !scheduledGravity && scheduledFixed == 0 && !loadEligible;

        private const float Depth = FootContactGeometryLimits.OrdinaryPenetrationMeters;
        private const float GapLimit = FootContactGeometryLimits.RecoveryMaxAnchorGapMeters;

        internal static bool CanCompareKnee(in Frame f, Phase phase)
        {
            var b = f.Pose.Box;
            return phase == Phase.Lift && f.FivePoint && f.Valid && b.Box >= -Depth &&
                b.Box <= FootContactGeometryLimits.RecoveryMaxGapMeters && f.Pose.Gap <= GapLimit &&
                FootBoxGeometry.Inside(b.FootAnkle, b.FootR, b.Center, b.Half, b.FloorCenter, b.FloorHalf);
        }

        internal static SwingKneeCandidate.Result CompareKnee(in RightKneePivot.Frame b, in LegMeasuredAnchors anchors,
            SwingKneeCandidate.Pose pose, in KneeReferenceState k, in PhysicsStepContext step, in SourceStepContext source)
        {
            if (!anchors.Valid) return new SwingKneeCandidate.Result { Reason = SwingKneeCandidate.Status.Unavailable };
            // Each lane starts from the measured hip and the committed hip attitude.
            // Keep the translation gaps between the actual upper/lower knee and ankle anchors.
            var g = pose.Reference;
            g.Hip = anchors.UpperHip;
            g.UpperSpan = Quaternion.Inverse(pose.PhysicalUpper) * (anchors.UpperKnee - anchors.UpperHip);
            g.LowerSpan = Quaternion.Inverse(pose.PhysicalLower) * (b.LowerAnkle - anchors.LowerKnee);
            g.Center = b.Center; g.Half = b.Half; g.FloorCenter = b.FloorCenter; g.FloorHalf = b.FloorHalf;
            pose.Reference = g;
            pose.PhysicalKnee = anchors.LowerKnee; pose.AnkleGap = b.FootAnkle - b.LowerAnkle;
            pose.PhysicalFootRelative = Quaternion.Inverse(pose.PhysicalLower) * b.FootR;
            pose.ObservedBox = b.Box; pose.LowerAnkleY = b.LowerAnkle.y;
            return SwingKneeCandidate.Evaluate(pose, k, step, source, measureRoom: true);
        }

        internal static Result Compare(in Frame f, Phase phase, bool unloaded, in Forecast.Motion m)
        {
            if (!f.Valid) return default;
            if (!unloaded) return new Result { State = Decision.Loaded };
            var issued = Forecast.Predict(f.Pose, m);
            if (!issued.Predicted) return new Result { State = Decision.MissingCommand };
            var box = f.Pose.Box;
            return CompareEndpoint(f, phase, m, box.LowerAnkle + f.LowerVelocity * box.Dt,
                issued.Foot, issued.R, issued.Box, issued.State == Forecast.Status.OutsideFootprint);
        }

        internal static Result WithIssuedLower(in Frame f, Phase phase, in Forecast.Motion m, in Result original, Vector3 lower)
        {
            // The committed foot control, footprint and sole are identical in both comparisons.
            if (!original.Predicted) return original;
            return CompareEndpoint(f, phase, m, lower, original.FootPoint, original.FootRotation, original.Sole, original.State == Decision.Outside);
        }

        internal static Result CompareEndpoint(in Frame f, Phase phase, in Forecast.Motion m, Vector3 lower,
            Vector3 footPoint, Quaternion footRotation, float sole, bool outside)
        {
            var pose = f.Pose;
            var box = pose.Box;
            var gap = footPoint - lower;
            if (!PelvisParentFrame.V(lower) || !float.IsFinite(gap.magnitude)) return new Result { State = Decision.MissingCommand };
            var r = new Result { Predicted = true, LowerY = lower.y, FootPoint = footPoint, FootRotation = footRotation, FootY = footPoint.y, Sole = sole,
                Gap = gap.magnitude, AppliedY = m.AppliedLinearA.y, TrialY = m.AppliedLinearA.y, TrialGap = gap.magnitude,
                Cap = Mathf.Min(m.Tether.BaseCap, m.Tether.SelectedCap) };
            if (outside ||
                !FootBoxGeometry.Inside(box.FootAnkle, box.FootR, box.Center, box.Half, box.FloorCenter, box.FloorHalf))
                r.State = Decision.Outside;
            else if (!f.FivePoint) r.State = Decision.NoFloorWitness;
            else if (phase != Phase.Lift) r.State = Decision.OtherPhase;
            else if (box.Box < -Depth || box.Box > FootContactGeometryLimits.RecoveryMaxGapMeters) r.State = Decision.OutsideStartBand;
            else if (sole >= -Depth) r.State = Decision.Clear;
            else if (pose.Gap > GapLimit) r.State = Decision.StartGap;
            else if (!float.IsFinite(m.Tether.BaseCap) || !float.IsFinite(m.Tether.SelectedCap) ||
                r.Cap <= 0f || m.AppliedLinearA.magnitude > r.Cap + .00001f ||
                !PelvisParentFrame.V(m.Tether.Raw) ||
                (Vector3.ClampMagnitude(m.Tether.Raw, m.Tether.SelectedCap) - m.AppliedLinearA).sqrMagnitude > 1e-8f)
                r.State = Decision.ProtectedBudget;
            else if (r.AppliedY >= 0f) r.State = Decision.NoDownwardTether;
            else
            {
                float response = box.Dt * box.Dt * Mathf.Max(0f, 1f - m.LinearDrag * box.Dt);
                r.AvailableRise = -r.AppliedY * response;
                float excess = (lower.y - box.LowerAnkle.y) - (footPoint.y - box.FootAnkle.y);
                if (excess <= 0f) { r.State = Decision.CommonDescent; return r; }
                // At most recover the excess foot descent and return to the
                // shallower of the current sole height and the surface. The exact
                // vertical slice of the 5 mm sphere keeps the actual X/Z request.
                float wanted = Mathf.Min(r.AvailableRise, Mathf.Min(excess, Mathf.Min(0f, box.Box) - sole));
                float roomSq = GapLimit * GapLimit - gap.x * gap.x - gap.z * gap.z;
                if (roomSq < 0f) { r.State = Decision.Gap; return r; }
                float half = Mathf.Sqrt(roomSq);
                float rise = Mathf.Min(wanted, half - gap.y);
                if (rise < -half - gap.y || rise <= 0f) { r.State = Decision.Gap; return r; }
                if (rise < KneeGroundTarget.MinGain) { r.State = Decision.Subthreshold; return r; }
                r.TrialY = Mathf.Min(0f, r.AppliedY + rise / response);
                r.Rise = (r.TrialY - r.AppliedY) * response;
                r.TrialGap = (gap + Vector3.up * r.Rise).magnitude;
                r.State = Decision.Trial;
            }
            return r;
        }

        // Retrospective geometry only. H144 already contains both ankle heights and
        // the full gap length. Keeping X/Z fixed makes this same calculation possible
        // for runs that did not yet record the actual issued acceleration.
        internal static bool MeasuredRoom(float startGap, float nextSole, float lowerDelta, float footDelta,
            float nextGap, float nextSignedGapY, out float needed, out float excess, out float gapAtDepth)
        {
            needed = Mathf.Max(0f, -Depth - nextSole);
            excess = Mathf.Max(0f, lowerDelta - footDelta);
            gapAtDepth = float.NaN;
            if (!float.IsFinite(nextSole) || !float.IsFinite(lowerDelta) || !float.IsFinite(footDelta) ||
                !float.IsFinite(nextGap) || !float.IsFinite(nextSignedGapY) || nextGap < 0f ||
                Mathf.Abs(nextSignedGapY) > nextGap + .000001f || !float.IsFinite(needed) || !float.IsFinite(excess)) return false;
            float lateralSq = Mathf.Max(0f, nextGap * nextGap - nextSignedGapY * nextSignedGapY);
            float shiftedY = nextSignedGapY + needed;
            gapAtDepth = Mathf.Sqrt(lateralSq + shiftedY * shiftedY);
            return startGap >= 0f && startGap <= GapLimit && needed > 0f && excess >= needed && gapAtDepth <= GapLimit;
        }

        // Only the known change in vertical displacement is applied to the next
        // measurement. This is a conditioned estimate; the alternative never ran.
        internal static bool ReadTrial(in Result r, in Frame next, out float sole, out float gap)
        {
            sole = gap = float.NaN;
            if (r.State != Decision.Trial || !next.Valid) return false;
            var b = next.Pose.Box;
            sole = b.Box + r.Rise;
            gap = (b.FootAnkle - b.LowerAnkle + Vector3.up * r.Rise).magnitude;
            return b.Box < -Depth && r.Sole + r.Rise >= -Depth && sole >= -Depth && gap <= GapLimit &&
                FootBoxGeometry.Inside(b.FootAnkle, b.FootR, b.Center, b.Half, b.FloorCenter, b.FloorHalf);
        }
    }
}
