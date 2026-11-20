using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Angular = MakemitAGA.ActiveRagdollLab.Physics.RightFootTorqueForecast;
using Band = MakemitAGA.ActiveRagdollLab.Physics.RightFootTorqueForecast.HeightBand;
using Drive = MakemitAGA.ActiveRagdollLab.Physics.FootAnkleSourceDrivenRelativePdIsolation;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One unconstrained foot before contact. All orientations rotate about the same
    // advanced physical COM. The held control quaternion is an attainable-height
    // diagnostic; it is never written back to the source, body, or controller.
    internal static class RightFootLandingForecast
    {
        internal enum Status : byte { MissingController, Geometry, InvalidMotion, Loaded, BodyMismatch, Fault,
            OutsideFootprint, ProtectInBand, Above, GoalOutsideBand, OverGap, MissingDrive, NoBudget, NoGain, Favorable }
        [System.Flags]
        internal enum Reject : byte { None=0, Budget=1, FootprintOrUpperBand=2, AnkleGap=4, Rotation=8, Subthreshold=16, NonFinite=32 }
        internal struct Pose
        {
            internal RightKneePivot.Frame Box;
            internal Quaternion Goal;
            internal Vector3 Com, Velocity, W;
            internal float PresentGoal, Gap;
        }
        internal struct Motion
        {
            internal Angular.BodyMotion Angular;
            internal Vector3 AppliedLinearA;
            internal float LinearDrag;
            internal RightAnkleLandingSequence.Tether Tether;
        }
        internal struct Pair
        {
            internal float LowerMass, LowerLever;
            internal Vector3 LowerInertia, RealizedRelativeA;
            internal Quaternion LowerPrincipal;
        }
        internal struct Result
        {
            internal Status State;
            internal Vector3 Com, Foot, W, DeltaTorque;
            internal Quaternion R;
            internal float Box, GoalAtFoot, GoalAtCom, TrialBox, RelativeIncrement, ExtraGapBound;
            internal byte Level, Tried;
            internal Reject Rejected;
            internal float BestSafeGain;
            internal bool EntersBand => State==Status.Favorable && Angular.Band(TrialBox)==Band.InBand;
            internal bool Predicted => State >= Status.OutsideFootprint;
            internal bool Opportunity => State >= Status.OverGap;
        }
        internal static bool Valid(in Pose p) =>
            Angular.ValidGeometry(p.Box) && Angular.ValidRotation(p.Goal) &&
            HipPointPairMobility.V(p.Com) && HipPointPairMobility.V(p.Velocity) && HipPointPairMobility.V(p.W) &&
            PelvisParentFrame.F(p.Gap) && p.Gap >= 0f && RightKneePivot.Near(p.Gap, (p.Box.LowerAnkle-p.Box.FootAnkle).magnitude) &&
            RightKneePivot.Near(Angular.Clearance(p.Box, p.Box.FootAnkle, p.Goal), p.PresentGoal);

        internal static Result Predict(in Pose p, in Motion m)
        {
            if (!Valid(p)) return new Result { State = Status.Geometry };
            if (!PelvisParentFrame.F(m.LinearDrag) || m.LinearDrag < 0f || !HipPointPairMobility.V(m.AppliedLinearA) ||
                !RightThighTracking.Same(p.W, m.Angular.W) ||
                !Angular.TryIssuedVelocity(m.Angular, p.Box.Dt, out _, out var w))
                return new Result { State = Status.InvalidMotion };
            float dt = p.Box.Dt;
            var r = new Result {
                Com = p.Com + (p.Velocity + m.AppliedLinearA * dt) * (Mathf.Max(0f, 1f-m.LinearDrag*dt)*dt),
                W = Vector3.ClampMagnitude(w, m.Angular.MaxW)
            };
            var offset = Quaternion.Inverse(p.Box.FootR) * (p.Box.FootAnkle-p.Com);
            r.R = FootRelativeReference.Advance(p.Box.FootR, r.W, dt);
            r.Foot = r.Com + r.R * offset;
            r.Box = Angular.Clearance(p.Box, r.Foot, r.R);
            r.TrialBox = r.Box;
            r.GoalAtFoot = Angular.Clearance(p.Box, r.Foot, p.Goal);
            r.GoalAtCom = Angular.Clearance(p.Box, r.Com + p.Goal*offset, p.Goal);
            if (!HipPointPairMobility.V(r.Com) || !HipPointPairMobility.V(r.Foot) ||
                !PelvisParentFrame.F(r.Box) || !PelvisParentFrame.F(r.GoalAtFoot) || !PelvisParentFrame.F(r.GoalAtCom))
                return new Result { State = Status.InvalidMotion };

            if (!Inside(p.Box, r.Foot, r.R)) r.State = Status.OutsideFootprint;
            else if (Angular.Band(r.Box) == Band.InBand ||
                (Angular.Band(p.Box.Box) == Band.InBand && Angular.Band(p.PresentGoal) != Band.InBand))
                r.State = Status.ProtectInBand;
            else if (Angular.Band(r.Box) == Band.Above) r.State = Status.Above;
            else if (Angular.Band(r.GoalAtFoot) != Band.InBand || Angular.Band(r.GoalAtCom) != Band.InBand)
                r.State = Status.GoalOutsideBand;
            else r.State = p.Gap > FootContactGeometryLimits.RecoveryMaxAnchorGapMeters ? Status.OverGap : Status.MissingDrive;
            return r;
        }

        private static bool Inside(in RightKneePivot.Frame p, Vector3 ankle, Quaternion r) =>
            FootBoxGeometry.Inside(ankle, r, p.Center, p.Half, p.FloorCenter, p.FloorHalf);
        private static Vector3 PairColumn(in Angular.BodyMotion foot, in Pair pair, Vector3 axis) =>
            HipPointPairMobility.InverseTensor(foot.Inertia, foot.WorldPrincipal, axis) +
            HipPointPairMobility.InverseTensor(pair.LowerInertia, pair.LowerPrincipal, axis);

        // Only solve once the issued forecast establishes a geometric opportunity.
        // Gyro terms cancel in this same-state DIFFERENCE from the actual post-clamp
        // ankle torque. Both the command and the lower's opposite reaction retain
        // the original actuator limits. All linear commands remain those actually issued.
        internal static void Compare(in Pose p, in Motion m, in Pair pair, ref Result r)
        {
            if (r.State != Status.MissingDrive ||
                !HipPointPairMobility.ValidBody(pair.LowerMass, pair.LowerInertia, pair.LowerPrincipal) ||
                !HipPointPairMobility.V(pair.RealizedRelativeA) || !PelvisParentFrame.F(pair.LowerLever) || pair.LowerLever < 0f)
                return;
            var a = m.Angular;
            if (!HipPointPairMobility.TryColumns(PairColumn(a,pair,Vector3.right), PairColumn(a,pair,Vector3.up),
                PairColumn(a,pair,Vector3.forward), out var mobility)) return;
            var error = FootRelativeReference.Log(p.Goal * Quaternion.Inverse(r.R));
            float angle = error.magnitude, dt = p.Box.Dt;
            if (angle <= 1e-7f) { r.State = Status.NoGain; return; }
            var response = mobility.Solve(error/angle);
            var offset = Quaternion.Inverse(p.Box.FootR) * (p.Box.FootAnkle-p.Com);
            r.State = Status.NoBudget;
            float previous = 0f;
            for (byte level=1; level<=3; level++)
            {
                float increment = Mathf.Min(Drive.MaxRelativeAngularAcceleration * (1 << (level-1)) / 16f, angle/(dt*dt));
                if (increment == previous) break;
                previous = increment; r.Tried++;
                var extra = response * increment;
                var relative = pair.RealizedRelativeA + error * (increment/angle);
                if (!HipPointPairMobility.V(extra) || relative.magnitude > Drive.MaxRelativeAngularAcceleration ||
                    (a.Drive+extra).magnitude > Drive.MaxInternalTorqueNm) { r.Rejected |= Reject.Budget; continue; }
                r.State = Status.NoGain;
                var trial = a;
                trial.Drive += extra;
                if (!Angular.TryIssuedVelocity(trial, dt, out _, out var w)) { r.Rejected |= Reject.NonFinite; continue; }
                var rotation = FootRelativeReference.Advance(p.Box.FootR, Vector3.ClampMagnitude(w, a.MaxW), dt);
                var foot = r.Com + rotation * offset;
                float box = Angular.Clearance(p.Box, foot, rotation);
                // The lower baseline translation is unknown. Bound the CHANGE from
                // the two rigid rotations. PRE_N+1 reports the actual issued ankle gap.
                float lowerA = HipPointPairMobility.InverseTensor(pair.LowerInertia, pair.LowerPrincipal, -extra).magnitude;
                float extraGap = (foot-r.Foot).magnitude + pair.LowerLever*lowerA*dt*dt;
                if (!PelvisParentFrame.F(box) || !PelvisParentFrame.F(extraGap)) { r.Rejected |= Reject.NonFinite; continue; }
                if (Angular.Band(box)==Band.Above || !Inside(p.Box, foot, rotation)) { r.Rejected |= Reject.FootprintOrUpperBand; continue; }
                if (p.Gap+extraGap > FootContactGeometryLimits.RecoveryMaxAnchorGapMeters) { r.Rejected |= Reject.AnkleGap; continue; }
                if (FootRelativeReference.Log(p.Goal*Quaternion.Inverse(rotation)).magnitude >= angle) { r.Rejected |= Reject.Rotation; continue; }
                float gain=box-r.Box;
                r.BestSafeGain=Mathf.Max(r.BestSafeGain,gain);
                if (gain < KneeGroundTarget.MinGain) { r.Rejected |= Reject.Subthreshold; continue; }
                r.State = Status.Favorable; r.Level = level; r.RelativeIncrement = increment;
                r.DeltaTorque = extra; r.ExtraGapBound = extraGap; r.TrialBox = box;
                return;
            }
        }
    }
}
