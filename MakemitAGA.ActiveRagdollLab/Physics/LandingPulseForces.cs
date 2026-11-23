using UnityEngine;
using Command = MakemitAGA.ActiveRagdollLab.Model.HipAnchorPairReport.Command;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // The original Hip pair and receiver are already queued in this FixedUpdate. Applying
    // only the difference at the original points preserves the selected wrench on each body.
    internal struct LandingPulseForces
    {
        internal Rigidbody Left, Right, Pelvis, Chest;
        internal Vector3 LeftForce, RightForce, LeftTorque, RightTorque;
        internal Vector3 LeftPoint, LeftSocket, RightPoint, RightSocket;
        internal Vector3 PelvisForce, ChestForce, PelvisTorque;

        internal static LandingPulseForces Create(Rigidbody left, Rigidbody right, Rigidbody pelvis, Rigidbody chest,
            in Command l, in Command r, in Model.PelvisHipAttitudeCommand from, in Command newL, in Command newR, in Model.PelvisHipAttitudeCommand to)
        {
            var delta = to.CommonAcceleration - from.CommonAcceleration;
            return new LandingPulseForces {
                Left=left, Right=right, Pelvis=pelvis, Chest=chest,
                LeftForce=newL.Force-l.Force, RightForce=newR.Force-r.Force,
                LeftTorque=newL.UpperActuatorTorque-l.UpperActuatorTorque, RightTorque=newR.UpperActuatorTorque-r.UpperActuatorTorque,
                LeftPoint=l.ChildPoint, LeftSocket=l.ParentPoint, RightPoint=r.ChildPoint, RightSocket=r.ParentPoint,
                PelvisForce=pelvis.mass*delta, ChestForce=chest.mass*delta, PelvisTorque=to.AppliedTorque-from.AppliedTorque
            };
        }

        internal bool TryApply(out int rollbackFailures)
        {
            rollbackFailures = 0;
            if (!HipPointPairMobility.V(LeftForce) || !HipPointPairMobility.V(RightForce) ||
                !HipPointPairMobility.V(LeftTorque) || !HipPointPairMobility.V(RightTorque) ||
                !HipPointPairMobility.V(PelvisForce) || !HipPointPairMobility.V(ChestForce) || !HipPointPairMobility.V(PelvisTorque))
                return false;
            int returned = 0;
            try
            {
                for (; returned < 9; returned++) Write(returned, 1f);
                return true;
            }
            catch
            {
                // Undo every returned call in reverse order, even if one undo fails.
                while (returned > 0)
                {
                    try { Write(--returned, -1f); }
                    catch { rollbackFailures++; }
                }
                return false;
            }
        }

        private void Write(int index, float sign)
        {
            switch (index)
            {
                case 0: Left.AddForceAtPosition(sign*LeftForce, LeftPoint, ForceMode.Force); break;
                case 1: Pelvis.AddForceAtPosition(-sign*LeftForce, LeftSocket, ForceMode.Force); break;
                case 2: Left.AddTorque(sign*LeftTorque, ForceMode.Force); break;
                case 3: Right.AddForceAtPosition(sign*RightForce, RightPoint, ForceMode.Force); break;
                case 4: Pelvis.AddForceAtPosition(-sign*RightForce, RightSocket, ForceMode.Force); break;
                case 5: Right.AddTorque(sign*RightTorque, ForceMode.Force); break;
                case 6: Pelvis.AddForce(sign*PelvisForce, ForceMode.Force); break;
                case 7: Chest.AddForce(sign*ChestForce, ForceMode.Force); break;
                case 8: Pelvis.AddTorque(sign*PelvisTorque, ForceMode.Force); break;
            }
        }
    }
}
