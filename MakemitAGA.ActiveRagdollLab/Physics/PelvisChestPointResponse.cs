using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Instantaneous response of Pelvis with the native Chest anchor at Chest COM.
    // Chest spin is independent: do NOT add Chest's spin tensor as if rigidly welded.
    // For l=Chest anchor-Pelvis COM, g=mc/(mp+mc), h=Hip lever-g*l:
    // J=Ip+mu*(|l|^2 I-l*l^T), alpha=J^-1(h x F), aHip=F/(mp+mc)+alpha x h.
    // This eliminates the linear Chest joint, not Knee/contact or both Hip controllers.
    internal struct PelvisChestPointResponse
    {
        internal bool Valid;
        internal float PelvisMass, ChestMass, TotalMass, ChestFraction, ReducedMass;
        internal Vector3 ChestLever, PelvisInertia;
        internal Quaternion PelvisPrincipal;
        internal HipPointPairMobility AngularInertia;

        internal static bool TryCreate(float pm, float cm, Vector3 inertia, Quaternion principal, Vector3 lever,
            out PelvisChestPointResponse model)
        {
            model = new PelvisChestPointResponse();
            if (!HipPointPairMobility.ValidBody(pm, inertia, principal) || !F(cm) || cm <= 0.000001f ||
                !V(lever) || !F(pm + cm)) return false;
            float total = pm + cm, mu = pm * (cm / total);
            Vector3 x = Column(new Vector3(1f, 0f, 0f), inertia, principal, lever, mu);
            Vector3 y = Column(new Vector3(0f, 1f, 0f), inertia, principal, lever, mu);
            Vector3 z = Column(new Vector3(0f, 0f, 1f), inertia, principal, lever, mu);
            if (!HipPointPairMobility.TryColumns(x, y, z, out HipPointPairMobility j)) return false;
            model = new PelvisChestPointResponse { Valid = true, PelvisMass = pm, ChestMass = cm,
                TotalMass = total, ChestFraction = cm / total, ReducedMass = mu,
                ChestLever = lever, PelvisInertia = inertia, PelvisPrincipal = principal, AngularInertia = j };
            return true;
        }
        private static Vector3 Column(Vector3 axis, Vector3 inertia, Quaternion q, Vector3 l, float mu)
        {
            Vector3 local = Quaternion.Inverse(q) * axis;
            return q * new Vector3(inertia.x * local.x, inertia.y * local.y, inertia.z * local.z) +
                mu * Vector3.Cross(l, Vector3.Cross(axis, l));
        }
        internal Vector3 AngularResponse(Vector3 pelvisForce, Vector3 pelvisTorque) =>
            AngularInertia.Solve(pelvisTorque - ChestFraction * Vector3.Cross(ChestLever, pelvisForce));
        // Read-only whole-parent input check. Both Hip wrenches and the ACTUALLY committed
        // receiver torque are supplied together; no currentForce sign guess or next-state fit.
        // The explicit gyro already present in controller torque stays in that torque. The
        // tested native response does not subtract another free-body gyro term here.
        internal Vector3 PredictAngular(Vector3 force, Vector3 torque, Vector3 w) =>
            AngularResponse(force, torque - ReducedMass * Vector3.Cross(ChestLever, Vector3.Cross(w, Vector3.Cross(w, ChestLever))));
        internal Vector3 PredictCom(Vector3 force, Vector3 alpha, Vector3 w) => force / TotalMass -
            ChestFraction * (Vector3.Cross(alpha, ChestLever) + Vector3.Cross(w, Vector3.Cross(w, ChestLever)));
        internal bool MatchesBody(float mass, Vector3 inertia, Quaternion principal) => Valid &&
            HipPointPairMobility.ValidBody(mass, inertia, principal) && mass == PelvisMass &&
            Same(inertia, PelvisInertia) && Mathf.Abs(principal.x * PelvisPrincipal.x + principal.y * PelvisPrincipal.y +
                principal.z * PelvisPrincipal.z + principal.w * PelvisPrincipal.w) > 0.99999f;
        internal bool SameAs(PelvisChestPointResponse other) => Valid && other.Valid && PelvisMass == other.PelvisMass &&
            ChestMass == other.ChestMass && TotalMass == other.TotalMass && ChestFraction == other.ChestFraction &&
            ReducedMass == other.ReducedMass && Same(ChestLever, other.ChestLever) && Same(PelvisInertia, other.PelvisInertia) &&
            Same(AngularInertia.X, other.AngularInertia.X) && Same(AngularInertia.Y, other.AngularInertia.Y) && Same(AngularInertia.Z, other.AngularInertia.Z);
        internal static bool Same(Vector3 a, Vector3 b) => a.x == b.x && a.y == b.y && a.z == b.z;
        private static bool V(Vector3 v) => HipPointPairMobility.V(v);
        private static bool F(float v) => HipPointPairMobility.F(v);
    }
}
