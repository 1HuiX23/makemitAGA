using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Symmetric positive 3x3 response/inertia and inverse. The free-body constructor
    // validates preparation/readback; constrained Pelvis uses TryColumns for inertia.
    // The bilateral compensated Hip solver uses translational masses directly.
    internal struct HipPointPairMobility
    {
        internal Vector3 X, Y, Z;
        private float _det;
        internal static bool ValidBody(float mass, Vector3 inertia, Quaternion principal) => P(mass) && Tensor(inertia) && Q(principal);
        internal static bool TryCreate(float childMass, Vector3 childInertia, Quaternion childPrincipal,
            Vector3 childLever, float parentMass, Vector3 parentInertia, Quaternion parentPrincipal,
            Vector3 parentLever, out HipPointPairMobility model)
        {
            model = new HipPointPairMobility();
            if (!P(childMass) || !P(parentMass) || !Tensor(childInertia) || !Tensor(parentInertia) ||
                !V(childLever) || !V(parentLever) || !Q(childPrincipal) || !Q(parentPrincipal)) return false;
            model.X = Column(new Vector3(1f, 0f, 0f), childMass, childInertia, childPrincipal, childLever, parentMass, parentInertia, parentPrincipal, parentLever);
            model.Y = Column(new Vector3(0f, 1f, 0f), childMass, childInertia, childPrincipal, childLever, parentMass, parentInertia, parentPrincipal, parentLever);
            model.Z = Column(new Vector3(0f, 0f, 1f), childMass, childInertia, childPrincipal, childLever, parentMass, parentInertia, parentPrincipal, parentLever);
            return TryColumns(model.X, model.Y, model.Z, out model);
        }
        internal static bool TryColumns(Vector3 x, Vector3 y, Vector3 z, out HipPointPairMobility model)
        {
            model = new HipPointPairMobility { X = x, Y = y, Z = z };
            model._det = Vector3.Dot(model.X, Vector3.Cross(model.Y, model.Z));
            float tolerance = 0.0001f * (1f + model.X.magnitude + model.Y.magnitude + model.Z.magnitude);
            return V(model.X) && V(model.Y) && V(model.Z) && P(model._det) && P(model.X.x) &&
                P(model.X.x * model.Y.y - model.X.y * model.Y.x) &&
                Mathf.Abs(model.X.y - model.Y.x) <= tolerance && Mathf.Abs(model.X.z - model.Z.x) <= tolerance && Mathf.Abs(model.Y.z - model.Z.y) <= tolerance;
        }
        private static Vector3 Column(Vector3 axis, float cm, Vector3 ci, Quaternion cq, Vector3 cr, float pm, Vector3 pi, Quaternion pq, Vector3 pr) =>
            axis * (1f / cm + 1f / pm) + Vector3.Cross(InverseTensor(ci, cq, Vector3.Cross(cr, axis)), cr) +
            Vector3.Cross(InverseTensor(pi, pq, Vector3.Cross(pr, axis)), pr);
        internal Vector3 Solve(Vector3 acceleration) => new Vector3(Vector3.Dot(acceleration, Vector3.Cross(Y, Z)),
            Vector3.Dot(X, Vector3.Cross(acceleration, Z)), Vector3.Dot(X, Vector3.Cross(Y, acceleration))) / _det;
        internal static Vector3 InverseTensor(Vector3 inertia, Quaternion principal, Vector3 torque)
        {
            Vector3 local = Quaternion.Inverse(principal) * torque;
            return principal * new Vector3(local.x / inertia.x, local.y / inertia.y, local.z / inertia.z);
        }
        internal static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z) && F(v.sqrMagnitude);
        internal static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool P(float x) => F(x) && x > 0.000001f;
        private static bool Tensor(Vector3 t) => V(t) && P(t.x) && P(t.y) && P(t.z);
        private static bool Q(Quaternion q) => F(q.x) && F(q.y) && F(q.z) && F(q.w) &&
            Mathf.Abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1f) < 0.001f;
    }
}
