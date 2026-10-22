using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // V04660 pure reference-planning math. No native access, force write or state history.
    // I = Ip_world + Ic_world + mu * (|r|^2 Identity - r r^T), r = Pelvis COM -> Chest COM pivot.
    // This is a rigid co-rotation planning model, NOT the articulated pair's exact actuator inertia.
    // For a unit axis n the relevant VECTOR torque norm is |I*n|, not n dot I*n.
    // The unchanged 1Nm budget covers this inertial term only. The other 1Nm of the unchanged
    // 2Nm Pelvis actuator cap remains margin for gyro/coupling/feedback; no gyro is cancelled here.
    internal struct PelvisReferenceInertiaBudget
    {
        private Vector3 _x, _y, _z;

        internal static bool TryCreate(Vector3 pelvisTensor, Quaternion pelvisPrincipalWorld,
            Vector3 chestTensor, Quaternion chestPrincipalWorld, Vector3 leverWorld,
            float pelvisMass, float chestMass, out PelvisReferenceInertiaBudget model)
        {
            model = default;
            if (!PositiveTensor(pelvisTensor) || !PositiveTensor(chestTensor) || !V(leverWorld) ||
                !P(pelvisMass) || !P(chestMass) || !Unit(ref pelvisPrincipalWorld) || !Unit(ref chestPrincipalWorld)) return false;
            float massSum = pelvisMass + chestMass;
            float mu = pelvisMass * (chestMass / massSum);
            if (!P(massSum) || !P(mu)) return false;
            model._x = Column(new Vector3(1f, 0f, 0f), pelvisTensor, pelvisPrincipalWorld, chestTensor, chestPrincipalWorld, leverWorld, mu);
            model._y = Column(new Vector3(0f, 1f, 0f), pelvisTensor, pelvisPrincipalWorld, chestTensor, chestPrincipalWorld, leverWorld, mu);
            model._z = Column(new Vector3(0f, 0f, 1f), pelvisTensor, pelvisPrincipalWorld, chestTensor, chestPrincipalWorld, leverWorld, mu);
            return V(model._x) && V(model._y) && V(model._z) && P(model._x.x) && P(model._y.y) && P(model._z.z);
        }

        private static Vector3 Column(Vector3 axis, Vector3 ip, Quaternion qp, Vector3 ic, Quaternion qc, Vector3 r, float mu) =>
            Tensor(ip, qp, axis) + Tensor(ic, qc, axis) + Vector3.Cross(r, Vector3.Cross(axis, r)) * mu;

        private static Vector3 Tensor(Vector3 diagonal, Quaternion principal, Vector3 value) =>
            principal * Vector3.Scale(diagonal, Quaternion.Inverse(principal) * value);

        internal Vector3 Apply(Vector3 alpha) => _x * alpha.x + _y * alpha.y + _z * alpha.z;

        internal bool TryAlphaCap(Vector3 direction, float scalarFallback, float maxAlpha, float torqueBudget,
            out float cap, out float inertiaNorm)
        {
            cap = scalarFallback; inertiaNorm = 0f;
            if (!V(direction) || !P(scalarFallback) || !P(maxAlpha) || !P(torqueBudget)) return false;
            float length = direction.magnitude;
            if (!F(length)) return false;
            // No requested axis: keep the frozen scalar value, rather than invent an axis from numerical noise.
            if (length <= 0.000001f) return true;
            inertiaNorm = Apply(direction / length).magnitude;
            if (!P(inertiaNorm)) return false;
            cap = Mathf.Min(maxAlpha, torqueBudget / inertiaNorm);
            return P(cap);
        }

        internal Vector3 RigidGyroEstimate(Vector3 w) => Vector3.Cross(w, Apply(w));

        private static bool Unit(ref Quaternion q)
        {
            float norm = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!P(norm)) return false;
            q = new Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm);
            return true;
        }
        private static bool PositiveTensor(Vector3 t) => V(t) && P(t.x) && P(t.y) && P(t.z);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z);
        private static bool P(float value) => F(value) && value > 0f;
        private static bool F(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
