using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;
using System.Globalization;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Ownership of the already-bounded v81 Upper torque, BEFORE Hip point forces.
    // The common vector clamp scales relative and transport terms together. The
    // subsequent capture/brake-floor delta belongs to relative control. Gyro and
    // parent transport remain external assistance; neither is mirrored to Pelvis.
    internal struct HipAngularRequest
    {
        internal bool Valid;
        internal Vector3 RelativeAlpha, TransportAlpha;
        internal Vector3 RelativeTorque, TransportTorque, GyroTorque;
        internal float ClosureNm;

        internal static bool TryCreate(Vector3 inertia, Quaternion principal, Vector3 w,
            Vector3 torque, Vector3 finalAlpha, Vector3 relativeRaw, Vector3 transportRaw,
            Vector3 beforeFloor, float maxAlpha, out HipAngularRequest request)
        {
            request = new HipAngularRequest();
            if (!HipPointPairMobility.ValidBody(1f, inertia, principal) || !V(w) || !V(torque) ||
                !V(finalAlpha) || !V(relativeRaw) || !V(transportRaw) || !V(beforeFloor) ||
                !HipPointPairMobility.F(maxAlpha) || maxAlpha <= 0f) return false;
            Vector3 raw = relativeRaw + transportRaw;
            if (!V(raw) || (Vector3.ClampMagnitude(raw, maxAlpha) - beforeFloor).magnitude > 0.0001f * (1f + maxAlpha) ||
                finalAlpha.magnitude > maxAlpha + 0.001f) return false;
            float scale = raw.magnitude > maxAlpha ? maxAlpha / raw.magnitude : 1f;
            // Subtract the explicitly selected, proportionally clamped transport;
            // this assigns final brake-floor shaping and roundoff to relative control.
            Vector3 transport = transportRaw * scale, relative = finalAlpha - transport;
            float qn = Mathf.Sqrt(principal.x * principal.x + principal.y * principal.y + principal.z * principal.z + principal.w * principal.w);
            Quaternion q = new Quaternion(principal.x / qn, principal.y / qn, principal.z / qn, principal.w / qn);
            Vector3 relTorque = Tensor(inertia, q, relative), transportTorque = Tensor(inertia, q, transport);
            Vector3 gyro = Vector3.Cross(w, Tensor(inertia, q, w));
            Vector3 residual = torque - relTorque - transportTorque - gyro;
            if (!V(relative) || !V(transport) || !V(relTorque) || !V(transportTorque) || !V(gyro) || !V(residual) ||
                residual.magnitude > 0.0001f * (1f + torque.magnitude + relTorque.magnitude + transportTorque.magnitude + gyro.magnitude)) return false;
            request = new HipAngularRequest { Valid = true, RelativeAlpha = relative, TransportAlpha = transport,
                RelativeTorque = relTorque, TransportTorque = transportTorque, GyroTorque = gyro, ClosureNm = residual.magnitude };
            return true;
        }
        internal bool SameAs(HipAngularRequest other) => Valid && other.Valid &&
            PelvisChestPointResponse.Same(RelativeAlpha, other.RelativeAlpha) && PelvisChestPointResponse.Same(TransportAlpha, other.TransportAlpha) &&
            PelvisChestPointResponse.Same(RelativeTorque, other.RelativeTorque) && PelvisChestPointResponse.Same(TransportTorque, other.TransportTorque) &&
            PelvisChestPointResponse.Same(GyroTorque, other.GyroTorque) && ClosureNm == other.ClosureNm;
        internal string Details(float q, bool reactionEnabled, Vector3 parentReaction) =>
            "hipAngularOwnership=V041070/hipRelativeReaction=" + reactionEnabled +
            "/requestNm(relative-transport-gyro)=" + S(RelativeTorque) + "|" + S(TransportTorque) + "|" + S(GyroTorque) +
            "/splitClosureNm=" + N(ClosureNm) + "/allocatedRelative-parentReactionNm=" + S(q * RelativeTorque) + "|" + S(parentReaction);
        private static string N(float v) => v.ToString("G9", CultureInfo.InvariantCulture);
        private static string S(Vector3 v) => "(" + N(v.x) + "," + N(v.y) + "," + N(v.z) + ")";
        private static Vector3 Tensor(Vector3 inertia, Quaternion q, Vector3 v)
        {
            Vector3 local = Quaternion.Inverse(q) * v;
            return q * new Vector3(inertia.x * local.x, inertia.y * local.y, inertia.z * local.z);
        }
        private static bool V(Vector3 v) => HipPointPairMobility.V(v);
    }
}
