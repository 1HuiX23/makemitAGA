using UnityEngine;
namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal static class FootBoxGeometry
    {
        internal const float SupportPlaneToleranceMeters = .0002f;
        internal const float GroundGoalUprightConeDegrees = 30f;
        // Full Box support function. center/half are scaled local offsets FROM the ankle.
        // Equivalent to all eight vertices, including a nonzero collider center and ankle anchor.
        internal static float Minimum(Vector3 ankle, Quaternion rotation, Vector3 center, Vector3 half)
        {
            Vector3 x = rotation * Vector3.right, y = rotation * Vector3.up, z = rotation * Vector3.forward;
            return (ankle + rotation * center).y - Mathf.Abs(x.y) * half.x - Mathf.Abs(y.y) * half.y - Mathf.Abs(z.y) * half.z;
        }
        internal static bool Inside(Vector3 ankle, Quaternion rotation, Vector3 center, Vector3 half, Vector3 floorCenter, Vector3 floorHalf)
        {
            Vector3 p = ankle + rotation * center;
            Vector3 x = rotation * Vector3.right, y = rotation * Vector3.up, z = rotation * Vector3.forward;
            float ex = Mathf.Abs(x.x) * half.x + Mathf.Abs(y.x) * half.y + Mathf.Abs(z.x) * half.z;
            float ez = Mathf.Abs(x.z) * half.x + Mathf.Abs(y.z) * half.y + Mathf.Abs(z.z) * half.z;
            return Mathf.Abs(p.x - floorCenter.x) + ex <= floorHalf.x && Mathf.Abs(p.z - floorCenter.z) + ez <= floorHalf.z;
        }
    }
}
