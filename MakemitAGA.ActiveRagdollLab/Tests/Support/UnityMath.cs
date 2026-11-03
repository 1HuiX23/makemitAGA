// Minimal test-only adapter for the pure trajectory helper. System.Numerics supplies
// quaternion algebra; this is NOT a Unity ABI, Rigidbody or PhysX implementation.
using System;
using NQuaternion = System.Numerics.Quaternion;
using NVector3 = System.Numerics.Vector3;

namespace UnityEngine
{
    internal struct Vector3
    {
        internal float x, y, z;
        internal Vector3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        internal static Vector3 zero => new Vector3();
        internal float sqrMagnitude => x * x + y * y + z * z;
        internal float magnitude => MathF.Sqrt(sqrMagnitude);
        internal void Normalize() { float length = magnitude; this = length > 1e-5f ? this / length : zero; }
        public static Vector3 operator *(Vector3 value, float scale) => new Vector3(value.x * scale, value.y * scale, value.z * scale);
        public static Vector3 operator /(Vector3 value, float scale) => value * (1f / scale);
    }

    internal struct Quaternion
    {
        internal float x, y, z, w;
        internal Quaternion(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }
        internal static Quaternion identity => new Quaternion(0f, 0f, 0f, 1f);
        private NQuaternion Value => new NQuaternion(x, y, z, w);
        private static Quaternion From(NQuaternion value) => new Quaternion(value.X, value.Y, value.Z, value.W);
        public static Quaternion operator *(Quaternion a, Quaternion b) => From(a.Value * b.Value);
        internal static Quaternion Inverse(Quaternion value) => From(NQuaternion.Inverse(value.Value));
        internal static Quaternion AngleAxis(float degrees, Vector3 axis)
        {
            if (axis.sqrMagnitude == 0f) return identity;
            return From(NQuaternion.CreateFromAxisAngle(NVector3.Normalize(new NVector3(axis.x, axis.y, axis.z)), degrees * Mathf.Deg2Rad));
        }
        internal void ToAngleAxis(out float degrees, out Vector3 axis)
        {
            var q = NQuaternion.Normalize(Value);
            float cosine = Math.Clamp(q.W, -1f, 1f);
            degrees = 2f * MathF.Acos(cosine) * Mathf.Rad2Deg;
            float sine = MathF.Sqrt(MathF.Max(0f, 1f - cosine * cosine));
            axis = sine < 1e-6f ? new Vector3(1f, 0f, 0f) : new Vector3(q.X / sine, q.Y / sine, q.Z / sine);
        }
    }

    internal static class Mathf
    {
        internal const float Deg2Rad = MathF.PI / 180f, Rad2Deg = 180f / MathF.PI;
        internal static float Max(float a, float b) => MathF.Max(a, b);
        internal static float Min(float a, float b) => MathF.Min(a, b);
        internal static float Abs(float value) => MathF.Abs(value);
        internal static float Sqrt(float value) => MathF.Sqrt(value);
        internal static float Sign(float value) => value >= 0f ? 1f : -1f;
    }
}
