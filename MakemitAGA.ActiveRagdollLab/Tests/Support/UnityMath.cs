// Minimal test-only adapter for the pure trajectory helper. System.Numerics supplies
// quaternion algebra; this is NOT a Unity ABI, Rigidbody or PhysX implementation.
using System;
using NQuaternion = System.Numerics.Quaternion;
using NVector3 = System.Numerics.Vector3;

namespace UnityEngine
{
    internal enum ForceMode { Force }

    internal struct Vector3
    {
        internal float x, y, z;
        internal Vector3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        internal static Vector3 zero => new Vector3();
        internal static Vector3 right => new Vector3(1f, 0f, 0f);
        internal static Vector3 up => new Vector3(0f, 1f, 0f);
        internal static Vector3 forward => new Vector3(0f, 0f, 1f);
        internal Vector3 normalized => magnitude > 1e-5f ? this / magnitude : zero;
        internal float sqrMagnitude => x * x + y * y + z * z;
        internal float magnitude => MathF.Sqrt(sqrMagnitude);
        internal void Normalize() { float length = magnitude; this = length > 1e-5f ? this / length : zero; }
        internal static float Dot(Vector3 a, Vector3 b) => a.x * b.x + a.y * b.y + a.z * b.z;
        internal static Vector3 Cross(Vector3 a, Vector3 b) => new Vector3(
            a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
        internal static Vector3 Scale(Vector3 a, Vector3 b) => new Vector3(a.x * b.x, a.y * b.y, a.z * b.z);
        internal static Vector3 ClampMagnitude(Vector3 value, float maxLength)
        {
            float lengthSquared = value.sqrMagnitude;
            return lengthSquared > maxLength * maxLength ? value / MathF.Sqrt(lengthSquared) * maxLength : value;
        }
        public static Vector3 operator -(Vector3 v) => new Vector3(-v.x, -v.y, -v.z);
        public static bool operator ==(Vector3 a, Vector3 b) => (a - b).sqrMagnitude < 1e-10f;
        public static bool operator !=(Vector3 a, Vector3 b) => !(a == b);
        public bool Equals(Vector3 other) => x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);
        public override bool Equals(object value) => value is Vector3 v && Equals(v);
        public override int GetHashCode() => HashCode.Combine(x, y, z);
        public static Vector3 operator +(Vector3 a, Vector3 b) => new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
        public static Vector3 operator -(Vector3 a, Vector3 b) => new Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
        public static Vector3 operator *(Vector3 value, float scale) => new Vector3(value.x * scale, value.y * scale, value.z * scale);
        public static Vector3 operator *(float scale, Vector3 value) => value * scale;
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
        public static Vector3 operator *(Quaternion q, Vector3 v)
        {
            NVector3 r = NVector3.Transform(new NVector3(v.x, v.y, v.z), q.Value);
            return new Vector3(r.X, r.Y, r.Z);
        }
        internal static Quaternion Inverse(Quaternion value) => From(NQuaternion.Inverse(value.Value));
        internal static Quaternion LookRotation(Vector3 forward, Vector3 up)
        {
            NVector3 z = NVector3.Normalize(new NVector3(forward.x, forward.y, forward.z));
            NVector3 x = NVector3.Normalize(NVector3.Cross(new NVector3(up.x, up.y, up.z), z));
            NVector3 y = NVector3.Cross(z, x);
            return From(NQuaternion.CreateFromRotationMatrix(new System.Numerics.Matrix4x4(
                x.X, x.Y, x.Z, 0f, y.X, y.Y, y.Z, 0f, z.X, z.Y, z.Z, 0f, 0f, 0f, 0f, 1f)));
        }
        internal static float Angle(Quaternion a, Quaternion b)
        {
            float dot = MathF.Min(MathF.Abs(NQuaternion.Dot(a.Value, b.Value)), 1f);
            return dot > 1f - 1e-6f ? 0f : MathF.Acos(dot) * 2f * Mathf.Rad2Deg;
        }
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
        internal const float PI = MathF.PI;
        internal static float Atan2(float y, float x) => MathF.Atan2(y, x);
        internal static float Max(float a, float b) => MathF.Max(a, b);
        internal static float Min(float a, float b) => MathF.Min(a, b);
        internal static float Abs(float value) => MathF.Abs(value);
        internal static float Sqrt(float value) => MathF.Sqrt(value);
        internal static float Clamp(float value, float min, float max) => Math.Clamp(value, min, max);
        internal static float Cos(float value) => MathF.Cos(value);
        internal static float Sin(float value) => MathF.Sin(value);
        internal static float Acos(float value) => MathF.Acos(value);
        internal static float Sign(float value) => value >= 0f ? 1f : -1f;
    }
}
