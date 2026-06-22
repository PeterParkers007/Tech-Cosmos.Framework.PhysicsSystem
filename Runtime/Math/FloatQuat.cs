using System;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    /// <summary>
    /// 引擎无关四元数，用于刚体朝向与 OBB 碰撞检测。
    /// </summary>
    [Serializable]
    public struct FloatQuat : IEquatable<FloatQuat>
    {
        public float x;
        public float y;
        public float z;
        public float w;

        public FloatQuat(float x, float y, float z, float w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        public static FloatQuat Identity => new FloatQuat(0f, 0f, 0f, 1f);

        public bool IsIdentity =>
            Math.Abs(x) < 1e-6f &&
            Math.Abs(y) < 1e-6f &&
            Math.Abs(z) < 1e-6f &&
            Math.Abs(w - 1f) < 1e-6f;

        public static FloatQuat FromAxisAngle(Float3 axis, float angleRadians)
        {
            Float3 normalized = Float3Math.Normalize(axis);
            float half = angleRadians * 0.5f;
            float sin = (float)Math.Sin(half);
            return new FloatQuat(
                normalized.x * sin,
                normalized.y * sin,
                normalized.z * sin,
                (float)Math.Cos(half));
        }

        public FloatQuat Normalized()
        {
            float mag = (float)Math.Sqrt(x * x + y * y + z * z + w * w);
            if (mag <= 1e-6f) return Identity;
            float inv = 1f / mag;
            return new FloatQuat(x * inv, y * inv, z * inv, w * inv);
        }

        public Float3 Rotate(Float3 vector)
        {
            FloatQuat q = Normalized();
            FloatQuat v = new FloatQuat(vector.x, vector.y, vector.z, 0f);
            FloatQuat result = Multiply(Multiply(q, v), q.Conjugate());
            return new Float3(result.x, result.y, result.z);
        }

        public FloatQuat Conjugate() => new FloatQuat(-x, -y, -z, w);

        public static FloatQuat Multiply(FloatQuat a, FloatQuat b)
        {
            return new FloatQuat(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
        }

        public static FloatQuat operator *(FloatQuat a, FloatQuat b) => Multiply(a, b);

        public bool Equals(FloatQuat other) =>
            x == other.x && y == other.y && z == other.z && w == other.w;

        public override bool Equals(object obj) => obj is FloatQuat other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(x, y, z, w);

        public override string ToString() => $"({x:F3}, {y:F3}, {z:F3}, {w:F3})";
    }
}
