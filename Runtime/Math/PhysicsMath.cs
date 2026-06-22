using System;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    internal static class PhysicsMath
    {
        public static Float3 Cross(Float3 a, Float3 b) =>
            new Float3(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);

        public static float Clamp(float value, float min, float max) => Math.Max(min, Math.Min(max, value));

        public static float Sqr(float value) => value * value;

        public static bool TryNormalize(Float3 value, out Float3 normalized, float epsilon = 1e-6f)
        {
            float magSq = Float3Math.Dot(value, value);
            if (magSq <= epsilon * epsilon)
            {
                normalized = Float3.Zero;
                return false;
            }

            float inv = 1f / (float)Math.Sqrt(magSq);
            normalized = new Float3(value.x * inv, value.y * inv, value.z * inv);
            return true;
        }

        public static Float3 ProjectOnPlane(Float3 vector, Float3 planeNormal)
        {
            float dot = Float3Math.Dot(vector, planeNormal);
            return vector - planeNormal * dot;
        }
    }
}
