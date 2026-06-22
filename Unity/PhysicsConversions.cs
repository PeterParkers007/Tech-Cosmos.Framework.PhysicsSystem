using TechCosmos.Core.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    internal static class PhysicsConversions
    {
        public static Float3 ToFloat3(Vector3 value) => new Float3(value.x, value.y, value.z);

        public static Vector3 ToVector3(Float3 value) => new Vector3(value.x, value.y, value.z);

        public static FloatQuat ToFloatQuat(Quaternion value) => new FloatQuat(value.x, value.y, value.z, value.w);

        public static Quaternion ToQuaternion(FloatQuat value) => new Quaternion(value.x, value.y, value.z, value.w);
    }
}
