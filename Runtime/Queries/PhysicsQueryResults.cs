using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    /// <summary>
    /// 射线描述。
    /// </summary>
    public readonly struct PhysicsRay
    {
        public readonly Float3 Origin;
        public readonly Float3 Direction;

        public PhysicsRay(Float3 origin, Float3 direction)
        {
            Origin = origin;
            Direction = PhysicsMath.TryNormalize(direction, out var normalized, 1e-6f)
                ? normalized
                : new Float3(0f, 0f, 1f);
        }

        public Float3 GetPoint(float distance) => Origin + Direction * distance;
    }

    /// <summary>
    /// 射线检测结果，字段命名对齐 Unity RaycastHit。
    /// </summary>
    public struct RaycastHit
    {
        public PhysicsBodyHandle body;
        public Float3 point;
        public Float3 normal;
        public float distance;
        public ColliderShapeType shapeType;
        public object userData;

        public bool HasHit => body.IsValid;
    }

    /// <summary>
    /// 球形/盒形重叠检测结果。
    /// </summary>
    public struct OverlapHit
    {
        public PhysicsBodyHandle body;
        public Float3 closestPoint;
        public float penetration;
        public object userData;

        public bool HasHit => body.IsValid;
    }

    /// <summary>
    /// 球形投射（SphereCast）结果。
    /// </summary>
    public struct SphereCastHit
    {
        public PhysicsBodyHandle body;
        public Float3 point;
        public Float3 normal;
        public float distance;
        public object userData;

        public bool HasHit => body.IsValid;
    }
}
