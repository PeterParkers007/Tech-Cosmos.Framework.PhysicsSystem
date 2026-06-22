using System;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    public enum ColliderShapeType
    {
        Sphere = 0,
        Box = 1,
        Capsule = 2
    }

    /// <summary>
    /// 统一碰撞体形状描述。LocalCenter / LocalRotation 定义相对刚体原点的偏移。
    /// </summary>
    [Serializable]
    public struct ColliderShape
    {
        public ColliderShapeType type;
        public Float3 localCenter;
        public FloatQuat localRotation;
        public Float3 size;
        public float radius;
        public float height;
        public int directionAxis;

        public static ColliderShape Sphere(float radius, Float3 localCenter = default)
        {
            return new ColliderShape
            {
                type = ColliderShapeType.Sphere,
                radius = Math.Max(radius, 1e-4f),
                localCenter = localCenter,
                localRotation = FloatQuat.Identity
            };
        }

        public static ColliderShape Box(Float3 halfExtents, Float3 localCenter = default, FloatQuat localRotation = default)
        {
            if (localRotation.IsIdentity) localRotation = FloatQuat.Identity;

            return new ColliderShape
            {
                type = ColliderShapeType.Box,
                size = new Float3(
                    Math.Max(halfExtents.x, 1e-4f),
                    Math.Max(halfExtents.y, 1e-4f),
                    Math.Max(halfExtents.z, 1e-4f)),
                localCenter = localCenter,
                localRotation = localRotation
            };
        }

        public static ColliderShape Capsule(float radius, float height, int directionAxis = 1, Float3 localCenter = default)
        {
            return new ColliderShape
            {
                type = ColliderShapeType.Capsule,
                radius = Math.Max(radius, 1e-4f),
                height = Math.Max(height, radius * 2f),
                directionAxis = Math.Clamp(directionAxis, 0, 2),
                localCenter = localCenter,
                localRotation = FloatQuat.Identity
            };
        }

        public Float3 GetWorldCenter(in Float3 bodyPosition, in FloatQuat bodyRotation)
        {
            Float3 offset = bodyRotation.Rotate(localCenter);
            return bodyPosition + offset;
        }

        public FloatQuat GetWorldRotation(in FloatQuat bodyRotation) => bodyRotation * localRotation;

        public void ComputeWorldBounds(
            in Float3 bodyPosition,
            in FloatQuat bodyRotation,
            out Float3 min,
            out Float3 max)
        {
            switch (type)
            {
                case ColliderShapeType.Sphere:
                {
                    Float3 center = GetWorldCenter(bodyPosition, bodyRotation);
                    min = center - new Float3(radius, radius, radius);
                    max = center + new Float3(radius, radius, radius);
                    break;
                }
                case ColliderShapeType.Box:
                {
                    ComputeBoxBounds(bodyPosition, bodyRotation, out min, out max);
                    break;
                }
                case ColliderShapeType.Capsule:
                {
                    ComputeCapsuleBounds(bodyPosition, bodyRotation, out min, out max);
                    break;
                }
                default:
                    min = bodyPosition;
                    max = bodyPosition;
                    break;
            }
        }

        private void ComputeBoxBounds(in Float3 bodyPosition, in FloatQuat bodyRotation, out Float3 min, out Float3 max)
        {
            FloatQuat worldRotation = GetWorldRotation(bodyRotation);
            Float3 center = GetWorldCenter(bodyPosition, bodyRotation);
            Float3 axisX = worldRotation.Rotate(new Float3(1f, 0f, 0f));
            Float3 axisY = worldRotation.Rotate(new Float3(0f, 1f, 0f));
            Float3 axisZ = worldRotation.Rotate(new Float3(0f, 0f, 1f));

            Float3 extent =
                new Float3(Math.Abs(axisX.x), Math.Abs(axisX.y), Math.Abs(axisX.z)) * size.x +
                new Float3(Math.Abs(axisY.x), Math.Abs(axisY.y), Math.Abs(axisY.z)) * size.y +
                new Float3(Math.Abs(axisZ.x), Math.Abs(axisZ.y), Math.Abs(axisZ.z)) * size.z;

            min = center - extent;
            max = center + extent;
        }

        private void ComputeCapsuleBounds(in Float3 bodyPosition, in FloatQuat bodyRotation, out Float3 min, out Float3 max)
        {
            Float3 center = GetWorldCenter(bodyPosition, bodyRotation);
            Float3 axis = GetCapsuleAxis(bodyRotation);
            float halfLine = Math.Max(0f, height * 0.5f - radius);
            Float3 top = center + axis * halfLine;
            Float3 bottom = center - axis * halfLine;
            Float3 r = new Float3(radius, radius, radius);

            min = new Float3(
                Math.Min(top.x, bottom.x) - r.x,
                Math.Min(top.y, bottom.y) - r.y,
                Math.Min(top.z, bottom.z) - r.z);
            max = new Float3(
                Math.Max(top.x, bottom.x) + r.x,
                Math.Max(top.y, bottom.y) + r.y,
                Math.Max(top.z, bottom.z) + r.z);
        }

        public Float3 GetCapsuleAxis(in FloatQuat bodyRotation)
        {
            FloatQuat worldRotation = GetWorldRotation(bodyRotation);
            return directionAxis switch
            {
                0 => worldRotation.Rotate(new Float3(1f, 0f, 0f)),
                2 => worldRotation.Rotate(new Float3(0f, 0f, 1f)),
                _ => worldRotation.Rotate(new Float3(0f, 1f, 0f))
            };
        }
    }
}
