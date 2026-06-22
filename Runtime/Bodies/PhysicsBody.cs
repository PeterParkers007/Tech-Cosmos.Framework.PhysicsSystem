using System;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    /// <summary>
    /// 创建刚体时的描述符，便于一次性配置。
    /// </summary>
    public struct PhysicsBodyDescriptor
    {
        public BodyType bodyType;
        public Float3 position;
        public FloatQuat rotation;
        public Float3 velocity;
        public Float3 angularVelocity;
        public float mass;
        public bool isTrigger;
        public bool useGravity;
        public float linearDrag;
        public float angularDrag;
        public CollisionFilter filter;
        public PhysicsMaterial material;
        public ColliderShape shape;
        public object userData;

        public static PhysicsBodyDescriptor DynamicSphere(Float3 position, float radius, float mass = 1f)
        {
            return new PhysicsBodyDescriptor
            {
                bodyType = BodyType.Dynamic,
                position = position,
                rotation = FloatQuat.Identity,
                mass = mass,
                useGravity = true,
                filter = CollisionFilter.Default,
                material = PhysicsMaterial.Default,
                shape = ColliderShape.Sphere(radius)
            };
        }

        public static PhysicsBodyDescriptor StaticBox(Float3 position, Float3 halfExtents)
        {
            return new PhysicsBodyDescriptor
            {
                bodyType = BodyType.Static,
                position = position,
                rotation = FloatQuat.Identity,
                mass = 0f,
                useGravity = false,
                filter = CollisionFilter.Default,
                material = PhysicsMaterial.Default,
                shape = ColliderShape.Box(halfExtents)
            };
        }
    }

    /// <summary>
    /// 运行时刚体。位置/速度由 <see cref="PhysicsWorld"/> 驱动更新。
    /// </summary>
    public sealed class PhysicsBody
    {
        public PhysicsBodyHandle Handle { get; internal set; }

        public BodyType BodyType { get; set; }
        public Float3 Position { get; internal set; }
        public FloatQuat Rotation { get; internal set; }
        public Float3 Velocity { get; set; }
        public Float3 AngularVelocity { get; set; }
        public float Mass { get; set; }
        public bool IsTrigger { get; set; }
        public bool UseGravity { get; set; }
        public float LinearDrag { get; set; }
        public float AngularDrag { get; set; }
        public CollisionFilter Filter { get; internal set; }
        public PhysicsMaterial Material { get; internal set; }
        public ColliderShape Shape { get; set; }
        public object UserData { get; set; }
        public bool IsSleeping { get; internal set; }
        public bool IsEnabled { get; set; } = true;

        internal int SlotIndex = -1;

        // 新增：per-body 性能/行为控制（超越 Unity Rigidbody 的黑箱）
        public int SolverIterationsOverride { get; set; }   // 0 = 使用世界默认
        public bool CcdEnabled { get; set; }
        public float CcdRadius { get; set; } = 0.1f;

        internal float InverseMass => BodyType == BodyType.Dynamic && Mass > 0f ? 1f / Mass : 0f;
        internal float InverseInertia => InverseMass; // 简化，实际应为张量

        public void WakeUp() => IsSleeping = false;

        public void MovePosition(Float3 target) => Position = target;
        public void MoveRotation(FloatQuat target) => Rotation = target.Normalized();

        public void ApplyForce(Float3 force)
        {
            if (BodyType != BodyType.Dynamic || Mass <= 0f) return;
            WakeUp();
            Velocity = Velocity + force * InverseMass;
        }

        public void ApplyTorque(Float3 torque)
        {
            if (BodyType != BodyType.Dynamic || Mass <= 0f) return;
            WakeUp();
            AngularVelocity = AngularVelocity + torque * InverseInertia;
        }

        public void ApplyForceAtPoint(Float3 force, Float3 worldPoint)
        {
            if (BodyType != BodyType.Dynamic || Mass <= 0f) return;
            WakeUp();
            Velocity = Velocity + force * InverseMass;
            Float3 r = worldPoint - Position;
            AngularVelocity = AngularVelocity + PhysicsMath.Cross(r, force) * InverseInertia;
        }

        public void ApplyImpulse(Float3 impulse)
        {
            if (BodyType != BodyType.Dynamic || Mass <= 0f) return;
            WakeUp();
            Velocity = Velocity + impulse * InverseMass;
        }

        public void ApplyAngularImpulse(Float3 impulse)
        {
            if (BodyType != BodyType.Dynamic || Mass <= 0f) return;
            WakeUp();
            AngularVelocity = AngularVelocity + impulse * InverseInertia;
        }

        public void ApplyLinearImpulse(Float3 impulse) => ApplyImpulse(impulse);

        public Float3 GetPointVelocity(Float3 worldPoint)
        {
            Float3 r = worldPoint - Position;
            return Velocity + PhysicsMath.Cross(AngularVelocity, r);
        }
    }
}
