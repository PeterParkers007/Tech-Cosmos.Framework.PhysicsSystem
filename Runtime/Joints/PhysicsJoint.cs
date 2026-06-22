using System;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    public enum JointType
    {
        Fixed = 0,
        Hinge = 1,
        Distance = 2,
        Spring = 3
    }

    public readonly struct PhysicsJointHandle : IEquatable<PhysicsJointHandle>
    {
        public readonly int Id;
        public readonly int Generation;

        public PhysicsJointHandle(int id, int generation)
        {
            Id = id;
            Generation = generation;
        }

        public static PhysicsJointHandle Invalid => new PhysicsJointHandle(-1, 0);

        public bool IsValid => Id >= 0;

        public bool Equals(PhysicsJointHandle other) => Id == other.Id && Generation == other.Generation;

        public override bool Equals(object obj) => obj is PhysicsJointHandle other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(Id, Generation);

        public override string ToString() => IsValid ? $"Joint({Id}:{Generation})" : "Joint(Invalid)";
    }

    /// <summary>
    /// 创建关节时的描述符。
    /// </summary>
    public struct PhysicsJointDescriptor
    {
        public JointType type;
        public PhysicsBodyHandle bodyA;
        public PhysicsBodyHandle bodyB;
        public Float3 localAnchorA;
        public Float3 localAnchorB;
        public Float3 localAxisA;
        public float minDistance;
        public float maxDistance;
        public float restLength;
        public float springStiffness;
        public float springDamping;
        public bool collideConnected;
    }

    /// <summary>
    /// 运行时关节约束。
    /// </summary>
    public sealed class PhysicsJoint
    {
        internal PhysicsJointHandle Handle { get; set; }

        public JointType Type { get; internal set; }
        public PhysicsBody BodyA { get; internal set; }
        public PhysicsBody BodyB { get; internal set; }
        public Float3 LocalAnchorA { get; set; }
        public Float3 LocalAnchorB { get; set; }
        public Float3 LocalAxisA { get; set; }
        public float MinDistance { get; set; }
        public float MaxDistance { get; set; }
        public float RestLength { get; set; }
        public float SpringStiffness { get; set; }
        public float SpringDamping { get; set; }
        public bool CollideConnected { get; set; }
        public bool IsEnabled { get; set; } = true;

        public Float3 GetWorldAnchorA() => BodyA.Position + BodyA.Rotation.Rotate(LocalAnchorA);

        public Float3 GetWorldAnchorB() => BodyB.Position + BodyB.Rotation.Rotate(LocalAnchorB);

        public Float3 GetWorldAxisA()
        {
            if (PhysicsMath.TryNormalize(BodyA.Rotation.Rotate(LocalAxisA), out Float3 axis))
                return axis;
            return BodyA.Rotation.Rotate(new Float3(0f, 1f, 0f));
        }
    }
}
