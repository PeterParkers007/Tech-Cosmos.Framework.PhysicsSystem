using System;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    /// <summary>
    /// 物理层掩码，语义对齐 Unity LayerMask。
    /// </summary>
    [Serializable]
    public struct PhysicsLayerMask : IEquatable<PhysicsLayerMask>
    {
        public int value;

        public PhysicsLayerMask(int value) => this.value = value;

        public static PhysicsLayerMask Everything => new PhysicsLayerMask(~0);
        public static PhysicsLayerMask Nothing => new PhysicsLayerMask(0);

        public bool ContainsLayer(int layer) => layer >= 0 && layer < 32 && (value & (1 << layer)) != 0;

        public bool Intersects(in PhysicsLayerMask other) => (value & other.value) != 0;

        public bool CanCollideWith(in CollisionFilter other) => Intersects(other.LayerMask);

        public static implicit operator int(PhysicsLayerMask mask) => mask.value;

        public static implicit operator PhysicsLayerMask(int mask) => new PhysicsLayerMask(mask);

        public bool Equals(PhysicsLayerMask other) => value == other.value;

        public override bool Equals(object obj) => obj is PhysicsLayerMask other && Equals(other);

        public override int GetHashCode() => value;
    }

    /// <summary>
    /// 刚体类型，对齐 Unity Rigidbody 的 Static / Kinematic / Dynamic 语义。
    /// </summary>
    public enum BodyType
    {
        Static = 0,
        Kinematic = 1,
        Dynamic = 2
    }

    /// <summary>
    /// 碰撞过滤：层 + 自定义组，便于项目扩展过滤规则。
    /// </summary>
    [Serializable]
    public struct CollisionFilter : IEquatable<CollisionFilter>
    {
        public int layer;
        public PhysicsLayerMask LayerMask;
        public int group;
        public int ignoreGroup;

        public CollisionFilter(int layer, PhysicsLayerMask layerMask, int group = 0, int ignoreGroup = 0)
        {
            this.layer = layer;
            LayerMask = layerMask;
            this.group = group;
            this.ignoreGroup = ignoreGroup;
        }

        public static CollisionFilter Default => new CollisionFilter(0, PhysicsLayerMask.Everything);

        public bool CanInteractWith(in CollisionFilter other)
        {
            if (group != 0 && group == other.ignoreGroup) return false;
            if (ignoreGroup != 0 && ignoreGroup == other.group) return false;
            return LayerMask.ContainsLayer(other.layer) && other.LayerMask.ContainsLayer(layer);
        }

        public bool Equals(CollisionFilter other) =>
            layer == other.layer &&
            LayerMask.Equals(other.LayerMask) &&
            group == other.group &&
            ignoreGroup == other.ignoreGroup;

        public override bool Equals(object obj) => obj is CollisionFilter other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(layer, LayerMask, group, ignoreGroup);
    }

    /// <summary>
    /// 物理材质：弹性与摩擦。
    /// </summary>
    [Serializable]
    public struct PhysicsMaterial
    {
        public float bounciness;
        public float friction;
        public float bouncinessCombine;
        public float frictionCombine;

        public PhysicsMaterial(float bounciness = 0f, float friction = 0.5f)
        {
            this.bounciness = bounciness;
            this.friction = friction;
            bouncinessCombine = 0.5f;
            frictionCombine = 0.5f;
        }

        public static PhysicsMaterial Default => new PhysicsMaterial(0f, 0.5f);

        public float CombineBounciness(in PhysicsMaterial other)
        {
            return Math.Max(bounciness, other.bounciness);
        }

        public float CombineFriction(in PhysicsMaterial other)
        {
            return Math.Max(friction, other.friction);
        }
    }

    /// <summary>
    /// 世界级物理参数。所有参数均可运行时动态调整，实现完全性能可控。
    /// 相比 Unity PhysX，提供更多细粒度控制（substeps、CCD、per-body overrides、profiling）。
    /// </summary>
    [Serializable]
    public struct PhysicsSettings
    {
        public Float3 gravity;
        public int velocityIterations;
        public int positionIterations;
        public float linearSleepThreshold;
        public float angularSleepThreshold;
        public float broadPhaseCellSize;

        // 新增：性能与确定性控制
        public float fixedTimeStep;      // 推荐 1/60f 或 1/50f
        public int maxSubsteps;          // 防止 spiral of death
        public bool enableSleeping;
        public bool enableCcd;
        public float ccdThreshold;       // 速度阈值触发 CCD

        public static PhysicsSettings Default => new PhysicsSettings
        {
            gravity = new Float3(0f, -9.81f, 0f),
            velocityIterations = 4,
            positionIterations = 2,
            linearSleepThreshold = 0.01f,
            angularSleepThreshold = 0.01f,
            broadPhaseCellSize = 2f,
            fixedTimeStep = 1f / 60f,
            maxSubsteps = 4,
            enableSleeping = true,
            enableCcd = false,
            ccdThreshold = 3f
        };
    }

    /// <summary>
    /// 稳定句柄，用于跨帧引用刚体。
    /// </summary>
    public readonly struct PhysicsBodyHandle : IEquatable<PhysicsBodyHandle>
    {
        public readonly int Id;
        public readonly int Generation;

        public PhysicsBodyHandle(int id, int generation)
        {
            Id = id;
            Generation = generation;
        }

        public static PhysicsBodyHandle Invalid => new PhysicsBodyHandle(-1, 0);

        public bool IsValid => Id >= 0;

        public bool Equals(PhysicsBodyHandle other) => Id == other.Id && Generation == other.Generation;

        public override bool Equals(object obj) => obj is PhysicsBodyHandle other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(Id, Generation);

        public override string ToString() => IsValid ? $"Body({Id}:{Generation})" : "Body(Invalid)";
    }

    /// <summary>
    /// 模拟性能统计。暴露给用户实现完全性能监控与调优（超越 Unity Physics 的黑箱）。
    /// </summary>
    public struct SimulationStats
    {
        public float deltaTime;
        public int substeps;
        public double integrationTimeMs;
        public double broadphaseTimeMs;
        public double narrowphaseTimeMs;
        public double solverTimeMs;
        public int contactCount;
        public float totalTimeMs => (float)(integrationTimeMs + broadphaseTimeMs + narrowphaseTimeMs + solverTimeMs);
    }
}
