using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    /// <summary>
    /// 碰撞体基类，对齐 Unity Collider 用法。
    /// </summary>
    public abstract class CosmosCollider : MonoBehaviour
    {
        [SerializeField] protected Vector3 center;
        [SerializeField] protected bool isTrigger;
        [SerializeField] protected PhysicMaterial legacyMaterial;
        [SerializeField] protected float bounciness = 0f;
        [SerializeField] protected float friction = 0.5f;

        private PhysicsBody _body;
        private CosmosRigidbody _rigidbody;

        public bool IsTrigger => isTrigger;
        public CosmosRigidbody AttachedRigidbody => _rigidbody;

        public Bounds bounds
        {
            get
            {
                var shape = BuildShape();
                shape.ComputeWorldBounds(
                    PhysicsConversions.ToFloat3(transform.position),
                    PhysicsConversions.ToFloatQuat(transform.rotation),
                    out var min,
                    out var max);

                Vector3 minV = PhysicsConversions.ToVector3(min);
                Vector3 maxV = PhysicsConversions.ToVector3(max);
                var bounds = new Bounds();
                bounds.SetMinMax(minV, maxV);
                return bounds;
            }
        }

        protected virtual void Awake()
        {
            _rigidbody = GetComponent<CosmosRigidbody>();
        }

        protected virtual void OnValidate()
        {
            if (_rigidbody != null)
                _rigidbody.RefreshShape();
        }

        internal void BindBody(PhysicsBody body) => _body = body;

        internal CollisionFilter BuildFilter()
        {
            return new CollisionFilter(gameObject.layer, PhysicsLayerMask.Everything);
        }

        internal PhysicsMaterial BuildMaterial()
        {
            if (legacyMaterial != null)
            {
                return new PhysicsMaterial(legacyMaterial.bounciness, legacyMaterial.dynamicFriction);
            }

            return new PhysicsMaterial(bounciness, friction);
        }

        public abstract ColliderShape BuildShape();
    }
}
