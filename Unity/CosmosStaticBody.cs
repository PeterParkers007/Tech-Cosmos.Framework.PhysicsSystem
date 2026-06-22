using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    /// <summary>
    /// 无刚体的静态碰撞体：适用于地面、墙体等静态几何。
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class CosmosStaticBody : MonoBehaviour
    {
        private PhysicsBody _body;
        private CosmosCollider _collider;

        private void Awake()
        {
            _collider = GetComponent<CosmosCollider>();
            if (_collider == null)
            {
                Debug.LogError("[CosmosStaticBody] Missing CosmosCollider on " + name, this);
                enabled = false;
                return;
            }

            var descriptor = new PhysicsBodyDescriptor
            {
                bodyType = BodyType.Static,
                position = PhysicsConversions.ToFloat3(transform.position),
                rotation = PhysicsConversions.ToFloatQuat(transform.rotation),
                filter = _collider.BuildFilter(),
                material = _collider.BuildMaterial(),
                shape = _collider.BuildShape(),
                userData = _collider,
                isTrigger = _collider.IsTrigger
            };

            _body = CosmosPhysics.World.CreateBody(descriptor);
            _collider.BindBody(_body);
        }

        private void OnEnable()
        {
            if (_body != null) _body.IsEnabled = true;
        }

        private void OnDisable()
        {
            if (_body != null) _body.IsEnabled = false;
        }

        private void OnDestroy()
        {
            if (_body != null)
                CosmosPhysics.World.DestroyBody(_body.Handle);
        }
    }
}
