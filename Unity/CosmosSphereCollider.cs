using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    [DisallowMultipleComponent]
    public sealed class CosmosSphereCollider : CosmosCollider
    {
        [SerializeField] private float radius = 0.5f;

        public float Radius
        {
            get => radius;
            set
            {
                radius = Mathf.Max(value, 0.0001f);
                if (AttachedRigidbody != null)
                    AttachedRigidbody.RefreshShape();
            }
        }

        public override ColliderShape BuildShape()
        {
            return ColliderShape.Sphere(radius, PhysicsConversions.ToFloat3(center));
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = isTrigger ? Color.cyan : Color.green;
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawWireSphere(center, radius);
        }
    }
}
