using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    [DisallowMultipleComponent]
    public sealed class CosmosBoxCollider : CosmosCollider
    {
        [SerializeField] private Vector3 size = Vector3.one;

        public Vector3 Size
        {
            get => size;
            set
            {
                size = new Vector3(
                    Mathf.Max(value.x, 0.0001f),
                    Mathf.Max(value.y, 0.0001f),
                    Mathf.Max(value.z, 0.0001f));
                if (AttachedRigidbody != null)
                    AttachedRigidbody.RefreshShape();
            }
        }

        public override ColliderShape BuildShape()
        {
            Vector3 halfExtents = size * 0.5f;
            return ColliderShape.Box(
                PhysicsConversions.ToFloat3(halfExtents),
                PhysicsConversions.ToFloat3(center));
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = isTrigger ? Color.cyan : Color.green;
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawWireCube(center, size);
        }
    }
}
