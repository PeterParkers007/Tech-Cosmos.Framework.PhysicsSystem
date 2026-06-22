using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    [DisallowMultipleComponent]
    public sealed class CosmosCapsuleCollider : CosmosCollider
    {
        [SerializeField] private float radius = 0.5f;
        [SerializeField] private float height = 2f;
        [SerializeField] private int direction = 1;

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

        public float Height
        {
            get => height;
            set
            {
                height = Mathf.Max(value, radius * 2f);
                if (AttachedRigidbody != null)
                    AttachedRigidbody.RefreshShape();
            }
        }

        public override ColliderShape BuildShape()
        {
            return ColliderShape.Capsule(
                radius,
                height,
                direction,
                PhysicsConversions.ToFloat3(center));
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = isTrigger ? Color.cyan : Color.green;
            Gizmos.matrix = transform.localToWorldMatrix;
            Vector3 axis = direction switch
            {
                0 => Vector3.right,
                2 => Vector3.forward,
                _ => Vector3.up
            };

            float halfLine = Mathf.Max(0f, height * 0.5f - radius);
            Vector3 top = center + axis * halfLine;
            Vector3 bottom = center - axis * halfLine;
            Gizmos.DrawWireSphere(top, radius);
            Gizmos.DrawWireSphere(bottom, radius);
        }
    }
}
