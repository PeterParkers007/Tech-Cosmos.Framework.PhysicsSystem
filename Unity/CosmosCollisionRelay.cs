using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;
using UnityEngine.Events;

namespace TechCosmos.PhysicsSystem.Unity
{
    /// <summary>
    /// 将纯物理碰撞事件转发为 Unity 风格消息，便于迁移现有脚本。
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class CosmosCollisionRelay : MonoBehaviour
    {
        [System.Serializable]
        public sealed class CollisionEvent : UnityEvent<CosmosCollider> { }

        public CollisionEvent onCollisionEnter = new CollisionEvent();
        public CollisionEvent onCollisionStay = new CollisionEvent();
        public CollisionEvent onCollisionExit = new CollisionEvent();
        public CollisionEvent onTriggerEnter = new CollisionEvent();
        public CollisionEvent onTriggerStay = new CollisionEvent();
        public CollisionEvent onTriggerExit = new CollisionEvent();

        private CosmosCollider _selfCollider;

        private void Awake()
        {
            _selfCollider = GetComponent<CosmosCollider>();
            CosmosPhysics.World.CollisionEnter += HandleCollisionEnter;
            CosmosPhysics.World.CollisionStay += HandleCollisionStay;
            CosmosPhysics.World.CollisionExit += HandleCollisionExit;
        }

        private void OnDestroy()
        {
            if (CosmosPhysicsWorld.ActiveWorld == null) return;
            CosmosPhysics.World.CollisionEnter -= HandleCollisionEnter;
            CosmosPhysics.World.CollisionStay -= HandleCollisionStay;
            CosmosPhysics.World.CollisionExit -= HandleCollisionExit;
        }

        private void HandleCollisionEnter(CollisionEnterEvent evt) => Dispatch(evt.BodyA, evt.BodyB, evt.IsTrigger, onCollisionEnter, onTriggerEnter);
        private void HandleCollisionStay(CollisionStayEvent evt) => Dispatch(evt.BodyA, evt.BodyB, evt.IsTrigger, onCollisionStay, onTriggerStay);
        private void HandleCollisionExit(CollisionExitEvent evt) => Dispatch(evt.BodyA, evt.BodyB, evt.IsTrigger, onCollisionExit, onTriggerExit);

        private void Dispatch(
            PhysicsBodyHandle bodyA,
            PhysicsBodyHandle bodyB,
            bool isTrigger,
            CollisionEvent collisionEvent,
            CollisionEvent triggerEvent)
        {
            if (_selfCollider == null) return;
            if (!CosmosPhysics.World.TryGetBody(bodyA, out var a) ||
                !CosmosPhysics.World.TryGetBody(bodyB, out var b))
            {
                return;
            }

            CosmosCollider other = null;
            if (ReferenceEquals(a.UserData, _selfCollider))
                other = b.UserData as CosmosCollider;
            else if (ReferenceEquals(b.UserData, _selfCollider))
                other = a.UserData as CosmosCollider;

            if (other == null) return;

            if (isTrigger)
                triggerEvent.Invoke(other);
            else
                collisionEvent.Invoke(other);
        }
    }
}
