using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    /// <summary>
    /// 通用关节组件：Fixed / Hinge / Distance / Spring。
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class CosmosPhysicsJoint : MonoBehaviour
    {
        [SerializeField] private JointType jointType = JointType.Fixed;
        [SerializeField] private CosmosRigidbody connectedBody;
        [SerializeField] private Vector3 localAnchor = Vector3.zero;
        [SerializeField] private Vector3 connectedAnchor = Vector3.zero;
        [SerializeField] private Vector3 axis = Vector3.up;
        [SerializeField] private float minDistance;
        [SerializeField] private float maxDistance;
        [SerializeField] private float restLength;
        [SerializeField] private float springStiffness = 50f;
        [SerializeField] private float springDamping = 5f;
        [SerializeField] private bool collideConnected;

        private CosmosRigidbody _localBody;
        private PhysicsJoint _joint;

        public PhysicsJoint Joint => _joint;

        private void Awake()
        {
            _localBody = GetComponent<CosmosRigidbody>();
            if (_localBody == null || connectedBody == null)
            {
                Debug.LogError("[CosmosPhysicsJoint] Requires CosmosRigidbody on self and connected body.", this);
                enabled = false;
                return;
            }

            CreateJoint();
        }

        private void OnDestroy()
        {
            if (_joint != null)
                CosmosPhysics.World.DestroyJoint(_joint.Handle);
            _joint = null;
        }

        private void CreateJoint()
        {
            if (_localBody?.Body == null || connectedBody?.Body == null) return;

            var descriptor = new PhysicsJointDescriptor
            {
                type = jointType,
                bodyA = _localBody.Body.Handle,
                bodyB = connectedBody.Body.Handle,
                localAnchorA = PhysicsConversions.ToFloat3(localAnchor),
                localAnchorB = PhysicsConversions.ToFloat3(connectedAnchor),
                localAxisA = PhysicsConversions.ToFloat3(axis.normalized),
                minDistance = minDistance,
                maxDistance = maxDistance,
                restLength = restLength > 0f ? restLength : Vector3.Distance(localAnchor, connectedAnchor),
                springStiffness = springStiffness,
                springDamping = springDamping,
                collideConnected = collideConnected
            };

            _joint = CosmosPhysics.World.CreateJoint(descriptor);
        }
    }
}
