using System.Collections.Generic;
using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    /// <summary>
    /// 对齐 Unity Rigidbody 的轻量替代组件，驱动纯物理 <see cref="PhysicsBody"/>。
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class CosmosRigidbody : MonoBehaviour
    {
        private static readonly List<CosmosRigidbody> Registered = new List<CosmosRigidbody>(128);

        [SerializeField] private BodyType bodyType = BodyType.Dynamic;
        [SerializeField] private float mass = 1f;
        [SerializeField] private bool useGravity = true;
        [SerializeField] private bool isKinematic;
        [SerializeField] private float linearDrag;
        [SerializeField] private float angularDrag;
        [SerializeField] private bool isTrigger;
        [SerializeField] private bool ccdEnabled;
        [SerializeField] private float ccdRadius;

        private PhysicsBody _body;
        private CosmosCollider _collider;

        public PhysicsBody Body => _body;
        public bool IsSleeping => _body != null && _body.IsSleeping;

        public float Mass
        {
            get => mass;
            set
            {
                mass = Mathf.Max(value, 0.0001f);
                if (_body != null) _body.Mass = mass;
            }
        }

        public bool UseGravity
        {
            get => useGravity;
            set
            {
                useGravity = value;
                if (_body != null) _body.UseGravity = value;
            }
        }

        public bool IsKinematic
        {
            get => isKinematic;
            set
            {
                isKinematic = value;
                ApplyBodyType();
            }
        }

        public Vector3 Velocity
        {
            get => _body != null ? PhysicsConversions.ToVector3(_body.Velocity) : Vector3.zero;
            set
            {
                if (_body != null) _body.Velocity = PhysicsConversions.ToFloat3(value);
            }
        }

        public Vector3 AngularVelocity
        {
            get => _body != null ? PhysicsConversions.ToVector3(_body.AngularVelocity) : Vector3.zero;
            set
            {
                if (_body != null) _body.AngularVelocity = PhysicsConversions.ToFloat3(value);
            }
        }

        private void Awake()
        {
            _collider = GetComponent<CosmosCollider>();
            if (_collider == null)
            {
                Debug.LogError("[CosmosRigidbody] Missing CosmosCollider on " + name, this);
                enabled = false;
                return;
            }

            Registered.Add(this);
            CreateOrRefreshBody();
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
            Registered.Remove(this);
            if (_body != null)
                CosmosPhysics.World.DestroyBody(_body.Handle);
            _body = null;
        }

        internal void RefreshShape()
        {
            if (_body == null || _collider == null) return;
            _body.Shape = _collider.BuildShape();
            _body.IsTrigger = isTrigger || _collider.IsTrigger;
        }

        private void CreateOrRefreshBody()
        {
            var world = CosmosPhysics.World;
            if (_body == null)
            {
                var descriptor = new PhysicsBodyDescriptor
                {
                    bodyType = ResolveBodyType(),
                    position = PhysicsConversions.ToFloat3(transform.position),
                    rotation = PhysicsConversions.ToFloatQuat(transform.rotation),
                    mass = mass,
                    useGravity = useGravity,
                    linearDrag = linearDrag,
                    angularDrag = angularDrag,
                    isTrigger = isTrigger || _collider.IsTrigger,
                    filter = _collider.BuildFilter(),
                    material = _collider.BuildMaterial(),
                    shape = _collider.BuildShape(),
                    userData = _collider
                };

                _body = world.CreateBody(descriptor);
                _body.CcdEnabled = ccdEnabled;
                _body.CcdRadius = ccdRadius;
                _collider.BindBody(_body);
            }
            else
            {
                ApplyBodyType();
                RefreshShape();
            }
        }

        private BodyType ResolveBodyType()
        {
            if (isKinematic) return BodyType.Kinematic;
            return bodyType;
        }

        private void ApplyBodyType()
        {
            if (_body == null) return;
            _body.BodyType = ResolveBodyType();
            _body.Mass = _body.BodyType == BodyType.Dynamic ? mass : 0f;
        }

        public void MovePosition(Vector3 position)
        {
            if (_body == null) return;
            _body.MovePosition(PhysicsConversions.ToFloat3(position));
            if (_body.BodyType != BodyType.Dynamic)
                transform.position = position;
        }

        public void MoveRotation(Quaternion rotation)
        {
            if (_body == null) return;
            _body.MoveRotation(PhysicsConversions.ToFloatQuat(rotation));
            if (_body.BodyType != BodyType.Dynamic)
                transform.rotation = rotation;
        }

        public void AddForce(Vector3 force, ForceMode mode = ForceMode.Force)
        {
            if (_body == null) return;

            switch (mode)
            {
                case ForceMode.Impulse:
                case ForceMode.VelocityChange:
                    _body.ApplyImpulse(PhysicsConversions.ToFloat3(force));
                    break;
                default:
                    _body.ApplyForce(PhysicsConversions.ToFloat3(force) * Time.fixedDeltaTime);
                    break;
            }
        }

        public void WakeUp()
        {
            _body?.WakeUp();
        }

        internal void SyncTransformFromBody()
        {
            if (_body == null || _body.BodyType != BodyType.Dynamic) return;
            transform.SetPositionAndRotation(
                PhysicsConversions.ToVector3(_body.Position),
                PhysicsConversions.ToQuaternion(_body.Rotation));
        }

        internal static void SyncAllTransforms()
        {
            for (int i = 0; i < Registered.Count; i++)
                Registered[i].SyncTransformFromBody();
        }
    }
}
