using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    /// <summary>
    /// 场景级物理驱动器：在 FixedUpdate 中推进纯物理世界，并同步 Transform。
    /// 挂到场景中即可替代 Unity 内置 Physics 模拟循环。
    /// </summary>
    [DefaultExecutionOrder(-200)]
    [DisallowMultipleComponent]
    public sealed class CosmosPhysicsWorld : MonoBehaviour
    {
        [SerializeField] private PhysicsSettingsAsset settingsAsset;
        [SerializeField] private bool simulateInFixedUpdate = true;
        [SerializeField] private bool autoSyncTransforms = true;

        private PhysicsWorld _world;

        public static CosmosPhysicsWorld Instance { get; private set; }
        public static PhysicsWorld ActiveWorld => Instance != null ? Instance._world : null;

        public PhysicsWorld World => _world;

        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Debug.LogWarning("[CosmosPhysics] Duplicate CosmosPhysicsWorld detected. Destroying duplicate.");
                Destroy(this);
                return;
            }

            Instance = this;
            _world = settingsAsset != null
                ? PhysicsWorld.Create(settingsAsset.ToRuntimeSettings())
                : PhysicsWorld.Create(PhysicsSettings.Default);

            PhysicsHub.DefaultWorld = _world;
        }

        private void FixedUpdate()
        {
            if (!simulateInFixedUpdate || _world == null) return;
            _world.Simulate(Time.fixedDeltaTime);
            if (autoSyncTransforms)
                CosmosRigidbody.SyncAllTransforms();
        }

        public void Simulate(float deltaTime)
        {
            _world?.Simulate(deltaTime);
            if (autoSyncTransforms)
                CosmosRigidbody.SyncAllTransforms();
        }

        private void OnDestroy()
        {
            if (Instance == this)
            {
                Instance = null;
                PhysicsHub.Reset();
            }
        }
    }
}
