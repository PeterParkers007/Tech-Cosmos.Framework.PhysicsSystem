using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    [CreateAssetMenu(fileName = "CosmosPhysicsSettings", menuName = "Tech-Cosmos/Physics/Settings")]
    public sealed class PhysicsSettingsAsset : ScriptableObject
    {
        public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
        [Min(1)] public int velocityIterations = 4;
        [Min(1)] public int positionIterations = 2;
        public float linearSleepThreshold = 0.01f;
        public float angularSleepThreshold = 0.01f;
        public float broadPhaseCellSize = 2f;

        public PhysicsSettings ToRuntimeSettings()
        {
            return new PhysicsSettings
            {
                gravity = PhysicsConversions.ToFloat3(gravity),
                velocityIterations = velocityIterations,
                positionIterations = positionIterations,
                linearSleepThreshold = linearSleepThreshold,
                angularSleepThreshold = angularSleepThreshold,
                broadPhaseCellSize = broadPhaseCellSize
            };
        }
    }
}
