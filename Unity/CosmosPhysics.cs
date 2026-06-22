using TechCosmos.Core.Runtime;
using TechCosmos.PhysicsSystem.Runtime;
using UnityEngine;

namespace TechCosmos.PhysicsSystem.Unity
{
    /// <summary>
    /// Unity 层 RaycastHit，字段对齐 UnityEngine.RaycastHit，便于无痛替换 Physics.Raycast。
    /// </summary>
    public struct CosmosRaycastHit
    {
        public Vector3 point;
        public Vector3 normal;
        public float distance;
        public CosmosCollider collider;
        public CosmosRigidbody rigidbody;
        public Transform transform;

        internal static CosmosRaycastHit FromRuntime(in RaycastHit hit, PhysicsWorld world)
        {
            var result = new CosmosRaycastHit
            {
                point = PhysicsConversions.ToVector3(hit.point),
                normal = PhysicsConversions.ToVector3(hit.normal),
                distance = hit.distance
            };

            if (world.TryGetBody(hit.body, out var body) && body.UserData is CosmosCollider cosmosCollider)
            {
                result.collider = cosmosCollider;
                result.rigidbody = cosmosCollider.AttachedRigidbody;
                result.transform = cosmosCollider.transform;
            }

            return result;
        }
    }

    /// <summary>
    /// 静态查询 API，语义对齐 UnityEngine.Physics，底层走纯物理 <see cref="PhysicsWorld"/>。
    /// </summary>
    public static class CosmosPhysics
    {
        public static PhysicsWorld World
        {
            get => CosmosPhysicsWorld.ActiveWorld ?? PhysicsHub.DefaultWorld;
            set
            {
                if (value == null) throw new System.ArgumentNullException(nameof(value));
                PhysicsHub.DefaultWorld = value;
            }
        }

        public static Vector3 Gravity
        {
            get => PhysicsConversions.ToVector3(World.Gravity);
            set => World.Gravity = PhysicsConversions.ToFloat3(value);
        }

        public static bool Raycast(Vector3 origin, Vector3 direction, out CosmosRaycastHit hitInfo, float maxDistance = Mathf.Infinity, int layerMask = Physics.DefaultRaycastLayers, QueryTriggerInteraction queryTriggerInteraction = QueryTriggerInteraction.UseGlobal)
        {
            hitInfo = default;
            if (!World.Raycast(
                    PhysicsConversions.ToFloat3(origin),
                    PhysicsConversions.ToFloat3(direction),
                    out RaycastHit hit,
                    maxDistance,
                    layerMask))
            {
                return false;
            }

            if (!PassesTriggerFilter(hit, queryTriggerInteraction))
                return false;

            hitInfo = CosmosRaycastHit.FromRuntime(hit, World);
            return true;
        }

        public static bool Raycast(Ray ray, out CosmosRaycastHit hitInfo, float maxDistance = Mathf.Infinity, int layerMask = Physics.DefaultRaycastLayers, QueryTriggerInteraction queryTriggerInteraction = QueryTriggerInteraction.UseGlobal)
        {
            return Raycast(ray.origin, ray.direction, out hitInfo, maxDistance, layerMask, queryTriggerInteraction);
        }

        public static bool SphereCast(Vector3 origin, float radius, Vector3 direction, out CosmosRaycastHit hitInfo, float maxDistance = Mathf.Infinity, int layerMask = Physics.DefaultRaycastLayers, QueryTriggerInteraction queryTriggerInteraction = QueryTriggerInteraction.UseGlobal)
        {
            hitInfo = default;
            if (!World.SphereCast(
                    PhysicsConversions.ToFloat3(origin),
                    radius,
                    PhysicsConversions.ToFloat3(direction),
                    out SphereCastHit hit,
                    maxDistance,
                    layerMask))
            {
                return false;
            }

            if (!World.TryGetBody(hit.body, out var body))
                return false;

            if (!PassesTriggerFilter(body.IsTrigger, queryTriggerInteraction))
                return false;

            hitInfo = new CosmosRaycastHit
            {
                point = PhysicsConversions.ToVector3(hit.point),
                normal = PhysicsConversions.ToVector3(hit.normal),
                distance = hit.distance,
                collider = body.UserData as CosmosCollider,
                rigidbody = (body.UserData as CosmosCollider)?.AttachedRigidbody,
                transform = (body.UserData as CosmosCollider)?.transform
            };
            return true;
        }

        public static CosmosCollider[] OverlapSphere(Vector3 position, float radius, int layerMask = Physics.AllLayers, QueryTriggerInteraction queryTriggerInteraction = QueryTriggerInteraction.UseGlobal)
        {
            var buffer = new OverlapHit[64];
            int count = World.OverlapSphere(PhysicsConversions.ToFloat3(position), radius, buffer, layerMask);
            var results = new CosmosCollider[count];
            int write = 0;

            for (int i = 0; i < count; i++)
            {
                if (!World.TryGetBody(buffer[i].body, out var body)) continue;
                if (!PassesTriggerFilter(body.IsTrigger, queryTriggerInteraction)) continue;
                if (body.UserData is CosmosCollider collider)
                    results[write++] = collider;
            }

            if (write == count) return results;
            var trimmed = new CosmosCollider[write];
            System.Array.Copy(results, trimmed, write);
            return trimmed;
        }

        public static int OverlapSphereNonAlloc(Vector3 position, float radius, CosmosCollider[] results, int layerMask = Physics.AllLayers, QueryTriggerInteraction queryTriggerInteraction = QueryTriggerInteraction.UseGlobal)
        {
            if (results == null) return 0;

            var buffer = new OverlapHit[results.Length];
            int count = World.OverlapSphere(PhysicsConversions.ToFloat3(position), radius, buffer, layerMask);
            int write = 0;

            for (int i = 0; i < count && write < results.Length; i++)
            {
                if (!World.TryGetBody(buffer[i].body, out var body)) continue;
                if (!PassesTriggerFilter(body.IsTrigger, queryTriggerInteraction)) continue;
                if (body.UserData is CosmosCollider collider)
                    results[write++] = collider;
            }

            return write;
        }

        private static bool PassesTriggerFilter(in RaycastHit hit, QueryTriggerInteraction interaction)
        {
            if (!World.TryGetBody(hit.body, out var body)) return false;
            return PassesTriggerFilter(body.IsTrigger, interaction);
        }

        private static bool PassesTriggerFilter(bool isTrigger, QueryTriggerInteraction interaction)
        {
            return interaction switch
            {
                QueryTriggerInteraction.Ignore => !isTrigger,
                QueryTriggerInteraction.Collide => true,
                _ => isTrigger || Physics.queriesHitTriggers
            };
        }
    }
}
