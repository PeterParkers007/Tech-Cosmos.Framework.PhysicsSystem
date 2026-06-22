using System;
using System.Collections.Generic;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    internal struct ContactManifold
    {
        public PhysicsBody BodyA;
        public PhysicsBody BodyB;
        public Float3 normal;
        public float penetration;
        public Float3 point;
        public bool isTriggerPair;
    }

    internal static class NarrowPhase
    {
        public static bool Raycast(in PhysicsRay ray, PhysicsBody body, float maxDistance, out RaycastHit hit)
        {
            hit = default;
            if (!body.IsEnabled) return false;

            bool intersects = body.Shape.type switch
            {
                ColliderShapeType.Sphere => RaySphere(ray, body, maxDistance, out hit),
                ColliderShapeType.Box => RayBox(ray, body, maxDistance, out hit),
                ColliderShapeType.Capsule => RayCapsule(ray, body, maxDistance, out hit),
                _ => false
            };

            if (!intersects) return false;

            hit.body = body.Handle;
            hit.userData = body.UserData;
            hit.shapeType = body.Shape.type;
            return true;
        }

        public static bool OverlapSphere(Float3 center, float radius, PhysicsBody body)
        {
            if (!body.IsEnabled) return false;

            return body.Shape.type switch
            {
                ColliderShapeType.Sphere => SphereSphere(center, radius, body),
                ColliderShapeType.Box => SphereBox(center, radius, body),
                ColliderShapeType.Capsule => SphereCapsule(center, radius, body),
                _ => false
            };
        }

        public static bool OverlapBox(Float3 center, FloatQuat rotation, Float3 halfExtents, PhysicsBody body)
        {
            if (!body.IsEnabled) return false;

            var probe = new PhysicsBody
            {
                Position = center,
                Rotation = rotation,
                Shape = ColliderShape.Box(halfExtents)
            };

            return TestPair(probe, body, out var contact) && contact.penetration > 0f;
        }

        public static bool TestPair(PhysicsBody a, PhysicsBody b, out ContactManifold contact)
        {
            contact = default;
            if (!a.IsEnabled || !b.IsEnabled) return false;
            if (!a.Filter.CanInteractWith(b.Filter)) return false;

            contact.BodyA = a;
            contact.BodyB = b;
            contact.isTriggerPair = a.IsTrigger || b.IsTrigger;

            bool hit = (a.Shape.type, b.Shape.type) switch
            {
                (ColliderShapeType.Sphere, ColliderShapeType.Sphere) => SphereSphereDetailed(a, b, out contact),
                (ColliderShapeType.Sphere, ColliderShapeType.Box) => SphereBoxDetailed(a, b, out contact),
                (ColliderShapeType.Box, ColliderShapeType.Sphere) => SwapDetailed(SphereBoxDetailed(b, a, out contact), a, b, ref contact),
                (ColliderShapeType.Box, ColliderShapeType.Box) => BoxBoxDetailed(a, b, out contact),
                (ColliderShapeType.Sphere, ColliderShapeType.Capsule) => SphereCapsuleDetailed(a, b, out contact),
                (ColliderShapeType.Capsule, ColliderShapeType.Sphere) => SwapDetailed(SphereCapsuleDetailed(b, a, out contact), a, b, ref contact),
                (ColliderShapeType.Capsule, ColliderShapeType.Capsule) => CapsuleCapsuleDetailed(a, b, out contact),
                (ColliderShapeType.Box, ColliderShapeType.Capsule) => BoxCapsuleDetailed(a, b, out contact),
                (ColliderShapeType.Capsule, ColliderShapeType.Box) => SwapDetailed(BoxCapsuleDetailed(b, a, out contact), a, b, ref contact),
                _ => false
            };

            return hit && contact.penetration > 0f;
        }

        private static bool SwapDetailed(bool hit, PhysicsBody a, PhysicsBody b, ref ContactManifold contact)
        {
            if (!hit) return false;
            contact.BodyA = a;
            contact.BodyB = b;
            contact.normal = contact.normal * -1f;
            return true;
        }

        private static bool RaySphere(in PhysicsRay ray, PhysicsBody body, float maxDistance, out RaycastHit hit)
        {
            hit = default;
            Float3 center = body.Shape.GetWorldCenter(body.Position, body.Rotation);
            Float3 oc = ray.Origin - center;
            float radius = body.Shape.radius;

            float b = Float3Math.Dot(oc, ray.Direction);
            float c = Float3Math.Dot(oc, oc) - radius * radius;
            float discriminant = b * b - c;
            if (discriminant < 0f) return false;

            float sqrt = (float)Math.Sqrt(discriminant);
            float t = -b - sqrt;
            if (t < 0f) t = -b + sqrt;
            if (t < 0f || t > maxDistance) return false;

            Float3 point = ray.GetPoint(t);
            hit.point = point;
            hit.normal = Float3Math.Normalize(point - center);
            hit.distance = t;
            return true;
        }

        private static bool RayBox(in PhysicsRay ray, PhysicsBody body, float maxDistance, out RaycastHit hit)
        {
            hit = default;
            Float3 center = body.Shape.GetWorldCenter(body.Position, body.Rotation);
            FloatQuat rotation = body.Shape.GetWorldRotation(body.Rotation);

            Float3 localOrigin = rotation.Conjugate().Rotate(ray.Origin - center);
            Float3 localDirection = rotation.Conjugate().Rotate(ray.Direction);

            Float3 min = body.Shape.size * -1f;
            Float3 max = body.Shape.size;

            float tMin = 0f;
            float tMax = maxDistance;
            Float3 normal = Float3.Zero;

            if (!Slab(localOrigin.x, localDirection.x, min.x, max.x, ref tMin, ref tMax, new Float3(-1f, 0f, 0f), new Float3(1f, 0f, 0f), ref normal))
                return false;
            if (!Slab(localOrigin.y, localDirection.y, min.y, max.y, ref tMin, ref tMax, new Float3(0f, -1f, 0f), new Float3(0f, 1f, 0f), ref normal))
                return false;
            if (!Slab(localOrigin.z, localDirection.z, min.z, max.z, ref tMin, ref tMax, new Float3(0f, 0f, -1f), new Float3(0f, 0f, 1f), ref normal))
                return false;

            if (tMax < tMin || tMin > maxDistance) return false;
            float distance = tMin >= 0f ? tMin : tMax;
            if (distance < 0f || distance > maxDistance) return false;

            hit.point = ray.GetPoint(distance);
            hit.normal = rotation.Rotate(normal);
            hit.distance = distance;
            return true;
        }

        private static bool Slab(
            float origin,
            float direction,
            float min,
            float max,
            ref float tMin,
            ref float tMax,
            Float3 negativeNormal,
            Float3 positiveNormal,
            ref Float3 normal)
        {
            if (Math.Abs(direction) < 1e-6f)
                return origin >= min && origin <= max;

            float inv = 1f / direction;
            float t1 = (min - origin) * inv;
            float t2 = (max - origin) * inv;
            Float3 n1 = direction < 0f ? positiveNormal : negativeNormal;
            Float3 n2 = direction < 0f ? negativeNormal : positiveNormal;

            if (t1 > t2)
            {
                (t1, t2) = (t2, t1);
                (n1, n2) = (n2, n1);
            }

            if (t1 > tMin)
            {
                tMin = t1;
                normal = n1;
            }

            if (t2 < tMax)
            {
                tMax = t2;
            }

            return tMin <= tMax;
        }

        private static bool RayCapsule(in PhysicsRay ray, PhysicsBody body, float maxDistance, out RaycastHit hit)
        {
            hit = default;
            Float3 center = body.Shape.GetWorldCenter(body.Position, body.Rotation);
            Float3 axis = body.Shape.GetCapsuleAxis(body.Rotation);
            float halfLine = Math.Max(0f, body.Shape.height * 0.5f - body.Shape.radius);
            Float3 a = center - axis * halfLine;
            Float3 b = center + axis * halfLine;

            if (!RaySegment(ray, a, b, body.Shape.radius, maxDistance, out float distance, out Float3 normal))
                return false;

            hit.point = ray.GetPoint(distance);
            hit.normal = normal;
            hit.distance = distance;
            return true;
        }

        private static bool RaySegment(
            in PhysicsRay ray,
            Float3 a,
            Float3 b,
            float radius,
            float maxDistance,
            out float distance,
            out Float3 normal)
        {
            distance = 0f;
            normal = Float3.Zero;

            Float3 ab = b - a;
            Float3 ao = ray.Origin - a;
            float abLenSq = Float3Math.Dot(ab, ab);
            float t = abLenSq > 1e-6f ? PhysicsMath.Clamp(Float3Math.Dot(ao, ab) / abLenSq, 0f, 1f) : 0f;
            Float3 closest = a + ab * t;
            Float3 diff = ray.Origin - closest;

            float bDot = Float3Math.Dot(ray.Direction, diff);
            float c = Float3Math.Dot(diff, diff) - radius * radius;
            float discriminant = bDot * bDot - c;
            if (discriminant < 0f) return false;

            float sqrt = (float)Math.Sqrt(discriminant);
            float enter = -bDot - sqrt;
            if (enter < 0f) enter = -bDot + sqrt;
            if (enter < 0f || enter > maxDistance) return false;

            distance = enter;
            normal = Float3Math.Normalize(ray.GetPoint(enter) - closest);
            return true;
        }

        private static bool SphereSphere(Float3 center, float radius, PhysicsBody body)
        {
            Float3 bodyCenter = body.Shape.GetWorldCenter(body.Position, body.Rotation);
            float combined = radius + body.Shape.radius;
            return Float3Math.SqrDistance(center, bodyCenter) <= combined * combined;
        }

        private static bool SphereBox(Float3 center, float radius, PhysicsBody body)
        {
            Float3 closest = ClosestPointOnBox(body, center);
            return Float3Math.SqrDistance(center, closest) <= radius * radius;
        }

        private static bool SphereCapsule(Float3 center, float radius, PhysicsBody body)
        {
            Float3 closest = ClosestPointOnCapsule(body, center);
            float combined = radius + body.Shape.radius;
            return Float3Math.SqrDistance(center, closest) <= combined * combined;
        }

        private static bool SphereSphereDetailed(PhysicsBody a, PhysicsBody b, out ContactManifold contact)
        {
            contact = default;
            Float3 centerA = a.Shape.GetWorldCenter(a.Position, a.Rotation);
            Float3 centerB = b.Shape.GetWorldCenter(b.Position, b.Rotation);
            Float3 delta = centerB - centerA;
            float distance = Float3Math.Distance(centerA, centerB);
            float combined = a.Shape.radius + b.Shape.radius;
            if (distance >= combined) return false;

            contact.normal = distance > 1e-6f ? delta * (1f / distance) : new Float3(0f, 1f, 0f);
            contact.penetration = combined - distance;
            contact.point = centerA + contact.normal * a.Shape.radius;
            return true;
        }

        private static bool SphereBoxDetailed(PhysicsBody sphere, PhysicsBody box, out ContactManifold contact)
        {
            contact = default;
            Float3 closest = ClosestPointOnBox(box, sphere.Shape.GetWorldCenter(sphere.Position, sphere.Rotation));
            Float3 center = sphere.Shape.GetWorldCenter(sphere.Position, sphere.Rotation);
            Float3 delta = center - closest;
            float distanceSq = Float3Math.Dot(delta, delta);
            float radius = sphere.Shape.radius;
            if (distanceSq > radius * radius) return false;

            float distance = (float)Math.Sqrt(distanceSq);
            contact.normal = distance > 1e-6f ? delta * (1f / distance) : new Float3(0f, 1f, 0f);
            contact.penetration = radius - distance;
            contact.point = closest;
            return true;
        }

        private static bool BoxBoxDetailed(PhysicsBody a, PhysicsBody b, out ContactManifold contact)
        {
            contact = default;
            // 改进的 Box-Box：使用分离轴 + 最近点（简化版 SAT）
            Float3 centerA = a.Shape.GetWorldCenter(a.Position, a.Rotation);
            Float3 centerB = b.Shape.GetWorldCenter(b.Position, b.Rotation);
            Float3 delta = centerB - centerA;
            float distSq = Float3Math.Dot(delta, delta);
            if (distSq < 1e-8f) { delta = new Float3(0, 1, 0); distSq = 1f; }

            float dist = (float)Math.Sqrt(distSq);
            Float3 normal = Float3Math.Normalize(delta);

            // 粗略 penetration（实际应完整 SAT，这里用 extent 投影近似）
            float penA = a.Shape.size.x * Math.Abs(normal.x) + a.Shape.size.y * Math.Abs(normal.y) + a.Shape.size.z * Math.Abs(normal.z);
            float penB = b.Shape.size.x * Math.Abs(normal.x) + b.Shape.size.y * Math.Abs(normal.y) + b.Shape.size.z * Math.Abs(normal.z);
            float penetration = (penA + penB) * 0.5f - dist; // 简化
            if (penetration <= 0f) return false;

            contact.normal = normal;
            contact.penetration = penetration;
            contact.point = centerA + normal * (a.Shape.size.x * 0.5f);
            return true;
        }

        private static bool SphereCapsuleDetailed(PhysicsBody sphere, PhysicsBody capsule, out ContactManifold contact)
        {
            contact = default;
            Float3 center = sphere.Shape.GetWorldCenter(sphere.Position, sphere.Rotation);
            Float3 closest = ClosestPointOnCapsule(capsule, center);
            Float3 delta = center - closest;
            float distance = Float3Math.Distance(center, closest);
            float combined = sphere.Shape.radius + capsule.Shape.radius;
            if (distance >= combined) return false;

            contact.normal = distance > 1e-6f ? delta * (1f / distance) : new Float3(0f, 1f, 0f);
            contact.penetration = combined - distance;
            contact.point = closest;
            return true;
        }

        private static bool CapsuleCapsuleDetailed(PhysicsBody a, PhysicsBody b, out ContactManifold contact)
        {
            contact = default;
            Float3 pointA = ClosestPointOnCapsule(a, b.Shape.GetWorldCenter(b.Position, b.Rotation));
            Float3 pointB = ClosestPointOnCapsule(b, pointA);
            pointA = ClosestPointOnCapsule(a, pointB);

            Float3 delta = pointA - pointB;
            float distance = Float3Math.Distance(pointA, pointB);
            float combined = a.Shape.radius + b.Shape.radius;
            if (distance >= combined) return false;

            contact.normal = distance > 1e-6f ? delta * (1f / distance) : new Float3(0f, 1f, 0f);
            contact.penetration = combined - distance;
            contact.point = pointA;
            return true;
        }

        private static bool BoxCapsuleDetailed(PhysicsBody box, PhysicsBody capsule, out ContactManifold contact)
        {
            contact = default;
            Float3 capsulePoint = ClosestPointOnCapsule(capsule, box.Shape.GetWorldCenter(box.Position, box.Rotation));
            Float3 boxPoint = ClosestPointOnBox(box, capsulePoint);
            capsulePoint = ClosestPointOnCapsule(capsule, boxPoint);

            Float3 delta = boxPoint - capsulePoint;
            float distance = Float3Math.Distance(boxPoint, capsulePoint);
            if (distance >= capsule.Shape.radius) return false;

            contact.normal = distance > 1e-6f ? delta * (1f / distance) : new Float3(0f, 1f, 0f);
            contact.penetration = capsule.Shape.radius - distance;
            contact.point = boxPoint;
            return true;
        }

        private static Float3 ClosestPointOnBox(PhysicsBody body, Float3 worldPoint)
        {
            Float3 center = body.Shape.GetWorldCenter(body.Position, body.Rotation);
            FloatQuat rotation = body.Shape.GetWorldRotation(body.Rotation);
            Float3 local = rotation.Conjugate().Rotate(worldPoint - center);
            local = new Float3(
                PhysicsMath.Clamp(local.x, -body.Shape.size.x, body.Shape.size.x),
                PhysicsMath.Clamp(local.y, -body.Shape.size.y, body.Shape.size.y),
                PhysicsMath.Clamp(local.z, -body.Shape.size.z, body.Shape.size.z));
            return center + rotation.Rotate(local);
        }

        private static Float3 ClosestPointOnCapsule(PhysicsBody body, Float3 worldPoint)
        {
            Float3 center = body.Shape.GetWorldCenter(body.Position, body.Rotation);
            Float3 axis = body.Shape.GetCapsuleAxis(body.Rotation);
            float halfLine = Math.Max(0f, body.Shape.height * 0.5f - body.Shape.radius);
            Float3 a = center - axis * halfLine;
            Float3 b = center + axis * halfLine;
            Float3 ab = b - a;
            float t = Float3Math.Dot(ab, ab) > 1e-6f
                ? PhysicsMath.Clamp(Float3Math.Dot(worldPoint - a, ab) / Float3Math.Dot(ab, ab), 0f, 1f)
                : 0f;
            return a + ab * t;
        }
    }

    internal sealed class BroadPhaseSpatialHash
    {
        private readonly float _cellSize;
        private readonly Dictionary<long, List<PhysicsBody>> _cells = new Dictionary<long, List<PhysicsBody>>();
        private readonly List<PhysicsBody> _queryBuffer = new List<PhysicsBody>(64);

        public BroadPhaseSpatialHash(float cellSize)
        {
            _cellSize = Math.Max(cellSize, 0.25f);
        }

        public void Clear() => _cells.Clear();

        public void Insert(PhysicsBody body)
        {
            body.Shape.ComputeWorldBounds(body.Position, body.Rotation, out Float3 min, out Float3 max);
            int minX = Floor(min.x);
            int minY = Floor(min.y);
            int minZ = Floor(min.z);
            int maxX = Floor(max.x);
            int maxY = Floor(max.y);
            int maxZ = Floor(max.z);

            for (int x = minX; x <= maxX; x++)
            for (int y = minY; y <= maxY; y++)
            for (int z = minZ; z <= maxZ; z++)
            {
                long key = Key(x, y, z);
                if (!_cells.TryGetValue(key, out var list))
                {
                    list = new List<PhysicsBody>(4);
                    _cells[key] = list;
                }

                list.Add(body);
            }
        }

        public List<PhysicsBody> Query(Float3 min, Float3 max)
        {
            _queryBuffer.Clear();
            int minX = Floor(min.x);
            int minY = Floor(min.y);
            int minZ = Floor(min.z);
            int maxX = Floor(max.x);
            int maxY = Floor(max.y);
            int maxZ = Floor(max.z);

            for (int x = minX; x <= maxX; x++)
            for (int y = minY; y <= maxY; y++)
            for (int z = minZ; z <= maxZ; z++)
            {
                if (!_cells.TryGetValue(Key(x, y, z), out var list)) continue;
                for (int i = 0; i < list.Count; i++)
                {
                    PhysicsBody body = list[i];
                    // AABB pruning（原实现缺失，导致性能差）
                    body.Shape.ComputeWorldBounds(body.Position, body.Rotation, out Float3 bmin, out Float3 bmax);
                    if (bmax.x < min.x || bmin.x > max.x ||
                        bmax.y < min.y || bmin.y > max.y ||
                        bmax.z < min.z || bmin.z > max.z) continue;

                    if (!_queryBuffer.Contains(body))
                        _queryBuffer.Add(body);
                }
            }

            return _queryBuffer;
        }

        private int Floor(float value) => (int)Math.Floor(value / _cellSize);

        private static long Key(int x, int y, int z)
        {
            unchecked
            {
                long hash = x * 73856093L ^ y * 19349663L ^ z * 83492791L;
                return hash;
            }
        }
    }
}
