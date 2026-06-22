using System;
using System.Collections.Generic;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    /// <summary>
    /// 连续碰撞检测：对高速刚体沿运动轨迹做 TOI 扫描，防止穿透。
    /// </summary>
    internal static class CcdSolver
    {
        private const int ToiBinarySearchIterations = 12;
        private const float ToiSurfaceEpsilon = 0.001f;

        public static bool NeedsCcd(PhysicsBody body, in PhysicsSettings settings)
        {
            if (body.BodyType != BodyType.Dynamic || !body.IsEnabled)
                return false;

            if (body.CcdEnabled)
                return true;

            if (!settings.enableCcd)
                return false;

            float speedSq = Float3Math.Dot(body.Velocity, body.Velocity);
            float threshold = Math.Max(settings.ccdThreshold, 0.01f);
            return speedSq >= threshold * threshold;
        }

        public static void ApplyMotion(
            PhysicsBody body,
            Float3 startPosition,
            float deltaTime,
            IReadOnlyList<PhysicsBody> bodies)
        {
            Float3 displacement = body.Velocity * deltaTime;
            float distance = (float)Math.Sqrt(Float3Math.Dot(displacement, displacement));
            if (distance <= 1e-6f)
            {
                body.Position = startPosition;
                return;
            }

            float sweepRadius = GetSweepRadius(body);
            float bestToi = 1f;
            ContactManifold bestContact = default;
            bool foundHit = false;

            for (int i = 0; i < bodies.Count; i++)
            {
                PhysicsBody other = bodies[i];
                if (ReferenceEquals(other, body) || !other.IsEnabled) continue;
                if (other.BodyType == BodyType.Dynamic && !other.CcdEnabled) continue;
                if (!body.Filter.CanInteractWith(other.Filter)) continue;

                if (SweepSphereAgainstBody(startPosition, displacement, sweepRadius, body, other, out float toi, out ContactManifold contact)
                    && toi < bestToi)
                {
                    bestToi = toi;
                    bestContact = contact;
                    foundHit = true;
                }
            }

            if (!foundHit)
            {
                body.Position = startPosition + displacement;
                return;
            }

            float travel = Math.Max(0f, bestToi * distance - ToiSurfaceEpsilon);
            body.Position = startPosition + displacement * (travel / distance);

            float normalSpeed = Float3Math.Dot(body.Velocity, bestContact.normal);
            if (normalSpeed < 0f)
                body.Velocity = body.Velocity - bestContact.normal * normalSpeed;

            body.WakeUp();
        }

        public static float GetSweepRadius(PhysicsBody body)
        {
            if (body.CcdRadius > 0f)
                return body.CcdRadius;

            switch (body.Shape.type)
            {
                case ColliderShapeType.Sphere:
                    return body.Shape.radius;
                case ColliderShapeType.Box:
                {
                    Float3 s = body.Shape.size;
                    return (float)Math.Sqrt(s.x * s.x + s.y * s.y + s.z * s.z);
                }
                case ColliderShapeType.Capsule:
                {
                    float halfLine = Math.Max(0f, body.Shape.height * 0.5f - body.Shape.radius);
                    return body.Shape.radius + halfLine;
                }
                default:
                    return 0.1f;
            }
        }

        private static bool SweepSphereAgainstBody(
            Float3 start,
            Float3 displacement,
            float radius,
            PhysicsBody movingBody,
            PhysicsBody other,
            out float toi,
            out ContactManifold contact)
        {
            contact = default;
            toi = 1f;

            var probe = new PhysicsBody
            {
                Position = start,
                Rotation = movingBody.Rotation,
                Shape = ColliderShape.Sphere(radius),
                BodyType = BodyType.Dynamic,
                Filter = movingBody.Filter,
                IsEnabled = true
            };

            if (!NarrowPhase.TestPair(probe, other, out contact))
            {
                probe.Position = start + displacement;
                if (!NarrowPhase.TestPair(probe, other, out _))
                    return false;
            }

            float lo = 0f;
            float hi = 1f;
            ContactManifold hitContact = default;

            for (int i = 0; i < ToiBinarySearchIterations; i++)
            {
                float mid = (lo + hi) * 0.5f;
                probe.Position = start + displacement * mid;
                probe.Shape = ColliderShape.Sphere(radius);

                if (NarrowPhase.TestPair(probe, other, out hitContact))
                    hi = mid;
                else
                    lo = mid;
            }

            toi = hi;
            contact = hitContact;
            contact.BodyA = movingBody;
            contact.BodyB = other;
            if (Float3Math.Dot(contact.normal, displacement) > 0f)
                contact.normal = contact.normal * -1f;

            return true;
        }
    }
}
