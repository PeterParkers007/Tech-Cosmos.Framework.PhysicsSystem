using System;
using System.Collections.Generic;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    internal static class CollisionSolver
    {
        public static void Resolve(ref ContactManifold contact, in PhysicsSettings settings, bool applyWarmStart, out float normalImpulse)
        {
            normalImpulse = 0f;
            if (contact.isTriggerPair) return;

            PhysicsBody a = contact.BodyA;
            PhysicsBody b = contact.BodyB;
            if (a.BodyType == BodyType.Static && b.BodyType == BodyType.Static) return;

            float invMassA = a.InverseMass;
            float invMassB = b.InverseMass;
            float invMassSum = invMassA + invMassB;
            if (invMassSum <= 0f) return;

            if (applyWarmStart && contact.warmStartNormalImpulse > 0f)
            {
                Float3 warmImpulse = contact.normal * contact.warmStartNormalImpulse;
                if (a.BodyType == BodyType.Dynamic)
                    a.Velocity = a.Velocity - warmImpulse * invMassA;
                if (b.BodyType == BodyType.Dynamic)
                    b.Velocity = b.Velocity + warmImpulse * invMassB;
            }

            const float baumgarte = 0.2f;
            const float slop = 0.005f;
            float penetration = Math.Max(contact.penetration - slop, 0f);
            Float3 correction = contact.normal * (baumgarte * penetration / invMassSum);
            if (a.BodyType == BodyType.Dynamic)
                a.Position = a.Position - correction * invMassA;
            if (b.BodyType == BodyType.Dynamic)
                b.Position = b.Position + correction * invMassB;

            Float3 relativeVelocity = b.Velocity - a.Velocity;
            float normalVelocity = Float3Math.Dot(relativeVelocity, contact.normal);
            if (normalVelocity > 0f) return;

            float restitution = a.Material.CombineBounciness(b.Material);
            float impulseScalar = -(1f + restitution) * normalVelocity / invMassSum;
            Float3 impulse = contact.normal * impulseScalar;
            normalImpulse = impulseScalar;

            if (a.BodyType == BodyType.Dynamic)
            {
                a.Velocity = a.Velocity - impulse * invMassA;
                a.WakeUp();
            }

            if (b.BodyType == BodyType.Dynamic)
            {
                b.Velocity = b.Velocity + impulse * invMassB;
                b.WakeUp();
            }

            Float3 tangent = relativeVelocity - contact.normal * normalVelocity;
            if (PhysicsMath.TryNormalize(tangent, out Float3 tangentDir))
            {
                float friction = a.Material.CombineFriction(b.Material);
                Float3 frictionImpulse = tangentDir * (-Float3Math.Dot(relativeVelocity, tangentDir) / invMassSum * friction);
                if (a.BodyType == BodyType.Dynamic)
                    a.Velocity = a.Velocity - frictionImpulse * invMassA;
                if (b.BodyType == BodyType.Dynamic)
                    b.Velocity = b.Velocity + frictionImpulse * invMassB;
            }
        }

        public static void IntegrateVelocity(PhysicsBody body, in PhysicsSettings settings, float deltaTime)
        {
            if (body.BodyType != BodyType.Dynamic || !body.IsEnabled) return;

            if (body.UseGravity)
                body.Velocity = body.Velocity + settings.gravity * deltaTime;

            if (body.LinearDrag > 0f)
            {
                float drag = Math.Max(0f, 1f - body.LinearDrag * deltaTime);
                body.Velocity = body.Velocity * drag;
            }

            if (body.AngularDrag > 0f)
            {
                float drag = Math.Max(0f, 1f - body.AngularDrag * deltaTime);
                body.AngularVelocity = body.AngularVelocity * drag;
            }
        }

        public static void UpdateSleeping(PhysicsBody body, in PhysicsSettings settings)
        {
            if (body.BodyType != BodyType.Dynamic || !body.IsEnabled) return;

            float speedSq = Float3Math.Dot(body.Velocity, body.Velocity);
            body.IsSleeping = speedSq <= settings.linearSleepThreshold * settings.linearSleepThreshold;
        }

        public static void Integrate(PhysicsBody body, in PhysicsSettings settings, float deltaTime)
        {
            if (body.BodyType != BodyType.Dynamic || !body.IsEnabled) return;

            IntegrateVelocity(body, settings, deltaTime);
            body.Position = body.Position + body.Velocity * deltaTime;
            UpdateSleeping(body, settings);
        }
    }
}
