using System;
using System.Collections.Generic;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    internal static class JointSolver
    {
        private const float PositionCorrection = 0.6f;
        private const float MaxLinearCorrection = 0.25f;

        public static void Solve(IReadOnlyList<PhysicsJoint> joints, float deltaTime)
        {
            if (deltaTime <= 0f) return;

            for (int i = 0; i < joints.Count; i++)
            {
                PhysicsJoint joint = joints[i];
                if (!joint.IsEnabled || joint.BodyA == null || joint.BodyB == null) continue;
                if (!joint.BodyA.IsEnabled || !joint.BodyB.IsEnabled) continue;

                switch (joint.Type)
                {
                    case JointType.Fixed:
                        SolveFixed(joint);
                        break;
                    case JointType.Hinge:
                        SolveHinge(joint);
                        break;
                    case JointType.Distance:
                        SolveDistance(joint);
                        break;
                    case JointType.Spring:
                        SolveSpring(joint, deltaTime);
                        break;
                }
            }
        }

        private static void SolveFixed(PhysicsJoint joint)
        {
            Float3 anchorA = joint.GetWorldAnchorA();
            Float3 anchorB = joint.GetWorldAnchorB();
            Float3 error = anchorB - anchorA;
            ApplyAnchorCorrection(joint.BodyA, joint.BodyB, error);
            ApplyAnchorVelocityConstraint(joint.BodyA, joint.BodyB, anchorA, anchorB);
        }

        private static void SolveHinge(PhysicsJoint joint)
        {
            // 当前实现：球窝约束（锚点重合），铰链轴用于后续旋转约束扩展。
            SolveFixed(joint);
        }

        private static void SolveDistance(PhysicsJoint joint)
        {
            Float3 anchorA = joint.GetWorldAnchorA();
            Float3 anchorB = joint.GetWorldAnchorB();
            Float3 delta = anchorB - anchorA;
            float distance = (float)Math.Sqrt(Float3Math.Dot(delta, delta));
            if (distance <= 1e-6f) return;

            Float3 direction = delta * (1f / distance);

            if (joint.MaxDistance > 0f && distance > joint.MaxDistance)
                ApplyDirectionCorrection(joint.BodyA, joint.BodyB, direction, distance - joint.MaxDistance);

            if (joint.MinDistance > 0f && distance < joint.MinDistance)
                ApplyDirectionCorrection(joint.BodyA, joint.BodyB, direction * -1f, joint.MinDistance - distance);

            ApplyDirectionVelocityConstraint(joint.BodyA, joint.BodyB, direction, anchorA, anchorB);
        }

        private static void SolveSpring(PhysicsJoint joint, float deltaTime)
        {
            Float3 anchorA = joint.GetWorldAnchorA();
            Float3 anchorB = joint.GetWorldAnchorB();
            Float3 delta = anchorB - anchorA;
            float distance = (float)Math.Sqrt(Float3Math.Dot(delta, delta));
            if (distance <= 1e-6f) return;

            Float3 direction = delta * (1f / distance);
            float rest = joint.RestLength > 0f ? joint.RestLength : distance;
            Float3 relativeVelocity = joint.BodyB.Velocity - joint.BodyA.Velocity;
            float relSpeed = Float3Math.Dot(relativeVelocity, direction);

            float force = joint.SpringStiffness * (distance - rest) + joint.SpringDamping * relSpeed;
            Float3 impulse = direction * (-force * deltaTime);

            if (joint.BodyA.BodyType == BodyType.Dynamic && joint.BodyA.InverseMass > 0f)
            {
                joint.BodyA.Velocity = joint.BodyA.Velocity - impulse * joint.BodyA.InverseMass;
                joint.BodyA.WakeUp();
            }

            if (joint.BodyB.BodyType == BodyType.Dynamic && joint.BodyB.InverseMass > 0f)
            {
                joint.BodyB.Velocity = joint.BodyB.Velocity + impulse * joint.BodyB.InverseMass;
                joint.BodyB.WakeUp();
            }
        }

        private static void ApplyAnchorCorrection(PhysicsBody a, PhysicsBody b, Float3 error)
        {
            float invMassA = a.InverseMass;
            float invMassB = b.InverseMass;
            float invMassSum = invMassA + invMassB;
            if (invMassSum <= 0f) return;

            Float3 correction = error * (PositionCorrection / invMassSum);
            float maxLen = Float3Math.Dot(correction, correction);
            if (maxLen > MaxLinearCorrection * MaxLinearCorrection && maxLen > 1e-8f)
            {
                float scale = MaxLinearCorrection / (float)Math.Sqrt(maxLen);
                correction = correction * scale;
            }

            if (a.BodyType == BodyType.Dynamic)
                a.Position = a.Position + correction * invMassA;
            if (b.BodyType == BodyType.Dynamic)
                b.Position = b.Position - correction * invMassB;
        }

        private static void ApplyDirectionCorrection(PhysicsBody a, PhysicsBody b, Float3 direction, float error)
        {
            float invMassA = a.InverseMass;
            float invMassB = b.InverseMass;
            float invMassSum = invMassA + invMassB;
            if (invMassSum <= 0f) return;

            Float3 correction = direction * (error * PositionCorrection / invMassSum);
            if (a.BodyType == BodyType.Dynamic)
                a.Position = a.Position + correction * invMassA;
            if (b.BodyType == BodyType.Dynamic)
                b.Position = b.Position - correction * invMassB;
        }

        private static void ApplyAnchorVelocityConstraint(PhysicsBody a, PhysicsBody b, Float3 anchorA, Float3 anchorB)
        {
            Float3 relativeVelocity = b.Velocity - a.Velocity;
            Float3 delta = anchorB - anchorA;
            if (!PhysicsMath.TryNormalize(delta, out Float3 direction))
                return;

            ApplyDirectionVelocityConstraint(a, b, direction, anchorA, anchorB);
        }

        private static void ApplyDirectionVelocityConstraint(PhysicsBody a, PhysicsBody b, Float3 direction, Float3 anchorA, Float3 anchorB)
        {
            Float3 relativeVelocity = b.Velocity - a.Velocity;
            float normalSpeed = Float3Math.Dot(relativeVelocity, direction);
            if (Math.Abs(normalSpeed) <= 1e-6f) return;

            float invMassA = a.InverseMass;
            float invMassB = b.InverseMass;
            float invMassSum = invMassA + invMassB;
            if (invMassSum <= 0f) return;

            Float3 impulse = direction * (-normalSpeed / invMassSum);
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
        }
    }
}
