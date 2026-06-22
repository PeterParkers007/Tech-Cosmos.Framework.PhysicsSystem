using System;

namespace TechCosmos.PhysicsSystem.Runtime
{
    public readonly struct CollisionEnterEvent
    {
        public readonly PhysicsBodyHandle BodyA;
        public readonly PhysicsBodyHandle BodyB;
        public readonly object UserDataA;
        public readonly object UserDataB;
        public readonly bool IsTrigger;

        public CollisionEnterEvent(
            PhysicsBodyHandle bodyA,
            PhysicsBodyHandle bodyB,
            object userDataA,
            object userDataB,
            bool isTrigger)
        {
            BodyA = bodyA;
            BodyB = bodyB;
            UserDataA = userDataA;
            UserDataB = userDataB;
            IsTrigger = isTrigger;
        }
    }

    public readonly struct CollisionStayEvent
    {
        public readonly PhysicsBodyHandle BodyA;
        public readonly PhysicsBodyHandle BodyB;
        public readonly object UserDataA;
        public readonly object UserDataB;
        public readonly bool IsTrigger;

        public CollisionStayEvent(
            PhysicsBodyHandle bodyA,
            PhysicsBodyHandle bodyB,
            object userDataA,
            object userDataB,
            bool isTrigger)
        {
            BodyA = bodyA;
            BodyB = bodyB;
            UserDataA = userDataA;
            UserDataB = userDataB;
            IsTrigger = isTrigger;
        }
    }

    public readonly struct CollisionExitEvent
    {
        public readonly PhysicsBodyHandle BodyA;
        public readonly PhysicsBodyHandle BodyB;
        public readonly object UserDataA;
        public readonly object UserDataB;
        public readonly bool IsTrigger;

        public CollisionExitEvent(
            PhysicsBodyHandle bodyA,
            PhysicsBodyHandle bodyB,
            object userDataA,
            object userDataB,
            bool isTrigger)
        {
            BodyA = bodyA;
            BodyB = bodyB;
            UserDataA = userDataA;
            UserDataB = userDataB;
            IsTrigger = isTrigger;
        }
    }
}
