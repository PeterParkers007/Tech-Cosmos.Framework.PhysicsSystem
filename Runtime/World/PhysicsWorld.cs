using System;
using System.Collections.Generic;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    internal struct CachedContact
    {
        public ContactManifold manifold;
        public float normalImpulse;
    }

    /// <summary>
    /// 纯物理世界：模拟、查询、碰撞回调的统一入口。
    /// API 语义对齐 Unity Physics，但不依赖 UnityEngine。
    /// </summary>
    public sealed class PhysicsWorld
    {
        private PhysicsSettings _settings;
        private readonly BroadPhaseSpatialHash _broadPhase;
        private readonly List<PhysicsBody> _bodies = new List<PhysicsBody>(128);
        private readonly List<PhysicsBody> _activeBodies = new List<PhysicsBody>(128);
        private readonly Dictionary<PhysicsBodyHandle, PhysicsBody> _lookup = new Dictionary<PhysicsBodyHandle, PhysicsBody>();
        private readonly HashSet<(int, int)> _activePairs = new HashSet<(int, int)>();
        private readonly HashSet<(int, int)> _newPairsScratch = new HashSet<(int, int)>();
        private readonly Dictionary<(int, int), CachedContact> _contactCache = new Dictionary<(int, int), CachedContact>();
        private readonly List<ContactManifold> _contacts = new List<ContactManifold>(64);
        private readonly List<(int, int)> _staleCacheKeys = new List<(int, int)>(32);
        private readonly List<RaycastHit> _raycastScratch = new List<RaycastHit>(32);
        private readonly List<PhysicsJoint> _joints = new List<PhysicsJoint>(32);
        private readonly Dictionary<PhysicsJointHandle, PhysicsJoint> _jointLookup = new Dictionary<PhysicsJointHandle, PhysicsJoint>();
        private readonly HashSet<(int, int)> _nonCollidingJointPairs = new HashSet<(int, int)>();
        private int _nextId;
        private int _nextGeneration = 1;
        private int _nextJointId;
        private int _nextJointGeneration = 1;

        public PhysicsSettings Settings => _settings;

        public Float3 Gravity
        {
            get => _settings.gravity;
            set => _settings.gravity = value;
        }

        public event Action<CollisionEnterEvent> CollisionEnter;
        public event Action<CollisionStayEvent> CollisionStay;
        public event Action<CollisionExitEvent> CollisionExit;

        public IReadOnlyList<PhysicsJoint> Joints => _joints;

        public PhysicsWorld() : this(PhysicsSettings.Default) { }

        public PhysicsWorld(PhysicsSettings settings)
        {
            _settings = settings;
            _broadPhase = new BroadPhaseSpatialHash(settings.broadPhaseCellSize);
        }

        public static PhysicsWorld Create(in PhysicsSettings settings) => new PhysicsWorld(settings);

        public PhysicsBody CreateBody(in PhysicsBodyDescriptor descriptor)
        {
            var body = new PhysicsBody
            {
                Handle = new PhysicsBodyHandle(_nextId++, _nextGeneration),
                BodyType = descriptor.bodyType,
                Position = descriptor.position,
                Rotation = descriptor.rotation.IsIdentity ? FloatQuat.Identity : descriptor.rotation.Normalized(),
                Velocity = descriptor.velocity,
                AngularVelocity = descriptor.angularVelocity,
                Mass = descriptor.bodyType == BodyType.Dynamic ? Math.Max(descriptor.mass, 1e-4f) : 0f,
                IsTrigger = descriptor.isTrigger,
                UseGravity = descriptor.useGravity,
                LinearDrag = descriptor.linearDrag,
                AngularDrag = descriptor.angularDrag,
                Filter = descriptor.filter,
                Material = descriptor.material,
                Shape = descriptor.shape,
                UserData = descriptor.userData,
                SlotIndex = _bodies.Count
            };

            _bodies.Add(body);
            _lookup[body.Handle] = body;
            return body;
        }

        public bool TryGetBody(PhysicsBodyHandle handle, out PhysicsBody body) => _lookup.TryGetValue(handle, out body);

        public void DestroyBody(PhysicsBodyHandle handle)
        {
            if (!TryGetBody(handle, out var body)) return;

            _lookup.Remove(handle);
            body.IsEnabled = false;
            body.Handle = new PhysicsBodyHandle(body.Handle.Id, _nextGeneration++);
            RemoveCachedContactsForBody(body.Handle.Id);
            RemoveJointsForBody(body.Handle.Id);
        }

        public PhysicsJoint CreateJoint(in PhysicsJointDescriptor descriptor)
        {
            if (!TryGetBody(descriptor.bodyA, out PhysicsBody bodyA) || !TryGetBody(descriptor.bodyB, out PhysicsBody bodyB))
                throw new InvalidOperationException("CreateJoint requires valid body handles.");

            var joint = new PhysicsJoint
            {
                Handle = new PhysicsJointHandle(_nextJointId++, _nextJointGeneration),
                Type = descriptor.type,
                BodyA = bodyA,
                BodyB = bodyB,
                LocalAnchorA = descriptor.localAnchorA,
                LocalAnchorB = descriptor.localAnchorB,
                LocalAxisA = descriptor.localAxisA,
                MinDistance = descriptor.minDistance,
                MaxDistance = descriptor.maxDistance,
                RestLength = descriptor.restLength,
                SpringStiffness = descriptor.springStiffness,
                SpringDamping = descriptor.springDamping,
                CollideConnected = descriptor.collideConnected
            };

            _joints.Add(joint);
            _jointLookup[joint.Handle] = joint;
            RefreshJointCollisionFilter(joint);
            return joint;
        }

        public bool TryGetJoint(PhysicsJointHandle handle, out PhysicsJoint joint) =>
            _jointLookup.TryGetValue(handle, out joint);

        public void DestroyJoint(PhysicsJointHandle handle)
        {
            if (!TryGetJoint(handle, out var joint)) return;

            _jointLookup.Remove(handle);
            _joints.Remove(joint);
            RemoveJointCollisionFilter(joint);
            joint.IsEnabled = false;
            joint.Handle = new PhysicsJointHandle(joint.Handle.Id, _nextJointGeneration++);
        }

        public SimulationStats LastStats { get; private set; }

        /// <summary>
        /// 固定步长模拟（推荐用于游戏循环）。内部自动 substep，防止大 deltaTime 导致不稳定。
        /// 完全可控：通过 Settings.fixedTimeStep / maxSubsteps 调节性能与精度。
        /// </summary>
        public void Simulate(float deltaTime)
        {
            if (deltaTime <= 0f) return;

            float fixedDt = _settings.fixedTimeStep > 0f ? _settings.fixedTimeStep : 1f / 60f;
            int substeps = 0;
            float remaining = deltaTime;

            var stats = new SimulationStats { deltaTime = deltaTime };

            while (remaining > 0f && substeps < _settings.maxSubsteps)
            {
                float stepDt = Math.Min(remaining, fixedDt);
                SimulateSingleStep(stepDt, ref stats);
                remaining -= stepDt;
                substeps++;
            }

            if (remaining > 0f)
            {
                // Runtime 层无 UnityEngine 引用，由上层 CosmosPhysicsWorld 记录
            }

            stats.substeps = substeps;
            LastStats = stats;
        }

        private void SimulateSingleStep(float deltaTime, ref SimulationStats stats)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();

            // 1. Integrate & collect active
            _activeBodies.Clear();
            for (int i = 0; i < _bodies.Count; i++)
            {
                PhysicsBody body = _bodies[i];
                if (!body.IsEnabled) continue;
                if (body.BodyType == BodyType.Dynamic && (!_settings.enableSleeping || !body.IsSleeping))
                {
                    Float3 startPosition = body.Position;
                    CollisionSolver.IntegrateVelocity(body, _settings, deltaTime);
                    if (CcdSolver.NeedsCcd(body, _settings))
                        CcdSolver.ApplyMotion(body, startPosition, deltaTime, _bodies);
                    else
                        body.Position = startPosition + body.Velocity * deltaTime;
                    CollisionSolver.UpdateSleeping(body, _settings);
                }
                _activeBodies.Add(body);
            }
            stats.integrationTimeMs += sw.Elapsed.TotalMilliseconds; sw.Restart();

            // 2. Broadphase
            _broadPhase.Clear();
            for (int i = 0; i < _activeBodies.Count; i++)
                _broadPhase.Insert(_activeBodies[i]);
            stats.broadphaseTimeMs += sw.Elapsed.TotalMilliseconds; sw.Restart();

            // 3. Narrowphase + events
            _contacts.Clear();
            _newPairsScratch.Clear();

            for (int i = 0; i < _activeBodies.Count; i++)
            {
                PhysicsBody a = _activeBodies[i];
                a.Shape.ComputeWorldBounds(a.Position, a.Rotation, out Float3 min, out Float3 max);
                List<PhysicsBody> candidates = _broadPhase.Query(min, max);

                for (int j = 0; j < candidates.Count; j++)
                {
                    PhysicsBody b = candidates[j];
                    if (ReferenceEquals(a, b) || a.Handle.Id > b.Handle.Id) continue;
                    if (ShouldSkipJointCollision(a.Handle.Id, b.Handle.Id)) continue;

                    int idA = a.Handle.Id;
                    int idB = b.Handle.Id;
                    var pairKey = (idA, idB);

                    if (!NarrowPhase.TestPair(a, b, out ContactManifold contact))
                    {
                        if (!_contactCache.TryGetValue(pairKey, out CachedContact persisted))
                            continue;

                        contact = persisted.manifold;
                        contact.BodyA = a;
                        contact.BodyB = b;
                    }

                    ApplyCachedContactData(ref contact, pairKey);
                    _contacts.Add(contact);
                    _newPairsScratch.Add(pairKey);

                    bool wasActive = _activePairs.Contains(pairKey);
                    if (!wasActive)
                        CollisionEnter?.Invoke(new CollisionEnterEvent(a.Handle, b.Handle, a.UserData, b.UserData, contact.isTriggerPair));
                    else
                        CollisionStay?.Invoke(new CollisionStayEvent(a.Handle, b.Handle, a.UserData, b.UserData, contact.isTriggerPair));
                }
            }

            foreach (var pair in _activePairs)
            {
                if (_newPairsScratch.Contains(pair)) continue;
                if (!TryFindBodiesByPair(pair, out var bodyA, out var bodyB)) continue;
                CollisionExit?.Invoke(new CollisionExitEvent(bodyA.Handle, bodyB.Handle, bodyA.UserData, bodyB.UserData, bodyA.IsTrigger || bodyB.IsTrigger));
                _contactCache.Remove(pair);
            }

            _activePairs.Clear();
            foreach (var pair in _newPairsScratch)
                _activePairs.Add(pair);

            RemoveStaleContactCacheEntries();

            stats.narrowphaseTimeMs += sw.Elapsed.TotalMilliseconds; sw.Restart();

            // 4. Solver
            int iters = _settings.velocityIterations;
            for (int i = 0; i < _contacts.Count; i++)
            {
                ContactManifold contact = _contacts[i];
                float accumulatedImpulse = 0f;

                for (int iteration = 0; iteration < iters; iteration++)
                {
                    CollisionSolver.Resolve(ref contact, _settings, iteration == 0, out float normalImpulse);
                    accumulatedImpulse += normalImpulse;
                }

                _contacts[i] = contact;
                UpdateCachedNormalImpulse(contact, accumulatedImpulse);
            }

            JointSolver.Solve(_joints, deltaTime);
            stats.solverTimeMs += sw.Elapsed.TotalMilliseconds;
            stats.contactCount = _contacts.Count;
        }

        public bool Raycast(Float3 origin, Float3 direction, out RaycastHit hit, float maxDistance = float.MaxValue, int layerMask = ~0)
        {
            hit = default;
            if (!PhysicsMath.TryNormalize(direction, out Float3 normalized)) return false;

            var ray = new PhysicsRay(origin, normalized);
            _raycastScratch.Clear();

            for (int i = 0; i < _bodies.Count; i++)
            {
                PhysicsBody body = _bodies[i];
                if (!body.IsEnabled) continue;
                if ((layerMask & (1 << body.Filter.layer)) == 0) continue;
                if (NarrowPhase.Raycast(ray, body, maxDistance, out RaycastHit candidate))
                    _raycastScratch.Add(candidate);
            }

            if (_raycastScratch.Count == 0) return false;

            RaycastHit closest = _raycastScratch[0];
            for (int i = 1; i < _raycastScratch.Count; i++)
            {
                if (_raycastScratch[i].distance < closest.distance)
                    closest = _raycastScratch[i];
            }

            hit = closest;
            return true;
        }

        public bool SphereCast(Float3 origin, float radius, Float3 direction, out SphereCastHit hit, float maxDistance = float.MaxValue, int layerMask = ~0)
        {
            hit = default;
            if (!PhysicsMath.TryNormalize(direction, out Float3 normalized)) return false;

            var ray = new PhysicsRay(origin, normalized);
            SphereCastHit closest = default;
            bool found = false;

            for (int i = 0; i < _bodies.Count; i++)
            {
                PhysicsBody body = _bodies[i];
                if (!body.IsEnabled) continue;
                if ((layerMask & (1 << body.Filter.layer)) == 0) continue;

                var probe = new PhysicsBody
                {
                    Position = origin,
                    Rotation = FloatQuat.Identity,
                    Shape = ColliderShape.Sphere(radius)
                };

                for (float t = 0f; t <= maxDistance; t += Math.Max(radius * 0.5f, 0.05f))
                {
                    probe.Position = ray.GetPoint(t);
                    if (!NarrowPhase.TestPair(probe, body, out ContactManifold contact)) continue;

                    closest = new SphereCastHit
                    {
                        body = body.Handle,
                        point = contact.point,
                        normal = contact.normal,
                        distance = t,
                        userData = body.UserData
                    };
                    found = true;
                    break;
                }
            }

            if (!found) return false;
            hit = closest;
            return true;
        }

        public int OverlapSphere(Float3 center, float radius, OverlapHit[] results, int layerMask = ~0)
        {
            if (results == null || results.Length == 0) return 0;

            int count = 0;
            for (int i = 0; i < _bodies.Count && count < results.Length; i++)
            {
                PhysicsBody body = _bodies[i];
                if (!body.IsEnabled) continue;
                if ((layerMask & (1 << body.Filter.layer)) == 0) continue;
                if (!NarrowPhase.OverlapSphere(center, radius, body)) continue;

                results[count++] = new OverlapHit
                {
                    body = body.Handle,
                    closestPoint = body.Shape.GetWorldCenter(body.Position, body.Rotation),
                    userData = body.UserData
                };
            }

            return count;
        }

        public int OverlapBox(Float3 center, FloatQuat rotation, Float3 halfExtents, OverlapHit[] results, int layerMask = ~0)
        {
            if (results == null || results.Length == 0) return 0;

            int count = 0;
            for (int i = 0; i < _bodies.Count && count < results.Length; i++)
            {
                PhysicsBody body = _bodies[i];
                if (!body.IsEnabled) continue;
                if ((layerMask & (1 << body.Filter.layer)) == 0) continue;
                if (!NarrowPhase.OverlapBox(center, rotation, halfExtents, body)) continue;

                results[count++] = new OverlapHit
                {
                    body = body.Handle,
                    closestPoint = body.Shape.GetWorldCenter(body.Position, body.Rotation),
                    userData = body.UserData
                };
            }

            return count;
        }

        public int OverlapSphereAll(Float3 center, float radius, List<OverlapHit> results, int layerMask = ~0)
        {
            if (results == null) throw new ArgumentNullException(nameof(results));
            results.Clear();

            var buffer = new OverlapHit[Math.Max(_bodies.Count, 8)];
            int count = OverlapSphere(center, radius, buffer, layerMask);
            for (int i = 0; i < count; i++)
                results.Add(buffer[i]);
            return count;
        }

        private bool TryFindBodiesByPair((int, int) pair, out PhysicsBody bodyA, out PhysicsBody bodyB)
        {
            bodyA = null;
            bodyB = null;
            for (int i = 0; i < _bodies.Count; i++)
            {
                PhysicsBody body = _bodies[i];
                if (body.Handle.Id == pair.Item1) bodyA = body;
                if (body.Handle.Id == pair.Item2) bodyB = body;
            }

            return bodyA != null && bodyB != null;
        }

        private void ApplyCachedContactData(ref ContactManifold contact, (int idA, int idB) pairKey)
        {
            if (!_contactCache.TryGetValue(pairKey, out CachedContact cached))
            {
                _contactCache[pairKey] = new CachedContact { manifold = contact, normalImpulse = 0f };
                return;
            }

            contact.warmStartNormalImpulse = cached.normalImpulse;

            if (Float3Math.Dot(cached.manifold.normal, contact.normal) < 0f)
                contact.normal = contact.normal * -1f;

            float alignment = Float3Math.Dot(cached.manifold.normal, contact.normal);
            if (alignment > 0.5f && PhysicsMath.TryNormalize(
                    cached.manifold.normal * 0.25f + contact.normal * 0.75f,
                    out Float3 blendedNormal))
            {
                contact.normal = blendedNormal;
            }

            _contactCache[pairKey] = new CachedContact
            {
                manifold = contact,
                normalImpulse = cached.normalImpulse
            };
        }

        private void UpdateCachedNormalImpulse(in ContactManifold contact, float normalImpulse)
        {
            var pairKey = (contact.BodyA.Handle.Id, contact.BodyB.Handle.Id);
            if (!_contactCache.TryGetValue(pairKey, out CachedContact cached))
                return;

            cached.manifold = contact;
            cached.normalImpulse = normalImpulse;
            _contactCache[pairKey] = cached;
        }

        private void RemoveStaleContactCacheEntries()
        {
            _staleCacheKeys.Clear();
            foreach (var pair in _contactCache.Keys)
            {
                if (!_newPairsScratch.Contains(pair))
                    _staleCacheKeys.Add(pair);
            }

            for (int i = 0; i < _staleCacheKeys.Count; i++)
                _contactCache.Remove(_staleCacheKeys[i]);
        }

        private void RemoveCachedContactsForBody(int bodyId)
        {
            _staleCacheKeys.Clear();
            foreach (var pair in _contactCache.Keys)
            {
                if (pair.Item1 == bodyId || pair.Item2 == bodyId)
                    _staleCacheKeys.Add(pair);
            }

            for (int i = 0; i < _staleCacheKeys.Count; i++)
                _contactCache.Remove(_staleCacheKeys[i]);
        }

        private bool ShouldSkipJointCollision(int idA, int idB)
        {
            if (idA > idB)
                (idA, idB) = (idB, idA);
            return _nonCollidingJointPairs.Contains((idA, idB));
        }

        private void RefreshJointCollisionFilter(PhysicsJoint joint)
        {
            int idA = joint.BodyA.Handle.Id;
            int idB = joint.BodyB.Handle.Id;
            if (idA > idB)
                (idA, idB) = (idB, idA);

            var pairKey = (idA, idB);
            if (!joint.CollideConnected)
                _nonCollidingJointPairs.Add(pairKey);
            else
                _nonCollidingJointPairs.Remove(pairKey);
        }

        private void RemoveJointCollisionFilter(PhysicsJoint joint)
        {
            int idA = joint.BodyA.Handle.Id;
            int idB = joint.BodyB.Handle.Id;
            if (idA > idB)
                (idA, idB) = (idB, idA);
            _nonCollidingJointPairs.Remove((idA, idB));
        }

        private void RemoveJointsForBody(int bodyId)
        {
            for (int i = _joints.Count - 1; i >= 0; i--)
            {
                PhysicsJoint joint = _joints[i];
                if (joint.BodyA.Handle.Id != bodyId && joint.BodyB.Handle.Id != bodyId) continue;
                DestroyJoint(joint.Handle);
            }
        }
    }
}
