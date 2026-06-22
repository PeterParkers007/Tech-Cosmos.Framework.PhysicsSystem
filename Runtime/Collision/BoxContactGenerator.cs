using System;
using TechCosmos.Core.Runtime;

namespace TechCosmos.PhysicsSystem.Runtime
{
    /// <summary>
    /// OBB-OBB SAT + Sutherland-Hodgman 接触面裁剪，生成稳定接触点。
    /// </summary>
    internal static class BoxContactGenerator
    {
        private const int MaxClipVerts = 8;

        private static readonly Float3[] ClipBufferA = new Float3[MaxClipVerts];
        private static readonly Float3[] ClipBufferB = new Float3[MaxClipVerts];

        private struct ObBox
        {
            public Float3 center;
            public Float3 axis0;
            public Float3 axis1;
            public Float3 axis2;
            public Float3 halfExtents;
        }

        private enum SatAxisSource
        {
            None = 0,
            FaceA0,
            FaceA1,
            FaceA2,
            FaceB0,
            FaceB1,
            FaceB2,
            EdgeCross
        }

        public static bool Generate(PhysicsBody a, PhysicsBody b, out ContactManifold contact)
        {
            contact = default;
            ObBox boxA = BuildObBox(a);
            ObBox boxB = BuildObBox(b);
            Float3 t = boxB.center - boxA.center;

            float minOverlap = float.MaxValue;
            Float3 bestAxis = Float3.Zero;
            SatAxisSource bestSource = SatAxisSource.None;

            if (!TestAxis(boxA.axis0, t, boxA, boxB, SatAxisSource.FaceA0, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(boxA.axis1, t, boxA, boxB, SatAxisSource.FaceA1, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(boxA.axis2, t, boxA, boxB, SatAxisSource.FaceA2, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(boxB.axis0, t, boxA, boxB, SatAxisSource.FaceB0, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(boxB.axis1, t, boxA, boxB, SatAxisSource.FaceB1, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(boxB.axis2, t, boxA, boxB, SatAxisSource.FaceB2, ref minOverlap, ref bestAxis, ref bestSource)) return false;

            if (!TestAxis(PhysicsMath.Cross(boxA.axis0, boxB.axis0), t, boxA, boxB, SatAxisSource.EdgeCross, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(PhysicsMath.Cross(boxA.axis0, boxB.axis1), t, boxA, boxB, SatAxisSource.EdgeCross, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(PhysicsMath.Cross(boxA.axis0, boxB.axis2), t, boxA, boxB, SatAxisSource.EdgeCross, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(PhysicsMath.Cross(boxA.axis1, boxB.axis0), t, boxA, boxB, SatAxisSource.EdgeCross, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(PhysicsMath.Cross(boxA.axis1, boxB.axis1), t, boxA, boxB, SatAxisSource.EdgeCross, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(PhysicsMath.Cross(boxA.axis1, boxB.axis2), t, boxA, boxB, SatAxisSource.EdgeCross, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(PhysicsMath.Cross(boxA.axis2, boxB.axis0), t, boxA, boxB, SatAxisSource.EdgeCross, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(PhysicsMath.Cross(boxA.axis2, boxB.axis1), t, boxA, boxB, SatAxisSource.EdgeCross, ref minOverlap, ref bestAxis, ref bestSource)) return false;
            if (!TestAxis(PhysicsMath.Cross(boxA.axis2, boxB.axis2), t, boxA, boxB, SatAxisSource.EdgeCross, ref minOverlap, ref bestAxis, ref bestSource)) return false;

            if (minOverlap <= 0f || !PhysicsMath.TryNormalize(bestAxis, out Float3 normal))
                return false;

            contact.normal = normal;
            contact.penetration = minOverlap;

            if (bestSource == SatAxisSource.EdgeCross || !TryClipContactPoint(boxA, boxB, bestSource, normal, out Float3 clipPoint))
                contact.point = ComputeFallbackContactPoint(boxA, boxB, normal);
            else
                contact.point = clipPoint;

            return true;
        }

        private static ObBox BuildObBox(PhysicsBody body)
        {
            FloatQuat rot = body.Shape.GetWorldRotation(body.Rotation);
            return new ObBox
            {
                center = body.Shape.GetWorldCenter(body.Position, body.Rotation),
                axis0 = rot.Rotate(new Float3(1f, 0f, 0f)),
                axis1 = rot.Rotate(new Float3(0f, 1f, 0f)),
                axis2 = rot.Rotate(new Float3(0f, 0f, 1f)),
                halfExtents = body.Shape.size
            };
        }

        private static bool TestAxis(
            Float3 axis,
            Float3 t,
            in ObBox a,
            in ObBox b,
            SatAxisSource source,
            ref float minOverlap,
            ref Float3 bestAxis,
            ref SatAxisSource bestSource)
        {
            if (!PhysicsMath.TryNormalize(axis, out Float3 normalized))
                return true;

            float ra = ProjectExtent(a, normalized);
            float rb = ProjectExtent(b, normalized);
            float distance = Math.Abs(Float3Math.Dot(t, normalized));
            float overlap = ra + rb - distance;
            if (overlap < 0f) return false;

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                bestAxis = Float3Math.Dot(t, normalized) >= 0f ? normalized : normalized * -1f;
                bestSource = source;
            }

            return true;
        }

        private static float ProjectExtent(in ObBox box, Float3 direction)
        {
            return box.halfExtents.x * Math.Abs(Float3Math.Dot(box.axis0, direction))
                 + box.halfExtents.y * Math.Abs(Float3Math.Dot(box.axis1, direction))
                 + box.halfExtents.z * Math.Abs(Float3Math.Dot(box.axis2, direction));
        }

        private static Float3 ComputeFallbackContactPoint(in ObBox a, in ObBox b, Float3 normal)
        {
            float supportA = ProjectExtent(a, normal);
            float supportB = ProjectExtent(b, normal * -1f);
            Float3 pointA = a.center + normal * supportA;
            Float3 pointB = b.center - normal * supportB;
            return (pointA + pointB) * 0.5f;
        }

        private static bool TryClipContactPoint(in ObBox a, in ObBox b, SatAxisSource source, Float3 normal, out Float3 contactPoint)
        {
            contactPoint = Float3.Zero;
            bool referenceIsA = source switch
            {
                SatAxisSource.FaceA0 or SatAxisSource.FaceA1 or SatAxisSource.FaceA2 => true,
                SatAxisSource.FaceB0 or SatAxisSource.FaceB1 or SatAxisSource.FaceB2 => false,
                _ => Float3Math.Dot(b.center - a.center, normal) >= 0f
            };

            ObBox reference = referenceIsA ? a : b;
            ObBox incident = referenceIsA ? b : a;
            Float3 referenceNormal = referenceIsA ? normal : normal * -1f;

            int referenceFace = GetReferenceFaceIndex(reference, referenceNormal);
            int incidentFace = GetIncidentFaceIndex(incident, referenceNormal * -1f);

            GetFaceVertices(reference, referenceFace, ClipBufferA, out int refCount);
            GetFaceVertices(incident, incidentFace, ClipBufferB, out int incCount);

            if (incCount < 3 || refCount < 3)
                return false;

            int clipCount = incCount;
            Float3[] source = ClipBufferB;
            Float3[] target = ClipBufferA;

            Float3 faceNormal = GetFaceNormal(reference, referenceFace);
            Float3 tangent0 = GetFaceTangent0(reference, referenceFace);
            Float3 tangent1 = PhysicsMath.Cross(faceNormal, tangent0);

            Float3 faceCenter = GetFaceCenter(reference, referenceFace);
            float extent0 = GetFaceExtent0(reference, referenceFace);
            float extent1 = GetFaceExtent1(reference, referenceFace);

            clipCount = ClipAgainstPlane(source, clipCount, target, faceCenter + tangent0 * extent0, tangent0 * -1f);
            if (clipCount < 1) return false;
            (source, target) = (target, source);

            clipCount = ClipAgainstPlane(source, clipCount, target, faceCenter - tangent0 * extent0, tangent0);
            if (clipCount < 1) return false;
            (source, target) = (target, source);

            clipCount = ClipAgainstPlane(source, clipCount, target, faceCenter + tangent1 * extent1, tangent1 * -1f);
            if (clipCount < 1) return false;
            (source, target) = (target, source);

            clipCount = ClipAgainstPlane(source, clipCount, target, faceCenter - tangent1 * extent1, tangent1);
            if (clipCount < 1) return false;

            contactPoint = ComputePolygonCentroid(target, clipCount);
            return true;
        }

        private static int GetReferenceFaceIndex(in ObBox box, Float3 outwardNormal)
        {
            int bestFace = 0;
            float bestDot = float.MinValue;
            for (int i = 0; i < 6; i++)
            {
                Float3 faceNormal = GetFaceNormal(box, i);
                float dot = Float3Math.Dot(faceNormal, outwardNormal);
                if (dot > bestDot)
                {
                    bestDot = dot;
                    bestFace = i;
                }
            }

            return bestFace;
        }

        private static int GetIncidentFaceIndex(in ObBox box, Float3 targetNormal)
        {
            int bestFace = 0;
            float bestDot = float.MaxValue;
            for (int i = 0; i < 6; i++)
            {
                Float3 faceNormal = GetFaceNormal(box, i);
                float dot = Float3Math.Dot(faceNormal, targetNormal);
                if (dot < bestDot)
                {
                    bestDot = dot;
                    bestFace = i;
                }
            }

            return bestFace;
        }

        private static Float3 GetFaceNormal(in ObBox box, int faceIndex) =>
            faceIndex switch
            {
                0 => box.axis0,
                1 => box.axis0 * -1f,
                2 => box.axis1,
                3 => box.axis1 * -1f,
                4 => box.axis2,
                _ => box.axis2 * -1f
            };

        private static Float3 GetFaceTangent0(in ObBox box, int faceIndex) =>
            faceIndex switch
            {
                0 or 1 => box.axis1,
                2 or 3 => box.axis0,
                _ => box.axis0
            };

        private static float GetFaceExtent0(in ObBox box, int faceIndex) =>
            faceIndex switch
            {
                0 or 1 => box.halfExtents.y,
                2 or 3 => box.halfExtents.x,
                _ => box.halfExtents.x
            };

        private static float GetFaceExtent1(in ObBox box, int faceIndex) =>
            faceIndex switch
            {
                0 or 1 => box.halfExtents.z,
                2 or 3 => box.halfExtents.z,
                _ => box.halfExtents.y
            };

        private static Float3 GetFaceCenter(in ObBox box, int faceIndex)
        {
            Float3 normal = GetFaceNormal(box, faceIndex);
            float extent = faceIndex switch
            {
                0 => box.halfExtents.x,
                1 => box.halfExtents.x,
                2 => box.halfExtents.y,
                3 => box.halfExtents.y,
                4 => box.halfExtents.z,
                _ => box.halfExtents.z
            };
            return box.center + normal * extent;
        }

        private static void GetFaceVertices(in ObBox box, int faceIndex, Float3[] vertices, out int count)
        {
            Float3 center = GetFaceCenter(box, faceIndex);
            Float3 tangent0 = GetFaceTangent0(box, faceIndex);
            Float3 normal = GetFaceNormal(box, faceIndex);
            Float3 tangent1 = PhysicsMath.Cross(normal, tangent0);
            float e0 = GetFaceExtent0(box, faceIndex);
            Float3 e0v = tangent0 * e0;
            float e1 = GetFaceExtent1(box, faceIndex);
            Float3 e1v = tangent1 * e1;

            vertices[0] = center - e0v - e1v;
            vertices[1] = center + e0v - e1v;
            vertices[2] = center + e0v + e1v;
            vertices[3] = center - e0v + e1v;
            count = 4;
        }

        private static int ClipAgainstPlane(Float3[] input, int inputCount, Float3[] output, Float3 planePoint, Float3 planeNormal)
        {
            if (inputCount == 0) return 0;

            int outputCount = 0;
            Float3 previous = input[inputCount - 1];
            float previousDistance = Float3Math.Dot(previous - planePoint, planeNormal);
            bool previousInside = previousDistance >= 0f;

            for (int i = 0; i < inputCount; i++)
            {
                Float3 current = input[i];
                float currentDistance = Float3Math.Dot(current - planePoint, planeNormal);
                bool currentInside = currentDistance >= 0f;

                if (currentInside)
                {
                    if (!previousInside)
                        output[outputCount++] = IntersectSegmentWithPlane(previous, current, planePoint, planeNormal);
                    output[outputCount++] = current;
                }
                else if (previousInside)
                {
                    output[outputCount++] = IntersectSegmentWithPlane(previous, current, planePoint, planeNormal);
                }

                previous = current;
                previousDistance = currentDistance;
                previousInside = currentInside;
            }

            return outputCount;
        }

        private static Float3 IntersectSegmentWithPlane(Float3 a, Float3 b, Float3 planePoint, Float3 planeNormal)
        {
            Float3 ab = b - a;
            float denom = Float3Math.Dot(ab, planeNormal);
            if (Math.Abs(denom) < 1e-8f)
                return a;

            float t = Float3Math.Dot(planePoint - a, planeNormal) / denom;
            t = PhysicsMath.Clamp(t, 0f, 1f);
            return a + ab * t;
        }

        private static Float3 ComputePolygonCentroid(Float3[] vertices, int count)
        {
            if (count <= 0) return Float3.Zero;
            if (count == 1) return vertices[0];

            Float3 sum = Float3.Zero;
            for (int i = 0; i < count; i++)
                sum = sum + vertices[i];

            return sum * (1f / count);
        }
    }
}
