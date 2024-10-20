#pragma once

#include <stdio.h>
#include <vector>
#include <string>
#include <execution>

#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "glm/gtx/vector_angle.hpp"

#include "Plane.h"
#include "Line.h"

#define MAX_ITERATIONS 64

namespace PhysicsCPU 
{
	class Physics
	{
	private:

        struct Common
        {
            glm::vec3 m_v3Gravity;
            int16_t m_nFps;

            float m_fDeltaTime;
            float m_fFixedDeltaTime;

            float m_fCurrentTime;
            float m_fMaxTime;

            int8_t m_nNumSubIntegrates;
        };

    public:
        struct Material
        {
            float m_fRestitution;
            float m_fFriction;
        };

        struct Triangle
        {
            int32_t m_nAId;
            int32_t m_nBId;
            int32_t m_nCId;
            glm::vec3 m_v3Normal;
        };

        struct RigidBody
        {
            float m_fMass;
            glm::mat3 m_mat3InvInertia;

            glm::vec3 m_v3Force;
            glm::vec3 m_v3LinearAcceleration;
            glm::vec3 m_v3LinearVelocity;
            glm::vec3 m_v3Position;

            glm::vec3 m_v3Torque;
            glm::vec3 m_v3AngularAcceleration;
            glm::vec3 m_v3AngularVelocity;
            glm::quat m_quatOrientation;

            float m_fLinearDamping;
            float m_fAngularDamping;

            glm::mat4x4 m_matWorld;

            int32_t m_nConvexTriMeshId;
            int16_t m_nMaterialId;
            int32_t m_nHitId;

            glm::vec3 GetPointVelocity(glm::vec3 v3Point)
            {
                return (m_v3LinearVelocity + glm::cross(m_v3AngularVelocity, v3Point));
            }

            glm::mat3 GetInvInertia() 
            {
                glm::mat3 rot = glm::mat3_cast(m_quatOrientation);  // quaternion -> rotation matrix
                glm::mat3 inertiaWorld = rot * (m_mat3InvInertia * glm::transpose(rot));
                return inertiaWorld;
            }
            
        };

        struct Hit 
        {
            //RigidBody *m_pRigidBody1 = nullptr;
            RigidBody *m_pRigidBody2 = nullptr;

            glm::vec3 m_v3PointInWorld = glm::vec3(0, 0, 0);
            glm::vec3 m_v3NormalInWorld = glm::vec3(0, 0, 0);
            float m_fPenetration = 0.0f;

            struct Hit Invert()
            {
                struct Hit ret;

                //ret.m_pRigidBody1 = m_pRigidBody2;
                //ret.m_pRigidBody2 = m_pRigidBody1;
                ret.m_pRigidBody2 = m_pRigidBody2;

                ret.m_fPenetration = m_fPenetration;
                ret.m_v3PointInWorld = m_v3PointInWorld;
                ret.m_v3NormalInWorld = -m_v3NormalInWorld;

                return ret;
            }
        };

        struct Hits
        {
            std::vector<struct Hit> m_listHits;

            void Clear() 
            {
                m_listHits.clear();
            }
        };

        struct ConvexTriMesh
        {
            std::vector<glm::vec3> m_listVertices;
            std::vector<Triangle> m_listTriangles;

            std::vector<glm::vec3> m_listUniqueVertices;

            glm::vec3 m_v3LocalMin = glm::vec3(0, 0, 0);
            glm::vec3 m_v3LocalMax = glm::vec3(0, 0, 0);
            
            bool IsContainsPoint(std::vector<glm::vec3>* pList, glm::vec3 v3Point) 
            {
                for (int i = 0; i < (int)(*pList).size(); i++) 
                {
                    glm::vec3 v3Point2 = (*pList)[i];

                    if (v3Point == v3Point2) 
                    {
                        return true;
                    }
                }

                return false;
            }

            void Update() 
            {
                m_v3LocalMin = glm::vec3(+FLT_MAX, +FLT_MAX, +FLT_MAX);
                m_v3LocalMax = glm::vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

                for (int i = 0; i < m_listTriangles.size(); i++)
                {
                    Triangle *pTri = &(m_listTriangles[i]);

                    glm::vec3 v3A = m_listVertices[pTri->m_nAId];
                    glm::vec3 v3B = m_listVertices[pTri->m_nBId];
                    glm::vec3 v3C = m_listVertices[pTri->m_nCId];

                    // calculate normals
                    pTri->m_v3Normal = glm::normalize(glm::cross(v3B - v3A, v3C - v3A));

                    // min/max
                    // -> A
                    m_v3LocalMin.x = std::fmin(m_v3LocalMin.x, v3A.x);
                    m_v3LocalMin.y = std::fmin(m_v3LocalMin.y, v3A.y);
                    m_v3LocalMin.z = std::fmin(m_v3LocalMin.z, v3A.z);

                    m_v3LocalMax.x = std::fmax(m_v3LocalMax.x, v3A.x);
                    m_v3LocalMax.y = std::fmax(m_v3LocalMax.y, v3A.y);
                    m_v3LocalMax.z = std::fmax(m_v3LocalMax.z, v3A.z);

                    // -> B
                    m_v3LocalMin.x = std::fmin(m_v3LocalMin.x, v3B.x);
                    m_v3LocalMin.y = std::fmin(m_v3LocalMin.y, v3B.y);
                    m_v3LocalMin.z = std::fmin(m_v3LocalMin.z, v3B.z);
                   
                    m_v3LocalMax.x = std::fmax(m_v3LocalMax.x, v3B.x);
                    m_v3LocalMax.y = std::fmax(m_v3LocalMax.y, v3B.y);
                    m_v3LocalMax.z = std::fmax(m_v3LocalMax.z, v3B.z);

                    // -> C
                    m_v3LocalMin.x = std::fmin(m_v3LocalMin.x, v3C.x);
                    m_v3LocalMin.y = std::fmin(m_v3LocalMin.y, v3C.y);
                    m_v3LocalMin.z = std::fmin(m_v3LocalMin.z, v3C.z);
                    
                    m_v3LocalMax.x = std::fmax(m_v3LocalMax.x, v3C.x);
                    m_v3LocalMax.y = std::fmax(m_v3LocalMax.y, v3C.y);
                    m_v3LocalMax.z = std::fmax(m_v3LocalMax.z, v3C.z);
                }

                m_listUniqueVertices.clear();
                for (int i = 0; i < (int)m_listVertices.size(); i++) 
                {
                    glm::vec3 v3Point = m_listVertices[i];

                    if (false == IsContainsPoint(&m_listUniqueVertices, v3Point)) 
                    { 
                        m_listUniqueVertices.push_back(v3Point);
                    }
                }

            }
        };

        struct Common m_common;
        std::vector<struct Material> m_listMaterials;
        std::vector<struct ConvexTriMesh> m_listConvexTriMeshs;
        std::vector<struct RigidBody> m_listRigidBodies;
        std::vector<struct Hits> m_listHits;

        Physics(int16_t nFps = 60)
        {
            memset(&m_common, NULL, sizeof(struct Common));

            m_common.m_nFps = nFps;
            m_common.m_fFixedDeltaTime = 1.0f / (float)m_common.m_nFps;
            m_common.m_nNumSubIntegrates = 10;
        }

        ~Physics()
        {
            memset(&m_common, NULL, sizeof(struct Common));
            m_listRigidBodies.clear();
            m_listMaterials.clear();

            for (int i = 0; i < m_listConvexTriMeshs.size(); i++)
            {
                m_listConvexTriMeshs[i].m_listVertices.clear();
                m_listConvexTriMeshs[i].m_listTriangles.clear();
            }
            m_listConvexTriMeshs.clear();
        }

        void Update(float fDeltaTime) 
        {
            m_common.m_fDeltaTime = fDeltaTime;
            m_common.m_fMaxTime += fDeltaTime;
            
            for (; m_common.m_fCurrentTime < m_common.m_fMaxTime; m_common.m_fCurrentTime += m_common.m_fFixedDeltaTime)
            {
                FixedUpdate();
            }
        }

        void SetGravity(glm::vec3 v3Gravity) 
        {
            m_common.m_v3Gravity = v3Gravity;
        }

        glm::vec3 GetGravity() 
        {
            return m_common.m_v3Gravity;
        }

        int32_t GenMaterial() 
        {
            int32_t nId = (int32_t)m_listMaterials.size();

            struct Material material;
            material.m_fRestitution = 0.0f;
            material.m_fFriction = 0.0f;
            m_listMaterials.push_back(material);

            return nId;
        }

        struct Material* GetMaterial(int32_t nId) 
        {
            return &(m_listMaterials[nId]);
        }

        int32_t GenConvexTriMesh() 
        {
            int32_t nId = (int32_t)m_listConvexTriMeshs.size();

            struct ConvexTriMesh convexTriMesh;
            convexTriMesh.m_listVertices.clear();
            convexTriMesh.m_listTriangles.clear();
            convexTriMesh.m_v3LocalMin = glm::vec3(0.0f, 0.0f, 0.0f);
            convexTriMesh.m_v3LocalMax = glm::vec3(0.0f, 0.0f, 0.0f);
            m_listConvexTriMeshs.push_back(convexTriMesh);

            return nId;
        }

        struct ConvexTriMesh* GetConvexTriMesh(int32_t nId)
        {
            return &(m_listConvexTriMeshs[nId]);
        }

        int32_t GenRigidBody()
        {
            int32_t nId = (int32_t)m_listRigidBodies.size();

            struct RigidBody rigidBody;
            rigidBody.m_fMass = 0.0f;
            rigidBody.m_mat3InvInertia = glm::identity<glm::mat3>();
            rigidBody.m_v3Force = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3LinearAcceleration = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3LinearVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3Position = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3Torque = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3AngularAcceleration = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3AngularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_quatOrientation = glm::identity<glm::quat>();
            rigidBody.m_fLinearDamping = 0.0f;
            rigidBody.m_fAngularDamping = 0.0f;
            rigidBody.m_matWorld = glm::identity<glm::mat4>();
            rigidBody.m_nConvexTriMeshId = -1;
            rigidBody.m_nMaterialId = -1;
            
            int32_t nHitId = (int32_t)m_listHits.size();
            rigidBody.m_nHitId = nHitId;
            struct Hits hits;
            m_listHits.push_back(hits);

            m_listRigidBodies.push_back(rigidBody);

            return nId;
        }

        struct RigidBody* GetRigidBody(int32_t nId)
        {
            return &(m_listRigidBodies[nId]);
        }

        int32_t NumRigidBodies() 
        {
            return (int32_t)m_listRigidBodies.size();
        }

    private:

        struct Simplex
        {
        private:
            std::vector<glm::vec3> m_points;
            int m_size;

        public:
            Simplex()
                : m_size(0)
            {
                // Reserve space for 4 points (if necessary for performance)
                m_points.reserve(4);
            }

            Simplex& operator=(std::initializer_list<glm::vec3> list)
            {
                m_size = 0;
                m_points.clear();

                for (glm::vec3 point : list)
                {
                    if (m_size < 4) // Limiting to 4 points
                    {
                        m_points.push_back(point);
                        m_size++;
                    }
                }

                return *this;
            }

            void clear()
            {
                m_size = 0;
            }

            void push_front(glm::vec3 point)
            {
                if (m_size < 4) {
                    m_points.insert(m_points.begin(), point);
                    m_size++;
                }
                else {
                    m_points = { point, m_points[0], m_points[1], m_points[2] };
                }
            }

            glm::vec3& operator[](int i) { return m_points[i]; }
            size_t size() const { return m_size; }

            auto begin() const { return m_points.begin(); }
            auto end() const { return m_points.begin() + m_size; }  // Adjusted to handle dynamic size
        };

        glm::vec3 FindFurthestPoint(const std::vector<glm::vec3>& m_vertices, glm::vec3 direction)
        {
            glm::vec3 maxPoint;
            float maxDistance = -FLT_MAX;

            for (glm::vec3 vertex : m_vertices)
            {
                float distance = glm::dot(vertex, direction);
                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    maxPoint = vertex;
                }
            }

            return maxPoint;
        }

        // Minkowski-differencia támogatási függvénye
        glm::vec3 Support(const std::vector<glm::vec3>& colliderA, const std::vector<glm::vec3>& colliderB, const glm::vec3& direction, glm::vec3* pointA, glm::vec3* pointB)
        {
            glm::vec3 furthestA = FindFurthestPoint(colliderA, direction);
            glm::vec3 furthestB = FindFurthestPoint(colliderB, -direction);

            if (pointA) *pointA = furthestA;
            if (pointB) *pointB = furthestB;

            return furthestA - furthestB; // Minkowski-differencia pontja
        }

        bool SameDirection(const glm::vec3& direction, const glm::vec3& ao)
        {
            return glm::dot(direction, ao) > 0;
        }

        bool LineCase(Simplex& points, glm::vec3& direction)
        {
            glm::vec3 a = points[0];
            glm::vec3 b = points[1];

            glm::vec3 ab = b - a;
            glm::vec3 ao = -a;

            if (SameDirection(ab, ao))
            {
                direction = glm::cross(glm::cross(ab, ao), ab);
            }

            else
            {
                points = { a };
                direction = ao;
            }

            return false;
        }

        bool TriangleCase(Simplex& points, glm::vec3& direction)
        {
            glm::vec3 a = points[0];
            glm::vec3 b = points[1];
            glm::vec3 c = points[2];

            glm::vec3 ab = b - a;
            glm::vec3 ac = c - a;
            glm::vec3 ao = -a;

            glm::vec3 abc = glm::cross(ab, ac);

            if (SameDirection(glm::cross(abc, ac), ao))
            {
                if (SameDirection(ac, ao))
                {
                    points = { a, c };
                    direction = glm::cross(glm::cross(ac, ao), ac);
                }

                else
                {
                    return LineCase(points = { a, b }, direction);
                }
            }

            else {
                if (SameDirection(glm::cross(ab, abc), ao))
                {
                    return LineCase(points = { a, b }, direction);
                }

                else {
                    if (SameDirection(abc, ao))
                    {
                        direction = abc;
                    }

                    else
                    {
                        points = { a, c, b };
                        direction = -abc;
                    }
                }
            }

            return false;
        }

        bool TetrahedronCase(Simplex& points, glm::vec3& direction)
        {
            glm::vec3 a = points[0];
            glm::vec3 b = points[1];
            glm::vec3 c = points[2];
            glm::vec3 d = points[3];

            glm::vec3 ab = b - a;
            glm::vec3 ac = c - a;
            glm::vec3 ad = d - a;
            glm::vec3 ao = -a;

            glm::vec3 abc = glm::cross(ab, ac);
            glm::vec3 acd = glm::cross(ac, ad);
            glm::vec3 adb = glm::cross(ad, ab);

            if (SameDirection(abc, ao))
            {
                return TriangleCase(points = { a, b, c }, direction);
            }

            if (SameDirection(acd, ao))
            {
                return TriangleCase(points = { a, c, d }, direction);
            }

            if (SameDirection(adb, ao))
            {
                return TriangleCase(points = { a, d, b }, direction);
            }

            return true;
        }

        bool NextSimplex(Simplex& points, glm::vec3& direction)
        {
            switch (points.size())
            {
            case 2: return LineCase(points, direction);
            case 3: return TriangleCase(points, direction);
            case 4: return TetrahedronCase(points, direction);
            }

            // never should be here
            return false;
        }

        bool GJK(/*const std::vector<glm::vec3>& colliderA, const std::vector<glm::vec3>& colliderB, glm::mat4 transformA, glm::mat4 transformB,*/RigidBody* pRigidBody1, RigidBody* pRigidBody2, Simplex& points)
        {
            const std::vector<glm::vec3>& colliderA0 = m_listConvexTriMeshs[pRigidBody1->m_nConvexTriMeshId].m_listUniqueVertices;
            const std::vector<glm::vec3>& colliderB0 = m_listConvexTriMeshs[pRigidBody2->m_nConvexTriMeshId].m_listUniqueVertices;
            const glm::mat4 transformA = pRigidBody1->m_matWorld;
            const glm::mat4 transformB = pRigidBody2->m_matWorld;

            std::vector<glm::vec3> colliderA;
            for (int i = 0; i < (int)colliderA0.size(); i++)
            {
                glm::vec3 v3In = colliderA0[i];
                glm::vec3 v3Out = glm::vec3(transformA * glm::vec4(v3In, 1.0f));
                colliderA.push_back(v3Out);
            }

            std::vector<glm::vec3> colliderB;
            for (int i = 0; i < (int)colliderB0.size(); i++)
            {
                glm::vec3 v3In = colliderB0[i];
                glm::vec3 v3Out = glm::vec3(transformB * glm::vec4(v3In, 1.0f));
                colliderB.push_back(v3Out);
            }

            glm::vec3 direction = glm::sphericalRand(1.0f);

            glm::vec3 pointA, pointB; // A két test legközelebbi pontjai
            glm::vec3 support = Support(colliderA, colliderB, direction, &pointA, &pointB);

            // Simplex is an array of points, max count is 4
            points.push_front(support);

            // New direction is towards the origin
            direction = -support;

            for (int iterations = 0; iterations < MAX_ITERATIONS; iterations++)
            {
                support = Support(colliderA, colliderB, direction, &pointA, &pointB);

                /*if (dot(support, direction) <= 0)
                {
                    return false; // no collision
                }*/

                points.push_front(support);

                if (NextSimplex(points, direction))
                {
                    return true;
                }

                if (direction.length() < 0.001f)
                {
                    direction = glm::sphericalRand(1.0f);
                }
            }

            return false;
        }

        void GetFaceNormals(
            const std::vector<glm::vec3>& polytope,
            const std::vector<size_t>& faces,

            std::vector<glm::vec4>* normals,
            size_t* minTriangle)
        {
            float  minDistance = FLT_MAX;

            for (size_t i = 0; i < faces.size(); i += 3)
            {
                glm::vec3 a = polytope[faces[i]];
                glm::vec3 b = polytope[faces[i + 1]];
                glm::vec3 c = polytope[faces[i + 2]];

                glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
                float distance = dot(normal, a);

                if (distance < 0)
                {
                    normal *= -1;
                    distance *= -1;
                }

                (*normals).emplace_back(normal, distance);

                if (distance < minDistance)
                {
                    (*minTriangle) = i / 3;
                    minDistance = distance;
                }
            }
        }

        void AddIfUniqueEdge(
            std::vector<std::pair<size_t, size_t>>& edges,
            const std::vector<size_t>& faces,
            size_t a,
            size_t b)
        {
            auto reverse = std::find(                       //      0--<--3
                edges.begin(),                              //     / \ B /   A: 2-0
                edges.end(),                                //    / A \ /    B: 0-2
                std::make_pair(faces[b], faces[a]) //   1-->--2
            );

            if (reverse != edges.end())
            {
                edges.erase(reverse);
            }

            else
            {
                edges.emplace_back(faces[a], faces[b]);
            }
        }

        bool EPA(
            /*const std::vector<glm::vec3>& colliderA,
            const std::vector<glm::vec3>& colliderB,
            glm::mat4 transformA, glm::mat4 transformB,*/
            RigidBody* pRigidBody1, RigidBody* pRigidBody2,
            const Simplex& simplex,
            Hit *pHit)
        {
            std::vector<glm::vec3>& colliderA0 = m_listConvexTriMeshs[pRigidBody1->m_nConvexTriMeshId].m_listUniqueVertices;
            std::vector<glm::vec3>& colliderB0 = m_listConvexTriMeshs[pRigidBody2->m_nConvexTriMeshId].m_listUniqueVertices;
            glm::mat4 transformA = pRigidBody1->m_matWorld;
            glm::mat4 transformB = pRigidBody2->m_matWorld;

            std::vector<glm::vec3> colliderA;
            for (int i = 0; i < (int)colliderA0.size(); i++)
            {
                glm::vec3 v3In = colliderA0[i];
                glm::vec3 v3Out = glm::vec3(transformA * glm::vec4(v3In, 1.0f));
                colliderA.push_back(v3Out);
            }

            std::vector<glm::vec3> colliderB;
            for (int i = 0; i < (int)colliderB0.size(); i++)
            {
                glm::vec3 v3In = colliderB0[i];
                glm::vec3 v3Out = glm::vec3(transformB * glm::vec4(v3In, 1.0f));
                colliderB.push_back(v3Out);
            }

            std::vector<glm::vec3> polytope(simplex.begin(), simplex.end());
            std::vector<size_t> faces =
            {
                0, 1, 2,
                0, 3, 1,
                0, 2, 3,
                1, 3, 2
            };

            std::vector<glm::vec4> normals;
            size_t minFace;
            GetFaceNormals(polytope, faces, &normals, &minFace);

            glm::vec3 minNormal;
            float minDistance = FLT_MAX;

            glm::vec3 pointA, pointB; // A két test legközelebbi pontjai

            bool bHasCollision = false;

            for (int iterations = 0; /*minDistance == FLT_MAX &&*/false == bHasCollision && iterations < MAX_ITERATIONS; iterations++)
            {
                minNormal = glm::vec3(normals[minFace]);
                minDistance = normals[minFace].w;

                glm::vec3 support = Support(colliderA, colliderB, minNormal, &pointA, &pointB);
                float sDistance = glm::dot(minNormal, support);

                bHasCollision = true;
                if (abs(sDistance - minDistance) > 0.0001f)
                {
                    //minDistance = FLT_MAX;
                    bHasCollision = false;

                    std::vector<std::pair<size_t, size_t>> uniqueEdges;

                    for (size_t i = 0; i < normals.size(); i++)
                    {
                        if (SameDirection(normals[i], support))
                        {
                            size_t f = i * 3;

                            AddIfUniqueEdge(uniqueEdges, faces, f, f + 1);
                            AddIfUniqueEdge(uniqueEdges, faces, f + 1, f + 2);
                            AddIfUniqueEdge(uniqueEdges, faces, f + 2, f);

                            faces[f + 2] = faces.back(); faces.pop_back();
                            faces[f + 1] = faces.back(); faces.pop_back();
                            faces[f] = faces.back(); faces.pop_back();

                            normals[i] = normals.back(); // pop-erase
                            normals.pop_back();

                            i--;
                        }
                    }

                    std::vector<size_t> newFaces;
                    for (const auto& edge : uniqueEdges)
                    {
                        size_t edgeIndex1 = edge.first;
                        size_t edgeIndex2 = edge.second;

                        newFaces.push_back(edgeIndex1);
                        newFaces.push_back(edgeIndex2);
                        newFaces.push_back(polytope.size());
                    }

                    polytope.push_back(support);

                    std::vector<glm::vec4> newNormals;
                    size_t newMinFace;
                    GetFaceNormals(polytope, newFaces, &newNormals, &newMinFace);

                    float oldMinDistance = FLT_MAX;
                    for (size_t i = 0; i < normals.size(); i++)
                    {
                        if (normals[i].w < oldMinDistance)
                        {
                            oldMinDistance = normals[i].w;
                            minFace = i;
                        }
                    }

                    if (newNormals[newMinFace].w < oldMinDistance)
                    {
                        minFace = newMinFace + normals.size();
                    }

                    faces.insert(faces.end(), newFaces.begin(), newFaces.end());
                    normals.insert(normals.end(), newNormals.begin(), newNormals.end());
                }
            }

            if (true == bHasCollision)
            {
                pHit->m_v3NormalInWorld = glm::normalize(minNormal);
                pHit->m_fPenetration = std::fabsf(minDistance);
                pHit->m_v3PointInWorld = pointA;//(pointA + pointB) * 0.5f;
                pHit->m_pRigidBody2 = pRigidBody2;

                //points.Normal = minNormal;
                //points.PenetrationDepth = minDistance;
                //points.collisionPoint = (pointA + pointB) * 0.5f;
                return true;
            }

            return false;
        }


        void FixedUpdate() 
        {
            Integrate();
            UpdateTransforms();

            UpdateBVHTree();

            CollisionDetection();
            CollisionResponse();

            UpdateTransforms();
        }

        glm::quat ApplyAngularVelocity(glm::quat orientation, glm::vec3 angularVelocity, float deltaTime) 
        {
            if (glm::length(angularVelocity) < 0.001f) 
            {
                return orientation;
            }

            // Kiszámítjuk a forgási tengelyt és szöget
            glm::vec3 axis = glm::normalize(angularVelocity); // A forgás tengelye
            float angle = glm::length(angularVelocity) * deltaTime; // A forgás szöge (rad)

            // Létrehozzuk a kvaterniót a szögsebesség alapján
            glm::quat rotation = glm::angleAxis(angle, axis);

            // Alkalmazzuk a forgatást az eredeti kvaternióra
            return glm::normalize(rotation * orientation); // A forgást balról kell megszorozni
        }

        glm::vec3 ApplyLinearDamping(glm::vec3 velocity, float dampingFactor, float deltaTime) 
        {
            velocity *= (1.0f - dampingFactor * deltaTime);
            return velocity;
        }

        glm::vec3 ApplyAngularDamping(glm::vec3 angularVelocity, float dampingFactor, float deltaTime) 
        {
            angularVelocity *= (1.0f - dampingFactor * deltaTime);
            return angularVelocity;
        }

        void Integrate()
        {
            float dt = m_common.m_fFixedDeltaTime / (float)m_common.m_nNumSubIntegrates;

            //std::for_each(std::execution::par_unseq, std::begin(m_listRigidBodies), std::end(m_listRigidBodies), [&](struct RigidBody rigidBody)
            for(int id = 0; id < m_listRigidBodies.size(); id++)
            {
                struct RigidBody& rigidBody = m_listRigidBodies[id];

                if (rigidBody.m_fMass <= 0) 
                {
                    //return;
                    continue;
                }

                for (int i = 0; i < m_common.m_nNumSubIntegrates; i++)
                {
                    // Apply gravity and linear forces
                    rigidBody.m_v3LinearAcceleration = m_common.m_v3Gravity + (rigidBody.m_v3Force / rigidBody.m_fMass);
                    rigidBody.m_v3LinearVelocity += rigidBody.m_v3LinearAcceleration * dt;

                    // Calculate angular acceleration using the inverse inertia tensor
                    rigidBody.m_v3AngularAcceleration = rigidBody.GetInvInertia() * rigidBody.m_v3Torque; // invInertia used here
                    rigidBody.m_v3AngularVelocity += rigidBody.m_v3AngularAcceleration * dt;

                    // Apply damping (linear and angular)
                    rigidBody.m_v3LinearVelocity = ApplyLinearDamping(rigidBody.m_v3LinearVelocity, rigidBody.m_fLinearDamping, dt);
                    rigidBody.m_v3AngularVelocity = ApplyAngularDamping(rigidBody.m_v3AngularVelocity, rigidBody.m_fAngularDamping, dt);

                    // Apply the updated velocities to position and orientation
                    rigidBody.m_v3Position += rigidBody.m_v3LinearVelocity * dt;
                    rigidBody.m_quatOrientation = ApplyAngularVelocity(rigidBody.m_quatOrientation, rigidBody.m_v3AngularVelocity, dt);
                }

                //printf("linearVelocity: %.2f; %.2f; %.2f\n", rigidBody.m_v3LinearVelocity.x, rigidBody.m_v3LinearVelocity.y, rigidBody.m_v3LinearVelocity.z);
                
            }//);
        }

        void UpdateTransforms()
        {
            //std::for_each(std::execution::par_unseq, std::begin(m_listRigidBodies), std::end(m_listRigidBodies), [&](struct RigidBody rigidBody)
            for (int id = 0; id < m_listRigidBodies.size(); id++)
            {
                struct RigidBody& rigidBody = m_listRigidBodies[id];

                glm::mat4 matRotate = glm::toMat4(rigidBody.m_quatOrientation);
                glm::mat4 matTranslate = glm::translate(rigidBody.m_v3Position);

                rigidBody.m_matWorld = matTranslate * matRotate;
            }//);
        }

        void UpdateBVHTree() 
        {
        }

        bool CollisionDetection(RigidBody *pRigidBody1, RigidBody *pRigidBody2, Hits* pHits)
        {
            if (pRigidBody1->m_fMass <= 0.0f && pRigidBody2->m_fMass <= 0.0f) 
            {
                return false;
            }


            Simplex simplex;
            if (true == GJK(pRigidBody1, pRigidBody2, simplex)) 
            {
                Hit hit;
                if (true == EPA(pRigidBody1, pRigidBody2, simplex, &hit))
                {
                    pHits->m_listHits.push_back(hit);
                    return true;
                }
            }
            
            //return true;
            return false;
        }

        void CollisionDetection() 
        {
            for (int i = 0; i < m_listRigidBodies.size(); i++) 
            {
                struct RigidBody* pRigidBody1 = &(m_listRigidBodies[i]);
                struct Hits* pHits = &(m_listHits[pRigidBody1->m_nHitId]);

                pHits->Clear();
            }

            for (int i = 0; i < m_listRigidBodies.size(); i++)
            {
                struct RigidBody *pRigidBody1 = &(m_listRigidBodies[i]);
                struct Hits* pHits = &(m_listHits[pRigidBody1->m_nHitId]);

                for (int j = 0; j < m_listRigidBodies.size(); j++) 
                {
                    struct RigidBody* pRigidBody2 = &(m_listRigidBodies[j]);

                    if (i != j) 
                    {
                        CollisionDetection(pRigidBody1, pRigidBody2, pHits);

                        /*if (false == CollisionDetection(pRigidBody1, pRigidBody2, pHits))
                        {
                            CollisionDetection(pRigidBody2, pRigidBody1, pHits);
                        }*/
                    }

                }
            }
        }

        void ResolveCollisionWithFriction(RigidBody* body, glm::vec3 collisionPoint, glm::vec3 normal)
        {
            int nMatId = body->m_nMaterialId;
            Material* pMaterial = &(m_listMaterials[nMatId]);

            glm::vec3 rA = collisionPoint - body->m_v3Position;
            glm::vec3 velocity = body->m_v3LinearVelocity + glm::cross(body->m_v3AngularVelocity, rA);
            float velocityAlongNormal = glm::dot(normal, velocity);

            //if (velocityAlongNormal > 0) return;
            
            float j = -(1.0f + pMaterial->m_fRestitution) * velocityAlongNormal;
            j /= (1.0f / body->m_fMass + glm::dot(glm::cross(body->GetInvInertia() * glm::cross(rA, normal), rA), normal));

            glm::vec3 impulse = j * normal;

            body->m_v3LinearVelocity += impulse / body->m_fMass; // impulzus változás
            body->m_v3AngularVelocity += body->GetInvInertia() * glm::cross(rA, impulse); // perdület változás

            // ***** Súrlódás hozzáadása *****
            glm::vec3 tangentVelocity = velocity - (velocityAlongNormal * normal);

            if (glm::length(tangentVelocity) > 0.0001f)
            {
                // Kiszervezett változók
                glm::vec3 r = collisionPoint - body->m_v3Position;
                glm::vec3 t = glm::normalize(tangentVelocity);
                float m_inv = 1.0f / body->m_fMass;

                float velocityAlongTangent = glm::dot(tangentVelocity, t);
                if (velocityAlongTangent < 0) return;

                // Számítsd ki az angularImpulse-t
                glm::vec3 crossRT = glm::cross(r, t);
                glm::vec3 angularImpulse = glm::cross(body->GetInvInertia() * crossRT, r);

                // A súrlódási impulzus kiszámítása
                float denominator = m_inv + glm::dot(angularImpulse, t);
                float jt = -glm::dot(tangentVelocity, t) / denominator;

                // Maximális súrlódási impulzus
                float maxFrictionImpulse = body->m_fMass * pMaterial->m_fFriction; // Tömeg és súrlódási tényező szorzataként

                // Az impulzus vektor
                glm::vec3 frictionImpulse = jt * t;

                // Korlátozd az impulzus értékét a maximális súrlódási impulzusra
                if (glm::length(frictionImpulse) > maxFrictionImpulse)
                {
                    frictionImpulse = glm::normalize(frictionImpulse) * maxFrictionImpulse;
                }

                // Lineáris sebesség frissítése súrlódással
                body->m_v3LinearVelocity += frictionImpulse / body->m_fMass;

                // Szögsebesség frissítése súrlódással
                glm::vec3 angularFrictionImpulse = glm::cross(r, frictionImpulse);
                glm::vec3 angularFrictionAcceleration = body->GetInvInertia() * angularFrictionImpulse;
                body->m_v3AngularVelocity += angularFrictionAcceleration;
            }
        }

        void ResolvePenetration(Hit hit) 
        {
            RigidBody* bodyB = hit.m_pRigidBody2;

            glm::vec3 normal = glm::normalize(hit.m_v3NormalInWorld);  // Ütközési normál
            float penetration = hit.m_fPenetration;  // Penetráció mélysége

            {
                // Az eltolás mértéke a penetráció mélysége alapján
                glm::vec3 correctionB(0.0f);

                if (bodyB->m_fMass > 0.0f)
                {
                    correctionB = normal * penetration/* * ratioB*/;
                    bodyB->m_v3Position += correctionB;
                }
            }
        }

        // ez egy jobb széttolás. tesztelni kell majd
        /*
void ResolveCollisionWithPenalization(RigidBody* body, RigidBody* otherBody, glm::vec3 collisionPoint, glm::vec3 normal, float fPenetration)
{
    // Rugó állandó (k), amely meghatározza, milyen erős az eltoló erő
    const float k = 1000.0f; // Ez állítható érték, minél nagyobb, annál erősebb az eltoló erő
    const float damping = 0.1f; // Csillapítási tényező a túlzott oszcilláció elkerülésére

    // Számítsuk ki a penetrációból származó korrekciós erőt
    glm::vec3 correctionForce = k * fPenetration * normal;

    // Csillapítás hozzáadása a sebességek függvényében
    glm::vec3 relativeVelocity = body->m_v3LinearVelocity - otherBody->m_v3LinearVelocity;
    glm::vec3 dampingForce = damping * glm::dot(relativeVelocity, normal) * normal;

    // Teljes korrekciós erő
    glm::vec3 totalCorrectionForce = correctionForce - dampingForce;

    // Alkalmazzuk a korrekciós erőt a testekre
    body->m_v3LinearVelocity += totalCorrectionForce / body->m_fMass;
    otherBody->m_v3LinearVelocity -= totalCorrectionForce / otherBody->m_fMass;

    // Pozíció frissítése a penetráció minimalizálása érdekében
    float positionalCorrectionFactor = 0.2f; // Állítható érték a penetráció fokozatos csökkentéséhez
    glm::vec3 positionalCorrection = positionalCorrectionFactor * fPenetration * normal;

    body->m_v3Position += positionalCorrection / 2.0f;  // Két test egyenlő mértékben mozogjon
    otherBody->m_v3Position -= positionalCorrection / 2.0f;
}
        */

        void CollisionResponse() 
        {
            for (int i = 0; i < (int)m_listHits.size(); i++)
            {
                Hits* pHits = &(m_listHits[i]);

                if (0 == pHits->m_listHits.size()) 
                {
                    continue;
                }

                Hit* pHit = &(pHits->m_listHits[0]);
                for (int j = 0; j < (int)pHits->m_listHits.size(); j++)
                {
                    if (pHits->m_listHits[j].m_fPenetration > pHit->m_fPenetration)
                    {
                        pHit = &(pHits->m_listHits[j]);
                    }
                }
                ResolvePenetration(*pHit);

                for (int j = 0; j < (int)pHits->m_listHits.size(); j++)
                {
                    Hit* pHit = &(pHits->m_listHits[j]);

                    if (pHit->m_pRigidBody2->m_fMass > 0.0f)
                    {
                        ResolveCollisionWithFriction(pHit->m_pRigidBody2, pHit->m_v3PointInWorld, pHit->m_v3NormalInWorld);
                    }

                }
            }

            

        }
	};
}
