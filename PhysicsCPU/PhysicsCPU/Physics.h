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

namespace PhysicsCPU 
{
	class Physics
	{
	private:
        static bool IsParallel(glm::vec3 v3Dir1, glm::vec3 v3Dir2)
        {
            float fAngleRad = glm::angle(v3Dir1, v3Dir2);
            float fAngle = glm::degrees(fAngleRad);
            return ((std::fabs(fAngle) < 0.1f) || (std::fabs(fAngle) > 179.9f));
        }

        static bool IntersectPlaneLine(Plane* pPlane, Line* pLine, glm::vec3 *pRet)
        {
            float denom = glm::dot(pPlane->m_v3Normal, pLine->GetDir());

            if (glm::abs(denom) > 0.001f) 
            {
                float t = glm::dot(pPlane->m_v3Pos - pLine->m_v3A, pPlane->m_v3Normal) / denom;

                if (t < 0.0f) { return false; }
                glm::vec3 v3IntersectPoint = pLine->m_v3A + (t * pLine->GetDir());
                float t_max = glm::distance(pLine->m_v3A, pLine->m_v3B);
                if (t > t_max) { return false; }

                (*pRet) = v3IntersectPoint;
                return true;
            }

            return false;
        }

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

            glm::vec3 GetPointVelocity(glm::vec3 v3Point)
            {
                return (m_v3LinearVelocity + glm::cross(m_v3AngularVelocity, v3Point));
            }

            glm::mat3 GetInvInertia() 
            {
                glm::mat3 rot = glm::mat3_cast(m_quatOrientation);  // quaternion -> rotation matrix
                glm::mat3 inertiaWorld = rot * m_mat3InvInertia * glm::transpose(rot);
                return inertiaWorld;
            }
            
        };

        struct Hit 
        {
            RigidBody *m_pRigidBody1 = nullptr;
            RigidBody *m_pRigidBody2 = nullptr;

            glm::vec3 m_v3PointInWorld = glm::vec3(0, 0, 0);
            glm::vec3 m_v3NormalInWorld = glm::vec3(0, 0, 0);
            float m_fPenetration = 0.0f;

            struct Hit Invert() 
            {
                struct Hit ret;

                ret.m_pRigidBody1 = m_pRigidBody2;
                ret.m_pRigidBody2 = m_pRigidBody1;
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
            std::vector<glm::vec3> m_listUniqueNormals;
            std::vector<glm::vec3> m_listUniqueEdges;

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

            bool IsContainsDir(std::vector<glm::vec3> *pList, glm::vec3 v3Dir)
            {
                for (int i = 0; i < (int)(*pList).size(); i++)
                {
                    glm::vec3 v3Dir2 = (*pList)[i];
                    if (true == Physics::IsParallel(v3Dir, v3Dir2)) 
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
                m_listUniqueNormals.clear();
                m_listUniqueEdges.clear();
                for (int i = 0; i < (int)m_listVertices.size(); i++) 
                {
                    glm::vec3 v3Point = m_listVertices[i];

                    if (false == IsContainsPoint(&m_listUniqueVertices, v3Point)) 
                    { 
                        m_listUniqueVertices.push_back(v3Point); 
                    }
                }
                for (int i = 0; i < (int)m_listTriangles.size(); i++)
                {
                    Triangle* pTri = &(m_listTriangles[i]);

                    if (false == IsContainsDir(&m_listUniqueNormals, pTri->m_v3Normal)) 
                    { 
                        m_listUniqueNormals.push_back(pTri->m_v3Normal); 
                    }
                }
                for (int i = 0; i < (int)m_listTriangles.size(); i++)
                {
                    Triangle* pTri = &(m_listTriangles[i]);

                    glm::vec3 v3A = m_listVertices[pTri->m_nAId];
                    glm::vec3 v3B = m_listVertices[pTri->m_nBId];
                    glm::vec3 v3C = m_listVertices[pTri->m_nCId];

                    Line line1(v3A, v3B);
                    Line line2(v3B, v3C);
                    Line line3(v3C, v3A);
                    if (false == IsContainsDir(&m_listUniqueEdges, line1.GetDir())) 
                    { 
                        m_listUniqueEdges.push_back(line1.GetDir()); 
                    }
                    if (false == IsContainsDir(&m_listUniqueEdges, line2.GetDir())) 
                    { 
                        m_listUniqueEdges.push_back(line2.GetDir()); 
                    }
                    if (false == IsContainsDir(&m_listUniqueEdges, line3.GetDir())) 
                    { 
                        m_listUniqueEdges.push_back(line3.GetDir()); 
                    }
                }

            }
        };

        struct Common m_common;
        std::vector<struct Material> m_listMaterials;
        std::vector<struct ConvexTriMesh> m_listConvexTriMeshs;
        std::vector<struct RigidBody> m_listRigidBodies;

        Physics(int16_t nFps = 60)
        {
            memset(&m_common, NULL, sizeof(struct Common));

            m_common.m_nFps = nFps;
            m_common.m_fFixedDeltaTime = 1.0f / (float)m_common.m_nFps;
            m_common.m_nNumSubIntegrates = 5;
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
        void FixedUpdate() 
        {
            Integrate();
            UpdateTransforms();

            UpdateBVHTree();

            CollisionDetectionAndResponse();
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

        void GetMinMax(RigidBody *pRigidBody, Plane *pPlane, float *p_fMin, float *p_fMax)
        {
            *p_fMin = +FLT_MAX;
            *p_fMax = -FLT_MAX;

            struct ConvexTriMesh* pConvexTriMesh = &(m_listConvexTriMeshs[pRigidBody->m_nConvexTriMeshId]);

            for (int i = 0; i < (int)pConvexTriMesh->m_listUniqueVertices.size(); i++) 
            {
                glm::vec3 v3LocalPos = pConvexTriMesh->m_listUniqueVertices[i];
                glm::vec3 v3Pos = pRigidBody->m_matWorld * glm::vec4(v3LocalPos, 1.0f);

                float t = pPlane->GetDistance(v3Pos);

                if (t < (*p_fMin)) { (*p_fMin) = t; }
                if (t > (*p_fMax)) { (*p_fMax) = t; }
            }
        }

        glm::vec4 SAT(RigidBody* pRigidBody1, RigidBody* pRigidBody2) 
        {
            struct ConvexTriMesh* pConvexTriMesh1 = &(m_listConvexTriMeshs[pRigidBody1->m_nConvexTriMeshId]);
            struct ConvexTriMesh* pConvexTriMesh2 = &(m_listConvexTriMeshs[pRigidBody2->m_nConvexTriMeshId]);

            glm::vec4 ret(0, 0, 0, 0);
            float fMinPenetration = FLT_MAX;

            // 1/3 plane-point
            for (int i = 0; i < (int)pConvexTriMesh1->m_listUniqueNormals.size(); i++)
            {
                glm::vec3 v3LocalPos = pRigidBody1->m_v3Position;
                glm::vec3 v3LocalNormal = pConvexTriMesh1->m_listUniqueNormals[i];
                glm::vec3 v3Pos = glm::vec3(pRigidBody1->m_matWorld * glm::vec4(v3LocalPos, 1.0f));
                glm::vec3 v3Normal = glm::normalize(glm::vec3(pRigidBody1->m_matWorld * glm::vec4(v3LocalNormal, 0.0f)));

                Plane plane(v3Pos, v3Normal);

                float fMin1, fMax1;
                GetMinMax(pRigidBody1, &plane, &fMin1, &fMax1);
                float fMin2, fMax2;
                GetMinMax(pRigidBody2, &plane, &fMin2, &fMax2);

                float fGlobalMin = (fMin1 < fMin2) ? fMin1 : fMin2;
                float fGlobalMax = (fMax1 > fMax2) ? fMax1 : fMax2;
                float fGlobalDistance = fGlobalMax - fGlobalMin;
                float fDist1 = fMax1 - fMin1;
                float fDist2 = fMax2 - fMin2;

                if (fGlobalDistance > (fDist1 + fDist2)) 
                {
                    return glm::vec4(0, 0, 0, 0);
                }

                float fPenetration = (fDist1 + fDist2) - fGlobalDistance;
                if (fPenetration < fMinPenetration)
                {
                    fMinPenetration = fPenetration;
                    ret = glm::vec4(plane.m_v3Normal, fMinPenetration);
                }

            }

            // 2/3 plane-point
            for (int i = 0; i < (int)pConvexTriMesh2->m_listUniqueNormals.size(); i++)
            {
                glm::vec3 v3LocalPos = pRigidBody2->m_v3Position;
                glm::vec3 v3LocalNormal = pConvexTriMesh2->m_listUniqueNormals[i];
                glm::vec3 v3Pos = glm::vec3(pRigidBody2->m_matWorld * glm::vec4(v3LocalPos, 1.0f));
                glm::vec3 v3Normal = glm::normalize(glm::vec3(pRigidBody2->m_matWorld * glm::vec4(v3LocalNormal, 0.0f)));

                Plane plane(v3LocalPos, v3Normal);

                float fMin1, fMax1;
                GetMinMax(pRigidBody1, &plane, &fMin1, &fMax1);
                float fMin2, fMax2;
                GetMinMax(pRigidBody2, &plane, &fMin2, &fMax2);

                float fGlobalMin = (fMin1 < fMin2) ? fMin1 : fMin2;
                float fGlobalMax = (fMax1 > fMax2) ? fMax1 : fMax2;
                float fGlobalDistance = fGlobalMax - fGlobalMin;
                float fDist1 = fMax1 - fMin1;
                float fDist2 = fMax2 - fMin2;

                if (fGlobalDistance > (fDist1 + fDist2))
                {
                    return glm::vec4(0, 0, 0, 0);
                }

                float fPenetration = (fDist1 + fDist2) - fGlobalDistance;
                if (fPenetration < fMinPenetration)
                {
                    fMinPenetration = fPenetration;
                    ret = glm::vec4(plane.m_v3Normal, fMinPenetration);
                }

            }

            // 3/3 edge-edge
            glm::vec3 v3LocalPos = pRigidBody1->m_v3Position;
            glm::vec3 v3Pos = glm::vec3(pRigidBody1->m_matWorld * glm::vec4(v3LocalPos, 1.0f));
            for (int i = 0; i < (int)pConvexTriMesh1->m_listUniqueEdges.size(); i++) 
            {
                glm::vec3 v3LocalDir = pConvexTriMesh1->m_listUniqueEdges[i];
                glm::vec3 v3Dir1 = glm::vec3(pRigidBody1->m_matWorld * glm::vec4(v3LocalDir, 0.0f));

                for (int j = 0; j < (int)pConvexTriMesh2->m_listUniqueEdges.size(); j++) 
                {
                    glm::vec3 v3LocalDir = pConvexTriMesh2->m_listUniqueEdges[j];
                    glm::vec3 v3Dir2 = glm::vec3(pRigidBody2->m_matWorld * glm::vec4(v3LocalDir, 0.0f));

                    if (true == Physics::IsParallel(v3Dir1, v3Dir2))
                    {
                        continue;
                    }

                    glm::vec3 v3Normal = glm::normalize(glm::cross(v3Dir1, v3Dir2));

                    Plane plane(v3Pos, v3Normal);

                    float fMin1, fMax1;
                    GetMinMax(pRigidBody1, &plane, &fMin1, &fMax1);
                    float fMin2, fMax2;
                    GetMinMax(pRigidBody2, &plane, &fMin2, &fMax2);

                    float fGlobalMin = (fMin1 < fMin2) ? fMin1 : fMin2;
                    float fGlobalMax = (fMax1 > fMax2) ? fMax1 : fMax2;
                    float fGlobalDistance = fGlobalMax - fGlobalMin;
                    float fDist1 = fMax1 - fMin1;
                    float fDist2 = fMax2 - fMin2;

                    if (fGlobalDistance > (fDist1 + fDist2))
                    {
                        return glm::vec4(0, 0, 0, 0);
                    }

                    float fPenetration = (fDist1 + fDist2) - fGlobalDistance;
                    if (fPenetration < fMinPenetration)
                    {
                        fMinPenetration = fPenetration;
                        ret = glm::vec4(plane.m_v3Normal, fMinPenetration);
                    }
                }
            }

            return ret;
        }

        Triangle* FindBestTriangle(RigidBody* pRigidBody, glm::vec3 v3Dir) 
        {
            Triangle* pRet = nullptr;

            struct ConvexTriMesh* pConvexTriMesh = &(m_listConvexTriMeshs[pRigidBody->m_nConvexTriMeshId]);

            for (int i = 0; i < (int)pConvexTriMesh->m_listTriangles.size(); i++)
            {
                Triangle *pTriangle = &(pConvexTriMesh->m_listTriangles[i]);
                glm::vec3 v3LocalNormal = pTriangle->m_v3Normal;
                glm::vec3 v3Normal = glm::normalize(glm::vec3(pRigidBody->m_matWorld * glm::vec4(v3LocalNormal, 0.0f)));

                if (nullptr == pRet) 
                { 
                    pRet = pTriangle; 
                }
                else if (glm::angle(v3Dir, v3Normal) < glm::angle(v3Dir, pRet->m_v3Normal))
                {
                    pRet = pTriangle;
                }
            }

            return pRet;
        }

        void FindSameTriangles(RigidBody* pRigidBody, Triangle* pBestTriangle, std::vector<Triangle*>* pRet) 
        {
            glm::vec3 v3BestNormal = glm::normalize(glm::vec3(pRigidBody->m_matWorld * glm::vec4(pBestTriangle->m_v3Normal, 0.0f)));

            struct ConvexTriMesh* pConvexTriMesh = &(m_listConvexTriMeshs[pRigidBody->m_nConvexTriMeshId]);

            for (int i = 0; i < (int)pConvexTriMesh->m_listTriangles.size(); i++)
            {
                Triangle* pTriangle = &(pConvexTriMesh->m_listTriangles[i]);
                glm::vec3 v3LocalNormal = pTriangle->m_v3Normal;
                glm::vec3 v3Normal = glm::normalize(glm::vec3(pRigidBody->m_matWorld * glm::vec4(v3LocalNormal, 0.0f)));

                float fAngle = glm::angle(v3BestNormal, v3Normal);

                if (fAngle < glm::radians(0.1f)) 
                {
                    (*pRet).push_back(pTriangle);
                }

            }
        }

        void GeneratePlanesAndLines(RigidBody* pRigidBody, std::vector<Triangle*> *pListTriangles, std::vector<Plane> *pListPlanes, std::vector<Line>* pListLines)
        {
            struct ConvexTriMesh* pConvexTriMesh = &(m_listConvexTriMeshs[pRigidBody->m_nConvexTriMeshId]);

            for (int i = 0; i < (int)(*pListTriangles).size(); i++) 
            {
                Triangle* pTriangle = (*pListTriangles)[i];

                glm::vec3 v3LocalA = pConvexTriMesh->m_listVertices[pTriangle->m_nAId];
                glm::vec3 v3LocalB = pConvexTriMesh->m_listVertices[pTriangle->m_nBId];
                glm::vec3 v3LocalC = pConvexTriMesh->m_listVertices[pTriangle->m_nCId];

                glm::vec3 v3A = glm::vec3(pRigidBody->m_matWorld * glm::vec4(v3LocalA, 1.0f));
                glm::vec3 v3B = glm::vec3(pRigidBody->m_matWorld * glm::vec4(v3LocalB, 1.0f));
                glm::vec3 v3C = glm::vec3(pRigidBody->m_matWorld * glm::vec4(v3LocalC, 1.0f));

                glm::vec3 v3Normal = glm::normalize(glm::vec3(pRigidBody->m_matWorld * glm::vec4(pTriangle->m_v3Normal, 0.0f)));

                // front
                Plane planeFront;
                planeFront.m_v3Pos = v3A;
                planeFront.m_v3Normal = v3Normal;
                
                // a
                Plane planeA;
                glm::vec3 v3AB = glm::normalize(v3B - v3A);
                planeA.m_v3Normal = glm::normalize(glm::cross(v3AB, v3Normal));
                planeA.m_v3Pos = v3A;

                // b
                Plane planeB;
                glm::vec3 v3BC = glm::normalize(v3C - v3B);
                planeB.m_v3Normal = glm::normalize(glm::cross(v3BC, v3Normal));
                planeB.m_v3Pos = v3B;

                // c
                Plane planeC;
                glm::vec3 v3CA = glm::normalize(v3A - v3C);
                planeC.m_v3Normal = glm::normalize(glm::cross(v3CA, v3Normal));
                planeC.m_v3Pos = v3C;

                // planes
                (*pListPlanes).push_back(planeFront);
                (*pListPlanes).push_back(planeA);
                (*pListPlanes).push_back(planeB);
                (*pListPlanes).push_back(planeC);

                // lines
                Line lineAB(v3B, v3A);
                Line lineBC(v3C, v3B);
                Line lineCA(v3A, v3C);

                (*pListLines).push_back(lineAB);
                (*pListLines).push_back(lineBC);
                (*pListLines).push_back(lineCA);
            }
        }

        bool IsInside(glm::vec3 v3Point, Plane *pListPlanes) 
        {
            for (int i = 0; i < 4; i++)
            {
                Plane plane = pListPlanes[i];
                if (plane.GetDistance(v3Point) > 0.001f) 
                {
                    return false;
                }
            }
            return true;
        }

        void GenerateHits(RigidBody* pRigidBody1, RigidBody* pRigidBody2, glm::vec3 v3Normal, float fPenetration,  Plane *pConvexPlanes1, Plane *pConvexPlanes2, Line *pLines1, Line *pLines2, struct Hits *pHits)
        {
            // RigidBody1 pontok benne vannak RigidBody2-ben?
            for (int i = 0; i < 3; i++) 
            {
                glm::vec3 v3Point = pLines1[i].m_v3A;

                if (true == IsInside(v3Point, pConvexPlanes2)) 
                {
                    struct Hit hit;
                    hit.m_pRigidBody1 = pRigidBody1;
                    hit.m_pRigidBody2 = pRigidBody2;
                    hit.m_v3PointInWorld = v3Point;
                    hit.m_v3NormalInWorld = v3Normal;
                    hit.m_fPenetration = std::fabs(fPenetration);

                    (*pHits).m_listHits.push_back(hit);
                }
            }
            
            // RigidBody1 edges-ei metszi RigidBody2 plane-eit?
            for (int i = 0; i < 3; i++) 
            {
                Line *pLine = &(pLines1[i]);

                for (int j = 0; j < 4; j++) 
                {
                    Plane *pPlane = &(pConvexPlanes2[j]);

                    glm::vec3 v3Point;
                    if (true == Physics::IntersectPlaneLine(pPlane, pLine, &v3Point) && true == IsInside(v3Point, pConvexPlanes2))
                    {
                        struct Hit hit;
                        hit.m_pRigidBody1 = pRigidBody1;
                        hit.m_pRigidBody2 = pRigidBody2;
                        hit.m_v3PointInWorld = v3Point;
                        hit.m_v3NormalInWorld = v3Normal;
                        hit.m_fPenetration = std::fabs(fPenetration);

                        (*pHits).m_listHits.push_back(hit);
                    }

                }
            }

        }

        bool CollisionDetection(RigidBody *pRigidBody1, RigidBody *pRigidBody2, struct Hits *pHits)
        {
            if (pRigidBody1->m_fMass <= 0.0f && pRigidBody2->m_fMass <= 0.0f) 
            {
                return false;
            }

            glm::vec4 separate = SAT(pRigidBody1, pRigidBody2);
            glm::vec3 v3SeparateNormal = glm::vec3(separate.x, separate.y, separate.z);
            float fPenetration = separate.w;

            if (glm::length(v3SeparateNormal) < 0.001f)
            {
                //printf("Coll false\n");
                return false;
            }
            else
            {
                //printf("Coll ok\n");
            }

            // Find best triangle
            glm::vec3 v3RB1Dir;
            {
                glm::vec3 v3ToRB2 = glm::normalize(pRigidBody2->m_v3Position - pRigidBody1->m_v3Position);
                if (glm::angle(v3SeparateNormal, v3ToRB2) < glm::radians(90.0f)) { v3RB1Dir = v3SeparateNormal; }
                else { v3RB1Dir = -v3SeparateNormal; }
            }

            glm::vec3 v3RB2Dir;
            {
                glm::vec3 v3ToRB1 = glm::normalize(pRigidBody1->m_v3Position - pRigidBody2->m_v3Position);
                if (glm::angle(v3SeparateNormal, v3ToRB1) < glm::radians(90.0f)) { v3RB2Dir = v3SeparateNormal; }
                else { v3RB2Dir = -v3SeparateNormal; }
            }

            Triangle *pRB1BestTriangle = FindBestTriangle(pRigidBody1, v3RB1Dir);
            Triangle *pRB2BestTriangle = FindBestTriangle(pRigidBody2, v3RB2Dir);

            // Find same triangles
            std::vector<Triangle*> listRB1LocalTriangles;
            std::vector<Triangle*> listRB2LocalTriangles;
            FindSameTriangles(pRigidBody1, pRB1BestTriangle, &listRB1LocalTriangles);
            FindSameTriangles(pRigidBody2, pRB2BestTriangle, &listRB2LocalTriangles);

            // Generate planes
            std::vector<Plane> listRB1Planes;
            std::vector<Plane> listRB2Planes;
            std::vector<Line> listRB1Lines;
            std::vector<Line> listRB2Lines;
            GeneratePlanesAndLines(pRigidBody1, &listRB1LocalTriangles, &listRB1Planes, &listRB1Lines);
            GeneratePlanesAndLines(pRigidBody2, &listRB2LocalTriangles, &listRB2Planes, &listRB2Lines);
            
            pHits->Clear();
            for (int i = 0; i < (int)listRB1LocalTriangles.size(); i++)
            {
                Plane *pConvexPlanes1 = &(listRB1Planes[i * 4]);
                Line* pLines1 = &(listRB1Lines[i * 3]);

                for (int j = 0; j < (int)listRB2LocalTriangles.size(); j++)
                {
                    Plane* pConvexPlanes2 = &(listRB2Planes[j * 4]);
                    Line* pLines2 = &(listRB2Lines[j * 3]);

                    GenerateHits(pRigidBody1, pRigidBody2, v3RB1Dir, fPenetration, pConvexPlanes1, pConvexPlanes2, pLines1, pLines2, pHits);

                    struct Hits hits2;
                    GenerateHits(pRigidBody2, pRigidBody1, v3RB2Dir, fPenetration, pConvexPlanes2, pConvexPlanes1, pLines2, pLines1, &hits2);

                    for (int j = 0; j < (int)hits2.m_listHits.size(); j++)
                    {
                        struct Hit hit2 = hits2.m_listHits[j];
                        pHits->m_listHits.push_back( hit2.Invert() );
                    }

                }
            }

            return ((int)pHits->m_listHits.size() > 0);
            //return true;
        }

        void CollisionDetectionAndResponse() 
        {
            for (int i = 0; i < m_listRigidBodies.size(); i++)
            {
                struct RigidBody *pRigidBody1 = &(m_listRigidBodies[i]);

                for (int j = 0; j < m_listRigidBodies.size(); j++) 
                {
                    struct RigidBody* pRigidBody2 = &(m_listRigidBodies[j]);

                    if (i < j) 
                    {
                        struct Hits hits;
                        if (true == CollisionDetection(pRigidBody1, pRigidBody2, &hits)) 
                        {
                            CollisionResponse(&hits);
                        }
                    }

                }
            }
        }

        bool IsContains(std::vector<glm::vec3> *pListNormals, glm::vec3 v3Normal) 
        {
            for (int i = 0; i < (int)(*pListNormals).size(); i++)
            {
                float fAngleRad = glm::angle((*pListNormals)[i], v3Normal);
                float fAngle = glm::degrees(fAngleRad);

                if (fAngle < 0.1f)
                {
                    return true;
                }
            }

            return false;
        }

        void ResolveCollisionWithFriction(RigidBody* body, glm::vec3 collisionPoint, glm::vec3 normal)
        {
            int nMatId = body->m_nMaterialId;
            Material* pMaterial = &(m_listMaterials[nMatId]);

            glm::vec3 rA = collisionPoint - body->m_v3Position;
            glm::vec3 velocity = body->m_v3LinearVelocity + glm::cross(body->m_v3AngularVelocity, rA);
            float velocityAlongNormal = glm::dot(normal, velocity);

            if (velocityAlongNormal > 0) return;
            
            float j = -(1.0f + pMaterial->m_fRestitution) * velocityAlongNormal;
            j /= (1.0f / body->m_fMass + glm::dot(glm::cross(body->GetInvInertia() * glm::cross(rA, normal), rA), normal));

            glm::vec3 impulse = j * normal;

            body->m_v3LinearVelocity += impulse / body->m_fMass; // impulzus változás
            body->m_v3AngularVelocity += glm::cross(rA, impulse) * body->GetInvInertia(); // perdület változás

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

        void ResolveCollisionWithFriction(RigidBody* body, RigidBody* otherBody, glm::vec3 collisionPoint, glm::vec3 normal)
        {
            int nMatId1 = body->m_nMaterialId;
            int nMatId2 = otherBody->m_nMaterialId;
            Material* pMaterial1 = &(m_listMaterials[nMatId1]);
            Material* pMaterial2 = &(m_listMaterials[nMatId2]);

            glm::vec3 rA = collisionPoint - body->m_v3Position;
            glm::vec3 rB = collisionPoint - otherBody->m_v3Position;

            glm::vec3 vA = body->m_v3LinearVelocity + glm::cross(body->m_v3AngularVelocity, rA);
            glm::vec3 vB = otherBody->m_v3LinearVelocity + glm::cross(otherBody->m_v3AngularVelocity, rB);

            glm::vec3 relativeVelocity = vA - vB;
            float velocityAlongNormal = glm::dot(normal, relativeVelocity);

            if (velocityAlongNormal > 0) return;
            
            float restitution = (pMaterial1->m_fRestitution + pMaterial2->m_fRestitution) / 2.0f;
            float j = -(1.0f + restitution) * velocityAlongNormal;
            j /= (1.0f / body->m_fMass + 1.0f / otherBody->m_fMass +
                glm::dot(glm::cross(body->GetInvInertia() * glm::cross(rA, normal), rA), normal) +
                glm::dot(glm::cross(otherBody->GetInvInertia() * glm::cross(rB, normal), rB), normal));

            glm::vec3 impulse = j * normal;

            body->m_v3LinearVelocity += impulse / body->m_fMass;
            body->m_v3AngularVelocity += glm::cross(rA, impulse) * body->GetInvInertia();

            otherBody->m_v3LinearVelocity -= impulse / otherBody->m_fMass;
            otherBody->m_v3AngularVelocity -= glm::cross(rB, impulse) * otherBody->GetInvInertia();

            // ***** Súrlódás hozzáadása *****
            glm::vec3 tangentVelocity = relativeVelocity - (velocityAlongNormal * normal);

            if (glm::length(tangentVelocity) > 0.0001f)
            {
                glm::vec3 tangent = glm::normalize(tangentVelocity);

                float velocityAlongTangent = glm::dot(relativeVelocity, tangent);
                if (velocityAlongTangent < 0) return;

                // Kiszámítjuk az angularImpulse-okat
                glm::vec3 crossRT_A = glm::cross(rA, tangent);
                glm::vec3 angularImpulse_A = glm::cross(body->GetInvInertia() * crossRT_A, rA);

                glm::vec3 crossRT_B = glm::cross(rB, tangent);
                glm::vec3 angularImpulse_B = glm::cross(otherBody->GetInvInertia() * crossRT_B, rB);

                // Súrlódási impulzus kiszámítása
                float m_inv_A = 1.0f / body->m_fMass;
                float m_inv_B = 1.0f / otherBody->m_fMass;

                float denominator = m_inv_A + m_inv_B +
                    glm::dot(angularImpulse_A, tangent) +
                    glm::dot(angularImpulse_B, tangent);

                float jt = -velocityAlongTangent / denominator;

                // Maximális súrlódási impulzus
                float maxFrictionImpulse = (body->m_fMass + otherBody->m_fMass) * (pMaterial1->m_fFriction + pMaterial2->m_fFriction) / 2.0f;

                // Az impulzus vektor
                glm::vec3 frictionImpulse = jt * tangent;

                // Korlátozd az impulzus értékét a maximális súrlódási impulzusra
                if (glm::length(frictionImpulse) > maxFrictionImpulse)
                {
                    frictionImpulse = glm::normalize(frictionImpulse) * maxFrictionImpulse;
                }

                // Lineáris sebesség frissítése súrlódással
                body->m_v3LinearVelocity += frictionImpulse / body->m_fMass;
                otherBody->m_v3LinearVelocity -= frictionImpulse / otherBody->m_fMass;

                // Szögsebesség frissítése súrlódással
                glm::vec3 angularFrictionImpulse_A = glm::cross(rA, frictionImpulse);
                glm::vec3 angularFrictionAcceleration_A = body->GetInvInertia() * angularFrictionImpulse_A;
                body->m_v3AngularVelocity += angularFrictionAcceleration_A;

                glm::vec3 angularFrictionImpulse_B = glm::cross(rB, frictionImpulse);
                glm::vec3 angularFrictionAcceleration_B = otherBody->GetInvInertia() * angularFrictionImpulse_B;
                otherBody->m_v3AngularVelocity -= angularFrictionAcceleration_B;
            }
        }

        void ResolvePenetration(Hit hit) 
        {
            //RigidBody* bodyA = hit.m_pRigidBody1;
            RigidBody* bodyB = hit.m_pRigidBody2;

            glm::vec3 normal = glm::normalize(hit.m_v3NormalInWorld);  // Ütközési normál
            float penetration = hit.m_fPenetration;  // Penetráció mélysége

            //float massA = bodyA->m_fMass <= 0.0f ? 1000000.0f : bodyA->m_fMass;
            float massB = bodyB->m_fMass <= 0.0f ? 1000000.0f : bodyB->m_fMass;

            // A mozgó testeket a penetráció mélységének arányában kell eltolni
            //float totalMass = massA + massB;

            //if (totalMass > 0) 
            {
                // Az eltolás mértéke a penetráció mélysége alapján
                glm::vec3 correctionA(0.0f), correctionB(0.0f);

                /*if (bodyA->m_fMass > 0.0f)
                {
                    float ratioA = massB / totalMass;  // A test aránya
                    correctionA = -normal * penetration * ratioA;
                    bodyA->m_v3Position += correctionA;
                }*/
                
                if (bodyB->m_fMass > 0.0f)
                {
                    //float ratioB = massA / totalMass;  // B test aránya
                    correctionB = normal * penetration/* * ratioB*/;
                    bodyB->m_v3Position += correctionB;
                }
            }
        }

        void CollisionResponse(struct Hits *pHits) 
        {
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
                Hit *pHit = &(pHits->m_listHits[j]);

                /*if (pHit->m_fPenetration < 0.001f)
                {
                    continue;
                }*/

                // resolve collision
                if (pHit->m_pRigidBody1->m_fMass > 0.0f && pHit->m_pRigidBody2->m_fMass > 0.0f) 
                {
                    ResolveCollisionWithFriction(pHit->m_pRigidBody1, pHit->m_pRigidBody2, pHit->m_v3PointInWorld, -pHit->m_v3NormalInWorld);
                }
                else if (pHit->m_pRigidBody1->m_fMass > 0.0f) 
                {
                    ResolveCollisionWithFriction(pHit->m_pRigidBody1, pHit->m_v3PointInWorld, -pHit->m_v3NormalInWorld);
                }
                else if (pHit->m_pRigidBody2->m_fMass > 0.0f) 
                {
                    ResolveCollisionWithFriction(pHit->m_pRigidBody2, pHit->m_v3PointInWorld, pHit->m_v3NormalInWorld);
                }

            }

            

        }
	};
}
