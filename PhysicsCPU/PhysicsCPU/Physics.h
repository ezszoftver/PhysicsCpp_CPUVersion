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

        struct RigidBody
        {
            float m_fMass;

            glm::vec3 m_v3Force;
            glm::vec3 m_v3LinearAcceleration;
            glm::vec3 m_v3LinearVelocity;
            glm::vec3 m_v3Position;

            glm::vec3 m_v3Torque;
            glm::vec3 m_v3AngularAcceleration;
            glm::vec3 m_v3AngularVelocity;
            glm::vec3 m_v3Axis;
            float m_fAngle;

            float m_fLinearDamping;
            float m_fAngularDamping;

            glm::mat4x4 m_matWorld;

            int32_t m_nConvexTriMeshId;
            int16_t m_nMaterialId;
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
            rigidBody.m_v3Force = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3LinearAcceleration = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3LinearVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3Position = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3Torque = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3AngularAcceleration = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3AngularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
            rigidBody.m_v3Axis = glm::vec3(1, 0, 0);
            rigidBody.m_fAngle = 0.0f;
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

            CollisionDetection();
            CollisionResponse();
            UpdateTransforms();
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
                    // translate
                    rigidBody.m_v3LinearAcceleration = m_common.m_v3Gravity + (rigidBody.m_v3Force / rigidBody.m_fMass);
                    rigidBody.m_v3LinearVelocity += rigidBody.m_v3LinearAcceleration * dt;
                    rigidBody.m_v3Position += rigidBody.m_v3LinearVelocity * dt;

                    // rotate
                    rigidBody.m_v3AngularAcceleration = (rigidBody.m_v3Torque / rigidBody.m_fMass);
                    rigidBody.m_v3AngularVelocity += rigidBody.m_v3AngularAcceleration * dt;
                    
                    // damping
                    //rigidBody.m_v3LinearVelocity *= std::powf(rigidBody.m_fLinearDamping, dt);
                    //rigidBody.m_v3AngularVelocity *= std::powf(rigidBody.m_fAngularDamping, dt);

                    // axis, angle
                    float fDeltaAngle = glm::length(rigidBody.m_v3AngularVelocity) * dt;
                    glm::vec3 v3RotationAxis = glm::vec3(1, 0, 0);
                    if (glm::length(rigidBody.m_v3AngularVelocity) > 0.0001f) 
                    {
                        v3RotationAxis = glm::normalize(rigidBody.m_v3AngularVelocity);
                    }
                    rigidBody.m_v3Axis = glm::normalize(glm::rotate(rigidBody.m_v3Axis, fDeltaAngle, v3RotationAxis));
                    rigidBody.m_fAngle += fDeltaAngle;
                }
                
            }//);
        }

        void UpdateTransforms()
        {
            //std::for_each(std::execution::par_unseq, std::begin(m_listRigidBodies), std::end(m_listRigidBodies), [&](struct RigidBody rigidBody)
            for (int id = 0; id < m_listRigidBodies.size(); id++)
            {
                struct RigidBody& rigidBody = m_listRigidBodies[id];

                glm::quat quat = glm::angleAxis(rigidBody.m_fAngle, rigidBody.m_v3Axis);
                glm::mat4 matRotate = glm::toMat4(quat);
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

            glm::vec3 v3Dir = glm::vec3(0, 0, 0);
            float fMinPenetration = FLT_MAX;

            // 1/3 plane-point
            for (int i = 0; i < (int)pConvexTriMesh1->m_listUniqueNormals.size(); i++)
            {
                glm::vec3 v3LocalPos = pRigidBody1->m_v3Position;
                glm::vec3 v3LocalNormal = pConvexTriMesh1->m_listUniqueNormals[i];
                glm::vec3 v3Normal = glm::normalize(glm::vec3(pRigidBody1->m_matWorld * glm::vec4(v3LocalNormal, 0.0f)));

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
                    v3Dir = plane.m_v3Normal;
                }

            }

            // 2/3 plane-point
            for (int i = 0; i < (int)pConvexTriMesh2->m_listUniqueNormals.size(); i++)
            {
                glm::vec3 v3LocalPos = pRigidBody2->m_v3Position;
                glm::vec3 v3LocalNormal = pConvexTriMesh2->m_listUniqueNormals[i];
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
                    v3Dir = plane.m_v3Normal;
                }

            }

            // 3/3 edge-edge
            glm::vec3 v3LocalPos = pRigidBody1->m_v3Position;
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
                        v3Dir = plane.m_v3Normal;
                    }
                }
            }

            return glm::vec4(v3Dir, fMinPenetration);
        }

        bool CollisionDetection(RigidBody *pRigidBody1, RigidBody *pRigidBody2) 
        {
            glm::vec4 v4Result = SAT(pRigidBody1, pRigidBody2);
            glm::vec3 v3Dir = glm::vec3(v4Result.x, v4Result.y, v4Result.z);
            float fPenetration = v4Result.w;

            if (glm::length(v3Dir) > 0.001f)
            {
                printf("Coll true; penetration: %f\n", fPenetration);
            }
            else 
            {
                //printf("Coll false\n");
            }
            
            return false;
        }

        void CollisionDetection() 
        {
            for (int i = 0; i < m_listRigidBodies.size(); i++)
            {
                struct RigidBody *pRigidBody1 = &(m_listRigidBodies[i]);

                for (int j = 0; j < m_listRigidBodies.size(); j++) 
                {
                    struct RigidBody* pRigidBody2 = &(m_listRigidBodies[j]);

                    if (i < j) 
                    {
                        CollisionDetection(pRigidBody1, pRigidBody2);
                    }

                }
            }
        }

        void CollisionResponse() 
        {
        }
	};
}
