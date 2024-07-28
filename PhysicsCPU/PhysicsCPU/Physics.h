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

        struct ConvexTriMesh
        {
            std::vector<glm::vec3> m_listVertices;
            std::vector<Triangle> m_listTriangles;

            glm::vec3 m_v3LocalMin = glm::vec3(0, 0, 0);
            glm::vec3 m_v3LocalMax = glm::vec3(0, 0, 0);

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

            //UpdateBVHTree();

            //CollisionDetection();
            //CollisionResponse();
            //UpdateTransforms();
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

        void CollisionDetection() 
        {
        }

        void CollisionResponse() 
        {
        }
	};
}
