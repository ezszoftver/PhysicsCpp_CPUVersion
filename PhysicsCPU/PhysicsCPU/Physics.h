#pragma once

#include <stdio.h>
#include <vector>
#include <string>
#include <execution>

#include "glm.hpp"
#include "gtc/matrix_transform.hpp"
#include "gtx/euler_angles.hpp"
#include "gtx/vector_angle.hpp"

namespace PhysicsCPU 
{
	class Physics
	{
	private:
        struct Common
        {
            glm::vec3 m_v3Gravity;
            uint16_t m_nFps;

            float m_fDeltaTime;
            float m_fFixedDeltaTime;

            float m_fCurrentTime;
            float m_fMaxTime;

            uint8_t m_nNumSubIntegrates;
        };

    public:
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
            glm::vec3 m_v3Rotate;

            float m_fLinearDamping;
            float m_fAngularDamping;

            glm::mat4x4 m_m4World;
        };

        struct Material
        {
            float m_fRestitution;
            float m_fFriction;
        };

        struct ConvexTriMesh
        {
            struct RigidBody m_rigidBody;
            uint16_t m_nMaterialId;

            std::vector<glm::vec3> m_listPoints;
            std::vector<uint16_t> m_listIndices;
        };

        struct Common m_common;
        std::vector<struct RigidBody> m_listRigidBodies;
        std::vector<struct Material> m_listMaterials;
        std::vector<struct ConvexTriMesh> m_listConvexTriMeshs;

        Physics(uint16_t nFps = 60)
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
                m_listConvexTriMeshs[i].m_listPoints.clear();
                m_listConvexTriMeshs[i].m_listIndices.clear();
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
                    return;
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
                    rigidBody.m_v3Rotate += rigidBody.m_v3AngularVelocity * dt;

                    // damping
                    rigidBody.m_v3LinearVelocity *= std::powf(rigidBody.m_fLinearDamping, dt);
                    rigidBody.m_v3AngularVelocity *= std::powf(rigidBody.m_fAngularDamping, dt);
                }
                
            }//);
        }

        void UpdateTransforms()
        {
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
