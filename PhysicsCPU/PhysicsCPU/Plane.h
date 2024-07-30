#pragma once

#include "glm/glm.hpp"

namespace PhysicsCPU 
{
	class Plane 
	{
	public:
		glm::vec3 m_v3Pos;
		glm::vec3 m_v3Normal;

		Plane() 
		{
			m_v3Pos = glm::vec3(0, 0, 0);
			m_v3Normal = glm::vec3(0, 0, 0);
		}

		Plane(glm::vec3 v3Pos, glm::vec3 v3Normal) 
		{
			m_v3Pos = v3Pos;
			m_v3Normal = v3Normal;
		}

		float GetDistance(glm::vec3 v3Point) 
		{
			float t = glm::dot(m_v3Normal, v3Point - m_v3Pos);
			return t;
		}
	};
}