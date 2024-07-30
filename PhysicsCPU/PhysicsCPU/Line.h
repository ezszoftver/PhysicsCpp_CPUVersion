#include "glm/glm.hpp"

namespace PhysicsCPU
{
	class Line
	{
	public:
		glm::vec3 m_v3A;
		glm::vec3 m_v3B;

		Line() 
		{
			m_v3A = glm::vec3(0,0,0);
			m_v3B = glm::vec3(0, 0, 0);
		}

		Line(glm::vec3 v3A, glm::vec3 v3B) 
		{
			m_v3A = v3A;
			m_v3B = v3B;
		}

		glm::vec3 GetDir() 
		{
			return glm::normalize(m_v3B - m_v3A);
		}
	};
}