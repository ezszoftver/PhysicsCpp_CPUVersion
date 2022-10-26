#include <stdlib.h>

#include "Physics.h"

using namespace PhysicsCPU;

Physics* pPhysics = NULL;

int main(int argc, char **argv) 
{
	pPhysics = new Physics();
	pPhysics->SetGravity(glm::vec3(0, -1, 0));

	;

	delete pPhysics;
	pPhysics = NULL;

	return EXIT_SUCCESS;
}