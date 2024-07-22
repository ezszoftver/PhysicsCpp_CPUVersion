#include <stdlib.h>

#include <Windows.h>
#include <gl/GL.h>
#include <GLFW/glfw3.h>
GLFWwindow* pWindow;

#include "Physics.h"
using namespace PhysicsCPU;
Physics* pPhysics = NULL;

bool Init() 
{
	pPhysics = new Physics();
	pPhysics->SetGravity(glm::vec3(0, -1, 0));

	int matId = pPhysics->GenMaterial();
	Physics::Material* pMaterial = pPhysics->GetMaterial(matId);
	pMaterial->m_fRestitution = 0.0f;
	pMaterial->m_fFriction = 0.0f;

	int meshId = pPhysics->GenConvexTriMesh();
	Physics::ConvexTriMesh *pMesh = pPhysics->GetConvexTriMesh(meshId);
	pMesh->m_listVertices.push_back(glm::vec3(-1, -1, -1));
	pMesh->m_listVertices.push_back(glm::vec3(+1, -1, -1));
	pMesh->m_listVertices.push_back(glm::vec3(+1, +1, -1));
	pMesh->m_listVertices.push_back(glm::vec3(-1, +1, -1));
	pMesh->m_listVertices.push_back(glm::vec3(-1, -1, +1));
	pMesh->m_listVertices.push_back(glm::vec3(+1, -1, +1));
	pMesh->m_listVertices.push_back(glm::vec3(+1, +1, +1));
	pMesh->m_listVertices.push_back(glm::vec3(-1, +1, +1));
	Physics::Triangle triBack1;
	triBack1.m_nAId = 0;
	triBack1.m_nBId = 1;
	triBack1.m_nCId = 3;
	Physics::Triangle triBack2;
	triBack2.m_nAId = 1;
	triBack2.m_nBId = 2;
	triBack2.m_nCId = 3;
	Physics::Triangle triFront1;
	triFront1.m_nAId = 4;
	triFront1.m_nBId = 5;
	triFront1.m_nCId = 6;
	Physics::Triangle triFront2;
	triFront2.m_nAId = 6;
	triFront2.m_nBId = 7;
	triFront2.m_nCId = 4;
	Physics::Triangle triLeft1;
	triLeft1.m_nAId = 0;
	triLeft1.m_nBId = 3;
	triLeft1.m_nCId = 7;
	Physics::Triangle triLeft2;
	triLeft2.m_nAId = 0;
	triLeft2.m_nBId = 7;
	triLeft2.m_nCId = 4;
	Physics::Triangle triRight1;
	triRight1.m_nAId = 1;
	triRight1.m_nBId = 5;
	triRight1.m_nCId = 2;
	Physics::Triangle triRight2;
	triRight2.m_nAId = 2;
	triRight2.m_nBId = 5;
	triRight2.m_nCId = 6;
	Physics::Triangle triTop1;
	triTop1.m_nAId = 3;
	triTop1.m_nBId = 2;
	triTop1.m_nCId = 6;
	Physics::Triangle triTop2;
	triTop2.m_nAId = 6;
	triTop2.m_nBId = 7;
	triTop2.m_nCId = 3;
	Physics::Triangle triBottom1;
	triBottom1.m_nAId = 4;
	triBottom1.m_nBId = 5;
	triBottom1.m_nCId = 0;
	Physics::Triangle triBottom2;
	triBottom2.m_nAId = 0;
	triBottom2.m_nBId = 5;
	triBottom2.m_nCId = 1;
	pMesh->m_listTriangles.push_back(triBack1);
	pMesh->m_listTriangles.push_back(triBack2);
	pMesh->m_listTriangles.push_back(triFront1);
	pMesh->m_listTriangles.push_back(triFront2);
	pMesh->m_listTriangles.push_back(triLeft1);
	pMesh->m_listTriangles.push_back(triLeft2);
	pMesh->m_listTriangles.push_back(triRight1);
	pMesh->m_listTriangles.push_back(triRight2);
	pMesh->m_listTriangles.push_back(triTop1);
	pMesh->m_listTriangles.push_back(triTop2);
	pMesh->m_listTriangles.push_back(triBottom1);
	pMesh->m_listTriangles.push_back(triBottom2);
	pMesh->Update();

	int bodyId = pPhysics->GenRigidBody();
	Physics::RigidBody* pRigidBody = pPhysics->GetRigidBody(bodyId);
	pRigidBody->m_fMass = 0.0f;
	pRigidBody->m_v3Position = glm::vec3(0, 0, 0);
	pRigidBody->m_v3Axis = glm::vec3(1, 0, 0);
	pRigidBody->m_fAngle = 0.0f;
	pRigidBody->m_nConvexTriMeshId = meshId;
	pRigidBody->m_nMaterialId = matId;

	return true;
}

void Exit() 
{
	delete pPhysics;
	pPhysics = NULL;
}

void Update() 
{
	int nWidth, nHeight;
	glfwGetFramebufferSize(pWindow, &nWidth, &nHeight);
	glViewport(0, 0, nWidth, nHeight);

	glClearColor(0.5f, 0.5f, 1.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	;

	glfwSwapBuffers(pWindow);
	glfwPollEvents();
}

int main(int argc, char **argv) 
{
	glfwInit();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

	int nWindowWidth = 1280;
	int nWindowHeight = 720;
	pWindow = glfwCreateWindow(nWindowWidth, nWindowHeight, "PhysicsCPU - Test01", NULL, NULL);
	glfwMakeContextCurrent(pWindow);
	glfwSwapInterval(0);

	// center of monitor
	GLFWmonitor *pPrimaryMonitor = glfwGetPrimaryMonitor();
	const GLFWvidmode *pMode = glfwGetVideoMode(pPrimaryMonitor);
	int xpos = (pMode->width - nWindowWidth) / 2;
	int ypos = (pMode->height - nWindowHeight) / 2;
	glfwSetWindowPos(pWindow, xpos, ypos);

	if (false == Init()) 
	{
		return EXIT_FAILURE;
	}

	while (false == glfwWindowShouldClose(pWindow)) 
	{
		Update();
	}

	Exit();
	
	glfwTerminate();

	return EXIT_SUCCESS;
}