#include <stdlib.h>

#include <Windows.h>
#include <gl/GL.h>
#include <GLFW/glfw3.h>
GLFWwindow* pWindow;

#include "Physics.h"
using namespace PhysicsCPU;
Physics* pPhysics = NULL;

#include "Camera.h"
Camera m_Camera;

float m_fElapsedTime = 0.0f;
float m_fCurrentTime = 0.0f;
int nFPS = 0;
float fSec = 0.0f;

void CreateCube(glm::vec3 v3Position, glm::quat quatOrientation, glm::vec3 v3HalfSize, float fMass)
{
	int matId = pPhysics->GenMaterial();
	Physics::Material* pMaterial = pPhysics->GetMaterial(matId);
	pMaterial->m_fRestitution = 0.0f;
	pMaterial->m_fFriction = 0.0f;

	int meshId = pPhysics->GenConvexTriMesh();
	Physics::ConvexTriMesh* pMesh = pPhysics->GetConvexTriMesh(meshId);
	pMesh->m_listVertices.push_back(glm::vec3(-1 * v3HalfSize.x, -1 * v3HalfSize.y, -1 * v3HalfSize.z));
	pMesh->m_listVertices.push_back(glm::vec3(+1 * v3HalfSize.x, -1 * v3HalfSize.y, -1 * v3HalfSize.z));
	pMesh->m_listVertices.push_back(glm::vec3(+1 * v3HalfSize.x, +1 * v3HalfSize.y, -1 * v3HalfSize.z));
	pMesh->m_listVertices.push_back(glm::vec3(-1 * v3HalfSize.x, +1 * v3HalfSize.y, -1 * v3HalfSize.z));
	pMesh->m_listVertices.push_back(glm::vec3(-1 * v3HalfSize.x, -1 * v3HalfSize.y, +1 * v3HalfSize.z));
	pMesh->m_listVertices.push_back(glm::vec3(+1 * v3HalfSize.x, -1 * v3HalfSize.y, +1 * v3HalfSize.z));
	pMesh->m_listVertices.push_back(glm::vec3(+1 * v3HalfSize.x, +1 * v3HalfSize.y, +1 * v3HalfSize.z));
	pMesh->m_listVertices.push_back(glm::vec3(-1 * v3HalfSize.x, +1 * v3HalfSize.y, +1 * v3HalfSize.z));
	Physics::Triangle triBack1;
	triBack1.m_nAId = 3;
	triBack1.m_nBId = 1;
	triBack1.m_nCId = 0;
	Physics::Triangle triBack2;
	triBack2.m_nAId = 3;
	triBack2.m_nBId = 2;
	triBack2.m_nCId = 1;
	Physics::Triangle triFront1;
	triFront1.m_nAId = 4;
	triFront1.m_nBId = 5;
	triFront1.m_nCId = 6;
	Physics::Triangle triFront2;
	triFront2.m_nAId = 6;
	triFront2.m_nBId = 7;
	triFront2.m_nCId = 4;
	Physics::Triangle triLeft1;
	triLeft1.m_nAId = 7;
	triLeft1.m_nBId = 3;
	triLeft1.m_nCId = 0;
	Physics::Triangle triLeft2;
	triLeft2.m_nAId = 4;
	triLeft2.m_nBId = 7;
	triLeft2.m_nCId = 0;
	Physics::Triangle triRight1;
	triRight1.m_nAId = 2;
	triRight1.m_nBId = 5;
	triRight1.m_nCId = 1;
	Physics::Triangle triRight2;
	triRight2.m_nAId = 6;
	triRight2.m_nBId = 5;
	triRight2.m_nCId = 2;
	Physics::Triangle triTop1;
	triTop1.m_nAId = 6;
	triTop1.m_nBId = 2;
	triTop1.m_nCId = 3;
	Physics::Triangle triTop2;
	triTop2.m_nAId = 3;
	triTop2.m_nBId = 7;
	triTop2.m_nCId = 6;
	Physics::Triangle triBottom1;
	triBottom1.m_nAId = 0;
	triBottom1.m_nBId = 5;
	triBottom1.m_nCId = 4;
	Physics::Triangle triBottom2;
	triBottom2.m_nAId = 1;
	triBottom2.m_nBId = 5;
	triBottom2.m_nCId = 0;
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
	pRigidBody->m_fMass = fMass;
	pRigidBody->m_v3Position = v3Position;
	pRigidBody->m_quatOrientation = quatOrientation;
	pRigidBody->m_nConvexTriMeshId = meshId;
	pRigidBody->m_nMaterialId = matId;

	pRigidBody->m_fLinearDamping = 0.5f;
	pRigidBody->m_fAngularDamping = 0.5f;
}

bool Init() 
{
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPointSize(10.0f);
	glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

	// physics
	pPhysics = new Physics();
	pPhysics->SetGravity(glm::vec3(0.0f, -0.1f, 0.0f));

	CreateCube(glm::vec3(0, -1, 0), glm::quat(glm::radians(glm::vec3(-0.0f, 0.0f, 0.0f))), glm::vec3(3, 0.1, 3), 0.0f);

	CreateCube(glm::vec3(0, 0.75f, 0), glm::quat(glm::radians(glm::vec3(0.0f, 0.0f, 0.0f))), glm::vec3(0.5f, 0.5f, 0.5f), 1.0f);

	CreateCube(glm::vec3(0, 2.0f, 0), glm::quat(glm::radians(glm::vec3(0.0f, 0.0f, 0.0f))), glm::vec3(0.5f, 0.5f, 0.5f), 1.0f);

	// camera
	m_Camera.Init(glm::vec3(10, 1, 0), glm::vec3(0, 0, 0));
	// hide cursor
	glfwSetInputMode(pWindow, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

	return true;
}

void Exit() 
{
	// show cursor
	glfwSetInputMode(pWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	delete pPhysics;
	pPhysics = NULL;
}

void DebugDraw()
{
	// draw triangles
	glPushMatrix();
	glBegin(GL_TRIANGLES);
	glColor4f(1.0f, 0, 0, 1.0f);
	for (int i = 0; i < pPhysics->NumRigidBodies(); i++)
	{
		struct Physics::RigidBody* pRigidBody = pPhysics->GetRigidBody(i);
		struct Physics::ConvexTriMesh* pConvexTriMesh = pPhysics->GetConvexTriMesh(pRigidBody->m_nConvexTriMeshId);
		
		for (int j = 0; j < (int)pConvexTriMesh->m_listTriangles.size(); j++)
		{
			struct Physics::Triangle* pTriangle = &(pConvexTriMesh->m_listTriangles[j]);
		
			glm::vec3 v3LocalA = pConvexTriMesh->m_listVertices[pTriangle->m_nAId];
			glm::vec3 v3LocalB = pConvexTriMesh->m_listVertices[pTriangle->m_nBId];
			glm::vec3 v3LocalC = pConvexTriMesh->m_listVertices[pTriangle->m_nCId];
		
			glm::vec3 v3A = glm::vec3(pRigidBody->m_matWorld * glm::vec4(v3LocalA, 1));
			glm::vec3 v3B = glm::vec3(pRigidBody->m_matWorld * glm::vec4(v3LocalB, 1));
			glm::vec3 v3C = glm::vec3(pRigidBody->m_matWorld * glm::vec4(v3LocalC, 1));
		
			glVertex3f(v3A.x, v3A.y, v3A.z);
			glVertex3f(v3B.x, v3B.y, v3B.z);
			glVertex3f(v3C.x, v3C.y, v3C.z);
		}

	}
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glEnd();
	glPopMatrix();

	// draw hits
	/*glPushMatrix();
	for (int i = 0; i < pPhysics->NumRigidBodies(); i++) 
	{
		struct Physics::RigidBody* pRigidBody = pPhysics->GetRigidBody(i);
		struct Physics::Hits *pHits = &(pPhysics->m_listHits[pRigidBody->m_nHitId]);

		for (int j = 0; j < (int)pHits->m_listHits.size(); j++)
		{
			struct Physics::Hit *pHit = &(pHits->m_listHits[j]);
			glm::vec3 v3Point = pHit->m_v3PointInWorld;

			glBegin(GL_POINTS);
			{
				glColor4f(0, 1.0f, 0, 1.0f);
				glVertex3f(v3Point.x, v3Point.y, v3Point.z);
				glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
			}
			glEnd();

			glm::vec3 v3A = pHit->m_v3PointInWorld;
			glm::vec3 v3B = v3A + (pHit->m_v3NormalInWorld * pHit->m_fPenetration);

			glLineWidth(5.0);
			glBegin(GL_LINES);
			{
				glColor4f(0, 1.0f, 0, 1.0f);
				glVertex3f(v3A.x, v3A.y, v3A.z);
				glVertex3f(v3B.x, v3B.y, v3B.z);
				glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
			}
			glEnd();
			glLineWidth(1.0);
		}
	}
	glPopMatrix();*/
}

void Update() 
{
	// esc exit
	if (GLFW_PRESS == glfwGetKey(pWindow, GLFW_KEY_ESCAPE))
	{
		glfwSetWindowShouldClose(pWindow, true);
		return;
	}

	// update
	// delta time
	m_fElapsedTime = m_fCurrentTime;
	m_fCurrentTime = (float)glfwGetTime();
	float dt = m_fCurrentTime - m_fElapsedTime;

	if (dt <= 0.0f) { dt = 1.0f / 60.0f; }
	if (dt > (1.0f / 10.0f)){ dt = 1.0f / 10.0f; }

	// fps
	// print fps
	nFPS++;
	fSec += dt;
	if (fSec >= 1.0f)
	{
		std::string strTitle = "FPS: " + std::to_string(nFPS);
		glfwSetWindowTitle(pWindow, strTitle.c_str());

		nFPS = 0;
		fSec = 0.0f;
	}

	// physics
	pPhysics->Update(dt);

	// camera
	m_Camera.Update(dt);
	if (GLFW_PRESS == glfwGetKey(pWindow, GLFW_KEY_W)) 
	{
		m_Camera.MoveW();
	}
	if (GLFW_PRESS == glfwGetKey(pWindow, GLFW_KEY_S))
	{
		m_Camera.MoveS();
	}
	if (GLFW_PRESS == glfwGetKey(pWindow, GLFW_KEY_A))
	{
		m_Camera.MoveA();
	}
	if (GLFW_PRESS == glfwGetKey(pWindow, GLFW_KEY_D))
	{
		m_Camera.MoveD();
	}
	double fMouseX, fMouseY;
	glfwGetCursorPos(pWindow, &fMouseX, &fMouseY);
	glfwSetCursorPos(pWindow, 320, 240);
	int fDeltaMouseX = (int)fMouseX - 320;
	int fDeltaMouseY = (int)fMouseY - 240;
	m_Camera.Rotate(fDeltaMouseX, fDeltaMouseY);

	glm::vec3 v3CameraPos = m_Camera.GetPos();
	glm::vec3 v3CameraAt = m_Camera.GetAt();
	glm::vec3 v3CameraDir = glm::normalize(v3CameraAt - v3CameraPos);

	// draw
	int nWidth, nHeight;
	glfwGetFramebufferSize(pWindow, &nWidth, &nHeight);
	glViewport(0, 0, nWidth, nHeight);

	glClearColor(0.5f, 0.5f, 1.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// camera
	glm::mat4 matCameraView = glm::lookAtRH(v3CameraPos, v3CameraAt, glm::vec3(0, 1, 0));
	glm::mat4 matCameraProj = glm::perspectiveRH(glm::radians(45.0f), (float)nWidth / (float)nHeight, 0.01f, 1000.0f);

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(glm::value_ptr(matCameraProj));

	glMatrixMode(GL_MODELVIEW);
	glm::mat4 matWorld = glm::mat4(1.0f);
	glLoadMatrixf(glm::value_ptr(matCameraView * matWorld));

	DebugDraw();

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