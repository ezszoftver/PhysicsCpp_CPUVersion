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