#include <stdlib.h>

#include <Windows.h>
#include <gl/GL.h>
#include <GLFW/glfw3.h>
GLFWwindow* pWindow;
int nWidth, nHeight;

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

	pWindow = glfwCreateWindow(800, 600, "PhysicsCPU - Test01", NULL, NULL);
	glfwMakeContextCurrent(pWindow);
	glfwSwapInterval(0);

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