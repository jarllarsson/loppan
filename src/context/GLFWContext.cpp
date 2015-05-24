#include "GLFWContext.h"
#include <GLFW/glfw3.h>

GLFWContext* GLFWContext::m_instance = NULL;

GLFWContext::GLFWContext(const string& p_title, int p_width, int p_height)
{
	m_closeFlag = false;
	m_sizeDirty = false;
	m_width = max(1, p_width);
	m_height = max(1, p_height);
	m_title = p_title;

	// Init GLFW
	if (!glfwInit())
		exit(EXIT_FAILURE);

	// Register a callback for GLFW errors
	glfwSetErrorCallback(errorCallback);

	// Create the window
	m_windowHandle = glfwCreateWindow(m_width, m_height, m_title.c_str(), NULL, NULL);
	if (!m_windowHandle)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	// Set window to be used as current OpenGL context
	glfwMakeContextCurrent(m_windowHandle);

	m_instance = this;
}

GLFWContext::~GLFWContext()
{
	glfwDestroyWindow(m_windowHandle);
	glfwTerminate();
}

GLFWContext* GLFWContext::getInstance()
{
	return m_instance;
}

void GLFWContext::close()
{
	m_closeFlag = true;
}

void GLFWContext::resize(int p_w, int p_h, bool p_update)
{
	m_width = max(1, p_w);
	m_height = max(1, p_h);



}

void GLFWContext::setTitle(const string& p_title)
{
	m_title = p_title;
}

void GLFWContext::updateTitle(const string& p_appendMsg /*= ""*/)
{

}

bool GLFWContext::closeRequested() const
{
	return m_closeFlag;
}

bool GLFWContext::isSizeDirty()
{

}

pair<int, int> GLFWContext::getSize()
{

}

void GLFWContext::addSubProcess(IContextProcessable* p_proc)
{

}

void GLFWContext::removeSubProcessEntry(const IContextProcessable* p_proc)
{

}

bool GLFWContext::runSubProcesses(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

}

void GLFWContext::errorCallback(int error, const char* description)
{
	fputs(description, stderr);
}
