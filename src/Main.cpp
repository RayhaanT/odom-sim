#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <iostream>
#include <OpenGLHeaders/ShaderClass.h>
#include <OpenGLHeaders/texture.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#define STB_IMAGE_IMPLEMENTATION
#define ARRAY_SIZE(array) (sizeof((array)) / sizeof((array[0])))
#include <GLFW/stb_image.h>
#include <vector>
#include <odom.h>
#include <chrono>

float deltaTime = 0.0f;
float lastFrame = 0.0f;

const float dimension = 800;

glm::mat4 zoom;
double zoomScaleFactor = 1;
const float zoomStep = 0.05f;

unsigned int driveVAO;
unsigned int driveVBO;

XDrive chassis;

void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
	glViewport(0, 0, width, height);
}

glm::vec2 screenToWorldCoordinates(glm::vec2 screenPos) {
	float xpos = screenPos.x;
	float ypos = screenPos.y;
	xpos -= dimension / 2;
	ypos -= dimension / 2;
	ypos = -ypos;
	screenPos = glm::vec2(xpos, ypos) * (1.0f / dimension);
	screenPos *= (float)(1.0f / zoomScaleFactor) * 2.0f;
	//screenPos = glm::vec3(glm::vec4(screenPos, 0.0f, 0.0f) * pan);
	return screenPos;
}

glm::vec2 screenToWorldCoordinates(float x, float y) {
	return screenToWorldCoordinates(glm::vec2(x, y));
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
	if(yoffset > 0) {
		zoom = glm::scale(zoom, glm::vec3(1.0f + zoomStep));
		zoomScaleFactor *= (1 + zoomStep);
	}
	else {
		zoom = glm::scale(zoom, glm::vec3(1.0f - zoomStep));
		zoomScaleFactor *= (1 - zoomStep);
	}
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
	if (glfwGetKey(window, GLFW_KEY_W)) {
		// forward
	}
	if (glfwGetKey(window, GLFW_KEY_S)) {
		// backward
	}
	if (glfwGetKey(window, GLFW_KEY_A)) {
		// left
	}
	if (glfwGetKey(window, GLFW_KEY_D)) {
		// right
	}
}

void mouse_callback(GLFWwindow *window, double xpos, double ypos) {

}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (action == GLFW_PRESS)
		{
		}
	}
}

int main()
{
	//Initialize GLFW
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	//Create a GLFW Window
	GLFWwindow *window = glfwCreateWindow(dimension, dimension, "Odometry", NULL, NULL);
	glfwMakeContextCurrent(window);

	//glad init: intializes all OpenGL function pointers
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	//Set size of rendering window
	glViewport(0, 0, dimension, dimension);

	void framebuffer_size_callback(GLFWwindow * window, int width, int height);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);	
	
	float squareVerticesTextured[] {
		//Vertices            //Texture coords
		-1.0f, -1.0f, 0.0f,   0.0f, 1.0f,
		-1.0f,  1.0f, 0.0f,   0.0f, 0.0f,
		1.0f,  -1.0f, 0.0f,   1.0f, 1.0f,
		1.0f,  -1.0f, 0.0f,   1.0f, 1.0f,
		-1.0f,  1.0f, 0.0f,   0.0f, 0.0f,
		1.0f,   1.0f, 0.0f,   1.0f, 0.0f
	};

	float squareVertices[] {
		//Vertices         
		-1.0f, -1.0f, 0.0f,
		-1.0f,  1.0f, 0.0f,
		1.0f,  -1.0f, 0.0f,
		1.0f,  -1.0f, 0.0f,
		-1.0f,  1.0f, 0.0f,
		1.0f,   1.0f, 0.0f
	};

	//Background data
	unsigned int backgroundVAO;
	glGenVertexArrays(1, &backgroundVAO);
	glBindVertexArray(backgroundVAO);
	unsigned int backgroundVBO;
	glGenBuffers(1, &backgroundVBO);
	glBindBuffer(GL_ARRAY_BUFFER, backgroundVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(squareVerticesTextured), squareVerticesTextured, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	//Create a Vertex Array Object
	glGenVertexArrays(1, &driveVAO);
	glBindVertexArray(driveVAO);
	
	glGenBuffers(1, &driveVBO);
	glBindBuffer(GL_ARRAY_BUFFER, driveVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(squareVertices), squareVertices, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
	glEnableVertexAttribArray(0);

	glm::vec3 objectColour = glm::vec3(1.0f, 0.5f, 0.31f);
	glm::vec3 lightColour = glm::vec3(1.0f, 1.0f, 1.0f);

	Shader driveShader("Shaders/Vertex.vs", "Shaders/Fragment.fs");

	Shader backgroundShader("Shaders/VertexTexture.vs", "Shaders/FragmentTexture.fs");

	backgroundShader.use();
	backgroundShader.setInt("text", 1);
	unsigned int backgroundTexture;
	loadTexture(backgroundTexture, "textures/VexField.png");
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, backgroundTexture);

	glEnable(GL_DEPTH_TEST);
	//Set mouse input callback function
	void mouse_callback(GLFWwindow * window, double xpos, double ypos);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetKeyCallback(window, key_callback);
	glPointSize(8);
	glLineWidth(3);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//Render Loop
	while (!glfwWindowShouldClose(window))
	{
		if (glfwGetKey(window, GLFW_KEY_ESCAPE)) {
			glfwSetWindowShouldClose(window, true);
		}

		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Pass matrices to the shader through a uniform
		driveShader.use();
		driveShader.setMat4("model", zoom * chassis.getMatrix());

		driveShader.setVec3("colour", glm::vec3(1.0f));
		glBindVertexArray(driveVAO);
		glBindBuffer(GL_ARRAY_BUFFER, driveVBO);
		glDrawArrays(GL_TRIANGLES, 0, ARRAY_SIZE(squareVertices));
		// glDrawArrays(GL_POINTS, 0, numberOfPoints);

		//Draw Background
		glActiveTexture(GL_TEXTURE1);
		backgroundShader.use();
		backgroundShader.setMat4("model", zoom);
		glBindVertexArray(backgroundVAO);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDrawArrays(GL_TRIANGLES, 0, ARRAY_SIZE(squareVerticesTextured));

		//Swap buffer and poll IO events
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}
