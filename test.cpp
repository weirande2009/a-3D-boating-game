#include "./include/glad/glad.h"
//#include <irrklang32/irrklang.h>
//#pragma comment(lib, "irrklang.lib")
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "include/learnopengl/camera.h"
#include "include/learnopengl/filesystem.h"
#include "include/learnopengl/model.h"
#include "include/learnopengl/shader_m.h"
#include "include/stb_image.h"

#include "player.h"
#include "fixed_camera.h"
#include "config.h"
#include "fruit.h"

#include <iostream>
#include <btBulletDynamicsCommon.h>
#pragma comment(lib, "assimp.lib")

/* 地面正方体顶点坐标 */
float vertices[] = {
		-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, 0.0f,
		 0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, 0.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, 0.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, 0.0f,
		-0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, 0.0f,
		-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, 0.0f,

		-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, 0.0f,
		 0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, 0.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, 0.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, 0.0f,
		-0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, 0.0f,
		-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, 0.0f,

		-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, 1.0f,
		-0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, 1.0f,
		-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, 1.0f,
		-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, 1.0f,
		-0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, 1.0f,
		-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, 1.0f,

		 0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, 0.0f,
		 0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, 0.0f,
		 0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, 0.0f,
		 0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, 0.0f,
		 0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, 0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, 0.0f,

		-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, 1.0f,
		 0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, 1.0f,
		 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, 1.0f,
		 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, 1.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, 1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, 1.0f,

		-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, 1.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, 1.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, 1.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, 1.0f,
		-0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, 1.0f,
		-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, 1.0f
};

float groundVertices[] = {
	30, 0, 30,
	30, 0, -30,
	-30, 0, -30,
	30, 0, 30,
	-30, 0, 30,
	-30, 0, -30
};

glm::vec3 cubePositions[] = {
	glm::vec3(0.0f,  -1.0f, 0.0f)		// 地面位置，与world中内置地面坐标信息一致
	//glm::vec3(1.1f,  100.0f,  0.0f),
	//glm::vec3(1.5f, 35.2f, 0.0f),
	//glm::vec3(1.4f, 50.0f, 0.0f),
	//glm::vec3(1.3f, 80.4f, 0.0f),
	//glm::vec3(1.4f,  30.0f, 0.0f),
	//glm::vec3(1.3f, 20.0f, 0.0f),
	//glm::vec3(1.5f,  2.0f, -2.5f),
	//glm::vec3(1.5f,  5.2f, -1.5f),
	//glm::vec3(-1.3f,  1.0f, -1.5f)
};

// ------------------------------------------
// 全局变量
// ------------------------------------------
// 汽车
Player player(glm::vec3(0.0f, 0.05f, 0.0f));

// 相机
glm::vec3 cameraPos(0.0f, 4.0f, 20.0f);
Camera camera(cameraPos);
FixedCamera fixedCamera(cameraPos);
bool isCameraFixed = false;

// 光照相关属性
glm::vec3 lightPos(-1.0f, 1.0f, 1.0f);
glm::vec3 lightDirection = glm::normalize(lightPos);
glm::mat4 lightSpaceMatrix;

// 深度Map的ID
unsigned int depthMap;
unsigned int depthMapFBO;

// 将鼠标设置在屏幕中心
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;
// ------------------------------------------
// 函数声明
// ------------------------------------------
// 深度图配置
void depthMapFBOInit()
{

	glGenFramebuffers(1, &depthMapFBO);
	// 创建深度纹理
	glGenTextures(1, &depthMap);
	glBindTexture(GL_TEXTURE_2D, depthMap);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	float borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
	glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
	// 把生成的深度纹理作为帧缓冲的深度缓冲
	glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// ---------------------------------
// 相机位置更新
// ---------------------------------

void updateFixedCamera()
{
	// 自动逐渐复原Zoom为默认值
	camera.ZoomRecover();

	// 处理相机相对于车坐标系下的向量坐标转换为世界坐标系下的向量
	float angle = glm::radians(-player.getMidValYaw());
	glm::mat4 rotateMatrix(
		cos(angle), 0.0, sin(angle), 0.0,
		0.0, 1.0, 0.0, 0.0,
		-sin(angle), 0.0, cos(angle), 0.0,
		0.0, 0.0, 0.0, 1.0);
	glm::vec3 rotatedPosition = glm::vec3(rotateMatrix * glm::vec4(fixedCamera.getPosition(), 1.0));

	camera.FixView(rotatedPosition + player.getMidValPosition(), fixedCamera.getYaw() + player.getMidValYaw());
}

// ---------------------------------
// 键盘/鼠标监听 // 窗口相关函数
// ---------------------------------
// 按键回调函数，使得一次按键只触发一次事件
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_C && action == GLFW_PRESS) {
		isCameraFixed = !isCameraFixed;
		string info = isCameraFixed ? "切换为固定视角" : "切换为自由视角";
		std::cout << "[CAMERA]" << info << std::endl;
	}
	if (key == GLFW_KEY_X && action == GLFW_PRESS) {
		isPolygonMode = !isPolygonMode;
		if (isPolygonMode) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		else {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
		string info = isPolygonMode ? "切换为线框图渲染模式" : "切换为正常渲染模式";
		std::cout << "[POLYGON_MODE]" << info << std::endl;
	}
}

void handleKeyInput(GLFWwindow* window, float deltaTime)
{
	// esc退出
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (!isCameraFixed) {
		// 相机 WSAD 前后左右 Space上 左Ctrl下
		if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
			camera.ProcessKeyboard(FORWARD, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
			camera.ProcessKeyboard(BACKWARD, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
			camera.ProcessKeyboard(LEFT, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
			camera.ProcessKeyboard(RIGHT, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
			camera.ProcessKeyboard(UP, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
			camera.ProcessKeyboard(DOWN, deltaTime);
	}
	else {
		if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
			fixedCamera.ProcessKeyboard(CAMERA_LEFT, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
			fixedCamera.ProcessKeyboard(CAMERA_RIGHT, deltaTime);
	}

	// 车车移动
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
		player.ProcessKeyboard(CAR_FORWARD, deltaTime);

		// 只有车车动起来的时候才可以左右旋转
		if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
			player.ProcessKeyboard(CAR_LEFT, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
			player.ProcessKeyboard(CAR_RIGHT, deltaTime);

		if (isCameraFixed)
			camera.ZoomOut();
	}
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
		player.ProcessKeyboard(CAR_BACKWARD, deltaTime);

		// 同上
		if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
			player.ProcessKeyboard(CAR_LEFT, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
			player.ProcessKeyboard(CAR_RIGHT, deltaTime);

		if (isCameraFixed)
			camera.ZoomIn();
	}

	// 回调监听按键（一个按键只会触发一次事件）
	glfwSetKeyCallback(window, key_callback);
}

// 鼠标移动
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (!isCameraFixed) {
		if (firstMouse) {
			lastX = xpos;
			lastY = ypos;
			firstMouse = false;
		}

		float xoffset = xpos - lastX;
		float yoffset = lastY - ypos; // 坐标翻转以对应坐标系

		lastX = xpos;
		lastY = ypos;

		camera.ProcessMouseMovement(xoffset, yoffset);
	}
}

// 鼠标滚轮
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}

// 改变窗口大小的回调函数
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// 确保窗口匹配的新窗口尺寸
	glViewport(0, 0, width, height);
}

GLFWwindow* windowInit()
{
	// 初始化配置
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// 创建窗口
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, u8"CrazyFruit | Ceynri", NULL, NULL);
	if (window == NULL) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		system("pause");
		return NULL;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// 令GLFW捕捉用户的鼠标
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	return window;
}

bool init()
{
	// 加载所有OpenGL函数指针
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		system("pause");
		return false;
	}

	// 配置全局openGL状态
	glEnable(GL_DEPTH_TEST);

	return true;
}

class SkyBox {
protected:
	// 天空盒
	unsigned int cubemapTexture;
	unsigned int skyboxVAO, skyboxVBO;
	// 天空盒顶点数据
	const float skyboxVertices[36 * 3] = {
		// positions
		-1.0f, 1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, 1.0f, -1.0f,
		-1.0f, 1.0f, -1.0f,

		-1.0f, -1.0f, 1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f, 1.0f, -1.0f,
		-1.0f, 1.0f, -1.0f,
		-1.0f, 1.0f, 1.0f,
		-1.0f, -1.0f, 1.0f,

		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, 1.0f,
		1.0f, 1.0f, 1.0f,
		1.0f, 1.0f, 1.0f,
		1.0f, 1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f, 1.0f,
		-1.0f, 1.0f, 1.0f,
		1.0f, 1.0f, 1.0f,
		1.0f, 1.0f, 1.0f,
		1.0f, -1.0f, 1.0f,
		-1.0f, -1.0f, 1.0f,

		-1.0f, 1.0f, -1.0f,
		1.0f, 1.0f, -1.0f,
		1.0f, 1.0f, 1.0f,
		1.0f, 1.0f, 1.0f,
		-1.0f, 1.0f, 1.0f,
		-1.0f, 1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f, 1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f, 1.0f,
		1.0f, -1.0f, 1.0f
	};
	// 天空盒的面数据
	const vector<std::string> faces{
		FileSystem::getPath("asset/textures/skybox/right.tga"),
		FileSystem::getPath("asset/textures/skybox/left.tga"),
		FileSystem::getPath("asset/textures/skybox/top.tga"),
		FileSystem::getPath("asset/textures/skybox/bottom.tga"),
		FileSystem::getPath("asset/textures/skybox/front.tga"),
		FileSystem::getPath("asset/textures/skybox/back.tga")
	};

	// 将六份纹理加载为一个cubemap纹理
	unsigned int loadCubemap(vector<std::string> faces)
	{
		unsigned int textureID;
		glGenTextures(1, &textureID);
		glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

		int width, height, nrChannels;
		for (unsigned int i = 0; i < faces.size(); i++) {
			unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
			if (data) {
				glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
				stbi_image_free(data);
			}
			else {
				std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
				stbi_image_free(data);
			}
		}
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

		return textureID;
	}

public:

	SkyBox() {
		// skybox VAO
		glGenVertexArrays(1, &skyboxVAO);
		glGenBuffers(1, &skyboxVBO);
		glBindVertexArray(skyboxVAO);
		glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

		// 纹理加载
		cubemapTexture = loadCubemap(faces);
	}
	void render(Shader& shader)
	{
		// viewMatrix 通过构造，移除相机的移动
		glm::mat4 viewMatrix = glm::mat4(glm::mat3(camera.GetViewMatrix()));
		// 投影
		glm::mat4 projMatrix = camera.GetProjMatrix((float)SCR_WIDTH / (float)SCR_HEIGHT);

		shader.setMat4("view", viewMatrix);
		shader.setMat4("projection", projMatrix);

		glBindVertexArray(skyboxVAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glBindVertexArray(0);
	}

};

// 设置光照相关属性
void setLight(Shader& shader)
{
	shader.setVec3("viewPos", camera.Position);
	shader.setVec3("light.direction", lightDirection);
	shader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

	glActiveTexture(GL_TEXTURE15);
	glBindTexture(GL_TEXTURE_2D, depthMap);
}

// 渲染玩家
void renderPlayer(Model& model, glm::mat4 modelMatrix, Shader& shader)
{
	modelMatrix = glm::rotate(modelMatrix, glm::radians(player.getYaw() - player.getDelayYaw() / 2), WORLD_UP);
	// 抵消模型原本自带的旋转
	modelMatrix = glm::rotate(modelMatrix, glm::radians(180.0f), WORLD_UP);
	// 调整模型大小
   // modelMatrix = glm::scale(modelMatrix, glm::vec3(0.004f, 0.004f, 0.004f));

	// 应用变换矩阵
	shader.setMat4("model", modelMatrix);

	model.Draw(shader);
}

// 渲染摄像机
void renderCamera(Model& model, glm::mat4 modelMatrix, Shader& shader)
{
	modelMatrix = glm::rotate(modelMatrix, glm::radians(fixedCamera.getYaw() + player.getYaw() / 2), WORLD_UP);
	modelMatrix = glm::translate(modelMatrix, cameraPos);
	modelMatrix = glm::scale(modelMatrix, glm::vec3(0.01f, 0.01f, 0.01f));

	// 应用变换矩阵
	shader.setMat4("model", modelMatrix);

	model.Draw(shader);
}

// 渲染场景
void renderScene(Model& model, Shader& shader)
{
	// 视图转换
	glm::mat4 viewMatrix = camera.GetViewMatrix();
	shader.setMat4("view", viewMatrix);
	// 模型转换
	glm::mat4 modelMatrix = glm::mat4(1.0f);
	modelMatrix = glm::scale(modelMatrix, glm::vec3(10, 10, 10));
	modelMatrix = glm::translate(modelMatrix, glm::vec3(-1.5, -0.1, -2));
	shader.setMat4("model", modelMatrix);
	// 投影转换
	glm::mat4 projMatrix = camera.GetProjMatrix((float)SCR_WIDTH / (float)SCR_HEIGHT);
	shader.setMat4("projection", projMatrix);

	model.Draw(shader);
}
void renderPlayerAndCamera(Model& carModel, Model& cameraModel, Shader& shader)
{
	// 视图转换
	glm::mat4 viewMatrix = camera.GetViewMatrix();
	shader.setMat4("view", viewMatrix);
	// 投影转换
	glm::mat4 projMatrix = camera.GetProjMatrix((float)SCR_WIDTH / (float)SCR_HEIGHT);
	shader.setMat4("projection", projMatrix);

	// -------
	// 层级建模

	// 模型转换
	glm::mat4 modelMatrix = glm::mat4(1.0f);
	modelMatrix = glm::translate(modelMatrix, player.getMidValPosition());
	modelMatrix = glm::rotate(modelMatrix, glm::radians(player.getDelayYaw() / 2), WORLD_UP);

	// 渲染汽车
	renderPlayer(carModel, modelMatrix, shader);

	// 由于mat4作函数参数时为值传递，故不需要备份modelMatrix

	// 渲染相机
	renderCamera(cameraModel, modelMatrix, shader);
}

int main()
{
	// ------------------------------
	// 初始化
	// ------------------------------
	//irrklang::ISoundEngine* engine = irrklang::createIrrKlangDevice();

	// 窗口初始化
	GLFWwindow* window = windowInit();
	// OpenGL初始化
	bool isInit = init();
	if (window == NULL || !isInit) {
		return -1;
	}
	// 深度Map的FBO配置
	depthMapFBOInit();
	// 天空盒的配置
	SkyBox skybox;
	// ------------------------------
	// 构建和编译着色器
	// ------------------------------

	// 为所有物体添加光照和阴影的shader
	Shader shader("shader/light_and_shadow.vs", "shader/light_and_shadow.fs");
	// 从太阳平行光角度生成深度信息的shader
	Shader depthShader("shader/shadow_mapping_depth.vs", "shader/shadow_mapping_depth.fs");
	// 天空盒shader
	Shader skyboxShader("shader/skybox.vs", "shader/skybox.fs");


	PhysicalWorld world;

	// ------------------------------
	// 模型加载
	// ------------------------------
	// 地面
	Object_info ground_info;
	ground_info.setMass(0.0f);
	ground_info.setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
	Model ground_model(FileSystem::getPath("asset/models/obj/test/heliu.obj"), ground_info);
	Fruit ground(ground_model);
	ground.createFruitInPhysicalWorld(world);

	// 玩家模型
	//Object_info player_info;
	//player_info.setPosition(glm::vec3(5.0f, 5.0f, 0.0f));
	//Model playerModel(FileSystem::getPath("asset/models/obj/Player/player.obj"), player_info);
	//playerModel.CreateObjectInPhysicalWorld(world);
	//// 相机模型
	//Object_info camera_info;
	//Model cameraModel(FileSystem::getPath("asset/models/obj/camera-cube/camera-cube.obj"), camera_info);
	//// 静态场景模型
	//Object_info scene_info;
	//scene_info.setMass(0.0f);
	//Model sceneModel(FileSystem::getPath("asset/models/obj/Scene/scene.obj"), scene_info);
	// 水果
	FruitManager fruitManager(2, 2);


	shader.use();
	shader.setInt("diffuseTexture", 0);
	shader.setInt("shadowMap", 15); // 这里的15是指"GL_TEXTURE15"，需要与后面的对应

	skyboxShader.use();
	skyboxShader.setInt("skybox", 0);

	// timing 用来平衡不同电脑渲染水平所产生的速度变化
	float deltaTime = 0.0f;
	float lastFrame = 0.0f;


	// ---------------------------------
	// 循环渲染
	// ---------------------------------
	/* 记录出界的刚体下标的数组，便于删除对应在OpenGL世界中的物体 */
	std::vector<int> deleteObjIndex;

	while (!glfwWindowShouldClose(window)) {

		// 计算一帧的时间长度以便于使帧绘制速度均匀
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		world.stepSimulation(deleteObjIndex);

		// 监听按键
		handleKeyInput(window, deltaTime);
		//更新
//		fruitManager.deleteFruit(deleteObjIndex);
		moveObjToStart(world, deleteObjIndex);
		fruitManager.createFruit(world, deltaTime);
		fruitManager.update(world);

		// 渲染背景
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		// ---------------------------------
		// 模型渲染
		// ---------------------------------

		shader.use();

		// 设置光照相关属性
		setLight(shader);

		// 切换为相机固定时，需要每次帧修改相机状态
		if (isCameraFixed) {
			updateFixedCamera();
		}


		fruitManager.render(shader, camera);

		shader.use();
		ground.render(shader, camera);


		// 交换缓冲区和调查IO事件（按下的按键,鼠标移动等）
		glfwSwapBuffers(window);

		// 轮询事件
		glfwPollEvents();

		deleteObjIndex.clear();

	}
}