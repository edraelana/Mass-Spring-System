#include <glad\glad.h>
#include <GLFW\glfw3.h>
#include <glm\ext.hpp>
#include <iostream>
#include <vector>
#include <ctime>

using namespace std;
using namespace glm;

int screenWidth = 1000;
int screenHeight = 1000;

unsigned int pointMassVBO;
unsigned int pointMassVAO;
int num_point_masses;
float point_radius = 0.05;

unsigned int springVBO;
unsigned int springVAO;

unsigned int vertexShader;

unsigned int pointMassFragmentShader;

unsigned int pointMassShaderProgram;

// Simulation Structs
struct PointMass
{
	int index;
	vec3 pos;
	vec3 velocity;
	vec3 netForce;
	float weight;
};

struct Spring
{
	// These are the indices in the Point Mass Vector that
	// correspond to specific point masses
	int i;
	int j;
	// Spring constant
	float k;
	// Rest Length
	float l;
};

/*
	CAMERA CLASS TAKEN FROM: learnopengl.com
*/

// Not an infinite plane, has a fixed radius
struct Plane
{

	vec3 origin = vec3(0.0, -1.0, 0.0);
	vec3 normal = vec3(0.0, 1.0, 0.0);
	float radius = 20.0;

	bool intersect(vec3 pos)
	{
		float proj = dot((pos - origin), normal);
		if (proj < 0) {
			vec3 proj_point = pos - proj * normal;
			if (length(proj_point - origin) <= radius)
				return true;
		}
		return false;
	}
} ground;

struct Camera
{
	float cameraSpeed = 0.05f;

	float yaw = 270.0f;
	float pitch = 0.0f;

	vec3 cameraPos = vec3(0.0f, 0.0f, 3.0f);
	vec3 cameraFront = vec3(0.0f, 0.0f, -1.0f);
	vec3 cameraUp = vec3(0.0f, 1.0f, 0.0f);

	mat4 view()
	{
		vec3 right = normalize(cross(cameraFront, vec3(0.0f, 1.0f, 0.0f)));
		cameraUp = normalize(cross(right, cameraFront));
		return lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
	}
} camera;

bool firstMouse = true;
float lastX = (float) screenWidth / 2.0;
float lastY = (float) screenHeight / 2.0;

// Vector holding the point masses
vector<PointMass> pointMasses;

// Vector holding the springs
vector<Spring> springs;

// Simulation Variables
float dt = 0.0009;
int desiredFPS = 60;
float sec_per_frame = ((float) 10 / (float)desiredFPS);

bool cubeCollision = false;

vec3 gravity = vec3(0.0, -9.81, 0.0);
float damp_constant = 0.5;

	// Mass on Spring (MOS)
float MOS_mass = 1.0; // in kg
float MOS_springConstant = 20.0;
float MOS_springLength = 0.5;

	// Spring Pendulum (SP)
int SP_chainLength = 10;
float SP_massTotal = 0.5; // in kg
float SP_springConstants = 50.0;
float SP_springLengths = 0.1;

	// Cloth (CL)
int CL_width = 45;
int CL_height = 45;
float CL_massTotal = 10.0; // in kg
float CL_springConstants = 90.0;

	// Jelly Cube (JC)
int JC_base = 10;
int JC_width = 10;
int JC_height = 10;
float JC_massTotal = 10.0; // in kg
float JC_springConstants = 25.0;

// General vector to load vertices to VBO's
vector<float> vertices;

void genVBOsVAOs();
void initShaders();
void initPrograms();
void updateCamera();
void keyboard_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void createPointMass(vec3 pos, vec3 velocity, float weight);
void createSpring(int i, int j, float k, float l);
void initMassOnSpring();
void initSpringPendulum();
void initCloth();
void initJellyCube();
void connectAllPoints(PointMass p, int i, float r, float k);
void loadPointsSprings();
vec3 sumExternalForces(PointMass p);
vec3 computeCollision(PointMass p, Plane pl);
vec3 computeSpringForce(Spring spring);
void render();

const char *vertexShaderSource = "#version 330 core\n"
"layout (location = 0) in vec3 aPos;\n"
"uniform mat4 mvp;\n"
"void main()\n"
"{\n"
"	gl_Position = mvp * vec4(aPos, 1.0);\n"
"}\0";

const char *pointMassFragmentShaderSource = "#version 330 core\n"
"out vec4 FragColor;\n"
"void main()\n"
"{\n"
"	FragColor = vec4(0.855f, 0.647f, 0.125f, 1.0f);\n"
"}\n\0";

int main()
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	GLFWwindow* window = glfwCreateWindow(800, 800, "Learn OpenGL", NULL, NULL);
	if (window == NULL)
	{
		cout << "Failed to create GLFW window" << endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		cout << "Failed to initialize GLAD" << endl;
		return -1;
	}

	glViewport(0, 0, screenWidth, screenHeight);

	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetKeyCallback(window, keyboard_callback);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	genVBOsVAOs();
	initShaders();
	initPrograms();
	updateCamera();

	initMassOnSpring();
	loadPointsSprings();

	float deltaTime = 0.0f;
	float lastFrame = 0.0f;
	float currentFrame = 0.0f;

	while (!glfwWindowShouldClose(window))
	{
		//currentFrame = glfwGetTime();
		for (int iter = 0; iter < 16; iter++) {


			for (int i = 0; i < pointMasses.size(); i++)
			{
				pointMasses[i].netForce += sumExternalForces(pointMasses[i]);
			}

			for (int i = 0; i < springs.size(); i++)
			{
				vec3 force_on_i = computeSpringForce(springs[i]);
				pointMasses[springs[i].i].netForce += force_on_i;
				pointMasses[springs[i].j].netForce += (-force_on_i);
			}

			for (int i = 0; i < pointMasses.size(); i++)
			{
				pointMasses[i].velocity += pointMasses[i].netForce * pointMasses[i].weight * dt;
				pointMasses[i].pos += pointMasses[i].velocity * dt;
				pointMasses[i].netForce = vec3(0.0, 0.0, 0.0);
			}

			//deltaTime = deltaTime + (currentFrame - lastFrame);
		}
		//if (deltaTime > sec_per_frame)
		{
			loadPointsSprings();

			glfwSwapBuffers(window);

			//Render command
			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			render();

			deltaTime = 0;
			lastFrame = currentFrame;
		}
		glfwPollEvents();
	}

	glDeleteShader(vertexShader);
	glDeleteShader(pointMassFragmentShader);
	glfwTerminate();
	return 0;
}

void genVBOsVAOs()
{
	glGenBuffers(1, &pointMassVBO);
	glGenVertexArrays(1, &pointMassVAO);

	glGenBuffers(1, &springVBO);
	glGenVertexArrays(1, &springVAO);
}

void initShaders()
{
	// Vertex Shader
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
	glCompileShader(vertexShader);

	// Check if compilation is a success
	int success;
	char infoLog[512];

	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);

	if (!success)
	{
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << endl;
		exit(0);
	}


	// Point Mass Fragment Shader
	pointMassFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(pointMassFragmentShader, 1, &pointMassFragmentShaderSource, NULL);
	glCompileShader(pointMassFragmentShader);

	glGetShaderiv(pointMassFragmentShader, GL_COMPILE_STATUS, &success);

	if (!success)
	{
		glGetShaderInfoLog(pointMassFragmentShader, 512, NULL, infoLog);
		cout << "ERROR::SHADER::FRAGMENT::POINT::COMPILATION_FAILED\n" << infoLog << endl;
		exit(0);
	}
}

void initPrograms()
{
	pointMassShaderProgram = glCreateProgram();

	glAttachShader(pointMassShaderProgram, vertexShader);
	glAttachShader(pointMassShaderProgram, pointMassFragmentShader);
	glLinkProgram(pointMassShaderProgram);

	int success;
	char infoLog[512];

	glGetProgramiv(pointMassShaderProgram, GL_LINK_STATUS, &success);
	if (!success)
	{
		glGetProgramInfoLog(pointMassShaderProgram, 512, NULL, infoLog);
		cout << "ERROR::PROGRAM::POINT::LINKING_FAILED\n" << infoLog << endl;
		exit(0);
	}
}

void updateCamera()
{
	mat4 model;
	mat4 view;
	mat4 projection;
	mat4 mvp;

	model = mat4(1.0f);
	view = camera.view();
	projection = perspective(radians(45.0f), (float) screenWidth / screenHeight, 0.1f, 100.0f);

	mvp = projection * view * model;

	glUseProgram(pointMassShaderProgram);

	int mvpLoc = glGetUniformLocation(pointMassShaderProgram, "mvp");
	glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, value_ptr(mvp));
}

void keyboard_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	else if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
	{
		cubeCollision = false;
		initMassOnSpring();
	}

	else if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
	{
		cubeCollision = false;
		initSpringPendulum();
	}

	else if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
	{
		cubeCollision = false;
		initCloth();
	}

	else if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
	{
		cubeCollision = true;
		initJellyCube();
	}

	/*
		CAMERA CONTROLS TAKEN FROM: learnopengl.com
	*/

	else if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
	{
		camera.cameraPos += camera.cameraSpeed * camera.cameraFront;
		updateCamera();
	}
	else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	{
		camera.cameraPos -= camera.cameraSpeed * camera.cameraFront;
		updateCamera();
	}
	else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	{
		camera.cameraPos -= normalize(cross(camera.cameraFront, camera.cameraUp)) * camera.cameraSpeed;
		updateCamera();
	}
	else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	{
		camera.cameraPos += normalize(cross(camera.cameraFront, camera.cameraUp)) * camera.cameraSpeed;
		updateCamera();
	}
}

/*
	MOUSE PROCESSING FOR CAMERA TAKEN FROM: learnopengl.com
*/

void mouse_callback(GLFWwindow * window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = (float) xpos - (float) lastX;
	float yoffset = (float) lastY -  (float) ypos;
	lastX = xpos;
	lastY = ypos;

	float sensitivity = 0.05;
	xoffset *= sensitivity;
	yoffset *= sensitivity;

	camera.yaw += xoffset;
	camera.pitch += yoffset;

	if (camera.pitch > 89.0f)
		camera.pitch = 89.0f;
	if (camera.pitch < -89.0)
		camera.pitch = -89.0f;

	vec3 front;
	front.x = cos(radians(camera.yaw)) * cos(radians(camera.pitch));
	front.y = sin(radians(camera.pitch));
	front.z = sin(radians(camera.yaw)) * cos(radians(camera.pitch));
	camera.cameraFront = normalize(front);
	updateCamera();
}

void framebuffer_size_callback(GLFWwindow * window, int width, int height)
{
	glViewport(0, 0, width, height);
}

void createPointMass(vec3 pos, vec3 velocity, float weight)
{
	PointMass pm;
	pm.index = pointMasses.size();
	pm.pos = pos;
	pm.velocity = velocity;
	pm.weight = weight;
	pm.netForce = vec3(0.0, 0.0, 0.0);

	pointMasses.push_back(pm);
}

void createSpring(int i, int j, float k, float l)
{
	Spring s;
	s.i = i;
	s.j = j;
	s.k = k;
	s.l = l;

	springs.push_back(s);
}

void initMassOnSpring()
{
	pointMasses.clear();
	springs.clear();

	createPointMass(vec3(0.0, 0.5, 0.0), vec3(0.0, 0.0, 0.0), 0.0);
	createPointMass(vec3(0.0, 0.0, 0.0), vec3(0.0, 0.0, 0.0), ((float) 1.0)/ (float) MOS_mass);

	createSpring(0, 1, MOS_springConstant, MOS_springLength);
}

void initSpringPendulum()
{
	pointMasses.clear();
	springs.clear();

	createPointMass(vec3(0.0, 0.5, 0.0), vec3(0.0, 0.0, 0.0), 0.0);

	float mass = (float) SP_massTotal / (float) SP_chainLength;

	for (int i = 1; i <= (SP_chainLength - 1); i++)
	{
		createPointMass(vec3(0.0 + (i * SP_springLengths), 0.5, 0.0), vec3(0.0, 0.0, 0.0), (float) 1.0 / mass);
	}

	for (int i = 0; i < (SP_chainLength - 1); i++)
	{
		createSpring(i, i + 1, SP_springConstants, SP_springLengths);
	}
}

void initCloth()
{
	pointMasses.clear();
	springs.clear();
	float mass = CL_massTotal / (CL_width * CL_height);

	// Create top row of immovable point masses
	createPointMass(vec3(-1.0, 1.0, 0.0), vec3(0.0, 0.0, 0.0), 0.0);

	for (int i = 1; i < CL_width; i++)
	{
		if ( (i % 5 == 0) || (i == CL_width - 1) )
			createPointMass(vec3(-1.0 + ( i *( 2.0 / CL_width)), 1.0, 0.0), vec3(0.0, 0.0, 0.0), 0.0);
		else
			createPointMass(vec3(-1.0 + (i *(2.0 / CL_width)), 1.0, 0.0), vec3(0.0, 0.0, 0.0), 1.0 / mass);
	}

	// Create Remaining point masses
	for (int h = 1; h < CL_height; h++)
	{
		for (int w = 0; w < CL_width; w++)
		{
			createPointMass(vec3(-1.0 + (w *(2.0 / CL_width)), 1.0 - (h *(2.0 / CL_height)), -(h *(2.0 / CL_height))), vec3(0.0, 0.0, 0.0), 1.0 / mass);
		}
	}

	float diagonalDistance = length(pointMasses[0].pos - pointMasses[CL_width + 1].pos);
	
	for (int i = 0; i < pointMasses.size(); i++)
	{
		connectAllPoints(pointMasses[i], i, diagonalDistance + 0.001, CL_springConstants);
	}
	
}

void initJellyCube()
{
	pointMasses.clear();
	springs.clear();

	float mass = (float) JC_massTotal / (float) (JC_base * JC_width * JC_height);

	// Create Point Masses
	for (int h = 0; h < JC_height; h++)
	{
		for (int w = 0; w < JC_width; w++)
		{
			for (int b = 0; b < JC_base; b++)
			{
				createPointMass(vec3(-1.0 + (b *(2.0 / JC_base)), 3.0 - (h *(2.0 / JC_height)), -1.0 + (w *(2.0 / JC_width))), vec3(0.0, 0.0, 0.0), 1.0 / mass);
			}
		}
	}

	float diagonalDistance = length(pointMasses[0].pos - pointMasses[(JC_width* JC_base) + JC_base + 1].pos);

	for (int i = 0; i < pointMasses.size(); i++)
	{
		connectAllPoints(pointMasses[i], i, diagonalDistance + 0.001, JC_springConstants);
	}
}

void connectAllPoints(PointMass p, int i, float r, float k)
{
	bool spring_exists = false;

	for (int j = 0; j < pointMasses.size(); j++)
	{
		if ((i != j) && (length(pointMasses[j].pos - pointMasses[i].pos) <= r))
		{
			for (int k = 0; k < springs.size(); k++)
			{
				if ( ((springs[k].i == i) && (springs[k].j == j)) || ((springs[k].i == j) && (springs[k].j == i)) )
				{
					spring_exists = true;
					break;
				}
			}
			if (!spring_exists)
				createSpring(i, j, k, length(pointMasses[j].pos - pointMasses[i].pos));
			else
				spring_exists = false;
		}
	}
}

void loadPointsSprings()
{
	for (int i = 0; i < springs.size(); i++)
	{
		vertices.push_back(pointMasses[springs[i].i].pos.x);
		vertices.push_back(pointMasses[springs[i].i].pos.y);
		vertices.push_back(pointMasses[springs[i].i].pos.z);

		vertices.push_back(pointMasses[springs[i].j].pos.x);
		vertices.push_back(pointMasses[springs[i].j].pos.y);
		vertices.push_back(pointMasses[springs[i].j].pos.z);
	}

	glBindVertexArray(pointMassVAO);

	glBindBuffer(GL_ARRAY_BUFFER, pointMassVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STREAM_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	num_point_masses = vertices.size() / 3;

	// Clear vertices vector
	vertices.clear();
}

vec3 sumExternalForces(PointMass p)
{
	if (p.weight == 0.0) {
		return vec3(0.0, 0.0, 0.0);
	}

	vec3 F_grav = (float)((1.0) / p.weight) * gravity;
	vec3 F_penalty;
	
	if (cubeCollision)
	{
		F_penalty += computeCollision(p, ground);
	}
	else
		F_penalty = vec3(0.0, 0.0, 0.0);

	return  F_grav + F_penalty;
}

vec3 computeCollision(PointMass pm, Plane pl)
{
	if (pl.intersect(pm.pos)) {
		float proj = dot((pm.pos - pl.origin), pl.normal);
		vec3 proj_point = pm.pos - proj * pl.normal;

		createPointMass(proj_point, vec3(0.0, 0.0, 0.0), 0.0);
		createSpring(pm.index, pointMasses.size() - 1, 500.0, 0.0);

		vec3 F_penalty = computeSpringForce(springs[springs.size() - 1]);
		pointMasses.pop_back();
		springs.pop_back();

		return F_penalty;
	}
	else
		return vec3(0.0, 0.0, 0.0);
}

vec3 computeSpringForce(Spring spring)
{
	// Placeholder variables for the pointmasses
	PointMass pi = pointMasses[spring.i];
	PointMass pj = pointMasses[spring.j];

	vec3 F_spring_ij, F_damp_ij;

	// Formula for spring force taken from pdf
	F_spring_ij = (-spring.k) * (length(pi.pos - pj.pos) - spring.l) * ((pi.pos - pj.pos) / (length(pi.pos - pj.pos)));

	if (length(F_spring_ij) != 0)
	{
		// Normalized spring force needed for damping formula
		vec3 F_spring_ij_norm = F_spring_ij / length(F_spring_ij);
		// Damping formula for spring taken from assignment pdf
		F_damp_ij = (-damp_constant) * (dot(pi.velocity - pj.velocity, F_spring_ij_norm) / dot(F_spring_ij_norm, F_spring_ij_norm)) * F_spring_ij_norm;
	}
	else
	{
		F_damp_ij = vec3(0.0, 0.0, 0.0);
	}

	return F_spring_ij + F_damp_ij;
}

void render()
{
	glUseProgram(pointMassShaderProgram);

	// Draw the pointMasses
	glBindVertexArray(pointMassVAO);
	glDrawArrays(GL_LINES, 0, num_point_masses);
}

