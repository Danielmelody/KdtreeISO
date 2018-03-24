//
// Created by Danielhu on 2018/1/16.
//
#include <iostream>
#include <unordered_set>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include "Mesh.h"
#include "Topology.h"
#include "Octree.h"
#include "program.h"

using glm::vec3;
using glm::mat4;
using glm::normalize;
using glm::radians;

using namespace std;

const char *vert =
    "#version 330 core\n"
        "layout(location = 0) in vec3 position;\n"
        "layout(location = 1) in vec3 normal;\n"
        "\n"
        "uniform mat4 mvp;\n"
        "uniform mat4 m;\n"
        "smooth out vec3 fragNormal;\n"
        "\n"
        "void main() {\n"
        "    fragNormal = (m * vec4(normal, 1.0)).xyz;\n"
        "    gl_Position = mvp * vec4(position, 1.0);\n"
        "}";

const char *frag =
    "#version 330 core\n"
        "out vec3 color;\n"
        "\n"
        "smooth in vec3 fragNormal;\n"
        "\n"
        "uniform vec3 lightDir;\n"
        "uniform vec3 albedo;\n"
        "\n"
        "void main() {\n"
        "    color = albedo * max(dot(fragNormal, lightDir), 0.2f);\n"
        "    // color = (1.0 - albedo) * (step(dot(fragNormal,vec3(0, 0, -1)), 0.0) * 0.8 + 0.2);\n"
        "}";

float cameraOffset = 20.f;
double previousCursorX = 0.f;
double previousCursorY = 0.f;
float rotateX = 0.f;
float rotateY = 0.f;
bool pressing = false;
bool inited = false;

void addMesh(Mesh *mesh, GLuint &positionsBuffer, GLuint &normalsBuffer, GLuint &indicesBuffer) {
  glGenBuffers(1, &positionsBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, positionsBuffer);
  glBufferData(GL_ARRAY_BUFFER, mesh->positions.size() * sizeof(glm::vec3), &(mesh->positions[0]), GL_STATIC_DRAW);

  glGenBuffers(1, &normalsBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, normalsBuffer);
  glBufferData(GL_ARRAY_BUFFER, mesh->normals.size() * sizeof(glm::vec3), &(mesh->normals[0]), GL_STATIC_DRAW);

  glGenBuffers(1, &indicesBuffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indicesBuffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               mesh->indices.size() * sizeof(unsigned int),
               &(mesh->indices[0]),
               GL_STATIC_DRAW);
}

void drawMesh(Mesh *mesh, GLuint &positionsBuffer, GLuint &normalsBuffer, GLuint &indicesBuffer, Program &p, bool shaded, bool wireframe) {
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, positionsBuffer);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, normalsBuffer);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, nullptr);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indicesBuffer);

  if (wireframe) {
    p.setVec3("albedo", glm::vec3(1, 1, 1));
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, (GLsizei) mesh->indices.size(), GL_UNSIGNED_INT, nullptr);
  }

  if (shaded) {
    p.setVec3("albedo", glm::vec3(1, 0, 0));
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_TRIANGLES, (GLsizei) mesh->indices.size(), GL_UNSIGNED_INT, nullptr);
  }

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
}

void setUniforms(Program &program) {
  mat4 p = glm::perspective(radians(45.f), 4.f / 3.f, 0.1f, 1000.f);
  mat4 m = glm::rotate(mat4(), rotateX, vec3(0, 1, 0));
  m = glm::rotate(m, rotateY, vec3(1, 0, 0));
  mat4 v = glm::lookAt(vec3(0, 0, cameraOffset), vec3(0, 0, -1), vec3(0, 1, 0));
  program.setMat4("m", m);
  program.setMat4("mvp", p * v * m);
  program.setVec3("albedo", vec3(1.0f, 1.0f, 1.0f));
  program.setVec3("lightDir", normalize(vec3(0.f, 0.f, 1.f)));
}

void error(int error, const char *description) {
  cerr << error << ": " << description << endl;
}

void mouseInput(GLFWwindow *window, double x, double y) {
  if (pressing) {
    rotateX += 0.01 * (x - previousCursorX);
    rotateY += 0.01 * (y - previousCursorY);
    previousCursorX = x;
    previousCursorY = y;
  }
}

void scroll(GLFWwindow *window, double dx, double dy) {
  cameraOffset -= 0.2 * dy;
  pressing = true;
}

void press(GLFWwindow *window, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    pressing = true;
    glfwGetCursorPos(window, &previousCursorX, &previousCursorY);
  }
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
    pressing = false;
  }
}

int main() {
  if (!glfwInit()) {
    return -1;
  }
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

  GLFWwindow *window = glfwCreateWindow(640, 480, "Voxel World", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(window);

//   Initialize GLEW
  glewExperimental = GL_TRUE; // Needed for core profile
  if (glewInit() != GLEW_OK) {
    fprintf(stderr, "Failed to initialize GLEW\n");
    getchar();
    glfwTerminate();
    return -1;
  }

  cout << "OpenGL Version: " + string((const char *) glGetString(GL_VERSION)) << endl;

  glfwSetErrorCallback(error);
  glfwSetCursorPosCallback(window, mouseInput);
  glfwSetScrollCallback(window, scroll);
  glfwSetMouseButtonCallback(window, press);

  GLuint VertexArrayID;
  glGenVertexArrays(1, &VertexArrayID);
  glBindVertexArray(VertexArrayID);

  GLfloat lineWidthRange[2] = {0.0f, 0.0f};
  glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, lineWidthRange);

  GLuint positionsBuffers[2];
  GLuint normalsBuffers[2];
  GLuint indicesBuffers[2];
  // Sphere g(3, vec3(0, 0, 0));
  AABB g1(vec3(-2, -5, -2), vec3(2, 5, 2));
  AABB g2(vec3(-5, -0.5, -5), vec3(5, 0.5, 5));
  // AABB g(vec3(1.7, -3, -3), vec3(2.3, 3, 3));
  //Union gu(&g1, &g2);
  // Union g(&g1, &g2);
  // Intersection g(&g1, &g2);
  Difference g(&g2, &g1);

  float area = 12.f;
  int svoCull = 0;
  int faceCount = 0;
  auto octree = Octree::buildWithTopology(glm::vec3(-area / 2.f), vec3(area), 7, &g, svoCull);
//  int traditionCount = 0;
//  Octree::simplify(octree, 1e-3, &g, traditionCount);

  int edgeSimplifyCount = 0;
  int last = edgeSimplifyCount;
  Octree::edgeSimplify(octree, 0.01, 1e-2, &g, edgeSimplifyCount);
  cout << "edge simplify : " << edgeSimplifyCount - last << endl;


  auto *octreeVisual = new Mesh();
  unordered_set<Octree*> visualUtil;
  Octree::drawOctrees(octree.get(), octreeVisual, visualUtil);
  Mesh *mesh = Octree::generateMesh(octree, &g, faceCount);
  cout.setf(ios::scientific);
  cout << "triangle count: " << mesh->indices.size() / 3 << endl;
  cout << "vertex count: " << mesh->positions.size() / 3 << endl;
  // mesh->generateFlatNormals();

  Program program;
  if (!program.init(vert, frag)) {
    cerr << "glsl error" << endl;
    return -1;
  }

  addMesh(mesh, positionsBuffers[0], normalsBuffers[0], indicesBuffers[0]);
  addMesh(octreeVisual, positionsBuffers[1], normalsBuffers[1], indicesBuffers[1]);



  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    if (pressing || !inited) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      program.use();
      setUniforms(program);
      drawMesh(mesh, positionsBuffers[0], normalsBuffers[0], indicesBuffers[0], program, true, true);
      // drawMesh(octreeVisual, positionsBuffers[1], normalsBuffers[1], indicesBuffers[1], program, false, true);
      glfwSwapBuffers(window);
      inited = true;
    }
  }

  delete mesh;
  delete octreeVisual;

  glDeleteBuffers(1, &positionsBuffers[0]);
  glDeleteBuffers(1, &normalsBuffers[0]);
  glDeleteBuffers(1, &indicesBuffers[0]);
  glDeleteBuffers(1, &positionsBuffers[1]);
  glDeleteBuffers(1, &normalsBuffers[1]);
  glDeleteBuffers(1, &indicesBuffers[1]);
  glfwTerminate();
  return 0;
}