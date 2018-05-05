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
#include "Utils.h"

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
    "uniform float specular;\n"
    "\n"
    "void main() {\n"
    "    vec3 normal = normalize(fragNormal);"
    "    vec3 h = normalize(lightDir + vec3(0, 0, 1));"
    "    color = albedo * mix(max(dot(normal, lightDir), 0.2f) , pow(dot(normal, h), 64.f), specular);\n"
    "    // color = (1.0 - albedo) * (step(dot(normal,vec3(0, 0, -1)), 0.0) * 0.8 + 0.2);\n"
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

void drawMesh(Mesh *mesh,
              GLuint &positionsBuffer,
              GLuint &normalsBuffer,
              GLuint &indicesBuffer,
              Program &p,
              bool shaded,
              bool wireframe) {
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
  program.setFloat("specular", 0.f);
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

  GLuint positionsBuffers[3];
  GLuint normalsBuffers[3];
  GLuint indicesBuffers[3];
  Transform g(
      glm::rotate(
          glm::translate(mat4(1.f), vec3(0, 0, 0)),
          glm::radians(0.f),
          vec3(1, 1, 0)
      ),
//      new Difference(
          // new AABB(vec3(-4.9, -4.9, -1.5), vec3(4.9, 4.9, -0.5))
          // new Sphere(5)
//      )
//  new Difference(
//      new Union(
//          new AABB(vec3(-1.5f, -2, -1.5f), vec3(1.5f, 2, 1.5f)),
//          new AABB(vec3(-1, -5, -1), vec3(1, 5, 1))
//      ),
//      new Sphere(1.9f)
//  )

//      new Union(
//          new Torus(5.f, 1.f),
//          new ExpUnion(
//              new Sphere(2, vec3(1.5, 1.5, 0)),
//              new Sphere(2, vec3(-1.5, -1.5, 0)),
//              1
//          )
//      )
//      new Difference(
//          new AABB(vec3(-4.3f), vec3(4.3f))
//          new Sphere(3, vec3(0, 5, 0))
//      )
//      new Heart(5)
//      new AABB(vec3(-5), vec3(5))
//      new Difference(
//          new AABB(vec3(-4, -4, -0.2), vec3(4, 4, 0.2)),
//          new AABB(vec3(-1.5f), vec3(1.5f))
//      )
      new Sphere(4.3f)
  );
  int svoCull = 0;

  int depth = 6;
  Octree::setCellSize(0.5f);
  PositionCode sizeCode = PositionCode(1 << (depth - 1));

  auto octree = Octree::buildWithTopology(-sizeCode / 2, depth, &g, svoCull);
  // auto satOct = SatOctree::initialBuildFromOctree(octree, encodePosition(PositionCode(0)), 7);
  auto kdtree = Octree::generateKdtree(octree, -sizeCode / 2, sizeCode / 2, 0);

  auto *kdtreeVisual = new Mesh();
  Kdtree::drawKdtree(kdtree, kdtreeVisual, 1e1);

  QefSolver sum;

  cout.setf(ios::scientific);
//  int originReduction = 0;
//  for (int i = 0; i < 1; ++i) {
//    float threshold = std::pow(10.f, (float) i - 1.f);
//    cout << "Threshold : " << threshold << endl;
//    originReduction += Octree::simplify(octree, threshold, &g);
//    cout << "origin reduction : " << originReduction << endl;
//    int extendedReduction = 0;
//    Octree::extendedSimplify(octree, threshold, &g, extendedReduction);
//    cout << "extended reduction : " << extendedReduction << endl;
//  }

  auto *octreeVisual = new Mesh();
  unordered_set<Vertex *> visualUtil;
  Octree::drawOctrees(octree, octreeVisual, visualUtil);
  int intersectionPreservingVerticesCount = 0;
  bool intersectionFree = false;

  // Mesh *mesh = Octree::generateMesh(octree, &g, intersectionPreservingVerticesCount, intersectionFree);
  Mesh * mesh = Kdtree::extractMesh(kdtree, &g, 1e-1);
  cout << "intersectionFree: " << (intersectionFree ? "true" : "false") << endl;
  cout << "triangle count: " << mesh->indices.size() / 3 << endl;
//  cout << "vertex count: " << mesh->positions.size() << endl;
  cout << "intersection contours: " << intersectionPreservingVerticesCount << endl;
  mesh->generateFlatNormals();
//
  Program program;
  if (!program.init(vert, frag)) {
    cerr << "glsl error" << endl;
    return -1;
  }

  addMesh(mesh, positionsBuffers[0], normalsBuffers[0], indicesBuffers[0]);
  addMesh(octreeVisual, positionsBuffers[1], normalsBuffers[1], indicesBuffers[1]);
  addMesh(kdtreeVisual, positionsBuffers[2], normalsBuffers[2], indicesBuffers[2]);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    if (pressing || !inited) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      program.use();
      setUniforms(program);
      drawMesh(mesh, positionsBuffers[0], normalsBuffers[0], indicesBuffers[0], program, true, false);
      // drawMesh(octreeVisual, positionsBuffers[1], normalsBuffers[1], indicesBuffers[1], program, false, true);
      // drawMesh(kdtreeVisual, positionsBuffers[2], normalsBuffers[2], indicesBuffers[2], program, false, true);
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
  glDeleteBuffers(1, &positionsBuffers[2]);
  glDeleteBuffers(1, &normalsBuffers[2]);
  glDeleteBuffers(1, &indicesBuffers[2]);
  glfwTerminate();
  return 0;
}