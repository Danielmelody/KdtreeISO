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

static float cameraOffset = 20.f;
static double previousCursorX = 0.f;
static double previousCursorY = 0.f;
static float rotateX = 0.f;
static float rotateY = 0.f;
static bool pressing = false;
static bool inited = false;

struct MeshBuffer {
  GLuint positions;
  GLuint normals;
  GLuint indices;
  MeshBuffer() {
    glGenBuffers(1, &positions);
    glGenBuffers(1, &normals);
    glGenBuffers(1, &indices);
  }
  ~MeshBuffer() {
    glDeleteBuffers(1, &positions);
    glDeleteBuffers(1, &normals);
    glDeleteBuffers(1, &indices);
  }
};

void addMesh(Mesh *mesh, const MeshBuffer &buffer) {
  glBindBuffer(GL_ARRAY_BUFFER, buffer.positions);
  glBufferData(GL_ARRAY_BUFFER, mesh->positions.size() * sizeof(glm::vec3), &(mesh->positions[0]), GL_STATIC_DRAW);

  glBindBuffer(GL_ARRAY_BUFFER, buffer.normals);
  glBufferData(GL_ARRAY_BUFFER, mesh->normals.size() * sizeof(glm::vec3), &(mesh->normals[0]), GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer.indices);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               mesh->indices.size() * sizeof(unsigned int),
               &(mesh->indices[0]),
               GL_STATIC_DRAW);
}

void drawMesh(Mesh *mesh,
              const MeshBuffer &buffer,
              Program &p,
              bool shaded,
              bool wireframe) {
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, buffer.positions);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, buffer.normals);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, nullptr);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer.indices);

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

  MeshBuffer meshBuffers[4];

  Transform g(
      glm::rotate(
          glm::translate(mat4(1.f), vec3(0, 0, 0)),
          glm::radians(0.f),
          vec3(1, 0, 0)
      ),
      new Intersection(
          new AABB(vec3(-4.3, -4.3, -3.2), vec3(4.3, 4.3, -2.8)),
          new Sphere(4, vec3(0, 0, -3))
      )
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
//          new AABB(vec3(-4.3f), vec3(4.3f)),
//          new Sphere(3, vec3(0, 5, 0))
//      )
//      new Heart(5)
//      new AABB(vec3(-5), vec3(5))
//      new Difference(
//          new AABB(vec3(-4, -4, -0.2), vec3(4, 4, 0.2)),
//          new AABB(vec3(-1.5f), vec3(1.5f))
//      )
//      new Sphere(4.3f)
  );
  int svoCull = 0;
  int octDepth = 5;
  Octree::setCellSize(0.6f);
  PositionCode sizeCode = PositionCode(1 << (octDepth - 1));
  float threshold = 1e-2;

  auto octree = Octree::buildWithTopology(-sizeCode / 2, octDepth, &g, svoCull);
  auto *octreeVisual = new Mesh();
  unordered_set<Vertex *> visualUtil;
  Octree::drawOctrees(octree, octreeVisual, visualUtil);

  auto kdtree = Octree::generateKdtree(octree, -sizeCode / 2, sizeCode / 2, 0);
  auto *kdtreeVisual = new Mesh();
  Kdtree::drawKdtree(kdtree, kdtreeVisual, threshold);

  int intersectionPreservingVerticesCount = 0;
  bool intersectionFree = false;

//  Octree::simplify(octree, threshold, &g);
//  Mesh * mesh = Octree::extractMesh(octree, &g, intersectionPreservingVerticesCount, intersectionFree);
  Mesh *mesh = Kdtree::extractMesh(kdtree, &g, threshold);

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

  addMesh(mesh, meshBuffers[0]);
  addMesh(octreeVisual, meshBuffers[1]);
  addMesh(kdtreeVisual, meshBuffers[2]);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    if (pressing || !inited) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      program.use();
      setUniforms(program);
      drawMesh(mesh, meshBuffers[0], program, true, true);
      // drawMesh(octreeVisual, meshBuffers[1], program, false, true);
      // drawMesh(kdtreeVisual, meshBuffers[2], program, false, true);
      glfwSwapBuffers(window);
      inited = true;
    }
  }

  delete mesh;
  delete octreeVisual;
  glfwTerminate();
  return 0;
}