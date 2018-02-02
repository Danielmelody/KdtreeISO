//
// Created by Danielhu on 2018/1/16.
//
#include <iostream>
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
        "smooth out vec3 fragNormal;\n"
        "\n"
        "void main() {\n"
        "    fragNormal = normal;\n"
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
        "    color = albedo * max(dot(fragNormal, lightDir), 0.f);\n"
        "}";

void addMesh(Mesh *mesh, GLuint &positionsBuffer, GLuint &normalsBuffer, GLuint &indicesBuffer) {
  glGenBuffers(1, &positionsBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, positionsBuffer);
  glBufferData(GL_ARRAY_BUFFER, mesh->positions.size() * sizeof(glm::vec3), &(mesh->positions[0]), GL_STATIC_DRAW);

  glGenBuffers(1, &normalsBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, normalsBuffer);
  glBufferData(GL_ARRAY_BUFFER, mesh->normals.size() * sizeof(glm::vec3), &(mesh->normals[0]), GL_STATIC_DRAW);

  glGenBuffers(1, &indicesBuffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indicesBuffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh->indices.size() * sizeof(unsigned int), &(mesh->indices[0]), GL_STATIC_DRAW);
}

void drawMesh(Mesh *mesh, GLuint& positionsBuffer, GLuint& normalsBuffer, GLuint& indicesBuffer) {
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, positionsBuffer);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, normalsBuffer);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, nullptr);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indicesBuffer);
  glDrawElements(GL_TRIANGLES, (GLsizei) mesh->indices.size(), GL_UNSIGNED_INT, nullptr);

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
}

void setUniforms(Program &program, glm::vec3 position) {
  mat4 p = glm::perspective(radians(45.f), 4.f / 3.f, 0.1f, 1000.f);
  mat4 m(1.f);
  m = glm::translate(m, position);
  program.setMat4("mvp", p * m);
  program.setVec3("albedo", vec3(0.7f, 0.7f, 0.f));
  program.setVec3("lightDir", normalize(vec3(1.f, 1.f, 1.f)));
}

Mesh* rect() {
  auto * mesh = new Mesh();
  mesh->positions = {
      /* face 1 */
      vec3(-1.0, -1.0, -1.0),
      vec3(-1.0, 1.0, -1.0),
      vec3(1.0, 1.0, -1.0),
      vec3(1.0, -1.0, -1.0)
  };
  mesh->indices = {
      0, 1, 2, 0, 2, 3,
  };
  mesh->generateSharpNormals();
  return mesh;
}

Mesh* triangle() {
  auto mesh = new Mesh();
  mesh->positions = {
      vec3(-1.0f, -1.0f, 0.0f),
      vec3(1.0f, -1.0f, 0.0f),
      vec3(0.0f,  1.0f, 0.0f),
  };
  mesh->generateSharpNormals();
  mesh->indices = {0, 1, 2};
  return mesh;
}

void Error(int error, const char* description)
{
  cerr << error << ": " << description << endl;
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

  glfwSetErrorCallback(Error);

  GLuint VertexArrayID;
  glGenVertexArrays(1, &VertexArrayID);
  glBindVertexArray(VertexArrayID);

  GLuint positionsBuffer;
  GLuint normalsBuffer;
  GLuint indicesBuffer;
  Heart g1(vec3(0));
  Sphere g2(2.f, vec3(1, 1, 4));
  Union g(&g1, &g2);
  // Intersection g(&g1, &g2);
  // Difference g(&g1, &g2);

  float area = 32.f;
  Octree* octree = Octree::buildWithGeometry(glm::vec3(-area / 2.f), area, 8, &g);
  int cutNum = 0;
  Octree::simplify(octree, -1.f, &g, cutNum);

  Mesh* mesh = Octree::generateMesh(octree);
  cout << "triangle count: " << mesh->positions.size() << endl;
  // mesh->generateSharpNormals();

  for (auto& p :mesh->positions) {
    if (p == vec3(0.0)) {
      cerr << "no solution for qef" << endl;
    }
  }

  Program program;
  if (!program.init(vert, frag)) {
    cerr << "glsl error" << endl;
    return -1;
  }



  addMesh(mesh, positionsBuffer, normalsBuffer, indicesBuffer);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glClearColor(0.0f, 0.0f, 0.4f, 0.0f);


  while (!glfwWindowShouldClose(window)) {
     glfwPollEvents();
     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
     program.use();
     setUniforms(program, vec3(0, 0, -20));
     drawMesh(mesh, positionsBuffer, normalsBuffer, indicesBuffer);
     glfwSwapBuffers(window);
  }

  delete mesh;

  glDeleteBuffers(1, &positionsBuffer);
  glDeleteBuffers(1, &normalsBuffer);
  glDeleteBuffers(1, &indicesBuffer);
  // delete octree;
  glfwTerminate();
  return 0;
}