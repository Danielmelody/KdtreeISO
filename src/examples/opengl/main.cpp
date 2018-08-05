//
// Created by Danielhu on 2018/1/16.
//

#define GLM_FORCE_CTOR_INIT

#include <iostream>
#include <sstream>
#include <cmath>
#include <ctime>
#include <unordered_set>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include "svpng.inc"
#include "cxxopts.hpp"
#include "Mesh.h"
#include "Topology.h"
#include "VolumeData.h"
#include "Octree.h"
#include "program.h"
#include "Utils.h"

using glm::fvec3;
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
    "uniform float flag;"
    "uniform vec3 lightDir;\n"
    "uniform vec3 albedo;\n"
    "uniform float specular;\n"
    "\n"
    "void main() {\n"
    "    vec3 normal = normalize(fragNormal);"
    "    vec3 h = normalize(lightDir + vec3(0, 0, 1));"
    "    if(flag > 0.5) {\n"
    "       color = albedo * mix(max(abs(dot(normal, lightDir)), 0.0f) , pow(dot(normal, h), 64.f), specular);\n"
    "       // color = albedo;\n"
    "    } else {\n"
    "       color = vec3(0.0);\n"
    "    }\n"
    "    // color = (1.0 - albedo) * (step(dot(normal,vec3(0, 0, -1)), 0.0) * 0.8 + 0.2);\n"
    "}";

static float cameraOffset = 15.f;
static double previousCursorX = 0.f;
static double previousCursorY = 0.f;
constexpr unsigned width = 512;
constexpr unsigned height = 512;
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
  glBufferData(GL_ARRAY_BUFFER, mesh->positions.size() * sizeof(fvec3), &(mesh->positions[0]), GL_STATIC_DRAW);

  glBindBuffer(GL_ARRAY_BUFFER, buffer.normals);
  glBufferData(GL_ARRAY_BUFFER, mesh->normals.size() * sizeof(fvec3), &(mesh->normals[0]), GL_STATIC_DRAW);

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
    p.setVec3("albedo", fvec3(0, 0, 0));
    p.setFloat("flag", 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, (GLsizei) mesh->indices.size(), GL_UNSIGNED_INT, nullptr);
  }

  if (shaded) {
    p.setVec3("albedo", fvec3(1.0, 0.4, 0));
    p.setFloat("flag", 1);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_TRIANGLES, (GLsizei) mesh->indices.size(), GL_UNSIGNED_INT, nullptr);
  }

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
}

void setUniforms(Program &program) {
  mat4 p = glm::perspective(radians(70.f), float(width) / float(height), 0.1f, 1000.f);
  mat4 m = glm::rotate(mat4(), rotateX, fvec3(0, 1, 0));
  m = glm::rotate(m, rotateY, fvec3(1, 0, 0));
  mat4 v = glm::lookAt(fvec3(0, 0, cameraOffset), fvec3(0, 0, -1), fvec3(0, 1, 0));
  program.setMat4("m", m);
  program.setMat4("mvp", p * v * m);
  program.setVec3("albedo", fvec3(1.f, 1.f, 1.f));
  program.setFloat("specular", 0.f);
  program.setVec3("lightDir", normalize(fvec3(0.f, 0.f, 1.f)));
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

int main(int argc, const char **argv) {

  cxxopts::Options options("opengl_viewer", "An opengl viewer for paper: Discrete k-d Tree Hierarchy for Isosurface Extraction");
  options.add_options()
      ("e,error", "Error threshold.", cxxopts::value<float>()->default_value("1e-2"))
      ("s,structure", "oct/kd, extracting iso-surface using oct/kd tree.", cxxopts::value<std::string>()->default_value("kd"))
      ("rotateX", "Camera eular angle x.", cxxopts::value<float>()->default_value("0"))
      ("rotateY", "Camera eular angle y.", cxxopts::value<float>()->default_value("0"))
      ("v,volume", "Volume source")
      ("o,output", "Output first frame to a file.", cxxopts::value<std::string>()->default_value("null"))
      ("h,help", "Print help and exit.")
      ;

  auto parameters = options.parse(argc, argv);
  rotateX = parameters["rotateX"].as<float>();
  rotateY = parameters["rotateY"].as<float>();
  stringstream errorss;
  errorss.setf(ios::scientific);
  errorss << parameters["e"].as<float>();
  string windowName = parameters["s"].as<std::string>() + " " + errorss.str();

  if (parameters.count("help")) {
    cout << options.help() << std::endl;
    return 0;
  }

  if (parameters["s"].as<std::string>() != "oct" && parameters["s"].as<std::string>() != "kd") {
    cout << options.help() << std::endl;
    return 0;
  }

  if (!glfwInit()) {
    return -1;
  }
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

  GLFWwindow *window = glfwCreateWindow(width, height, windowName.c_str(), nullptr, nullptr);
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

  Transform topology(
      glm::rotate(
          glm::translate(mat4(1.f), fvec3(0)),
          glm::radians(0.f),
          fvec3(0, 0, 1)
      ),
//      new Difference(
//          new AABB(fvec3(-8), fvec3(8))
//          new AABB(fvec3(-3.5), fvec3(3.5, 3.5, 1.6))
//          new Sphere(5.5, fvec3(0))
//      )
      new Difference(
          new AABB(fvec3(-4), fvec3(4)),
//          new Union(
//          new Union(
//              new Cylinder(fvec3(0.f, 0.f, 4.1f))
//              new Transform(glm::rotate(mat4(), glm::radians(90.f), fvec3(1, 0, 0)),
//                            new Cylinder(fvec3(0.f, 0.f, 3.7f)))
//          )
              new Transform(glm::rotate(mat4(), glm::radians(90.f), fvec3(1, 0, 0)),
                            new Cylinder(fvec3(0.f, 0.f, 3.5f)))

      )
//  new Difference(
//      new Union(
//          new AABB(fvec3(-4, -4, -0.4f), fvec3(4, 4, -0.2f)),
//          new AABB(fvec3(-4, -4, 0.2f), fvec3(4, 4, 0.4f))
//      )
//  new Intersection(
//      new Sphere(5)
//      new AABB(fvec3(-5), fvec3(5, 5, 0))
//      )
//      new Sphere(5.2f)
//  )
//      new Difference(
//          new Heart(5),
//          new Difference(
//              new Cylinder(fvec3(0.f, 0.f, 3.7f)),
//              new Cylinder(fvec3(0.f, 0.f, 3.f))
//          )
//      )


//      new Intersection(
//          new AABB(fvec3(-4), fvec3(4))
//          new ExpUnion(
//              new Sphere(6, fvec3(4.5, 4.5, 0)),
//              new Sphere(6, fvec3(-4.5, -4.5, 0)),
//              1
//          )
//      )
//  new Intersection(
//      new AABB(fvec3(-4.3), fvec3(4.3)),
//      new Difference(
//          new Transform(glm::rotate(mat4(), glm::radians(90.f), fvec3(1, 0, 0)),
//                        new Cylinder(fvec3(0.f, 0.f, 4.3f))),
//          new Transform(glm::rotate(mat4(), glm::radians(90.f), fvec3(1, 0, 0)),
//                        new Cylinder(fvec3(0.f, 0.f, 3.8f)))
//      )
//  )
//      new Union(
//          new Intersection(
//              new Cylinder(fvec3(0, 0, 0.5)),
//              new AABB(fvec3(-6), fvec3(6.5))
//          ),
//          new Intersection(
//              new Cylinder(fvec3(0, 0, 5)),
//              new UnionList({
//                                new AABB(fvec3(-5, -5, -5), fvec3(5, -4.5, 5)),
//                                new AABB(fvec3(-5, -2.5, -5), fvec3(5, -2, 5)),
//                                new AABB(fvec3(-5, 0, -5), fvec3(5, 0.5, 5)),
//                                new AABB(fvec3(-5, 2.5, -5), fvec3(5, 3, 5)),
//                                new AABB(fvec3(-5, 5, -5), fvec3(5, 5.5, 5))
//                            })
//          )
//      )
//      new AABB(fvec3(-5), fvec3(5))
//      new Union(
//      new Union(
//      new Intersection(
//          new AABB(fvec3(-5, -5.5, -5), fvec3(5, -5, 5)),
//          new Cylinder(fvec3(0, 0, 5))
//      ),
//      new Sphere(5.0, fvec3(0, 0.6, 0))
//      )
//        new AABB(fvec3(-4, -4, 1), fvec3(4, 4, 4))
//      new Union(
//          new Union(
//              new Sphere(1),
//              new Sphere(1, fvec3(4, 4, 0))
//          ),
//          new Union(
//              new Sphere(1, fvec3(4, 0, 4)),
//              new Sphere(1, fvec3(0, 4, 4))
//          )
//      )
  );

  int octDepth = 7;
  RectilinearGrid::setUnitSize(0.2773);
  PositionCode sizeCode = PositionCode(1 << (octDepth - 1));
  float threshold = parameters["e"].as<float>();

  ScalarField *scalarField = &topology;
  if (parameters.count("volume")) {
    VolumeData volumeData(
        parameters["volume"].as<std::string>(),
        99,
        128,
        128,
        -sizeCode / 2,
        PositionCode(1)
    );
    volumeData.readTIFF();
    cout << "vulome read" << endl;
    scalarField = &volumeData;
  }

  clock_t begin = clock();

  Octree *octree = Octree::buildWithScalarField(-sizeCode / 2, octDepth, scalarField, parameters["s"].as<std::string>() == "kd");
  clock_t oct_build = clock();
  cout << "oct build time:" << (double) (oct_build - begin) / CLOCKS_PER_SEC << endl;
  Kdtree *kdtree = nullptr;
  auto *kdtreeVisual = new Mesh();

  int intersectionPreservingVerticesCount = 0;
  bool intersectionFree = false;
  Mesh *mesh(nullptr);
  if ((parameters["s"].as<std::string>()) == "kd") {
    cout << "extract using kdtree" << endl;
    kdtree = Kdtree::buildFromOctree(octree, -sizeCode / 2, sizeCode / 2, scalarField, 0);
    clock_t kdbuild = clock();
    Kdtree::drawKdtree(kdtree, kdtreeVisual, threshold);
    kdbuild = clock();
    cout << "kd build time:" << (double) (kdbuild - oct_build) / CLOCKS_PER_SEC << endl;
    mesh = Kdtree::extractMesh(kdtree, scalarField, threshold);
    clock_t kdextract = clock();
    cout << "kd extract time:" << (double) (kdextract - kdbuild) / CLOCKS_PER_SEC << endl;
  } else if ((parameters["s"].as<std::string>()) == "oct") {
    cout << "extract using octree" << endl;
    Octree::simplify(octree, threshold);
    mesh = Octree::extractMesh(octree, scalarField, intersectionPreservingVerticesCount, intersectionFree);
    clock_t octextract = clock();
    cout << "oct extract time:" << (double) (octextract - oct_build) / CLOCKS_PER_SEC << endl;
  }

  auto *octreeVisual = new Mesh();
  Octree::drawOctrees(octree, octreeVisual);

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
//      drawMesh(octreeVisual, meshBuffers[1], program, false, true);
//      drawMesh(kdtreeVisual, meshBuffers[2], program, false, true);
      glfwSwapBuffers(window);
      if (!inited) {
        stringstream outss;
        outss << parameters["o"].as<std::string>()
              << parameters["e"].as<float>() << "_" << parameters["s"].as<std::string>()
              << "_t" << mesh->indices.size() / 3;
        if (parameters["o"].as<std::string>() != "null") {
          unsigned char image[width * height * 3];
//          unsigned char *p = image;
//          glPixelStorei(GL_PACK_ALIGNMENT, 0);
//          glPixelStorei(GL_PACK_ROW_LENGTH, );
//          glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
//          glPixelStorei(GL_PACK_SKIP_ROWS, 0);
          glReadPixels(width * 1 / 2, height * 1 / 2, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
          FILE *fp = fopen((outss.str() + ".png").c_str(), "wb");
          svpng(fp, width, height, (unsigned char *) image, 0);
          fclose(fp);
          break;
        }
      }
      inited = true;
    }
  }

  delete mesh;
  delete octreeVisual;
  glfwTerminate();
  return 0;
}