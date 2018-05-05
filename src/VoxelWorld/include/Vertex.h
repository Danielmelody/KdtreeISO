//
// Created by Danielhu on 2018/5/1.
//

#ifndef VOXELWORLD_VERTEX_H
#define VOXELWORLD_VERTEX_H

#include <glm/glm.hpp>

struct Vertex {
  unsigned int vertexIndex;
  glm::vec3 hermiteP;
  glm::vec3 hermiteN;
  explicit Vertex(glm::vec3 hermiteP) : vertexIndex(0), hermiteP(hermiteP), hermiteN(glm::vec3(0)) {}
  Vertex() = default;
};

#endif //VOXELWORLD_VERTEX_H
