//
// Created by Danielhu on 2018/5/1.
//

#ifndef VOXELWORLD_VERTEX_H
#define VOXELWORLD_VERTEX_H

#include <glm/glm.hpp>

struct Vertex {
  unsigned int vertexIndex;
  glm::fvec3 hermiteP;
  glm::fvec3 hermiteN;
  explicit Vertex(glm::fvec3 hermiteP) : vertexIndex(0), hermiteP(hermiteP), hermiteN(glm::fvec3(0)) {}
  Vertex() = default;
};

#endif //VOXELWORLD_VERTEX_H
