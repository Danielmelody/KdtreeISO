//
// Created by Danielhu on 2018/5/1.
//

#ifndef VOXELWORLD_VERTEX_H
#define VOXELWORLD_VERTEX_H

#include <glm/glm.hpp>

struct Vertex {
  Vertex *parent;
  unsigned int vertexIndex;
  float error;
  glm::fvec3 hermiteP;
  glm::fvec3 hermiteN;
  explicit Vertex(glm::fvec3 hermiteP)
      : parent(nullptr), vertexIndex(0), error(-1.f), hermiteP(hermiteP), hermiteN(glm::fvec3(0)) {}
  Vertex() = default;
};

#endif //VOXELWORLD_VERTEX_H
