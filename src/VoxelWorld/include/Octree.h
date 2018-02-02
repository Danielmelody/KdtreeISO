//
// Created by Danielhu on 2018/1/14.
//

#ifndef VOXELWORLD_OCTREE_H
#define VOXELWORLD_OCTREE_H

#include <glm/glm.hpp>
#include "Topology.h"
#include "Qef.h"

class Octree {
public:
  static Octree* buildWithGeometry(glm::vec3 min, float size, int depth, Topology* geometry);
  static Octree* simplify(Octree* root, float threshold, Topology* geometry, int &count);
  static Mesh* generateMesh(Octree* root);
  void collapse(Topology* g);
  Octree(glm::vec3 min, float size, int depth):isLeaf(false), internal(false), min(min), size(size), depth(depth) {
    for (int i = 0; i < 8; ++i) {
      children[i] = nullptr;
    }
  };
  ~Octree() {
    for (int i = 0; i < 8; ++i) {
      delete children[i];
    }
  };
protected:
  static void generateVertexIndices(Octree* root, unsigned int& count, Mesh* mesh);
  static void contourCell(Octree* root, Mesh* mesh);
  static void contourFace(Octree* nodes[2], int dir, Mesh* mesh);
  static void contourEdge(Octree* nodes[4], int dir, Mesh* mesh);
  static void generateQuad(Octree* nodes[4], int dir, Mesh *mesh);
  static bool getSelfQef(Octree* node, Topology* geometry, QefSolver& qef);

  Octree* children[8];
  uint8_t cornerSigns[8];
  bool isLeaf;
  bool internal;
  glm::vec3 min;
  float size;
  int depth;
  QefSolver qef;
  float error;
  unsigned int vertexIndex;
  glm::vec3 hermiteP;
  glm::vec3 hermiteN;
};

#endif //VOXELWORLD_OCTREE_H
