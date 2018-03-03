//
// Created by Danielhu on 2018/1/14.
//

#ifndef VOXELWORLD_OCTREE_H
#define VOXELWORLD_OCTREE_H

#include <glm/glm.hpp>
#include "Topology.h"
#include "Qef.h"
#include <vector>
#include <unordered_set>

class Octree {
public:
  static Octree *buildWithTopology(glm::vec3 min,
                                   float size,
                                   int depth,
                                   Topology *topology,
                                   int &loselessCut);
  static void simplify(Octree *root, float threshold, Topology *geometry, int &count);
  static void compress(Octree *root, float threshold, Topology *geometry, int &count);

  static Mesh *generateMesh(Octree *root, Topology *geometry);
  float getError() { return error; }
  float getAdaptiveError() { return error / (size * size * size); }
  void collapse(Topology *g);
  Octree(glm::vec3 min, float size, int depth) :
      parent(nullptr),
      childIndex(-1),
      isLeaf(false),
      internal(false),
      min(min),
      size(size),
      depth(depth) {
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
  static void contourCell(Octree *root, Mesh *mesh, Topology *geometry);
  static void contourFace(Octree *nodes[2], int dir, Mesh *mesh, Topology *geometry);
  static void contourEdge(Octree *nodes[4], int dir, Mesh *mesh, Topology *geometry);
  static void generateVertexIndices(Octree *node, Mesh *mesh, Topology *geometry);
  static bool findFeatureNodes(Octree *node,
                               std::vector<Octree *> &results,
                               const int cornerDir,
                               bool subdivision,
                               const glm::vec3 &edgeP,
                               const glm::vec3 &normal);
  static void generateQuad(Octree **nodes, int dir, Mesh *mesh, Topology *g);
  static void generatePolygons(Octree **nodes, int dir, Mesh *mesh, Topology *g);
  static bool getSelfQef(Octree *node, Topology *geometry, QefSolver &qef);
  static Octree *buildRecursively(glm::vec3 min, float size, int depth, Topology *geometry);
  static Octree *losslessCompress(Octree *root, float threshold, Topology *geometry, int &count);

  static void calHermite(Octree *node, QefSolver &qef, Topology *g);
  Octree *parent;
  Octree *children[8];
  int8_t childIndex;
  uint8_t cornerSigns[8];
  std::unordered_set<Octree *> connections;
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
