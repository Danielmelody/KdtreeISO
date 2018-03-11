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
#include <memory>

class Octree {
public:
  static std::shared_ptr<Octree> buildWithTopology(glm::vec3 min,
                                                   glm::vec3 size,
                                                   int depth,
                                                   Topology *topology,
                                                   int &loselessCut);
  static void simplify(std::shared_ptr<Octree> root, float threshold, Topology *geometry, int &count);
  static std::shared_ptr<Octree> edgeSimplify(std::shared_ptr<Octree> root,
                                              float threshold,
                                              Topology *geometry,
                                              int &count);
  static void edgeCollapse(std::shared_ptr<Octree> &a,
                           std::shared_ptr<Octree> &b,
                           int dir,
                           float threshold,
                           Topology *geometry,
                           glm::vec3 faceMin,
                           float faceSize,
                           int &count);
  static void compress(std::shared_ptr<Octree> root, float threshold, Topology *geometry, int &count);

  static Mesh *generateMesh(std::shared_ptr<Octree> root, Topology *geometry, int &count);
  float getError() { return error; }
  // float getAdaptiveError() { return error / (size * size * size); }
  void collapse(Topology *g);
  Octree(glm::vec3 min, glm::vec3 size, int depth) :
      childIndex(-1),
      hasEdgeCollapse(false),
      isLeaf(false),
      internal(false),
      min(min),
      size(size),
      depth(depth) {
    for (int i = 0; i < 8; ++i) {
      children[i] = nullptr;
    }
  };
protected:
  static void contourCell(std::shared_ptr<Octree> root, Mesh *mesh, Topology *geometry, int &count);
  static void contourFace(std::shared_ptr<Octree> nodes[2], int dir, Mesh *mesh, Topology *geometry, int &count);
  static void contourEdge(std::shared_ptr<Octree> nodes[4], int dir, Mesh *mesh, Topology *geometry);
  static void combine(std::shared_ptr<Octree> &a,
                      std::shared_ptr<Octree> &b,
                      const QefSolver &combination,
                      glm::vec3 combinePos,
                      int &count);
  static void generateVertexIndices(std::shared_ptr<Octree> node, Mesh *mesh, Topology *geometry);
  static bool findFeatureNodes(std::shared_ptr<Octree> node,
                               std::vector<std::shared_ptr<Octree>> &results,
                               const int cornerDir,
                               bool subdivision,
                               const glm::vec3 &edgeP,
                               const glm::vec3 &normal);
  static bool intersectWithBrothers(int cornerDir, std::shared_ptr<Octree> node);
  static void generateQuad(std::shared_ptr<Octree> *nodes, int dir, Mesh *mesh, Topology *g);
  static void generatePolygons(std::shared_ptr<Octree> *nodes, int dir, Mesh *mesh, Topology *g);
  static bool getSelfQef(Octree *node, Topology *geometry, QefSolver &qef);
  static std::shared_ptr<Octree> buildRecursively(glm::vec3 min, glm::vec3 size, int depth, Topology *geometry);
  static std::shared_ptr<Octree> losslessCompress(std::shared_ptr<Octree> root,
                                                  float threshold,
                                                  Topology *geometry,
                                                  int &count);

  static void calHermite(Octree* node, QefSolver &qef, Topology *g);
  std::shared_ptr<Octree> children[8];
  int childIndex;
  uint8_t cornerSigns[8];
  bool hasEdgeCollapse;
  bool isLeaf;
  bool internal;
  glm::vec3 min;
  glm::vec3 size;
  int depth;
  QefSolver qef;
  float error;
  unsigned int vertexIndex;
  glm::vec3 hermiteP;
  glm::vec3 hermiteN;
};

#endif //VOXELWORLD_OCTREE_H
