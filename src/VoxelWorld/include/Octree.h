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

struct Vertex {
  unsigned int *clusterVertexIndex;
  glm::vec3 hermiteP;
  glm::vec3 hermiteN;
//  ~Vertex() {
//    delete clusterVertexIndex;
//  }
};

class Octree {
public:
  static std::shared_ptr<Octree> buildWithTopology(glm::vec3 min,
                                                   glm::vec3 size,
                                                   int depth,
                                                   Topology *topology,
                                                   int &loselessCut);
  static void simplify(std::shared_ptr<Octree> root, float threshold, Topology *geometry, int &count);
  static std::shared_ptr<Octree> edgeSimplify(std::shared_ptr<Octree> root,
                                              float roughnessT,
                                              float qefT,
                                              Topology *geometry,
                                              int &count);
  static std::shared_ptr<Octree> edgeClassifier(std::shared_ptr<Octree> root,
                                                float roughnessT,
                                                float qefT,
                                                int classifyDir,
                                                Topology *geometry,
                                                int &count);
  static void edgeCluster(std::shared_ptr<Octree> root,
                          Topology *geometry,
                          int &count);
  static void edgeCollapse(std::shared_ptr<Octree> &a,
                           std::shared_ptr<Octree> &b,
                           int dir,
                           float roughnessT,
                           float qefT,
                           Topology *geometry,
                           glm::vec3 faceMin,
                           float faceSize);
  static void compress(std::shared_ptr<Octree> root, float threshold, Topology *geometry, int &count);

  static Mesh *generateMesh(std::shared_ptr<Octree> root, Topology *geometry, int &count);
  static void drawOctrees(Octree *root, Mesh *mesh, std::unordered_set<Octree *> &visited);
  float getError() { return error; }
  // float getAdaptiveError() { return error / (size * size * size); }
  void collapse(Topology *g);
  Octree(glm::vec3 min, glm::vec3 size, int depth) :
      childIndex(-1),
      isLeaf(false),
      internal(false),
      min(min),
      size(size),
      depth(depth) {
    for (int i = 0; i < 8; ++i) {
      children[i] = nullptr;
    }
    cluster = new std::vector<std::shared_ptr<Octree> *>();
    clusterQef = new QefSolver();
    clusterMin = new glm::vec3(this->min);
    clusterSize = new glm::vec3(this->size);
  };
  ~Octree() {
//    if (cluster->empty()) {
//      delete cluster;
//      delete clusterMin;
//      delete clusterSize;
//      delete clusterQef;
//    }
  }
protected:
  static void contourCell(Octree *root, Mesh *mesh, Topology *geometry, int &count);
  static void contourFace(Octree *nodes[2], int dir, Mesh *mesh, Topology *geometry, int &count);
  static void contourEdge(Octree *nodes[4], int dir, Mesh *mesh, Topology *geometry);
  static void combine(std::shared_ptr<Octree> &a,
                      std::shared_ptr<Octree> &b,
                      Topology *g);
  static void generateVertexIndices(const std::shared_ptr<Octree> &node,
                                    Mesh *mesh,
                                    Topology *geometry,
                                    std::unordered_set<Octree *> &indexed);
  static bool findFeatureNodes(Octree *node,
                               std::vector<Octree *> &results,
                               const int cornerDir,
                               bool subdivision,
                               const glm::vec3 &edgeP,
                               const glm::vec3 &normal);
  static bool intersectWithBrothers(int cornerDir, Octree *node);
  static bool segmentFaceInterssection(const glm::vec3 &va,
                                       const glm::vec3 &vb,
                                       const glm::vec3 &min,
                                       const glm::vec3 max,
                                       int dir);
  static void generateQuad(Octree *nodes[4], int dir, Mesh *mesh, Topology *g);
  static void generatePolygons(Octree *nodes[4], int dir, Mesh *mesh, Topology *g);
  static void detectSharpTriangles(Vertex *vertices[3], Mesh *mesh, Topology *g);
  static bool getSelfQef(Octree *node, Topology *geometry, QefSolver &qef);
  static std::shared_ptr<Octree> buildRecursively(glm::vec3 min, glm::vec3 size, int depth, Topology *geometry);
  static std::shared_ptr<Octree> losslessCompress(std::shared_ptr<Octree> root,
                                                  float threshold,
                                                  Topology *geometry,
                                                  int &count);

  static void calHermite(Octree *node, QefSolver *qef, Topology *g);
  std::shared_ptr<Octree> children[8];
  int childIndex;
  uint8_t cornerSigns[8];
  std::vector<std::shared_ptr<Octree> *> *cluster;
  bool isLeaf;
  bool internal;
  glm::vec3 min;
  glm::vec3 size;
  glm::vec3 *clusterMin;
  glm::vec3 *clusterSize;
  int depth;
  QefSolver qef;
  QefSolver *clusterQef;
  float error;
  Vertex vertex;
};

#endif //VOXELWORLD_OCTREE_H
