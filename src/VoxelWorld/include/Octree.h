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
#include <unordered_map>
#include <memory>

struct Vertex {
  unsigned int vertexIndex;
  glm::vec3 hermiteP;
  glm::vec3 hermiteN;
public:
  Vertex() : vertexIndex(0), hermiteP(glm::vec3(0)), hermiteN(glm::vec3(0)) {}
};

class Octree {
public:
  static Octree *buildWithTopology(glm::vec3 min,
                                   glm::vec3 size,
                                   int depth,
                                   Topology *topology,
                                   int &loselessCut);
  static int simplify(Octree *root, float threshold, Topology *geometry);
  static void reverseExtendedSimplify(Octree *root, Topology *g);
  static Octree *extendedSimplify(Octree *root,
                                  float threshold,
                                  Topology *geometry,
                                  int &count);
  static Octree *edgeClassifier(Octree *root,
                                float threshold,
                                Topology *geometry);
  static void edgeCluster(Octree *root,
                          Topology *geometry,
                          int &count,
                          std::unordered_set<std::unordered_set<Octree *> *> &clusters
  );

  static void edgeCollapse(Octree *&a,
                           Octree *&b,
                           int dir,
                           float threshold,
                           Topology *geometry,
                           glm::vec3 faceMin,
                           float faceSize);
  static void cubeExtensionTest(Octree *a, Octree *b, int dir, float minSize);

  static Mesh *generateMesh(Octree *root,
                            Topology *geometry,
                            int &intersectionPreservingVerticesCount,
                            bool intersectionFree = true);
  static void drawOctrees(Octree *root, Mesh *mesh, std::unordered_set<Vertex *> &visited);
  float getError() { return error; }
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
    cluster = new std::unordered_set<Octree *>({this});
    clusterQef = new QefSolver();
    clusterMin = new glm::vec3(this->min);
    clusterSize = new glm::vec3(this->size);
    clusterVertex = &vertex;
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
  static void contourCell(Octree *root,
                          Mesh *mesh,
                          Topology *geometry,
                          int &intersectionPreservingVerticesCount,
                          bool intersectionFree);
  static void contourFace(Octree *nodes[2],
                          int dir,
                          Mesh *mesh,
                          Topology *geometry,
                          int &intersectionPreservingVerticesCount,
                          bool intersectionFree);
  static void contourEdge(Octree *nodes[4],
                          int dir,
                          Mesh *mesh,
                          Topology *geometry,
                          int &intersectionPreservingVerticesCount,
                          bool intersectionFree);
  static void combine(Octree *a, Octree *b, Topology *g);
  static void generateVertexIndices(Octree *node,
                                    Mesh *mesh,
                                    Topology *geometry,
                                    std::unordered_set<Vertex *> &indexed);
  static bool findFeatureNodes(Octree *node,
                               std::vector<Octree *> &results,
                               const int cornerDir,
                               bool subdivision,
                               const glm::vec3 &edgeP,
                               const glm::vec3 &normal);
  static bool intersectWithBrothers(int cornerDir, Octree *node);
  static bool isInterFreeCondition2Faild(const std::vector<Vertex *> &polygons,
                                         const glm::vec3 &p1,
                                         const glm::vec3 &p2);
  static void generateQuad(Octree *nodes[4],
                           int dir,
                           Mesh *mesh,
                           Topology *g,
                           int &intersectionPreservingVerticesCount,
                           bool intersectionFree);
  static void generatePolygons(Octree *nodes[4], int dir, Mesh *mesh, Topology *g);
  static void detectSharpTriangles(Vertex *vertices[3], Mesh *mesh, Topology *g);
  static bool getSelfQef(Octree *node, Topology *geometry, QefSolver &qef);
  static Octree *buildRecursively(glm::vec3 min, glm::vec3 size, int depth, Topology *geometry);
  static Octree *losslessCompress(Octree *root,
                                  float threshold,
                                  Topology *geometry,
                                  int &count);

  static void calHermite(Octree *node, QefSolver *qef, Topology *g, Vertex *vertex);
  Octree *children[8];
  int childIndex;
  uint8_t cornerSigns[8];
  bool isLeaf;
  bool internal;
  glm::vec3 min;
  glm::vec3 size;
  int depth;
  QefSolver qef;
  float error;
  Vertex vertex;
  std::unordered_map<Octree*, Vertex*> faceVertices;
  std::unordered_set<Octree *> *cluster;
  glm::vec3 *clusterMin;
  glm::vec3 *clusterSize;
  QefSolver *clusterQef;
  Vertex *clusterVertex;
};

#endif //VOXELWORLD_OCTREE_H
