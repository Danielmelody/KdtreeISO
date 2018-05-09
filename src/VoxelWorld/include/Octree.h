//
// Created by Danielhu on 2018/1/14.
//

#ifndef VOXELWORLD_OCTREE_H
#define VOXELWORLD_OCTREE_H

#include <glm/glm.hpp>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <tuple>
#include <memory>
#include "Topology.h"
#include "Vertex.h"
#include "Qef.h"
#include "Utils.h"
#include "Mesh.h"
#include "Kdtree.h"


typedef std::unordered_set<std::set<Vertex *>, ContainerHasher<std::set<Vertex *>>> EdgePolygonSet;

class Octree {
public:
  static Octree *buildWithTopology(PositionCode minCode, int depth, Topology *geometry);
  static void getSum(Octree *root, PositionCode minPos, PositionCode maxPos, QefSolver& out);
  static Kdtree *generateKdtree(Octree *root, PositionCode minCode, PositionCode maxCode, Topology *t, int depth);
  static int simplify(Octree *root, float threshold, Topology *geometry);

  static void cubeExtensionTest(Octree *a, Octree *b, int dir, float minSize);

  static Mesh *extractMesh(Octree *root,
                           Topology *geometry,
                           int &intersectionPreservingVerticesCount,
                           bool intersectionFree = true);
  static void drawOctrees(Octree *root, Mesh *mesh);
  float getError() { return grid.error; }
  Octree(PositionCode minCode, PositionCode maxCode, int depth) :
      grid(minCode, maxCode),
      childIndex(-1),
      isLeaf(false),
      min(codeToPos(minCode, RectilinearGrid::getUnitSize())),
      size(codeToPos(maxCode - minCode, RectilinearGrid::getUnitSize())),
      depth(depth) {};
  ~Octree() = default;
protected:
  static void contourCell(Octree *root,
                          Mesh *mesh,
                          Topology *geometry,
                          int &intersectionPreservingVerticesCount,
                          EdgePolygonSet &edgePolygonSet,
                          bool intersectionFree);
  static void contourFace(Octree *nodes[2],
                          int dir,
                          Mesh *mesh,
                          Topology *geometry,
                          int &intersectionPreservingVerticesCount,
                          EdgePolygonSet &edgePolygonSet,
                          bool intersectionFree);
  static void contourEdge(Octree *nodes[4],
                          int dir,
                          Mesh *mesh,
                          Topology *geometry,
                          int &intersectionPreservingVerticesCount,
                          EdgePolygonSet &edgePolygonSet,
                          bool intersectionFree);
  static void generateVertexIndices(Octree *node,
                                    Mesh *mesh,
                                    Topology *geometry,
                                    std::unordered_set<Vertex *> &indexed);
  static bool findFeatureNodes(Octree *node,
                               std::vector<Octree *> &results,
                               int cornerDir,
                               bool subdivision,
                               const glm::fvec3 &edgeP,
                               const glm::fvec3 &normal);
  static bool intersectWithBrothers(int cornerDir, Octree *node);
  static bool isInterFreeCondition2Faild(const std::vector<Vertex *> &polygons,
                                         const glm::fvec3 &p1,
                                         const glm::fvec3 &p2);
  static void generateQuad(Octree **nodes,
                           int dir,
                           Mesh *mesh,
                           Topology *g,
                           int &intersectionPreservingVerticesCount,
                           EdgePolygonSet &edgePolygonSet,
                           bool intersectionFree);
  static void generatePolygons(Octree *nodes[4], int dir, Mesh *mesh, Topology *g);
  static bool getSelfQef(Octree *node, Topology *geometry, QefSolver &qef);
  static void calHermite(Octree *node, QefSolver *qef, Topology *g, Vertex *vertex);
  RectilinearGrid grid;
  Octree *children[8] {nullptr};
  int childIndex;
  bool isLeaf;
  glm::fvec3 min;
  glm::fvec3 size;
  int depth;
  std::unordered_map<Octree *, Vertex *> faceVertices;
};

#endif //VOXELWORLD_OCTREE_H
