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
#include "VolumeData.h"
#include "Vertex.h"
#include "Qef.h"
#include "Utils.h"
#include "Mesh.h"
#include "Kdtree.h"

typedef std::unordered_set<std::set<Vertex *>, ContainerHasher<std::set<Vertex *>>> EdgePolygonSet;

struct Octree {
  public:
  static Octree *buildWithScalarField(PositionCode minCode, int depth, ScalarField *scalarField, bool as_mipmap);
  static void getSum(Octree *root, PositionCode minPos, PositionCode maxPos, QefSolver &out);
  static void simplify(Octree *root, float threshold);
  static void calClusterbility(Octree *root, ScalarField *s);

  static Mesh *extractMesh(Octree *root,
                           ScalarField *geometry,
                           int &intersectionPreservingVerticesCount,
                           bool intersectionFree = true);
  static void drawOctrees(Octree *root, Mesh *mesh);
  void combineComponents(ScalarField *s);
  Octree(PositionCode minCode, PositionCode maxCode, int depth) : grid(minCode, maxCode),
                                                                  childIndex(-1),
                                                                  isLeaf(false),
                                                                  depth(depth){};
  ~Octree() = default;

  protected:
  static void contourCell(Octree *root,
                          Mesh *mesh,
                          ScalarField *geometry,
                          int &intersectionPreservingVerticesCount,
                          EdgePolygonSet &edgePolygonSet,
                          bool intersectionFree,
                          float threshold);
  static void contourFace(Octree *nodes[2],
                          int dir,
                          Mesh *mesh,
                          ScalarField *geometry,
                          int &intersectionPreservingVerticesCount,
                          EdgePolygonSet &edgePolygonSet,
                          bool intersectionFree,
                          float threshold);
  static void contourEdge(Octree **nodes,
                          int dir,
                          int quadDir2,
                          ScalarField *geometry,
                          int &intersectionPreservingVerticesCount,
                          EdgePolygonSet &edgePolygonSet,
                          bool intersectionFree,
                          Mesh *mesh,
                          float threshold);
  static void generateVertexIndices(Octree *node,
                                    Mesh *mesh,
                                    ScalarField *geometry,
                                    std::unordered_set<Vertex *> &indexed);
  static void generateQuad(Octree **nodes,
                           int dir,
                           int quadDir2,
                           ScalarField *g,
                           int &intersectionPreservingVerticesCount,
                           EdgePolygonSet &edgePolygonSet,
                           bool intersectionFree,
                           Mesh *mesh,
                           float threshold);

  public:
  RectilinearGrid grid;
  Octree *children[8]{nullptr};
  int childIndex;
  bool isLeaf;
  bool clusterable{true};
  int depth;
};

#endif //VOXELWORLD_OCTREE_H
