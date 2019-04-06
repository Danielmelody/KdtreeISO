//
// Created by Danielhu on 2018/4/19.
//

#ifndef VOXELWORLD_KDTREE_H
#define VOXELWORLD_KDTREE_H

#include <vector>
#include <array>
#include "Utils.h"
#include "Qef.h"
#include "RectilinearGrid.h"
#include "Mesh.h"
#include "AxisAlignedLine.h"

struct Octree;

struct Kdtree {
  typedef std::array<Kdtree *, 2> FaceKd;
  typedef std::array<Kdtree *, 4> EdgeKd;
  RectilinearGrid grid;
  int planeDir;
  int depth;
  bool clusterable{true};
  Kdtree *children[2]{nullptr, nullptr};
  Kdtree(QefSolver sum,
         const PositionCode &minCode,
         const PositionCode &maxCode,
         int dir,
         int depth)
    : grid(minCode, maxCode, sum),
      planeDir(dir),
      depth(depth) {}
  inline bool isContouringLeaf(float threshold) const {
    if (!children[0] && !children[1]) {
      return true;
    }
    //    if (grid.error > threshold) {
    //      return false;
    //    }
    for (auto &v : grid.vertices) {
      if (v.error > threshold) {
        return false;
      }
    }
    return clusterable;
  }
  inline bool isLeaf() const {
    return !children[0] && !children[1];
  }
  inline int axis() {
    assert(!isLeaf());
    if (children[0]) {
      return children[0]->grid.maxCode[planeDir];
    }
    return children[1]->grid.minCode[planeDir];
  }
  inline Kdtree *getChild(int i, float threshold) {
    if (grid.approximate.error < threshold) {
      return this;
    }
    return children[i];
  }
  ~Kdtree() {
    delete children[0];
    delete children[1];
  }
  void combineQef();
  void calClusterability(ScalarField *t);
  static void generateVertexIndices(Kdtree *root, Mesh *mesh, ScalarField *t, float threshold);
  static void contourCell(Kdtree *node, Mesh *mesh, ScalarField *t, float threshold);
  static void contourFace(FaceKd &nodes,
                          int dir,
                          int axis,
                          Mesh *mesh,
                          ScalarField *t,
                          float threshold);
  static void detectQuad(EdgeKd &nodes, AALine line, float threshold);
  static void contourEdge(EdgeKd &nodes,
                          const AALine &line,
                          int quadDir1,
                          ScalarField *t,
                          float threshold,
                          Mesh *mesh);
  static void generateQuad(EdgeKd &nodes,
                           int quadDir1,
                           int quadDir2,
                           Mesh *mesh,
                           ScalarField *t,
                           float threshold);
  static int chooseAxisDir(QefSolver &qef, const PositionCode &minCode, const PositionCode &maxCode);
  static Kdtree *buildFromOctree(Octree *octree,
                                 const PositionCode &minCode,
                                 const PositionCode &maxCode,
                                 ScalarField *t,
                                 int depth);
  static void drawKdtree(Kdtree *root, Mesh *mesh, float threshold);
  static Mesh *extractMesh(Kdtree *root, ScalarField *t, float threshold);
};

#endif //VOXELWORLD_KDTREE_H
