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

class Octree;

struct Kdtree {
  typedef std::array<Kdtree *, 2> FaceKd;
  typedef std::array<Kdtree *, 4> EdgeKd;
  RectilinearGrid grid;
  int planeDir;
  int depth;
  bool clusterable {true};
  Kdtree *children[2]{nullptr, nullptr};
  Kdtree(QefSolver sum,
         PositionCode minCode,
         PositionCode maxCode,
         int dir,
         int depth)
      :
      grid(minCode, maxCode, sum),
      planeDir(dir),
      depth(depth) {}
  inline bool isLeaf(float threshold) { return clusterable && (grid.error < threshold || (!children[0] && !children[1])); }
  inline int axis() {
    assert(!isLeaf(-1));
    if (children[0]) {
      return children[0]->grid.maxCode[planeDir];
    }
    return children[1]->grid.minCode[planeDir];
  }
  inline Kdtree *getChild(int i, float threshold) {
    if (grid.error < threshold) {
      return this;
    }
    return children[i];
  }
  ~Kdtree() {
    delete children[0];
    delete children[1];
  }
  void combineQef();
  void calClusterability();
  static void generateVertexIndices(Kdtree *root, Mesh *mesh, Topology *t, float threshold);
  static void contourCell(Kdtree *node, Mesh *mesh, Topology *t, float threshold);
  static void contourFace(FaceKd &nodes,
                          int dir,
                          int axis,
                          Mesh *mesh,
                          Topology *t,
                          float threshold);
  static void detectQuad(EdgeKd &nodes, AALine line, float threshold);
  static void contourEdge(EdgeKd &nodes,
                           const AALine &line,
                           int quadDir1,
                           Topology *t,
                           float threshold,
                           Mesh *mesh);
  static void generateQuad(EdgeKd &nodes, int quadDir1, int quadDir2, Mesh *mesh, Topology *t);
  static int chooseAxisDir(Octree *octree, QefSolver &qef, PositionCode minCode, PositionCode maxCode);
  static Kdtree *buildFromOctree(Octree *octree, PositionCode minCode, PositionCode maxCode, Topology *t, int depth);
  static void drawKdtree(Kdtree *root, Mesh *mesh, float threshold);
  static Mesh *extractMesh(Kdtree *root, Topology *t, float threshold);
};


#endif //VOXELWORLD_KDTREE_H
