//
// Created by Danielhu on 2018/4/19.
//

#ifndef VOXELWORLD_KDTREE_H
#define VOXELWORLD_KDTREE_H

#include <vector>
#include <array>
#include "Utils.h"
#include "Qef.h"
#include "Mesh.h"
#include "AxisAlignedLine.h"

struct Kdtree {
  typedef std::array<Kdtree *, 2> FaceKd;
  typedef std::array<Kdtree *, 4> EdgeKd;
  QefSolver sum;
  PositionCode minCode;
  PositionCode maxCode;
  int planeDir;
  int depth;
  std::vector<Vertex> vertices;
  float error{0.f};
  Kdtree *children[2]{nullptr, nullptr};
  Kdtree(QefSolver sum,
         PositionCode minCode,
         PositionCode maxCode,
         int dir,
         int depth)
      :
      sum(sum),
      minCode(minCode),
      maxCode(maxCode),
      planeDir(dir),
      depth(depth) {
    vertices.resize(1);
    sum.solve(vertices[0].hermiteP, error);
  }
  inline bool isLeaf() { return !children[0] && !children[1]; }
  inline bool isLeaf(float threshold) { return error < threshold || (!children[0] && !children[1]); }
  inline int axis() {
    assert(!isLeaf());
    if (children[0]) {
      return children[0]->maxCode[planeDir];
    }
    return children[1]->minCode[planeDir];
  }
  inline Kdtree *getChild(int i, float threshold) {
    if (error < threshold) {
      return this;
    }
    return children[i];
  }
  ~Kdtree() {
    delete children[0];
    delete children[1];
  }
  static void drawKdtree(Kdtree *root, Mesh *mesh, float threshold);
  static Mesh *extractMesh(Kdtree *root, Topology *t, float threshold);
  static void generateVertexIndices(Kdtree *root, Mesh *mesh, Topology *t, float threshold);
  static void contourCell(Kdtree *node, Mesh *mesh, Topology *t, float threshold);
  static void contourFace(FaceKd &nodes,
                          int dir,
                          int axis,
                          Mesh *mesh,
                          Topology *t,
                          float threshold);

  static void contourEdge(EdgeKd &nodes,
                          const AALine &line,
                          Mesh *mesh,
                          Topology *t,
                          float threshold);
  static void detectQuad(EdgeKd &nodes, AALine line, float threshold);
  static void generateQuad(EdgeKd &nodes, Mesh *mesh, Topology *t);
};

#endif //VOXELWORLD_KDTREE_H
