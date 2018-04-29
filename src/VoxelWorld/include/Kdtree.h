//
// Created by Danielhu on 2018/4/19.
//

#ifndef VOXELWORLD_KDTREE_H
#define VOXELWORLD_KDTREE_H

#include "Utils.h"
#include "Qef.h"
#include "Mesh.h"

struct Kdtree {
  QefSolver sum;
  OctCodeType minCode;
  OctCodeType maxCode;
  Kdtree *left;
  Kdtree *right;
  int depth;
  Kdtree(QefSolver sum,
         OctCodeType minCode,
         OctCodeType maxCode,
         int depth,
         Kdtree *left = nullptr,
         Kdtree *right = nullptr)
      : sum(sum),
        minCode(minCode),
        maxCode(maxCode),
        left(left),
        right(right),
        depth(depth) {}
  ~Kdtree() {
    delete left;
    delete right;
  }
  static void drawKdtree(Kdtree *root, Mesh *mesh);
};

#endif //VOXELWORLD_KDTREE_H
