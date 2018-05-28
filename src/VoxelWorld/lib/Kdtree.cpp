//
// Created by Danielhu on 2018/4/22.
//


#include <set>
#include <map>
#include "Kdtree.h"
#include "Octree.h"
#include "Indicators.h"
#include "Mesh.h"
#include "AxisAlignedLine.h"

#pragma clang diagnostic ignored "-Wmissing-braces"

using glm::max;
using glm::min;

void Kdtree::calClusterability() {
  bool homogeneous = true;
  for (int i = 1; i < 8; ++i) {
    if (grid.cornerSigns[i] != grid.cornerSigns[0]) {
      homogeneous = false;
    }
  }
  if (homogeneous) {
    clusterable = false;
    return;
  }
  if (!children[0] && !children[1]) {
    clusterable = true;
    return;
  }
  if (children[0] && !children[1]) {
    clusterable = children[0]->clusterable;
    return;
  }
  if (children[1] && !children[0]) {
    clusterable = children[1]->clusterable;
    return;
  }
  // assume cal clusterable from bottom-up
  if (!children[0]->clusterable || !children[1]->clusterable) {
    clusterable = false;
    return;
  }
  for (int i = 0; i < 4; ++i) {
    int edgeMinIndex = cellProcFaceMask[planeDir * 4 + i][0];
    int edgeMaxIndex = cellProcFaceMask[planeDir * 4 + i][1];
    int signChanges = 0;
    for (int j = 0; j < 2; ++j) {
      if (children[j]->grid.cornerSigns[edgeMinIndex] != children[j]->grid.cornerSigns[edgeMaxIndex]) {
        signChanges++;
      }
    }
    if (signChanges > 1) {
      clusterable = false;
      return;
    }
  }
}

void Kdtree::combineQef() {
  if (!clusterable || isLeaf()) {
    return;
  }
  std::map<int, int> combineMaps[2];
  grid.calCornerComponents();
  for (int i = 0; i < 4; ++i) {
    int c = -1;
    for (int j = 0; j < 2; ++j) {
      if (grid.cornerSigns[cellProcFaceMask[planeDir * 4 + i][j]] != 0) {
        c = grid.componentIndices[cellProcFaceMask[planeDir * 4 + i][j]];
        break;
      }
    }
    if (c == -1) {
      continue;
    }
    for (int j = 0; j < 2; ++j) {
      auto child = children[j];
      if (child) {
        for (int k = 0; k < 2; ++k) {
          if (child->grid.cornerSigns[cellProcFaceMask[planeDir * 4 + i][k]] != 0) {
            int childC = child->grid.componentIndices[cellProcFaceMask[planeDir * 4 + i][k]];
            assert(child->grid.components[childC].pointCount > 0);
            combineMaps[j][c] = childC;
            break;
          }
        }
      }
    }
  }
  for (int i = 0; i < 2; ++i) {
    for (auto p : combineMaps[i]) {
      grid.components.at(p.first).combine(children[i]->grid.components.at(p.second));
    }
  }
}

Kdtree *Kdtree::buildFromOctree(Octree *octree, PositionCode minCode, PositionCode maxCode, Topology *t, int depth) {
  if (glm::any(glm::greaterThanEqual(minCode, maxCode))) {
    return nullptr;
  }
  QefSolver sum;
  Octree::getSum(octree, minCode, maxCode, sum);
  if (sum.pointCount == 0) {
    return nullptr;
  }
  PositionCode bestRightMinCode = maxCode, bestLeftMaxCode = minCode;
  float minErrorDiff = 1e20;
  QefSolver leftSum, rightSum;
  int dir = chooseAxisDir(octree, sum, minCode, maxCode);
  for (int axis = minCode[dir] + 1; axis < maxCode[dir]; ++axis) {
    PositionCode rightMinCode = minCode;
    rightMinCode[dir] = axis;
    PositionCode leftMaxCode = maxCode;
    leftMaxCode[dir] = axis;
    glm::fvec3 leftApproximate, rightApproximate;
    leftSum.reset();
    rightSum.reset();
    float leftError = 0.f;
    float rightError = 0.f;
    Octree::getSum(octree, minCode, leftMaxCode, leftSum);
    Octree::getSum(octree, rightMinCode, maxCode, rightSum);
    leftSum.solve(leftApproximate, leftError);
    rightSum.solve(rightApproximate, rightError);
//    leftError = leftSum.getVariance(leftApproximate)[dir];
//    rightError = rightSum.getVariance(rightApproximate)[dir];
    if (abs(rightError - leftError) < minErrorDiff) {
      minErrorDiff = abs(rightError - leftError);
      bestLeftMaxCode = leftMaxCode;
      bestRightMinCode = rightMinCode;
    }
  }
  auto kd = new Kdtree(sum, minCode, maxCode, dir, depth);
  kd->children[0] = buildFromOctree(octree, minCode, bestLeftMaxCode, t, depth + 1);
  kd->children[1] = buildFromOctree(octree, bestRightMinCode, maxCode, t, depth + 1);
  if (kd->isLeaf()) {
    kd->grid.assignSign(t);
    kd->grid.sampleQef(t);
  } else {
    kd->grid.assignSign(t);
    kd->calClusterability();
    kd->combineQef();
  }
  if (kd->clusterable) {
    for (int i = 0; i < kd->grid.components.size(); ++i) {
      kd->grid.solveComponent(i);
    }
  }
  return kd;
}

int Kdtree::chooseAxisDir(Octree *octree, QefSolver &qef, PositionCode minCode, PositionCode maxCode) {
  // naive approach
  int dir = 0;
  int strategy = 1;
  auto size = maxCode - minCode;
  int maxDir = 0, minDir = 1;
  if (size[1] > size[0]) {
    maxDir = 1;
    minDir = 0;
  }
  if (size[2] > size[maxDir]) {
    maxDir = 2;
  }
  if (size[minDir] > size[2]) {
    minDir = 2;
  }
  switch (strategy) {
  case 0:dir = maxDir;
    break;
  case 1:
    // variance approach
    glm::fvec3 approximate;
    float error;
    qef.solve(approximate, error);
    auto variance = qef.getVariance(approximate);
    if (variance[1] > variance[0]) {
      dir = 1;
    }
    if (variance[2] > variance[dir]) {
      dir = 2;
    }
    if (size[dir] < 2) {
      dir = maxDir;
    }
    break;
  }
  return dir;
}

void Kdtree::drawKdtree(Kdtree *root, Mesh *mesh, float threshold) {
  if (!root) {
    return;
  }
  if (root->isContouringLeaf(threshold)) {
    root->grid.draw(mesh);
    return;
  }
  drawKdtree(root->children[0], mesh, threshold);
  drawKdtree(root->children[1], mesh, threshold);
}

Mesh *Kdtree::extractMesh(Kdtree *root, Topology *t, float threshold) {
  Mesh *mesh = new Mesh;
  generateVertexIndices(root, mesh, t, threshold);
  contourCell(root, mesh, t, threshold);
  return mesh;
}

void Kdtree::generateVertexIndices(Kdtree *root, Mesh *mesh, Topology *t, float threshold) {
  if (!root) {
    return;
  }
  for (auto &v : root->grid.vertices) {
    mesh->addVertex(&v, t);
  }
  generateVertexIndices(root->children[0], mesh, t, threshold);
  generateVertexIndices(root->children[1], mesh, t, threshold);
}

AALine constructLine(const Kdtree::FaceKd &faceNodes, int side, int originFaceDir, int axis, float threshold) {
  AALine line;
  line.point[originFaceDir] = axis;
  assert(!faceNodes[side]->isContouringLeaf(threshold));
  line.dir = 3 - originFaceDir - faceNodes[side]->planeDir;
  line.point[faceNodes[side]->planeDir] = faceNodes[side]->axis();
  return line;
}

void Kdtree::contourCell(Kdtree *node, Mesh *mesh, Topology *t, float threshold) {
  if (!node || node->isContouringLeaf(threshold)) {
    return;
  }
  FaceKd faceNodes = {node->children[0], node->children[1]};
  contourFace(faceNodes, node->planeDir, node->axis(), mesh, t, threshold);
  contourCell(node->children[0], mesh, t, threshold);
  contourCell(node->children[1], mesh, t, threshold);
}

bool checkMinialFace(const Kdtree::FaceKd &nodes, int dir, PositionCode &faceMin, PositionCode &faceMax) {
  faceMax = min(nodes[0]->grid.maxCode, nodes[1]->grid.maxCode);
  faceMin = max(nodes[0]->grid.minCode, nodes[1]->grid.minCode);
  auto offset = faceMax - faceMin;
  return offset[(dir + 1) % 3] > 0 && offset[(dir + 2) % 3] > 0;
}

void Kdtree::contourFace(FaceKd &nodes,
                         const int dir,
                         const int axis,
                         Mesh *mesh,
                         Topology *t,
                         float threshold) {
  if (!nodes[0] || !nodes[1]) {
    return;
  }
  if (nodes[0]->isContouringLeaf(threshold) && nodes[1]->isContouringLeaf(threshold)) {
    return;
  }

  PositionCode faceMin, faceMax;
  if (!checkMinialFace(nodes, dir, faceMin, faceMax)) {
    return;
  }

  for (int i = 0; i < 2; ++i) {
    while (!nodes[i]->isContouringLeaf(threshold) && nodes[i]->planeDir == dir) {
      nodes[i] = nodes[i]->children[1 - i];
      if (!nodes[i]) {
        return;
      }
    }
  }

  for (int i = 0; i < 2; ++i) {
    if (!nodes[i]->isContouringLeaf(threshold)) {
      for (int j = 0; j < 2; ++j) {
        FaceKd nextFace = nodes;
        nextFace[i] = nodes[i]->children[j];
        contourFace(nextFace, dir, axis, mesh, t, threshold);
      }
      if (nodes[i]->axis() > faceMin[nodes[i]->planeDir] && nodes[i]->axis() < faceMax[nodes[i]->planeDir]) {
        EdgeKd edgeNodes = {nodes[0], nodes[0], nodes[1], nodes[1]};
        edgeNodes[i * 2] = nodes[i]->children[0];
        edgeNodes[i * 2 + 1] = nodes[i]->children[1];
        AALine line = constructLine(nodes, i, dir, axis, threshold);
        contourEdge(edgeNodes, line, nodes[i]->planeDir, t, threshold, mesh);
      }
      return;
    }
  }
}

bool checkMinialEdge(const Kdtree::EdgeKd &nodes, const AALine &line, PositionCode &minEnd, PositionCode &maxEnd) {
  minEnd = maxEnd = line.point;
  int dir = line.dir;
  minEnd[dir] = max(max(nodes[0]->grid.minCode, nodes[1]->grid.minCode),
                    max(nodes[2]->grid.minCode, nodes[3]->grid.minCode))[dir];
  maxEnd[dir] = min(min(nodes[0]->grid.maxCode, nodes[1]->grid.maxCode),
                    min(nodes[2]->grid.maxCode, nodes[3]->grid.maxCode))[dir];
  return minEnd[dir] < maxEnd[dir];
}

int nextQuadIndex(int dir1, int dir2, int planeDir, int i) {
  PositionCode pos;
  pos[dir1] = 1 - i % 2;
  pos[dir2] = 1 - i / 2;
  return pos[planeDir];
}

void Kdtree::detectQuad(EdgeKd &nodes, AALine line, float threshold) {
  for (int i = 0; i < 2; ++i) {
    while (
        nodes[i * 2] && nodes[i * 2 + 1]
            && !nodes[i * 2]->isContouringLeaf(threshold)
            && nodes[2 * i] == nodes[2 * i + 1]
            && nodes[i * 2]->planeDir != line.dir) {
      auto commonNode = nodes[i * 2];
      if (nodes[i * 2]->axis() == line.point[nodes[i * 2]->planeDir]) {
        nodes[i * 2] = commonNode->children[0];
        nodes[i * 2 + 1] = commonNode->children[1];
      } else if (nodes[i * 2]->axis() > line.point[nodes[i * 2]->planeDir]) {
        nodes[i * 2] = commonNode->children[0];
        nodes[i * 2 + 1] = commonNode->children[0];
      } else {
        nodes[i * 2] = commonNode->children[1];
        nodes[i * 2 + 1] = commonNode->children[1];
      }
    }
  }
}

void setQuadNode(Kdtree::EdgeKd &nodes, int i, Kdtree *p) {
  if (nodes[oppositeQuadIndex(i)] == nodes[i]) {
    nodes[oppositeQuadIndex(i)] = p;
  }
  nodes[i] = p;
}

void Kdtree::contourEdge(EdgeKd &nodes,
                         const AALine &line,
                         const int quadDir1,
                         Topology *t,
                         float threshold,
                         Mesh *mesh) {
  detectQuad(nodes, line, threshold);
  for (auto n : nodes) {
    if (!n) {
      return;
    }
  }
  assert(quadDir1 >= 0 && quadDir1 < 3);
  const int quadDir2 = 3 - quadDir1 - line.dir;
  PositionCode minEndCode, maxEndCode;
  if (!checkMinialEdge(nodes, line, minEndCode, maxEndCode)) {
    return;
  }
  glm::fvec3 minEnd = codeToPos(minEndCode, RectilinearGrid::getUnitSize());
  glm::fvec3 maxEnd = codeToPos(maxEndCode, RectilinearGrid::getUnitSize());
  for (int i = 0; i < 4; ++i) {
    if (nodes[i] != nodes[oppositeQuadIndex(i)]) {
      while (!nodes[i]->isContouringLeaf(threshold) && nodes[i]->planeDir != line.dir) {
        nodes[i] = nodes[i]->children[nextQuadIndex(quadDir1, quadDir2, nodes[i]->planeDir, i)];
        if (!nodes[i]) {
          return;
        }
      }
    }
  }
//  assert(nodes[0]->grid.minCode[quadDir1] <= nodes[1]->grid.minCode[quadDir1]);
//  assert(nodes[2]->grid.minCode[quadDir1] <= nodes[3]->grid.minCode[quadDir1]);
//  assert(nodes[0]->grid.minCode[quadDir2] <= nodes[2]->grid.minCode[quadDir2]);
//  assert(nodes[1]->grid.minCode[quadDir2] <= nodes[3]->grid.minCode[quadDir2]);

//  if ((maxEndCode - minEndCode)[0] == 1) {
//    mesh->drawAABBDebug(codeToPos(minEndCode, RectilinearGrid::getUnitSize()), codeToPos(maxEndCode, RectilinearGrid::getUnitSize()));
//  }

  if (nodes[0]->isContouringLeaf(threshold) && nodes[1]->isContouringLeaf(threshold) && nodes[2]->isContouringLeaf(threshold)
      && nodes[3]->isContouringLeaf(threshold)) {
    // only for debug
    if ((t->value(minEnd) >= 0 && t->value(maxEnd) >= 0) || (t->value(minEnd) < 0 && t->value(maxEnd) < 0)) {
      return;
    }
    generateQuad(nodes, quadDir1, quadDir2, mesh, t);
    return;
  }
  for (int i = 0; i < 4; ++i) {
    EdgeKd nextNodes = nodes;
    if (!nodes[i]->isContouringLeaf(threshold) && nodes[i]->planeDir == line.dir) {
      setQuadNode(nextNodes, i, nodes[i]->children[0]);
      contourEdge(nextNodes, line, quadDir1, t, threshold, mesh);
      nextNodes = nodes;
      setQuadNode(nextNodes, i, nodes[i]->children[1]);
      contourEdge(nextNodes, line, quadDir1, t, threshold, mesh);
      return;
    }
  }
}

void Kdtree::generateQuad(EdgeKd &nodes, int quadDir1, int quadDir2, Mesh *mesh, Topology *t) {
//  for (auto n : nodes) {
//    n->grid.vertices[0].hermiteP = codeToPos(n->grid.minCode + n->grid.maxCode, RectilinearGrid::getUnitSize()) / 2.f;
//    // mesh->drawAABBDebug(codeToPos(n->grid.minCode, RectilinearGrid::getUnitSize()), codeToPos(n->grid.maxCode, RectilinearGrid::getUnitSize()));
//  }

  // return;

//  assert(glm::all(glm::greaterThanEqual(nodes[1]->grid.minCode, nodes[0]->grid.minCode)));
//  assert(glm::all(glm::greaterThanEqual(nodes[3]->grid.minCode, nodes[2]->grid.minCode)));
//  assert(glm::all(glm::greaterThanEqual(nodes[3]->grid.minCode, nodes[0]->grid.minCode)));
//  assert(glm::all(glm::greaterThanEqual(nodes[3]->grid.minCode, nodes[1]->grid.minCode)));

//  for (auto &n : nodes) {
//    assert(n->clusterable);
//    for (auto c : n->grid.vertices) {
//      assert(c.vertexIndex);
//    }
//  }

  RectilinearGrid::generateQuad(nodes, quadDir1, quadDir2, mesh, t);
}
