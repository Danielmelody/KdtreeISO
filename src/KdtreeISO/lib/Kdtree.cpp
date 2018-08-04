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

void Kdtree::calClusterability(ScalarField *t) {
  bool selfClusterable = RectilinearGrid::calClusterability(
      children[0] ? &children[0]->grid : nullptr,
      children[1] ? &children[1]->grid : nullptr,
      planeDir, grid.minCode, grid.maxCode, t);
  if (!selfClusterable) {
    clusterable = false;
    return;
  }
  for (auto child : children) {
    if (child && !child->clusterable) {
      clusterable = false;
      return;
    }
  }
  clusterable = true;
  return;
}

void Kdtree::combineQef() {
  if (!clusterable || isLeaf()) {
    return;
  }
  RectilinearGrid::combineAAGrid(children[0] ? &children[0]->grid : nullptr,
                                 children[1] ? &children[1]->grid : nullptr,
                                 planeDir,
                                 &grid);
}

Kdtree *Kdtree::buildFromOctree(Octree *octree, PositionCode minCode, PositionCode maxCode, ScalarField *t, int depth) {
  if (glm::any(glm::greaterThanEqual(minCode, maxCode))) {
    return nullptr;
  }
  QefSolver sum;
  Octree::getSum(octree, minCode, maxCode, sum);
  if (sum.pointCount == 0) {
    return nullptr;
  }

  int strategy = 1;
  PositionCode bestRightMinCode = maxCode, bestLeftMaxCode = minCode;
  int dir = chooseAxisDir(octree, sum, minCode, maxCode);
  int minAxis = minCode[dir];
  int maxAxis = maxCode[dir];
  if (strategy == 0) {
    float error;
    glm::fvec3 approximate;
    sum.solve(approximate, error);
    int plane = static_cast<int>(std::round(approximate[dir] / RectilinearGrid::getUnitSize()));
    if (maxCode[dir] - minCode[dir] > 1) {
      plane = std::min(maxCode[dir] - 1, std::max(minCode[dir] + 1, plane));
      bestLeftMaxCode = maxCode;
      bestRightMinCode = minCode;
      bestLeftMaxCode[dir] = bestRightMinCode[dir] = plane;
    }
  } else {
    QefSolver leftSum, rightSum;
    float minError = 1e20;
    while (maxAxis - minAxis > 1) {
      int mid = (maxAxis + minAxis) / 2;
      PositionCode rightMinCode = minCode;
      rightMinCode[dir] = mid;
      PositionCode leftMaxCode = maxCode;
      leftMaxCode[dir] = mid;
      glm::fvec3 leftApproximate, rightApproximate;
      leftSum.reset();
      rightSum.reset();
      Octree::getSum(octree, minCode, leftMaxCode, leftSum);
      Octree::getSum(octree, rightMinCode, maxCode, rightSum);
      float leftError = 0.f;
      float rightError = 0.f;
      leftSum.solve(leftApproximate, leftError);
      rightSum.solve(rightApproximate, rightError);
      if (abs(leftError - rightError) < minError) {
        minError = abs(leftError - rightError);
        bestLeftMaxCode = leftMaxCode;
        bestRightMinCode = rightMinCode;
      }
      if (leftError > rightError) {
        maxAxis = mid;
      } else if (leftError < rightError) {
        minAxis = mid;
      } else {
        break;
      }
    }
  }
  auto kd = new Kdtree(sum, minCode, maxCode, dir, depth);
  kd->children[0] = buildFromOctree(octree, minCode, bestLeftMaxCode, t, depth + 1);
  kd->children[1] = buildFromOctree(octree, bestRightMinCode, maxCode, t, depth + 1);
  if (kd->isLeaf()) {
    kd->grid.assignSign(t);
    kd->grid.sampleQef(t, false);
  } else {
    kd->grid.assignSign(t);
    kd->calClusterability(t);
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
  if (strategy == 0) {
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
    dir = maxDir;
  } else {
    // variance approach
    glm::fvec3 approximate;
    float error;
    qef.solve(approximate, error);
    auto variance = qef.getVariance(approximate);
    int maxVarDir = 0, minVarDir = 1;
    if (variance[1] > variance[0]) {
      maxVarDir = 1;
      minVarDir = 0;
    }
    if (variance[2] > variance[maxVarDir]) {
      maxVarDir = 2;
    }
    if (variance[minVarDir] > variance[2]) {
      minVarDir = 2;
    }
    dir = maxVarDir;
    if (size[maxVarDir] < 2) {
      dir = 3 - maxVarDir - minVarDir;
      if (size[3 - maxVarDir - minVarDir] < 2) {
        dir = minVarDir;
      }
    }
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

Mesh *Kdtree::extractMesh(Kdtree *root, ScalarField *t, float threshold) {
  Mesh *mesh = new Mesh;
  generateVertexIndices(root, mesh, t, threshold);
  contourCell(root, mesh, t, threshold);
  return mesh;
}

void Kdtree::generateVertexIndices(Kdtree *root, Mesh *mesh, ScalarField *t, float threshold) {
  if (!root) {
    return;
  }
//  mesh->addVertex(&root->grid.approximate, t);
  for (int i = 0; i < root->grid.vertices.size(); ++i) {
    auto &v = root->grid.vertices[i];
    mesh->addVertex(&v, t);
//    v.hermiteN = glm::normalize(root->grid.components[i].averageNormalSum);
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

void Kdtree::contourCell(Kdtree *node, Mesh *mesh, ScalarField *t, float threshold) {
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
                         ScalarField *t,
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
                         ScalarField *t,
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

  if (nodes[0]->isContouringLeaf(threshold) && nodes[1]->isContouringLeaf(threshold)
      && nodes[2]->isContouringLeaf(threshold)
      && nodes[3]->isContouringLeaf(threshold)) {
    // only for debug
    if ((t->value(minEnd) >= 0 && t->value(maxEnd) >= 0) || (t->value(minEnd) < 0 && t->value(maxEnd) < 0)) {
//      return;
    }
    generateQuad(nodes, quadDir1, quadDir2, mesh, t, threshold);
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

void Kdtree::generateQuad(EdgeKd &nodes,
                          int quadDir1,
                          int quadDir2,
                          Mesh *mesh,
                          ScalarField *t,
                          float threshold) {
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

  RectilinearGrid::generateQuad(nodes, quadDir1, quadDir2, mesh, t, threshold);
}
