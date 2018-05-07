//
// Created by Danielhu on 2018/4/22.
//



#include "Kdtree.h"
#include "Octree.h"
#include "Indicators.h"
#include "Mesh.h"
#include "AxisAlignedLine.h"

#pragma clang diagnostic ignored "-Wmissing-braces"

using glm::max;
using glm::min;

void Kdtree::drawKdtree(Kdtree *root, Mesh *mesh, float threshold) {
  if (!root) {
    return;
  }
  if (root->isLeaf(threshold)) {
    vec3 size = codeToPos(root->maxCode - root->minCode, Octree::cellSize);
    vec3 min = codeToPos(root->minCode, Octree::cellSize);
    for (int i = 0; i < 12; ++i) {
      auto a = min_offset_subdivision(cellProcFaceMask[i][0]) * size + min;
      auto b = min_offset_subdivision(cellProcFaceMask[i][1]) * size + min;

      auto na = normalize(min_offset_subdivision(cellProcFaceMask[i][0]) - vec3(0.5f));
      auto nb = normalize(min_offset_subdivision(cellProcFaceMask[i][1]) - vec3(0.5f));

      mesh->positions.push_back(a);
      mesh->positions.push_back(a);
      mesh->positions.push_back(b);
      mesh->normals.push_back(na);
      mesh->normals.push_back(na);
      mesh->normals.push_back(nb);
      mesh->indices.push_back(static_cast<unsigned int &&>(mesh->indices.size()));
      mesh->indices.push_back(static_cast<unsigned int &&>(mesh->indices.size()));
      mesh->indices.push_back(static_cast<unsigned int &&>(mesh->indices.size()));
    }
    return;
  }
  drawKdtree(root->children[0], mesh, 0);
  drawKdtree(root->children[1], mesh, 0);
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
  mesh->addVertex(&root->vertices[0], t);
  if (root->error >= threshold) {
    generateVertexIndices(root->children[0], mesh, t, threshold);
    generateVertexIndices(root->children[1], mesh, t, threshold);
  }
}

AALine constructLine(const Kdtree::FaceKd &faceNodes, int side, int originFaceDir, int axis, float threshold) {
  AALine line;
  line.point[originFaceDir] = axis;
  assert(!faceNodes[side]->isLeaf(threshold));
  line.dir = 3 - originFaceDir - faceNodes[side]->planeDir;
  line.point[faceNodes[side]->planeDir] = faceNodes[side]->axis();
  return line;
}

void Kdtree::contourCell(Kdtree *node, Mesh *mesh, Topology *t, float threshold) {
  if (!node || node->isLeaf(threshold)) {
    return;
  }
  FaceKd faceNodes = {node->children[0], node->children[1]};
  contourFace(faceNodes, node->planeDir, node->axis(), mesh, t, threshold);
  contourCell(node->children[0], mesh, t, threshold);
  contourCell(node->children[1], mesh, t, threshold);
}

bool checkMinialFace(const Kdtree::FaceKd &nodes, int dir, PositionCode &faceMin, PositionCode &faceMax) {
  faceMax = min(nodes[0]->maxCode, nodes[1]->maxCode);
  faceMin = max(nodes[0]->minCode, nodes[1]->minCode);
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
  if (nodes[0]->isLeaf(threshold) && nodes[1]->isLeaf(threshold)) {
    return;
  }

  PositionCode faceMin, faceMax;
  if (!checkMinialFace(nodes, dir, faceMin, faceMax)) {
    return;
  }

  for (int i = 0; i < 2; ++i) {
    while (!nodes[i]->isLeaf(threshold) && nodes[i]->planeDir == dir) {
      nodes[i] = nodes[i]->children[1 - i];
      if (!nodes[i]) {
        return;
      }
    }
  }

  for (int i = 0; i < 2; ++i) {
    if (!nodes[i]->isLeaf(threshold)) {
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
  minEnd[dir] = max(max(nodes[0]->minCode, nodes[1]->minCode), max(nodes[2]->minCode, nodes[3]->minCode))[dir];
  maxEnd[dir] = min(min(nodes[0]->maxCode, nodes[1]->maxCode), min(nodes[2]->maxCode, nodes[3]->maxCode))[dir];
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
    while (!nodes[i * 2]->isLeaf(threshold)
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

constexpr int oppositeQuadIndex(int i) {
  return (i / 2) * 2 + 1 - i % 2;
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
  if (!nodes[0] || !nodes[1] || !nodes[2] || !nodes[3]) {
    return;
  }
  assert(quadDir1 >= 0 && quadDir1 < 3);
  const int quadDir2 = 3 - quadDir1 - line.dir;
  PositionCode minEndCode, maxEndCode;
  if (!checkMinialEdge(nodes, line, minEndCode, maxEndCode)) {
    return;
  }
  glm::vec3 minEnd = codeToPos(minEndCode, Octree::cellSize);
  glm::vec3 maxEnd = codeToPos(maxEndCode, Octree::cellSize);

  if (line.point[2] == 1 && line.point[1] == 0 && line.dir == 0) { ;
  }
  detectQuad(nodes, line, threshold);
  for (int i = 0; i < 4; ++i) {
    if (nodes[i] != nodes[oppositeQuadIndex(i)]) {
      while (!nodes[i]->isLeaf(threshold) && nodes[i]->planeDir != line.dir) {
        nodes[i] = nodes[i]->children[nextQuadIndex(quadDir1, quadDir2, nodes[i]->planeDir, i)];
        if (!nodes[i]) {
          return;
        }
      }
    }
  }
//  assert(nodes[0]->minCode[quadDir1] <= nodes[1]->minCode[quadDir1]);
//  assert(nodes[2]->minCode[quadDir1] <= nodes[3]->minCode[quadDir1]);
//  assert(nodes[0]->minCode[quadDir2] <= nodes[2]->minCode[quadDir2]);
//  assert(nodes[1]->minCode[quadDir2] <= nodes[3]->minCode[quadDir2]);

//  if ((maxEndCode - minEndCode)[0] == 1) {
//    mesh->drawAABBDebug(codeToPos(minEndCode, Octree::cellSize), codeToPos(maxEndCode, Octree::cellSize));
//  }

  if (nodes[0]->isLeaf(threshold) && nodes[1]->isLeaf(threshold) && nodes[2]->isLeaf(threshold)
      && nodes[3]->isLeaf(threshold)) {
    // only for debug
    if ((t->value(minEnd) >= 0 && t->value(maxEnd) >= 0) || (t->value(minEnd) < 0 && t->value(maxEnd) < 0)) {
      return;
    }
    generateQuad(nodes, mesh, t);
    return;
  }
  for (int i = 0; i < 4; ++i) {
    EdgeKd nextNodes = nodes;
    if (!nodes[i]->isLeaf(threshold) && nodes[i]->planeDir == line.dir) {
      setQuadNode(nextNodes, i, nodes[i]->children[0]);
      contourEdge(nextNodes, line, quadDir1, t, threshold, mesh);
      nextNodes = nodes;
      setQuadNode(nextNodes, i, nodes[i]->children[1]);
      contourEdge(nextNodes, line, quadDir1, t, threshold, mesh);
      return;
    }
  }
}

void Kdtree::generateQuad(EdgeKd &nodes, Mesh *mesh, Topology *t) {
//  for (auto n : nodes) {
//    n->vertices[0].hermiteP = codeToPos(n->minCode + n->maxCode, Octree::cellSize) / 2.f;
//    // mesh->drawAABBDebug(codeToPos(n->minCode, Octree::cellSize), codeToPos(n->maxCode, Octree::cellSize));
//  }

  // return;

  std::vector<Vertex *> polygons;

//  assert(glm::all(glm::greaterThanEqual(nodes[1]->minCode, nodes[0]->minCode)));
//  assert(glm::all(glm::greaterThanEqual(nodes[3]->minCode, nodes[2]->minCode)));
//  assert(glm::all(glm::greaterThanEqual(nodes[3]->minCode, nodes[0]->minCode)));
//  assert(glm::all(glm::greaterThanEqual(nodes[3]->minCode, nodes[1]->minCode)));
  polygons.push_back(&nodes[0]->vertices[0]);
  if (nodes[0] != nodes[1]) {
    polygons.push_back(&nodes[1]->vertices[0]);
  }
  polygons.push_back(&nodes[3]->vertices[0]);
  if (nodes[2] != nodes[3]) {
    polygons.push_back(&nodes[2]->vertices[0]);
  }
  for (int i = 0; i < polygons.size() - 2; ++i) {
    Vertex *triangles[] = {polygons[0], polygons[i + 1], polygons[i + 2]};
    mesh->addTriangle(triangles, t);
  }
}
