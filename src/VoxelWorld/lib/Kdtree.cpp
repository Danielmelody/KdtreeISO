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

AALine constructLine(const Kdtree::FaceKd &faceNodes, int side, int planeDir, int axis, float threshold) {
  AALine line;
  line.point[planeDir] = axis;
  if (!faceNodes[side]->isLeaf(threshold)) {
    line.dir = 3 - planeDir - faceNodes[side]->planeDir;
    line.point[faceNodes[side]->planeDir] = faceNodes[side]->axis();
  }
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

bool checkMinialFace(const Kdtree::FaceKd &nodes, int dir) {
  auto minialFace = min(nodes[0]->maxCode, nodes[1]->maxCode) - max(nodes[0]->minCode, nodes[1]->minCode);
  return minialFace[(dir + 1) % 3] > 0 && minialFace[(dir + 2) % 3] > 0;
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
  if (!checkMinialFace(nodes, dir)) {
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
  AALine line1 = constructLine(nodes, 0, dir, axis, threshold);
  AALine line2 = constructLine(nodes, 1, dir, axis, threshold);
  if (!nodes[0]->isLeaf(threshold)) {
    EdgeKd edgeNodes = {nodes[0]->children[0], nodes[0]->children[1], nodes[1], nodes[1]};
    contourEdge(edgeNodes, line1, mesh, t, threshold);
    FaceKd face1 = {nodes[0]->children[0], nodes[1]};
    FaceKd face2 = {nodes[0]->children[1], nodes[1]};
    contourFace(face1, dir, axis, mesh, t, threshold);
    contourFace(face2, dir, axis, mesh, t, threshold);
  } else if (!nodes[1]->isLeaf(threshold)) {
    EdgeKd edgeNodes2 = {nodes[0], nodes[0], nodes[1]->children[0], nodes[1]->children[1]};
    contourEdge(edgeNodes2, line2, mesh, t, threshold);
    FaceKd face1 = {nodes[0], nodes[1]->children[0]};
    FaceKd face2 = {nodes[0], nodes[1]->children[1]};
    contourFace(face1, dir, axis, mesh, t, threshold);
    contourFace(face2, dir, axis, mesh, t, threshold);
  }
}

void Kdtree::detectQuad(EdgeKd &nodes, AALine line, float threshold) {
  if (!nodes[0]->isLeaf(threshold)
      && nodes[0] == nodes[1]
      && nodes[0]->axis() == line.point[nodes[0]->planeDir]
      && nodes[0]->planeDir != line.dir) {
    auto commonNode = nodes[0];
    nodes[0] = commonNode->children[0];
    nodes[1] = commonNode->children[1];
  }
  if (!nodes[2]->isLeaf(threshold)
      && nodes[2] == nodes[3]
      && nodes[2]->axis() == line.point[nodes[2]->planeDir]
      && nodes[2]->planeDir != line.dir) {
    auto commonNode = nodes[2];
    nodes[2] = commonNode->children[0];
    nodes[3] = commonNode->children[1];
  }
}

void setEdgeNode(Kdtree::EdgeKd &nodes, int i, Kdtree *p) {
  nodes[i] = p;
  int oppositeI = (i / 2) * 2 + (1 - i % 2);
  if (nodes[oppositeI] == nodes[i]) {
    nodes[oppositeI] = nodes[i];
  }
}

bool checkMinialEdge(const Kdtree::EdgeKd &nodes, int dir) {
  PositionCode minCode =
      max(max(nodes[0]->minCode, nodes[1]->minCode), max(nodes[2]->minCode, nodes[3]->minCode));
  PositionCode maxCode =
      min(min(nodes[0]->maxCode, nodes[1]->maxCode), min(nodes[2]->maxCode, nodes[3]->maxCode));
  return minCode[dir] < maxCode[dir];
}

void Kdtree::contourEdge(EdgeKd &nodes, const AALine &line, Mesh *mesh, Topology *t, float threshold) {
  if (!nodes[0] || !nodes[1] || !nodes[2] || !nodes[3]) {
    return;
  }

  for (int i = 0; i < 4; ++i) {
    while (!nodes[i]->isLeaf(threshold) && nodes[i]->planeDir != line.dir) {
      auto relations = glm::greaterThan(nodes[i]->maxCode, line.point);
      if (relations[nodes[i]->planeDir]) {
        nodes[i] = nodes[i]->children[0];
      } else {
        nodes[i] = nodes[i]->children[1];
      }
      if (!nodes[i]) {
        return;
      }
    }
  }
  if (!checkMinialEdge(nodes, line.dir)) {
    return;
  }
  detectQuad(nodes, line, threshold);
  if (nodes[0]->isLeaf(threshold) && nodes[1]->isLeaf(threshold) && nodes[2]->isLeaf(threshold)
      && nodes[3]->isLeaf(threshold)) {
    generateQuad(nodes, mesh, t);
    return;
  }
  for (int i = 0; i < 4; ++i) {
    EdgeKd nextNodes = nodes;
    if (!nodes[i]->isLeaf() && nodes[i]->planeDir == line.dir) {
      setEdgeNode(nextNodes, i, nodes[i]->children[0]);
      contourEdge(nextNodes, line, mesh, t, threshold);
      nextNodes = nodes;
      setEdgeNode(nextNodes, i, nodes[i]->children[1]);
      contourEdge(nextNodes, line, mesh, t, threshold);
      return;
    }
  }
}

void Kdtree::generateQuad(EdgeKd &nodes, Mesh *mesh, Topology *t) {
//  for (auto n : nodes) {
//    // n->vertices[0].hermiteP = codeToPos(n->minCode + n->maxCode, Octree::cellSize) / 2.f;
//    mesh->drawAABBDebug(codeToPos(n->minCode, Octree::cellSize), codeToPos(n->maxCode, Octree::cellSize));
//  }

//  return;

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
