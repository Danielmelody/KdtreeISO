//
// Created by Danielhu on 2018/1/20.
//

#define GLM_ENABLE_EXPERIMENTAL
#define GLM_FORCE_CTOR_INIT

#include <unordered_map>
#include <unordered_set>
#include <numeric>
#include <set>
#include <glm/glm.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/fast_square_root.hpp>
#include "Octree.h"
#include "Mesh.h"
#include "Utils.h"
#include "Indicators.h"

Octree *Octree::buildWithScalarField(const PositionCode &minCode, int depth, ScalarField *scalarField, bool as_mipmap) {
  PositionCode sizeCode = PositionCode(1 << (depth - 1));
  auto root = new Octree(minCode, minCode + sizeCode, depth);
  assert(depth > 0);
  root->grid.assignSign(scalarField);
  bool noChildren = true;
  if (depth == 1) {
    if (!root->grid.isSigned) {
      delete root;
      return nullptr;
    }
    root->grid.sampleQef(scalarField, true);
    root->isLeaf = true;
  }
  else {
    PositionCode subSizeCode = PositionCode(static_cast<uint16_t>(1 << (depth - 2)));
    for (int i = 0; i < 8; ++i) {
      PositionCode subMinCode = minCode + subSizeCode * decodeCell(i);
      root->children[i] =
        buildWithScalarField(subMinCode, depth - 1, scalarField, as_mipmap);
      if (root->children[i]) {
        noChildren = false;
        root->children[i]->childIndex = static_cast<int8_t>(i);
        root->grid.allQef.combine(root->children[i]->grid.allQef);
      }
    }
    if (noChildren) {
      delete root;
      return nullptr;
    }
    calClusterbility(root, scalarField);
    if (root->clusterable && !as_mipmap) {
      root->combineComponents(scalarField);
    }
    root->isLeaf = false;
  }
  assert(root->grid.allQef.pointCount);
  assert(!isnan(root->grid.allQef.btb));
  if (!as_mipmap) {
    for (int i = 0; i < root->grid.components.size(); ++i) {
      root->grid.solveComponent(i);
    }
  }
  root->grid.solve(root->grid.allQef, root->grid.approximate);
  //  assert(root->grid.error >= -0.001f);
  return root;
}

void Octree::getSum(Octree *root, const PositionCode &minPos, const PositionCode &maxPos, QefSolver &out) {
  if (!root) {
    return;
  }
  if (glm::any(glm::greaterThanEqual(minPos, maxPos))) {
    return;
  }
  if (glm::any(glm::greaterThanEqual(minPos, root->grid.maxCode)) || glm::any(glm::lessThanEqual(maxPos, root->grid.minCode))) {
    return;
  }
  auto minPosBound = glm::max(root->grid.minCode, minPos);
  auto maxPosBound = glm::min(root->grid.maxCode, maxPos);
  if (minPosBound == root->grid.minCode && maxPosBound == root->grid.maxCode) {
    out.combine(root->grid.allQef);
    return;
  }
  for (int i = 0; i < 8; ++i) {
    getSum(root->children[i], minPosBound, maxPosBound, out);
  }
}

void Octree::simplify(Octree *root, float threshold) {
  if (!root) {
    return;
  }
  if (root->isLeaf) {
    return;
  }
  for (int i = 0; i < 8; ++i) {
    simplify(root->children[i], threshold);
  }
  if (root->clusterable && root->grid.approximate.error < threshold) {
    for (auto &child : root->children) {
      child = nullptr;
    }
    root->isLeaf = true;
  }
}

Mesh *Octree::extractMesh(Octree *root,
                          ScalarField *geometry,
                          int &intersectionPreservingVerticesCount,
                          bool intersectionFree) {
  assert(root);
  auto *mesh = new Mesh();
  std::unordered_set<Vertex *> indexed;
  EdgePolygonSet edgePolygonSet;
  generateVertexIndices(root, mesh, geometry, indexed);
  contourCell(root, mesh, geometry, intersectionPreservingVerticesCount, edgePolygonSet, intersectionFree, 0);
  return mesh;
}

void Octree::generateVertexIndices(Octree *node,
                                   Mesh *mesh,
                                   ScalarField *geometry,
                                   std::unordered_set<Vertex *> &indexed) {
  if (!node) {
    return;
  }
  for (int i = 0; i < 8; ++i) {
    generateVertexIndices(node->children[i], mesh, geometry, indexed);
  }
  if (node->isLeaf) {
    for (auto &v : node->grid.vertices) {
      mesh->addVertex(&v, geometry);
    }
  }
}

void Octree::contourCell(Octree *root,
                         Mesh *mesh,
                         ScalarField *geometry,
                         int &intersectionPreservingVerticesCount,
                         EdgePolygonSet &edgePolygonSet,
                         bool intersectionFree,
                         float threshold) {
  if (!root || root->isLeaf) {
    return;
  }
  for (int i = 0; i < 8; ++i) {
    contourCell(root->children[i],
                mesh,
                geometry,
                intersectionPreservingVerticesCount,
                edgePolygonSet,
                intersectionFree,
                threshold);
  }
  for (int i = 0; i < 12; ++i) {
    Octree *nodes[2] = {
      root->children[cellProcFaceMask[i][0]],
      root->children[cellProcFaceMask[i][1]],
    };
    contourFace(nodes,
                cellProcFaceMask[i][2],
                mesh,
                geometry,
                intersectionPreservingVerticesCount,
                edgePolygonSet,
                intersectionFree,
                threshold);
  }
  for (int i = 0; i < 6; ++i) {
    Octree *nodes[4];
    for (int j = 0; j < 4; ++j) {
      nodes[j] = root->children[cellProcEdgeMask[i][j]];
    }
    contourEdge(nodes,
                cellProcEdgeMask[i][4],
                (cellProcEdgeMask[i][4] + 2) % 3,
                geometry,
                intersectionPreservingVerticesCount,
                edgePolygonSet,
                intersectionFree,
                mesh,
                threshold);
  }
}

void Octree::contourFace(Octree *nodes[2],
                         int dir,
                         Mesh *mesh,
                         ScalarField *geometry,
                         int &intersectionPreservingVerticesCount,
                         EdgePolygonSet &edgePolygonSet,
                         bool intersectionFree,
                         float threshold) {
  if (!nodes[0] || !nodes[1]) {
    return;
  }
  if (nodes[0]->isLeaf && nodes[1]->isLeaf) {
    return;
  }
  // the subdivision of a face resulting 4 child faces;
  for (int i = 0; i < 4; ++i) {
    Octree *subdivision_face[2] = {nodes[0], nodes[1]};
    for (int j = 0; j < 2; ++j) {
      if (!subdivision_face[j]->isLeaf) {
        subdivision_face[j] = subdivision_face[j]->children[faceProcFaceMask[dir][i][j]];
      }
    }
    contourFace(subdivision_face,
                faceProcFaceMask[dir][i][2],
                mesh,
                geometry,
                intersectionPreservingVerticesCount,
                edgePolygonSet,
                intersectionFree,
                threshold);
  }
  for (int i = 0; i < 4; ++i) {
    Octree *edge_nodes[4];
    const int c[4] =
      {
        faceProcEdgeMask[dir][i][1],
        faceProcEdgeMask[dir][i][2],
        faceProcEdgeMask[dir][i][3],
        faceProcEdgeMask[dir][i][4],
      };
    for (int j = 0; j < 4; ++j) {
      const int order = faceNodeOrder[j];
      if (nodes[order]->isLeaf) {
        edge_nodes[j] = nodes[order];
      }
      else {
        edge_nodes[j] = nodes[order]->children[c[j]];
      }
    }
    if (dir == 0 && faceProcEdgeMask[dir][i][5] == 2) {
      ;
    }
    contourEdge(edge_nodes,
                faceProcEdgeMask[dir][i][5],
                dir,
                geometry,
                intersectionPreservingVerticesCount,
                edgePolygonSet,
                intersectionFree,
                mesh,
                threshold);
  }
}

void Octree::contourEdge(Octree **nodes,
                         int dir,
                         int quadDir2,
                         ScalarField *geometry,
                         int &intersectionPreservingVerticesCount,
                         EdgePolygonSet &edgePolygonSet,
                         bool intersectionFree,
                         Mesh *mesh,
                         float threshold) {
  if (!nodes[0] || !nodes[1] || !nodes[2] || !nodes[3]) {
    return;
  }

  if (nodes[0]->isLeaf && nodes[1]->isLeaf && nodes[2]->isLeaf && nodes[3]->isLeaf) {
    generateQuad(nodes,
                 dir,
                 quadDir2,
                 geometry,
                 intersectionPreservingVerticesCount,
                 edgePolygonSet,
                 intersectionFree,
                 mesh,
                 threshold);
    return;
  }
  int quadDir1 = 3 - dir - quadDir2;
  // the subdivision of a edge resulting 2 child edges;
  for (int i = 0; i < 2; ++i) {
    Octree *subdivision_edge[4];
    for (int j = 0; j < 4; ++j) {
      if (!nodes[j]->isLeaf) {

        PositionCode code;
        code[dir] = i;
        code[quadDir1] = (3 - j) % 2;
        code[quadDir2] = (3 - j) / 2;

        subdivision_edge[j] = nodes[j]->children[encodeCell(code)];
      }
      else {
        subdivision_edge[j] = nodes[j];
      }
    }
    contourEdge(subdivision_edge,
                dir,
                quadDir2,
                geometry,
                intersectionPreservingVerticesCount,
                edgePolygonSet,
                intersectionFree,
                mesh,
                threshold);
  }
}

void Octree::generateQuad(Octree **nodes,
                          int dir,
                          int quadDir2,
                          ScalarField *t,
                          int &intersectionPreservingVerticesCount,
                          EdgePolygonSet &edgePolygonSet,
                          bool intersectionFree,
                          Mesh *mesh,
                          float threshold) {
  std::array<Octree *, 4> array = {{nodes[0], nodes[1], nodes[2], nodes[3]}};
  RectilinearGrid::generateQuad(array, 3 - quadDir2 - dir, quadDir2, mesh, t, threshold);
}

void Octree::drawOctrees(Octree *root, Mesh *mesh) {
  if (!root) {
    return;
  }
  if (root->isLeaf) {
    root->grid.draw(mesh);
    return;
  }
  for (int i = 0; i < 8; ++i) {
    drawOctrees(root->children[i], mesh);
  }
}

void Octree::calClusterbility(Octree *root, ScalarField *s) {
  if (!root || root->isLeaf) {
    return;
  }
  for (int i = 0; i < 8; ++i) {
    if (root->children[i] && !root->children[i]->clusterable) {
      root->clusterable = false;
      return;
    }
  }
  for (int i = 0; i < 12; ++i) {
    int leftIndex = cellProcFaceMask[i][0];
    int rightIndex = cellProcFaceMask[i][1];
    auto left = root->children[leftIndex] ? &root->children[leftIndex]->grid : nullptr;
    auto right = root->children[rightIndex] ? &root->children[rightIndex]->grid : nullptr;
    ;
    auto dir = cellProcFaceMask[i][2];
    auto halfSize = (root->grid.maxCode - root->grid.minCode) / 2;
    auto minCode = root->grid.minCode + decodeCell(leftIndex) * halfSize;
    auto maxCode = root->grid.minCode + halfSize + decodeCell(rightIndex) * halfSize;
    bool clusterable = RectilinearGrid::calClusterability(left, right, dir, minCode, maxCode, s);
    if (!clusterable) {
      root->clusterable = false;
      return;
    }
  }
  root->clusterable = true;
}
void Octree::combineComponents(ScalarField *s) {
  assert(grid.components.empty());
  auto halfSize = (grid.maxCode - grid.minCode) / 2;
  RectilinearGrid xgridPool[2], ygridPool[4];
  RectilinearGrid *xgrids[2];
  for (int x = 0; x < 2; ++x) {
    auto yMinCode = PositionCode(x, 0, 0) * halfSize + grid.minCode;
    auto yMaxCode = PositionCode(x, 1, 1) * halfSize + halfSize + grid.minCode;
    RectilinearGrid *ygrids[2];
    for (int y = 0; y < 2; ++y) {
      auto zMinCode = PositionCode(x, y, 0) * halfSize + grid.minCode;
      auto zMaxCode = PositionCode(x, y, 1) * halfSize + halfSize + grid.minCode;
      auto l =
        children[encodeCell(PositionCode(x, y, 0))] ? &children[encodeCell(PositionCode(x, y, 0))]->grid : nullptr;
      auto r =
        children[encodeCell(PositionCode(x, y, 1))] ? &children[encodeCell(PositionCode(x, y, 1))]->grid : nullptr;
      if (!l && !r) {
        ygrids[y] = nullptr;
      }
      else {
        ygrids[y] = &ygridPool[x * 2 + y];
        ygrids[y]->minCode = zMinCode;
        ygrids[y]->maxCode = zMaxCode;
        ygrids[y]->assignSign(s);
        if (l)
          ygrids[y]->allQef.combine(l->allQef);
        if (r)
          ygrids[y]->allQef.combine(r->allQef);
        RectilinearGrid::combineAAGrid(l, r, 2, ygrids[y]);
      }
    }
    if (!ygrids[0] && !ygrids[1]) {
      xgrids[x] = nullptr;
    }
    else {
      xgrids[x] = &xgridPool[x];
      xgrids[x]->minCode = yMinCode;
      xgrids[x]->maxCode = yMaxCode;
      xgrids[x]->assignSign(s);
      if (ygrids[0])
        xgrids[x]->allQef.combine(ygrids[0]->allQef);
      if (ygrids[1])
        xgrids[x]->allQef.combine(ygrids[1]->allQef);
      RectilinearGrid::combineAAGrid(ygrids[0], ygrids[1], 1, xgrids[x]);
    }
  }
  RectilinearGrid::combineAAGrid(xgrids[0], xgrids[1], 0, &grid);
  std::set<Vertex *> coarserVertices;
  for (auto &v : grid.vertices) {
    coarserVertices.insert(&v);
  }

  for (auto child : children) {
    if (child) {
      for (Vertex &v : child->grid.vertices) {
        if (v.parent) {
          while (v.parent->parent) {
            v.parent = v.parent->parent;
          }
        }
        if (coarserVertices.find(v.parent) == coarserVertices.end()) {
          v.parent = nullptr;
        }
      }
    }
  }

  // to avoid a specific MC case
  int count = 0;
  for (auto c : grid.components) {
    count += c.pointCount;
  }
  if (count != grid.allQef.pointCount) {
    clusterable = false;
    for (auto child : children) {
      if (child) {
        for (Vertex &v : child->grid.vertices) {
          v.parent = nullptr;
        }
      }
    }
  }
}
