//
// Created by Danielhu on 2018/1/20.
//
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

using namespace glm;

float Octree::cellSize = 0.2f;

void Octree::setCellSize(float size) {
  cellSize = size;
}

bool Octree::getSelfQef(Octree *node, Topology *g, QefSolver &qef) {
  vec3 corners[8];
  int8_t mtlID = g->getMaterialID();
  for (int i = 0; i < 8; ++i) {
    corners[i] = min_offset_subdivision(i) * node->size + node->min;
    node->cornerSigns[i] = (uint8_t) (g->value(corners[i]) > 0. ? mtlID : 0);
  }
  bool intersect = false;
  for (int i = 0; i < 7; ++i) {
    if (node->cornerSigns[i] != node->cornerSigns[i + 1]) {
      intersect = true;
    }
  }
  if (!intersect) {
    return false;
  }
  for (int i = 0; i < 12; ++i) {
    vec3 p1 = corners[edge_map[i][0]];
    vec3 p2 = corners[edge_map[i][1]];
    if (node->cornerSigns[edge_map[i][0]] != node->cornerSigns[edge_map[i][1]]) {
      vec3 p, n;
      if (g->solve(p1, p2, p)) {
        g->normal(p, n);
        qef.add(p, n);
      }
    }
  }
  return true;
}

Octree *Octree::buildWithTopology(PositionCode minCode, int depth, Topology *topology, int &losslessCut) {
  auto root = samplerBuild(minCode, depth, topology);
  // root = losslessCompress(root, -1, topology, losslessCut);
  return root;
}

Octree *Octree::samplerBuild(PositionCode minCode, int depth, Topology *topology) {
  vec3 size = vec3(Octree::cellSize * (1 << (depth - 1)));
  vec3 min = codeToPos(minCode, Octree::cellSize);
  auto root = new Octree(min, size, depth);
  assert(depth > 0);
  root->minCode = minCode;
  PositionCode sizeCode = PositionCode(static_cast<uint16_t>(1 << (depth - 1)));
  root->maxCode = minCode + sizeCode;
  bool homogeneous = true;
  if (depth == 1) {
    if (!getSelfQef(root, topology, root->qef)) {
      delete root;
      return nullptr;
    }
    root->isLeaf = true;
    homogeneous = false;
  } else {
    PositionCode subSizeCode = PositionCode(static_cast<uint16_t >(1 << (depth - 2)));
    for (int i = 0; i < 8; ++i) {
      PositionCode subMinCode = minCode + subSizeCode * min_offset_subdivision_code(i);
      root->children[i] =
          samplerBuild(subMinCode, depth - 1, topology);
      homogeneous = homogeneous && !root->children[i];
      if (root->children[i]) {
        root->cornerSigns[i] = root->children[i]->cornerSigns[i];
        root->children[i]->childIndex = static_cast<int8_t>(i);
        root->qef.combine(root->children[i]->qef);
      }
    }
    root->isLeaf = false;
  }
  if (homogeneous) {
    delete root;
    return nullptr;
  }
  calHermite(root, &root->qef, topology, &root->vertex);
  root->clusterQef->set(root->qef);
  return root;
}

void Octree::getSum(Octree *root, PositionCode minPos, PositionCode maxPos, QefSolver &out) {
  if (!root) {
    return;
  }
  if (glm::any(glm::greaterThanEqual(minPos, maxPos))) {
    return;
  }
  if (glm::any(glm::greaterThanEqual(minPos, root->maxCode)) || glm::any(glm::lessThanEqual(maxPos, root->minCode))) {
    return;
  }
  minPos = glm::max(root->minCode, minPos);
  maxPos = glm::min(root->maxCode, maxPos);
  if (minPos == root->minCode && maxPos == root->maxCode) {
    out.combine(root->qef);
    return;
  }
  for (int i = 0; i < 8; ++i) {
    getSum(root->children[i], minPos, maxPos, out);
  }
}

Kdtree *Octree::generateKdtree(Octree *root, PositionCode minCode, PositionCode maxCode, Topology *t, int depth) {
  if (glm::any(glm::greaterThanEqual(minCode, maxCode))) {
    return nullptr;
  }
  QefSolver sum;
  getSum(root, minCode, maxCode, sum);
  if (sum.pointCount == 0) {
    return nullptr;
  }
  PositionCode bestRightMinCode = maxCode, bestLeftMaxCode = minCode;
  float minErrorDiff = 1e20;
  QefSolver leftSum, rightSum;

  auto size = maxCode - minCode;
  int dir = 0;
  if (size[1] > size[0]) {
    dir = 1;
  }
  if (size[2] > size[dir]) {
    dir = 2;
  }
  for (int axis = minCode[dir] + 1; axis < maxCode[dir]; ++axis) {
    PositionCode rightMinCode = minCode;
    rightMinCode[dir] = axis;
    PositionCode leftMaxCode = maxCode;
    leftMaxCode[dir] = axis;
    glm::vec3 leftApproximate, rightApproximate;
    leftSum.reset();
    rightSum.reset();
    float leftError = 0.f;
    float rightError = 0.f;
    getSum(root, minCode, leftMaxCode, leftSum);
    getSum(root, rightMinCode, maxCode, rightSum);
    if (leftSum.pointCount > 0) {
      leftSum.solve(leftApproximate, leftError);
    }
    if (rightSum.pointCount > 0) {
      rightSum.solve(rightApproximate, rightError);
    }
    if (abs(rightError - leftError) < minErrorDiff) {
      minErrorDiff = abs(rightError - leftError);
      bestLeftMaxCode = leftMaxCode;
      bestRightMinCode = rightMinCode;
    }
  }
  auto kd = new Kdtree(sum, minCode, maxCode, dir, depth);
  kd->children[0] = generateKdtree(root, minCode, bestLeftMaxCode, t, depth + 1);
  kd->children[1] = generateKdtree(root, bestRightMinCode, maxCode, t, depth + 1);
  kd->assignSign(t);
  kd->calClusterability();
  return kd;
}

int Octree::simplify(Octree *root, float threshold, Topology *geometry) {
  if (!root) {
    return 0;
  }
  if (root->isLeaf) {
    return 0;
  }
  int reduction = 0;
  int childCount = 0;
  for (auto &child : root->children) {
    reduction += simplify(child, threshold, geometry);
    if (child) {
      childCount++;
    }
  }
  if (root->error < threshold) {
    reduction += childCount - 1;
    for (auto &child: root->children) {
      child = nullptr;
    }
    root->isLeaf = true;
    root->clusterQef->set(root->qef);
  }
  return reduction;
}

void Octree::reverseExtendedSimplify(Octree *root, Topology *g) {
  if (!root) {
    return;
  }
  if (root->isLeaf) {
    if (root->cluster->size() > 1) {
      root->cluster = new std::unordered_set<Octree *>({root});
      root->clusterVertex = &root->vertex;
      calHermite(root, &root->qef, g, &root->vertex);
      root->clusterQef = new QefSolver();
      root->clusterQef->set(root->qef);
      root->clusterMin = new glm::vec3(root->min);
      root->clusterSize = new glm::vec3(root->size);
    }
    return;
  }
  for (int i = 0; i < 8; ++i) {
    reverseExtendedSimplify(root->children[i], g);
  }
}

Octree *Octree::extendedSimplify(Octree *root,
                                 float threshold,
                                 Topology *geometry,
                                 int &count) {
  edgeClassifier(root, geometry, threshold);
  std::unordered_set<std::unordered_set<Octree *> *> clusters;
  OptionalHierarchyClustering(root, threshold, geometry, count);
  edgeCluster(root, geometry, count, clusters);
  return root;
}

Octree *Octree::edgeClassifier(Octree *root, Topology *geometry, float threshold) {
  if (!root || root->isLeaf) {
    return root;
  }
  for (int i = 0; i < 8; ++i) {
    root->children[i] = edgeClassifier(root->children[i], geometry, threshold);
  }
  for (int i = 0; i < 12; ++i) {
    auto &a = root->children[cellProcFaceMask[i][0]];
    auto &b = root->children[cellProcFaceMask[i][1]];
    auto faceMin = root->min
        + directionMap(cellProcFaceMask[i][2]) * root->size / 2.f
        + faceSubDivision(cellProcFaceMask[i][2], i % 4) * root->size / 2.f;
    const int dir = cellProcFaceMask[i][2];
    edgeCollapse(a,
                 b,
                 dir,
                 threshold,
                 geometry,
                 faceMin,
                 root->size.x / 2.f);
  }
  int shrinkCount = 0;
  int childCount = 0;
  std::unordered_set<std::unordered_set<Octree *> *> clusters;
  float minError = 1000000.f;
  for (int i = 0; i < 8; ++i) {
    if (root->children[i] && clusters.find(root->cluster) == clusters.end()) {
      clusters.insert(root->cluster);
      shrinkCount += root->cluster->size() - 1;
      childCount++;
      minError = std::min(minError, root->clusterQef->getError(root->clusterVertex->hermiteP));
    }
  }
  return root;
}

void Octree::edgeCluster(Octree *root,
                         Topology *geometry,
                         int &count,
                         std::unordered_set<std::unordered_set<Octree *> *> &clusters) {
  if (!root) {
    return;
  }
  if (root->isLeaf) {
    if (clusters.find(root->cluster) != clusters.end()) {
      return;
    }
    clusters.insert(root->cluster);
    count += std::max(0, (int) root->cluster->size() - 1);
    if (root->cluster->size() > 1) {
      calHermite(root, root->clusterQef, geometry, root->clusterVertex);
      assert(root->clusterQef->pointCount != 0);
    }
  }
  for (int i = 0; i < 8; ++i) {
    edgeCluster(root->children[i], geometry, count, clusters);
  }
}

Octree *Octree::OptionalHierarchyClustering(Octree *root, float threshold, Topology *geometry, int &count) {
//  for(int i = 0; i  < 8; ++i) {
//
//  }
  if (!root || root->isLeaf) {
    return nullptr;
  }
  for (int i = 0; i < 8; ++i) {
    OptionalHierarchyClustering(root->children[i], threshold, geometry, count);
  }
  bool clusterable = true;
  std::unordered_set<std::unordered_set<Octree *> *> internalClusters;
  float maxChildClusterError = 0.f;
  if (root->error < threshold) {
    // glm
    for (int i = 0; i < 8; ++i) {
      if (root->children[i] && root->cluster->size() > 1) {
        vec3 clusterMin = *root->children[i]->clusterMin;
        vec3 clusterMax = *root->children[i]->clusterMin + *root->children[i]->clusterSize;
        vec3 cellMin = root->children[i]->min;
        vec3 cellMax = root->children[i]->min + root->children[i]->size;
        int validAxisNum = 0;
        bool intermediately = false;
        for (int i = 0; i < 3; ++i) {
          if (cellMin[i] <= clusterMin[i] && cellMax[i] >= clusterMax[i]) {
            validAxisNum++;
          } else if (cellMin[i] > clusterMin[i] && cellMax[i] < clusterMax[i]) {
            intermediately = true;
          }
        }
        if (!(validAxisNum == 2 && !intermediately)) {
          clusterable = false;
        }
        maxChildClusterError = std::max(root->error, maxChildClusterError);
        if (validAxisNum == 3) {
          internalClusters.insert(root->children[i]->cluster);
        }
      }
    }
    if (clusterable && !internalClusters.empty()) {
      for (int i = 0; i < 8; ++i) {
        auto &child = root->children[i];
        if (child && child->cluster->size() > 1) {
          child->cluster->erase(child);
          calClusterBounds(child->cluster);
          child->clusterQef->separate(child->qef);
          child = nullptr;
          count++;
        }
      }
      count--;
      root->isLeaf = true;
    }
    return root;
  }
  return root;
}

void Octree::calClusterBounds(std::unordered_set<Octree *> *cluster) {
  glm::vec3 min = std::accumulate(std::next(cluster->begin()),
                                  cluster->end(),
                                  (*cluster->begin())->min,
                                  [](glm::vec3 min, Octree *b) -> glm::vec3 { return glm::min(min, b->min); });
  glm::vec3 max = std::accumulate(std::next(cluster->begin()),
                                  cluster->end(),
                                  (*cluster->begin())->min + (*cluster->begin())->size,
                                  [](glm::vec3 max, Octree *b) { return glm::max(max, b->min + b->size); });
  *(*cluster->begin())->clusterMin = min;
  *(*cluster->begin())->clusterSize = max - min;
}

void Octree::cubeExtensionTest(Octree *a, Octree *b, int dir, float minSize) {
//  vec3 combineMin = glm::min(*a->clusterMin, *b->clusterMin);
//  vec3 combineSize = glm::max(*a->clusterMin + *a->clusterSize, *b->clusterMin + *b->clusterSize) - combineMin;
//  for (int i = 0; i < 8; ++i) {
//    vec3 corner = min_offset_subdivision(i) * combineSize + combineMin;
//
//  }

}

void Octree::edgeCollapse(Octree *&a,
                          Octree *&b,
                          int dir,
                          float threshold,
                          Topology *geometry,
                          vec3 faceMin,
                          float faceSize) {
  if (!a || !b) {
    return;
  }
  if (!a->isLeaf || !b->isLeaf) {
    for (int i = 0; i < 4; ++i) {
      vec3 subMin = faceMin + 0.5f * faceSize * faceSubDivision(dir, i);
      float subSize = faceSize / 2.f;

      if (!a->isLeaf && !b->isLeaf) {
        edgeCollapse(a->children[faceProcFaceMask[dir][i][0]],
                     b->children[faceProcFaceMask[dir][i][1]],
                     dir,
                     threshold,
                     geometry,
                     subMin,
                     subSize
        );
      } else if (a->isLeaf) {
        edgeCollapse(a, b->children[faceProcFaceMask[dir][i][1]], dir, threshold, geometry, subMin, subSize);
      } else {
        edgeCollapse(a->children[faceProcFaceMask[dir][i][0]], b, dir, threshold, geometry, subMin, subSize);
      }
    }
  }

  auto lastfaceNodeEndSign = -1;
  bool connectivity = false;

  for (int i = 0; i < 4; ++i) {
    auto faceNodeEndSign = 0;
    vec3 samplePoint = faceSubDivision(dir, i) * faceSize + faceMin;
    float density = geometry->value(samplePoint);
    if (density > 0) {
      faceNodeEndSign = geometry->getMaterialID();
    } else {
      faceNodeEndSign = 0;
    }
    if (faceNodeEndSign != lastfaceNodeEndSign && lastfaceNodeEndSign != -1) {
      connectivity = true;
    }
    lastfaceNodeEndSign = faceNodeEndSign;
  }

  if (!connectivity) {
    return;
  }

  if (a->cluster == b->cluster) {
    return;
  }

  vec3 minOffset = *b->clusterMin - *a->clusterMin;
  vec3 maxOffset = *b->clusterMin + *b->clusterSize - (*a->clusterMin + *a->clusterSize);

  for (int i = 0; i < 3; ++i) {
    if (i != dir) {
      if (minOffset[i] != 0 || maxOffset[i] != 0) {
        return;
      }
    }
  }

  QefSolver edgeQEF;
  edgeQEF.combine(*a->clusterQef);
  edgeQEF.combine(*b->clusterQef);
  vec3 combineP;
  float combineError;
  edgeQEF.solve(combineP, combineError);
  if (combineError < threshold) {
    combine(a, b, geometry);
  }
}

Mesh *Octree::extractMesh(Octree *root,
                          Topology *geometry,
                          int &intersectionPreservingVerticesCount,
                          bool intersectionFree) {
  assert(root);
  auto *mesh = new Mesh();
  std::unordered_set<Vertex *> indexed;
  EdgePolygonSet edgePolygonSet;
  generateVertexIndices(root, mesh, geometry, indexed);
  contourCell(root, mesh, geometry, intersectionPreservingVerticesCount, edgePolygonSet, intersectionFree);
  return mesh;
}

void Octree::generateVertexIndices(Octree *node,
                                   Mesh *mesh,
                                   Topology *geometry,
                                   std::unordered_set<Vertex *> &indexed) {
  if (!node) {
    return;
  }
  for (int i = 0; i < 8; ++i) {
    generateVertexIndices(node->children[i], mesh, geometry, indexed);
  }
  if (node->isLeaf) {
    if (indexed.find(node->clusterVertex) != indexed.end()) {
      return;
    }
    indexed.insert(node->clusterVertex);
    node->clusterVertex->vertexIndex = static_cast<unsigned int>(mesh->positions.size());
    mesh->positions.push_back(node->clusterVertex->hermiteP);
    mesh->normals.push_back(node->clusterVertex->hermiteN);
  }
}

void Octree::contourCell(Octree *root,
                         Mesh *mesh,
                         Topology *geometry,
                         int &intersectionPreservingVerticesCount,
                         EdgePolygonSet &edgePolygonSet,
                         bool intersectionFree) {
  if (!root || root->isLeaf) {
    return;
  }
  for (int i = 0; i < 8; ++i) {
    contourCell(root->children[i],
                mesh,
                geometry,
                intersectionPreservingVerticesCount,
                edgePolygonSet,
                intersectionFree);
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
                intersectionFree);
  }
  for (int i = 0; i < 6; ++i) {
    Octree *nodes[4];
    for (int j = 0; j < 4; ++j) {
      nodes[j] = root->children[cellProcEdgeMask[i][j]];
    }
    contourEdge(nodes,
                cellProcEdgeMask[i][4],
                mesh,
                geometry,
                intersectionPreservingVerticesCount,
                edgePolygonSet,
                intersectionFree);
  }
}

void Octree::contourFace(Octree *nodes[2],
                         int dir,
                         Mesh *mesh,
                         Topology *geometry,
                         int &intersectionPreservingVerticesCount,
                         EdgePolygonSet &edgePolygonSet,
                         bool intersectionFree) {
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
                intersectionFree);
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
      const int order = faceNodeOrder[dir][i][j];
      if (nodes[order]->isLeaf) {
        edge_nodes[j] = nodes[order];
      } else {
        edge_nodes[j] = nodes[order]->children[c[j]];
      }
    }
    contourEdge(edge_nodes,
                faceProcEdgeMask[dir][i][5],
                mesh,
                geometry,
                intersectionPreservingVerticesCount,
                edgePolygonSet,
                intersectionFree);
  }
}

void Octree::contourEdge(Octree *nodes[4],
                         int dir,
                         Mesh *mesh,
                         Topology *geometry,
                         int &intersectionPreservingVerticesCount,
                         EdgePolygonSet &edgePolygonSet,
                         bool intersectionFree) {
  if (!nodes[0] || !nodes[1] || !nodes[2] || !nodes[3]) {
    return;
  }

  if (nodes[0]->isLeaf && nodes[1]->isLeaf && nodes[2]->isLeaf && nodes[3]->isLeaf) {
    generateQuad(nodes, dir, mesh, geometry, intersectionPreservingVerticesCount, edgePolygonSet, intersectionFree);
    return;
  }

  // the subdivision of a edge resulting 2 child edges;
  for (int i = 0; i < 2; ++i) {
    Octree *subdivision_edge[4];
    for (int j = 0; j < 4; ++j) {
      if (!nodes[j]->isLeaf) {
        subdivision_edge[j] = nodes[j]->children[edgeProcEdgeMask[dir][i][j]];
      } else {
        subdivision_edge[j] = nodes[j];
      }
    }
    contourEdge(subdivision_edge,
                dir,
                mesh,
                geometry,
                intersectionPreservingVerticesCount,
                edgePolygonSet,
                intersectionFree);
  }
}

void Octree::combine(Octree *a, Octree *b, Topology *g) {

  auto bigger = a;
  auto smaller = b;

  if (b->cluster->size() > a->cluster->size()) {
    bigger = b;
    smaller = a;
  }
  for (auto s : *smaller->cluster) {
    bigger->cluster->insert(s);
  }
  bigger->clusterQef->combine(*smaller->clusterQef);

  vec3 biggerMax = *bigger->clusterSize + *bigger->clusterMin;
  vec3 smallerMax = *smaller->clusterSize + *smaller->clusterMin;

  vec3 combineMax = glm::max(biggerMax, smallerMax);
  vec3 combineMin = glm::min(*bigger->clusterMin, *smaller->clusterMin);
  *bigger->clusterMin = combineMin;
  *bigger->clusterSize = combineMax - combineMin;

//  auto smCluster = smaller->cluster;
//  auto smClusterQef = smaller->clusterQef;
//  auto smClusterMin = smaller->clusterMin;
//  auto smClusterSize = smaller->clusterSize;

  int count = 0;

  assert(bigger->clusterQef->pointCount > 0);

  for (auto oct : *(smaller->cluster)) {
    oct->cluster = bigger->cluster;
    oct->clusterQef = bigger->clusterQef;
    oct->clusterMin = bigger->clusterMin;
    oct->clusterSize = bigger->clusterSize;
    oct->clusterVertex = bigger->clusterVertex;
    count++;
  }
//  delete smCluster;
//  delete smClusterQef;
//  delete smClusterMin;
//  delete smClusterSize;
}

bool Octree::intersectWithBrothers(int cornerDir, Octree *node) {
  for (int i = 0; i < 3; ++i) {
    int edgeIndex = dirRelatedEdge[cornerDir][node->childIndex][i];
    if (edgeIndex >= 0) {
      if (node->cornerSigns[edge_map[edgeIndex][0]] != node->cornerSigns[edge_map[edgeIndex][1]]) {
        return true;
      }
    }
  }
  return true;
}

bool Octree::findFeatureNodes(Octree *node,
                              std::vector<Octree *> &results,
                              const int cornerDir,
                              bool subdivision,
                              const glm::vec3 &edgeP,
                              const glm::vec3 &normal) {
  if (!node) {
    return false;
  }

  if (!intersectWithBrothers(cornerDir, node)) {
    return false;
  }

  //float parallelrity = glm::dot(normalize(edgeP - node->vertex.hermiteP), normal);
  // if (node->isLeaf && node->qef.roughness > 1e-10) {
  results.push_back(node);
  return true;
  // }
  /*
  bool no_features = true;
  if (!node->isLeaf && subdivision) {
    for (int i = 0; i < 8; ++i) {
      auto spreadNodeIndex = planeSpreadingMask[cornerDir][i];
      auto spreadNode = node->children[spreadNodeIndex];
      if (findFeatureNodes(spreadNode, results, cornerDir, true, edgeP, normal)) {
        break;
      }
    }
  }

  int startIndex = 1;
  if (adjacentNodes[cornerDir][node->childIndex] == 1) {
    startIndex = 4;
  } else if (adjacentNodes[7 - cornerDir][node->childIndex] == 1) {
    startIndex = 7;
  } else if (cornerDir + node->childIndex == 7) {
    startIndex = 8;
  }
  for (int i = startIndex; i < 8; ++i) {
    auto spreadNodeIndex = planeSpreadingMask[cornerDir][i];
    auto spreadNode = node->parent->children[spreadNodeIndex];
    assert(spreadNode != node);
    if (!spreadNode) {
      break;
    }
    if (findFeatureNodes(spreadNode, results, cornerDir, true, edgeP, normal)) {
      no_features = false;
      break;
    }
  }
  if (no_features && !subdivision) {
    return findFeatureNodes(node->parent, results, cornerDir, false, edgeP, normal);
  }
  return true;
   */
}

bool Octree::isInterFreeCondition2Faild(const std::vector<Vertex *> &polygons, const vec3 &p1, const vec3 &p2) {
  int anotherV = 3;
  bool interSupportingEdge = false;

  for (int i = 2; i < polygons.size(); ++i) {
    vec3 baryPos;
    bool isInter = glm::intersectRayTriangle(p1,
                                             p2 - p1,
                                             polygons[0]->hermiteP,
                                             polygons[i - 1]->hermiteP,
                                             polygons[i]->hermiteP,
                                             baryPos);
    isInter = isInter && (baryPos.z > 0.f && baryPos.z < 1.f);
    if (isInter) {
      interSupportingEdge = true;
      anotherV = i % 3 + 1;
    }
  }
  if (polygons.size() == 3) {
    return !interSupportingEdge;
  } else {
    vec3 baryPos;
    bool interTetrahedron = glm::intersectRayTriangle(polygons[0]->hermiteP,
                                                      polygons[2]->hermiteP - polygons[0]->hermiteP,
                                                      p1,
                                                      p2,
                                                      polygons[anotherV]->hermiteP,
                                                      baryPos);
    interTetrahedron = interTetrahedron && (baryPos.z > 0.f && baryPos.z < 1.f);
    return !(interTetrahedron && interSupportingEdge);
  }
}

void Octree::generateQuad(Octree **nodes,
                          int dir,
                          Mesh *mesh,
                          Topology *g,
                          int &intersectionPreservingVerticesCount,
                          EdgePolygonSet &edgePolygonSet,
                          bool intersectionFree) {
  std::unordered_set<Vertex *> identifier;
  std::vector<Vertex *> polygon;
  bool condition1Failed = false;
  for (int i = 0; i < 4; ++i) {
    if (identifier.find(nodes[i]->clusterVertex) == identifier.end()) {
      polygon.push_back(nodes[i]->clusterVertex);
    }
    identifier.insert(nodes[i]->clusterVertex);
  }

  if (polygon.size() < 3) {
    return;
  }
  int sameCellIndex[2] = {2, 3};
  int firstConcaveFaceVertex = 0;
  for (int i = 0; i < 4; ++i) {
    int testDir = (dir + i / 2 + 1) % 3;
    int edgeAdjacentCellIndexA = edgeTestNodeOrder[i][0];
    int edgeAdjacentCellIndexB = edgeTestNodeOrder[i][1];
    Octree *a = nodes[edgeAdjacentCellIndexA];
    Octree *b = nodes[edgeAdjacentCellIndexB];
    if (a->cluster != b->cluster) {
      if (a->faceVertices.find(b) != a->faceVertices.end()) {
        firstConcaveFaceVertex = i;
        condition1Failed = true;
        continue;
      }
      vec3 faceMinA = vec3(std::numeric_limits<float>::max());
      vec3 faceMinB = faceMinA;
      vec3 faceMaxA = -vec3(std::numeric_limits<float>::max());
      vec3 faceMaxB = faceMaxA;
      for (int j = 0; j < 4; ++j) {
        int subIndexA = faceProcFaceMask[testDir][j][0];
        int subIndexB = faceProcFaceMask[testDir][j][1];
        vec3 cornerA = min_offset_subdivision(subIndexA) * (*a->clusterSize) + *a->clusterMin;
        vec3 cornerB = min_offset_subdivision(subIndexB) * (*b->clusterSize) + *b->clusterMin;
        faceMinA = glm::min(cornerA, faceMinA);
        faceMinB = glm::min(cornerB, faceMinB);
        faceMaxA = glm::max(cornerA, faceMaxA);
        faceMaxB = glm::max(cornerB, faceMaxB);
      }
      vec3 faceMin = glm::max(faceMinA, faceMinB);
      vec3 faceMax = glm::min(faceMaxA, faceMaxB);
      if (!segmentFaceIntersection(a->clusterVertex->hermiteP, b->clusterVertex->hermiteP, faceMin, faceMax, testDir)) {
        vec3 minEnd = faceMin + directionMap(dir) * (faceMax - faceMin);
        vec3 maxEnd = faceMax - directionMap(dir) * (faceMax - faceMin);
        glm::vec3 points[4] = {faceMin, minEnd, faceMax, maxEnd};
        vec3 massPointSum(0.f);
        int pointCount = 0;
        for (int k = 0; k < 4; ++k) {
          float v1 = g->value(points[k]);
          float v2 = g->value(points[(k + 1) % 4]);
          if ((v1 >= 0 && v2 < 0) || (v1 < 0 && v2 >= 0)) {
            vec3 x;
            g->solve(points[k], points[(k + 1) % 4], x);
            massPointSum += x;
            pointCount++;
          }
        }
        if (pointCount > 0) {
          firstConcaveFaceVertex = i;
          auto faceV = new Vertex(massPointSum / (float) pointCount);
          mesh->addVertex(faceV, g);
          a->faceVertices[b] = faceV;
          b->faceVertices[a] = faceV;
          condition1Failed = true;
        }
      }
    } else {
      sameCellIndex[0] = edgeAdjacentCellIndexA;
      sameCellIndex[1] = edgeAdjacentCellIndexB;
    }
  }

  int minCellIndex = 0;
  for (int i = 0; i < 4; ++i) {
    int edgeAdjacentCellIndexA = edgeTestNodeOrder[i][0];
    int edgeAdjacentCellIndexB = edgeTestNodeOrder[i][1];
    if (edgeAdjacentCellIndexA != sameCellIndex[0] && edgeAdjacentCellIndexA != sameCellIndex[1]
        && edgeAdjacentCellIndexB != sameCellIndex[0] && edgeAdjacentCellIndexB != sameCellIndex[1]) {
      minCellIndex = edgeAdjacentCellIndexA;
    }
  }

  Vertex edgeVertex;
  vec3 p1 = *nodes[minCellIndex]->clusterSize * min_offset_subdivision(edgeProcEdgeMask[dir][0][minCellIndex])
      + *nodes[minCellIndex]->clusterMin;
  vec3 p2 = *nodes[minCellIndex]->clusterSize * min_offset_subdivision(edgeProcEdgeMask[dir][1][minCellIndex])
      + *nodes[minCellIndex]->clusterMin;

  for (int i = 0; i < 4; ++i) {
    p1[dir] = std::max((*nodes[i]->clusterMin)[dir], p1[dir]);
    p2[dir] = std::min((*nodes[i]->clusterMin)[dir] + (*nodes[i]->clusterSize)[dir], p2[dir]);
  }

  float v1 = g->value(p1);
  float v2 = g->value(p2);
  if ((v1 > 0 && v2 > 0) || (v1 < 0 && v2 < 0)) {
    return;
  }

  std::set<Vertex *> polygonSet;

  for (auto v : polygon) {
    polygonSet.insert(v);
  }

  if (edgePolygonSet.find(polygonSet) != edgePolygonSet.end()) {
    return;
  }

  edgePolygonSet.insert(polygonSet);

  bool condition2Failed = isInterFreeCondition2Faild(polygon, p1, p2);
  if (polygon.size() > 3) {
    std::vector<Vertex *> reversePolygons = {polygon[1], polygon[2], polygon[3], polygon[0]};
    bool reverseCondition2Failed = isInterFreeCondition2Faild(reversePolygons, p1, p2);
    if (!reverseCondition2Failed) {
      /// NOTE: the swap here happens whether intersection-free or not
      polygon.swap(reversePolygons);
    }
    condition2Failed = condition2Failed && reverseCondition2Failed;
  }
  if (intersectionFree && (condition1Failed || condition2Failed)) {
    intersectionPreservingVerticesCount++;
    polygon.clear();
    if (!condition2Failed) {
      std::vector<int> concaveFlags;
      std::vector<Vertex *> convexPart;
      int concaveCount = 0;
      for (int i = 0; i < 4; ++i) {
        int index = (i + firstConcaveFaceVertex) % 4;
        auto faceIter = nodes[index]->faceVertices.find(nodes[(index + 1) % 4]);
        auto cellVertex = nodes[(index + 1) % 4]->clusterVertex;
        if (faceIter != nodes[index]->faceVertices.end()) {
          polygon.push_back(faceIter->second);
          concaveFlags.push_back(1);
          convexPart.push_back(faceIter->second);
          concaveCount++;
        }
        polygon.push_back(cellVertex);
        concaveFlags.push_back(0);
      }
      for (int i = 0; i < polygon.size() - 2; ++i) {
        Vertex *triangle[3] = {
            polygon[0], polygon[i + 1], polygon[i + 2]
        };
        mesh->addTriangle(triangle, g);
      }
    } else {
      g->solve(p1, p2, edgeVertex.hermiteP);
      mesh->addVertex(&edgeVertex, g);
      for (int i = 0; i < 4; ++i) {
        Octree *a = nodes[i];
        Octree *b = nodes[(i + 1) % 4];
        if (a != b) {
          polygon.push_back(a->clusterVertex);
          auto faceVIter = a->faceVertices.find(b);
          if (faceVIter != a->faceVertices.end()) {
            polygon.push_back(faceVIter->second);
            polygon.push_back(faceVIter->second);
          }
          polygon.push_back(b->clusterVertex);
        }
      }
      for (int i = 0; i < polygon.size() / 2; ++i) {
        Vertex *triangle[3] = {
            &edgeVertex, polygon[i * 2], polygon[i * 2 + 1]
        };
        mesh->addTriangle(triangle, g);
      }
    }
  } else {
    for (int i = 2; i < polygon.size(); ++i) {
      Vertex *triangle[3] = {
          polygon[0], polygon[i - 1], polygon[i]
      };
      mesh->addTriangle(triangle, g);
    }
  }
}

void Octree::generatePolygons(Octree *nodes[4], int dir, Mesh *mesh, Topology *g) {
  int minNodeIndex = -1;
  int maxDepth = -1;
  for (int i = 0; i < 4; ++i) {
    if (nodes[i]->depth > maxDepth) {
      minNodeIndex = i;
      maxDepth = nodes[i]->depth;
    }
  }
  int edgeIndex = processEdgeMask[dir][minNodeIndex];

  glm::vec3 p1 = *nodes[minNodeIndex]->clusterMin
      + min_offset_subdivision(edge_map[edgeIndex][0]) * (*nodes[minNodeIndex]->clusterSize);
  glm::vec3 p2 = *nodes[minNodeIndex]->clusterMin
      + min_offset_subdivision(edge_map[edgeIndex][1]) * (*nodes[minNodeIndex]->clusterSize);
  // uint8_t corner1 = nodes[minNodeIndex]->cornerSigns[edge_map[edgeIndex][0]];
  uint8_t corner2 = nodes[minNodeIndex]->cornerSigns[edge_map[edgeIndex][1]];
  glm::vec3 edgeP, normal;
  g->solve(p1, p2, edgeP);
  g->normal(edgeP, normal);
  std::vector<Octree *> polygons;

  for (int i = 0; i < 4; ++i) {
    float NDotDir = glm::dot(edgeProcDir(dir, i), normal);
    int dirSign = 1;
    if (NDotDir > 0.f && corner2 == 0) {
      dirSign = 0;
    }
    int cornerDir = planeSpreadingDir[dir][dirSign][i];
    findFeatureNodes(nodes[i],
                     polygons,
                     cornerDir,
                     false,
                     edgeP,
                     normal);
  }
  if (polygons.size() > 2) {
    for (int i = 2; i < polygons.size(); ++i) {
      mesh->indices.push_back(polygons[0]->vertex.vertexIndex);
      mesh->indices.push_back(polygons[i - 1]->vertex.vertexIndex);
      mesh->indices.push_back(polygons[i]->vertex.vertexIndex);
    }
  }
}

void Octree::drawOctrees(Octree *root, Mesh *mesh, std::unordered_set<Vertex *> &visited) {
  if (!root) {
    return;
  }
  if (root->isLeaf) {
    if (visited.find(root->clusterVertex) != visited.end()) {
      return;
    }
    visited.insert(root->clusterVertex);
    for (int i = 0; i < 12; ++i) {
      auto a = min_offset_subdivision(cellProcFaceMask[i][0]) * (*root->clusterSize) + (*root->clusterMin);
      auto b = min_offset_subdivision(cellProcFaceMask[i][1]) * (*root->clusterSize) + (*root->clusterMin);
//      auto a = min_offset_subdivision(cellProcFaceMask[i][0]) * (root->clusterSize) + (root->min);
//      auto b = min_offset_subdivision(cellProcFaceMask[i][1]) * (root->size) + (root->min);



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
  for (int i = 0; i < 8; ++i) {
    drawOctrees(root->children[i], mesh, visited);
  }
}

void Octree::calHermite(Octree *node, QefSolver *qef, Topology *g, Vertex *vertex) {
  auto &p = vertex->hermiteP;
  qef->solve(p, node->error);
  const auto min = *node->clusterMin - vec3(1e-6);
  const auto max = *node->clusterMin + (*node->clusterSize) + vec3(1e-6);
  if (p.x < min.x || p.x > max.x ||
      p.y < min.y || p.y > max.y ||
      p.z < min.z || p.z > max.z) {

    p = qef->massPointSum / (float) qef->pointCount;
    if (p.x < min.x || p.x > max.x ||
        p.y < min.y || p.y > max.y ||
        p.z < min.z || p.z > max.z) {
      p = glm::min(*node->clusterMin + (*node->clusterSize), p);
      p = glm::max(*node->clusterMin, p);
    }
  }

  g->normal(p, vertex->hermiteN);
}