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

bool Octree::getSelfQef(Octree *node, Topology *g, QefSolver &qef) {
  fvec3 corners[8];
  int8_t mtlID = g->getMaterialID();
  for (int i = 0; i < 8; ++i) {
    corners[i] = min_offset_subdivision(i) * node->size + node->min;
    node->grid.cornerSigns[i] = (uint8_t) (g->value(corners[i]) > 0. ? mtlID : 0);
  }
  bool intersect = false;
  for (int i = 0; i < 7; ++i) {
    if (node->grid.cornerSigns[i] != node->grid.cornerSigns[i + 1]) {
      intersect = true;
    }
  }
  if (!intersect) {
    return false;
  }
  for (int i = 0; i < 12; ++i) {
    fvec3 p1 = corners[edge_map[i][0]];
    fvec3 p2 = corners[edge_map[i][1]];
    if (node->grid.cornerSigns[edge_map[i][0]] != node->grid.cornerSigns[edge_map[i][1]]) {
      fvec3 p, n;
      if (g->solve(p1, p2, p)) {
        g->normal(p, n);
        qef.add(p, n);
      }
    }
  }
  return true;
}

Octree *Octree::buildWithTopology(PositionCode minCode, int depth, Topology *topology) {
  PositionCode sizeCode = PositionCode(1 << (depth - 1));
  auto root = new Octree(minCode, minCode + sizeCode, depth);
  assert(depth > 0);
  bool homogeneous = true;
  if (depth == 1) {
    if (!getSelfQef(root, topology, root->grid.allQef)) {
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
          buildWithTopology(subMinCode, depth - 1, topology);
      homogeneous = homogeneous && !root->children[i];
      if (root->children[i]) {
        root->grid.cornerSigns[i] = root->children[i]->grid.cornerSigns[i];
        root->children[i]->childIndex = static_cast<int8_t>(i);
        root->grid.allQef.combine(root->children[i]->grid.allQef);
      }
    }
    root->isLeaf = false;
  }
  if (homogeneous) {
    delete root;
    return nullptr;
  }
  assert(root->grid.allQef.pointCount);
  assert(!isnan(root->grid.allQef.btb));
  calHermite(root, &root->grid.allQef, topology, &root->vertex);
  return root;
}

void Octree::getSum(Octree *root, PositionCode minPos, PositionCode maxPos, QefSolver &out) {
  if (!root) {
    return;
  }
  if (glm::any(glm::greaterThanEqual(minPos, maxPos))) {
    return;
  }
  if (glm::any(glm::greaterThanEqual(minPos, root->grid.maxCode))
      || glm::any(glm::lessThanEqual(maxPos, root->grid.minCode))) {
    return;
  }
  minPos = glm::max(root->grid.minCode, minPos);
  maxPos = glm::min(root->grid.maxCode, maxPos);
  if (minPos == root->grid.minCode && maxPos == root->grid.maxCode) {
    out.combine(root->grid.allQef);
    return;
  }
  for (int i = 0; i < 8; ++i) {
    getSum(root->children[i], minPos, maxPos, out);
  }
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
  if (root->grid.error < threshold) {
    reduction += childCount - 1;
    for (auto &child: root->children) {
      child = nullptr;
    }
    root->isLeaf = true;
  }
  return reduction;
}

void Octree::cubeExtensionTest(Octree *a, Octree *b, int dir, float minSize) {
//  fvec3 combineMin = glm::min(a->min, b->min);
//  fvec3 combineSize = glm::max(a->min + a->size, b->min + b->size) - combineMin;
//  for (int i = 0; i < 8; ++i) {
//    fvec3 corner = min_offset_subdivision(i) * combineSize + combineMin;
//
//  }

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
    if (indexed.find(&node->vertex) != indexed.end()) {
      return;
    }
    indexed.insert(&node->vertex);
    node->vertex.vertexIndex = static_cast<unsigned int>(mesh->positions.size());
    mesh->positions.push_back(node->vertex.hermiteP);
    mesh->normals.push_back(node->vertex.hermiteN);
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

bool Octree::intersectWithBrothers(int cornerDir, Octree *node) {
  for (int i = 0; i < 3; ++i) {
    int edgeIndex = dirRelatedEdge[cornerDir][node->childIndex][i];
    if (edgeIndex >= 0) {
      if (node->grid.cornerSigns[edge_map[edgeIndex][0]] != node->grid.cornerSigns[edge_map[edgeIndex][1]]) {
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
                              const glm::fvec3 &edgeP,
                              const glm::fvec3 &normal) {
  if (!node) {
    return false;
  }

  if (!intersectWithBrothers(cornerDir, node)) {
    return false;
  }

  //float parallelrity = glm::dot(normalize(edgeP - node->vertex.hermiteP), normal);
  // if (node->isLeaf && node->grid.allQef.roughness > 1e-10) {
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

bool Octree::isInterFreeCondition2Faild(const std::vector<Vertex *> &polygons, const fvec3 &p1, const fvec3 &p2) {
  int anotherV = 3;
  bool interSupportingEdge = false;

  for (int i = 2; i < polygons.size(); ++i) {
    fvec3 baryPos;
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
    fvec3 baryPos;
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
    if (identifier.find(&nodes[i]->vertex) == identifier.end()) {
      polygon.push_back(&nodes[i]->vertex);
    }
    identifier.insert(&nodes[i]->vertex);
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
    if (a != b) {
      if (a->faceVertices.find(b) != a->faceVertices.end()) {
        firstConcaveFaceVertex = i;
        condition1Failed = true;
        continue;
      }
      fvec3 faceMinA = fvec3(std::numeric_limits<float>::max());
      fvec3 faceMinB = faceMinA;
      fvec3 faceMaxA = -fvec3(std::numeric_limits<float>::max());
      fvec3 faceMaxB = faceMaxA;
      for (int j = 0; j < 4; ++j) {
        int subIndexA = faceProcFaceMask[testDir][j][0];
        int subIndexB = faceProcFaceMask[testDir][j][1];
        fvec3 cornerA = min_offset_subdivision(subIndexA) * (a->size) + a->min;
        fvec3 cornerB = min_offset_subdivision(subIndexB) * (b->size) + b->min;
        faceMinA = glm::min(cornerA, faceMinA);
        faceMinB = glm::min(cornerB, faceMinB);
        faceMaxA = glm::max(cornerA, faceMaxA);
        faceMaxB = glm::max(cornerB, faceMaxB);
      }
      fvec3 faceMin = glm::max(faceMinA, faceMinB);
      fvec3 faceMax = glm::min(faceMaxA, faceMaxB);
      if (!segmentFaceIntersection(a->vertex.hermiteP, b->vertex.hermiteP, faceMin, faceMax, testDir)) {
        fvec3 minEnd = faceMin + directionMap(dir) * (faceMax - faceMin);
        fvec3 maxEnd = faceMax - directionMap(dir) * (faceMax - faceMin);
        glm::fvec3 points[4] = {faceMin, minEnd, faceMax, maxEnd};
        fvec3 massPointSum(0.f);
        int pointCount = 0;
        for (int k = 0; k < 4; ++k) {
          float v1 = g->value(points[k]);
          float v2 = g->value(points[(k + 1) % 4]);
          if ((v1 >= 0 && v2 < 0) || (v1 < 0 && v2 >= 0)) {
            fvec3 x;
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
  fvec3 p1 = nodes[minCellIndex]->size * min_offset_subdivision(edgeProcEdgeMask[dir][0][minCellIndex])
      + nodes[minCellIndex]->min;
  fvec3 p2 = nodes[minCellIndex]->size * min_offset_subdivision(edgeProcEdgeMask[dir][1][minCellIndex])
      + nodes[minCellIndex]->min;

  for (int i = 0; i < 4; ++i) {
    p1[dir] = std::max((nodes[i]->min)[dir], p1[dir]);
    p2[dir] = std::min((nodes[i]->min)[dir] + (nodes[i]->size)[dir], p2[dir]);
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
        auto cellVertex = &(nodes[(index + 1) % 4]->vertex);
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
          polygon.push_back(&a->vertex);
          auto faceVIter = a->faceVertices.find(b);
          if (faceVIter != a->faceVertices.end()) {
            polygon.push_back(faceVIter->second);
            polygon.push_back(faceVIter->second);
          }
          polygon.push_back(&b->vertex);
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

  glm::fvec3 p1 = nodes[minNodeIndex]->min
      + min_offset_subdivision(edge_map[edgeIndex][0]) * (nodes[minNodeIndex]->size);
  glm::fvec3 p2 = nodes[minNodeIndex]->min
      + min_offset_subdivision(edge_map[edgeIndex][1]) * (nodes[minNodeIndex]->size);
  // uint8_t corner1 = nodes[minNodeIndex]->grid.cornerSigns[edge_map[edgeIndex][0]];
  uint8_t corner2 = nodes[minNodeIndex]->grid.cornerSigns[edge_map[edgeIndex][1]];
  glm::fvec3 edgeP, normal;
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

void Octree::calHermite(Octree *node, QefSolver *qef, Topology *g, Vertex *vertex) {
  auto &p = vertex->hermiteP;
  qef->solve(p, node->grid.error);
  const auto min = node->min - fvec3(1e-6);
  const auto max = node->min + (node->size) + fvec3(1e-6);
  if (p.x < min.x || p.x > max.x ||
      p.y < min.y || p.y > max.y ||
      p.z < min.z || p.z > max.z) {

    p = qef->massPointSum / (float) qef->pointCount;
    if (p.x < min.x || p.x > max.x ||
        p.y < min.y || p.y > max.y ||
        p.z < min.z || p.z > max.z) {
      p = glm::min(node->min + (node->size), p);
      p = glm::max(node->min, p);
    }
  }
  g->normal(p, vertex->hermiteN);
}