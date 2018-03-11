//
// Created by Danielhu on 2018/1/20.
//
#include <glm/glm.hpp>
#include <Mesh.h>
#include "Octree.h"

using namespace glm;

const vec3 min_offset_subdivision(int i) {
  static const vec3 offsets[8] = {
      vec3(0.f, 0.f, 0.f),
      vec3(0.f, 0.f, 1.f),
      vec3(0.f, 1.f, 0.f),
      vec3(0.f, 1.f, 1.f),
      vec3(1.f, 0.f, 0.f),
      vec3(1.f, 0.f, 1.f),
      vec3(1.f, 1.f, 0.f),
      vec3(1.f, 1.f, 1.f),
  };
  assert(i >= 0 && i < 8);
  return offsets[i];
};

const vec3 directionMap(int i) {
  static const vec3 offsets[3] = {
      vec3(1.f, 0.f, 0.f),
      vec3(0.f, 1.f, 0.f),
      vec3(0.f, 0.f, 1.f),
  };
  assert(i >= 0 && i < 3);
  return offsets[i];
}

// from original dc implementation

const int edge_map[12][2] = {
    {0, 4}, {1, 5}, {2, 6}, {3, 7},    // x-axis
    {0, 2}, {1, 3}, {4, 6}, {5, 7},    // y-axis
    {0, 1}, {2, 3}, {4, 5}, {6, 7}     // z-axis
};

const int faceNodeOrder[3][4][4] = {
    {{0, 0, 1, 1}, {0, 0, 1, 1}, {0, 1, 1, 0}, {0, 1, 1, 0}},
    {{0, 1, 1, 0}, {0, 1, 1, 0}, {0, 0, 1, 1}, {0, 0, 1, 1}},
    {{0, 0, 1, 1}, {0, 0, 1, 1}, {0, 1, 1, 0}, {0, 1, 1, 0}},
};

const int cellProcFaceMask[12][3] =
    {{0, 4, 0}, {1, 5, 0}, {2, 6, 0}, {3, 7, 0}, {0, 2, 1}, {4, 6, 1}, {1, 3, 1}, {5, 7, 1}, {0, 1, 2}, {2, 3, 2},
     {4, 5, 2}, {6, 7, 2}};

const int cellProcEdgeMask[6][5] =
    {{0, 2, 3, 1, 0}, {4, 6, 7, 5, 0}, {0, 1, 5, 4, 1}, {2, 3, 7, 6, 1}, {0, 4, 6, 2, 2}, {1, 5, 7, 3, 2}};

const int faceProcFaceMask[3][4][3] = {
    {{4, 0, 0}, {5, 1, 0}, {6, 2, 0}, {7, 3, 0}},
    {{2, 0, 1}, {6, 4, 1}, {3, 1, 1}, {7, 5, 1}},
    {{1, 0, 2}, {3, 2, 2}, {5, 4, 2}, {7, 6, 2}}
};

const vec3 faceSubDivision(int dir, int i) {
  static const vec3 offsets[3][4] = {
      {vec3(0.f, 0.f, 0.f), vec3(0.f, 0.f, 1.f), vec3(0.f, 1.f, 0.f), vec3(0.f, 1.f, 1.f),},
      {vec3(0.f, 0.f, 0.f), vec3(1.f, 0.f, 0.f), vec3(0.f, 0.f, 1.f), vec3(1.f, 0.f, 1.f),},
      {vec3(0.f, 0.f, 0.f), vec3(0.f, 1.f, 0.f), vec3(1.f, 0.f, 0.f), vec3(1.f, 1.f, 0.f),},
  };
  assert(i >= 0 && i < 4);
  return offsets[dir][i];
};

const int faceProcEdgeMask[3][4][6] = {
    {{1, 4, 5, 1, 0, 1}, {1, 6, 7, 3, 2, 1}, {0, 4, 0, 2, 6, 2}, {0, 5, 1, 3, 7, 2}},
    {{0, 2, 0, 1, 3, 0}, {0, 6, 4, 5, 7, 0}, {1, 2, 6, 4, 0, 2}, {1, 3, 7, 5, 1, 2}},
    {{1, 1, 3, 2, 0, 0}, {1, 5, 7, 6, 4, 0}, {0, 1, 0, 4, 5, 1}, {0, 3, 2, 6, 7, 1}}
};

const vec3 edgeProcDir(int i, int j) {
  const vec3 dirs[3][4] = {
      {vec3(0.f, -1.f, -1.f), vec3(0.f, -1.f, 1.f), vec3(0.f, 1.f, 1.f), vec3(0.f, 1.f, -1.f),},
      {vec3(-1.f, 0.f, -1.f), vec3(-1.f, 0.f, 1.f), vec3(1.f, 0.f, 1.f), vec3(1.f, 0.f, -1.f),},
      {vec3(-1.f, -1.f, 0.f), vec3(1.f, -1.f, 0.f), vec3(1.f, 1.f, 0.f), vec3(-1.f, 1.f, 0.f),},
  };
  assert(i >= 0 && i < 3 && i >= 0 && i < 4);
  return dirs[i][j];
};

const int edgeProcEdgeMask[3][2][5] = {
    {{3, 1, 0, 2, 0}, {7, 5, 4, 6, 0}},
    {{5, 4, 0, 1, 1}, {7, 6, 2, 3, 1}},
    {{6, 2, 0, 4, 2}, {7, 3, 1, 5, 2}},
};

const int planeSpreadingDir[3][2][4] = {
    {{0, 2, 3, 1}, {4, 6, 7, 5}},
    {{0, 1, 5, 4}, {2, 3, 7, 6}},
    {{0, 4, 6, 2}, {1, 5, 7, 2}},
};
//
//const int planeSpreadingMask[8][8] = {
//    {0, 1, 2, 4, 3, 5, 6, 7},
//    {1, 0, 3, 5, 2, 4, 7, 6},
//    {2, 0, 3, 6, 1, 4, 7, 5},
//    {3, 1, 2, 7, 0, 5, 6, 4},
//    {4, 0, 5, 6, 1, 2, 7, 3},
//    {5, 1, 4, 7, 0, 3, 6, 2},
//    {6, 2, 4, 7, 0, 3, 5, 1},
//    {7, 3, 5, 6, 1, 2, 4, 0},
//};
//
//const int adjacentNodes[8][8] = {
//    {0, 1, 1, 0, 1, 0, 0, 0,},
//    {1, 0, 0, 1, 0, 1, 0, 0,},
//    {1, 0, 0, 1, 0, 0, 1, 0,},
//    {0, 1, 1, 0, 0, 0, 0, 1,},
//    {1, 0, 0, 0, 0, 1, 1, 0,},
//    {0, 1, 0, 0, 1, 0, 0, 1,},
//    {0, 0, 1, 0, 1, 0, 0, 1,},
//    {0, 0, 0, 1, 0, 1, 1, 0,},
//};

const int dirRelatedEdge[8][8][3] = {
    {
        {-1, -1, -1}, {-1, 2, 6}, {-1, 1, 10}, {-1, -1, 0},
        {-1, 5, 9}, {-1, -1, 4}, {-1, -1, 8}, {0, 4, 8},
    },
    {
        {-1, 3, 11}, {-1, -1, -1}, {-1, -1, 1}, {-1, 0, 10},
        {-1, -1, 5}, {-1, 4, 9}, {1, 5, 8}, {-1, -1, 8},
    },
    {
        {-1, 11, 3}, {-1, -1, 2}, {-1, -1, -1}, {-1, 0, 6},
        {-1, -1, 9}, {2, 4, 9}, {-1, 5, 8}, {-1, -1, 4},
    },
    {
        {-1, -1, 3}, {-1, 2, 11}, {-1, 1, 7}, {-1, -1, -1},
        {3, 5, 9}, {-1, -1, 9}, {-1, -1, 5}, {-1, 4, 8},
    },
    {
        {-1, 7, 11}, {-1, -1, 5}, {-1, -1, 10}, {0, 6, 10},
        {-1, -1, -1}, {-1, 2, 4}, {-1, 1, 8}, {-1, -1, 0},
    },
    {
        {-1, -1, 7}, {-1, 0, 11}, {1, 7, 10}, {-1, -1, 10},
        {-1, 3, 5}, {-1, -1, -1}, {-1, -1, 1}, {-1, 1, 8}
    },
    {
        {-1, -1, 11}, {2, 6, 11}, {-1, 7, 10}, {-1, -1, 6},
        {-1, 3, 9}, {-1, -1, 2}, {-1, -1, -1}, {-1, 1, 4},
    },
    {
        {3, 7, 11}, {-1, -1, 11}, {-1, -1, 7}, {-1, 6, 10},
        {-1, -1, 3}, {-1, 2, 9}, {-1, 1, 5}, {-1, -1, -1},
    }
};

const int processEdgeMask[3][4] = {{3, 2, 1, 0}, {7, 5, 6, 4}, {11, 10, 9, 8}};

const int triangleIndices[6] = {0, 1, 2, 0, 2, 3};
const int triangleIndicesFlip[6] = {0, 3, 2, 0, 2, 1};

bool Octree::getSelfQef(Octree* node, Topology *g, QefSolver &qef) {
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

std::shared_ptr<Octree> Octree::buildWithTopology(
    glm::vec3 min,
    vec3 size,
    int depth,
    Topology *topology,
    int &losslessCut) {
  auto root = buildRecursively(min, size, depth, topology);
  root = losslessCompress(root, -1, topology, losslessCut);
  return root;
}

std::shared_ptr<Octree> Octree::buildRecursively(glm::vec3 min, vec3 size, int depth, Topology *topology) {
  auto root = std::make_shared<Octree>(min, size, depth);
  assert(depth > 0);
  if (depth == 1) {
    if (!getSelfQef(root.get(), topology, root->qef)) {
      root->internal = true;
      return root;
    }
    calHermite(root.get(), root->qef, topology);
    root->isLeaf = true;
    return root;
  }
  for (int i = 0; i < 8; ++i) {
    long count = root->children[i].use_count();
    root->children[i] =
        buildRecursively(min + min_offset_subdivision(i) * size / 2.f, size / 2.f, depth - 1, topology);
    count = root->children[i].use_count();
    root->cornerSigns[i] = root->children[i]->cornerSigns[i];
    root->children[i]->childIndex = static_cast<int8_t>(i);
  }
  root->isLeaf = false;
  return root;
}

std::shared_ptr<Octree> Octree::losslessCompress(std::shared_ptr<Octree> root,
                                                 float threshold,
                                                 Topology *topology,
                                                 int &count) {
  if (!root) {
    return nullptr;
  }
  if (root->isLeaf) {
    if (root->internal) {
      count++;
      return nullptr;
    }
    return root;
  }

  bool internal = true;

  for (int i = 0; i < 8; ++i) {

    auto result = losslessCompress(root->children[i], threshold, topology, count);
    if (result == nullptr) {
      root->children[i] = nullptr;
    }
    internal = internal && (root->children[i] == nullptr);
  }

  if (internal) {
    count++;
    return nullptr;
  }

  return root;
}

void Octree::simplify(std::shared_ptr<Octree> root, float threshold, Topology *geometry, int &count) {
  if (!root) {
    return;
  }
  if (root->isLeaf) {
    return;
  }
  QefSolver sum;
  for (auto &child : root->children) {
    simplify(child, threshold, geometry, count);
    if (child) {
      sum.combine(child->qef);
    }
  }
  vec3 tempP;
  sum.solve(tempP, root->error);
  root->qef.set(sum);
  if (root->error < threshold) {
    // getSelfQef(root, geometry, root->qef);
    calHermite(root.get(), sum, geometry);
    count++;
    for (auto &child: root->children) {
      child = nullptr;
    }
    root->isLeaf = true;
  }
}

std::shared_ptr<Octree> Octree::edgeSimplify(std::shared_ptr<Octree> root,
                                             float threshold,
                                             Topology *geometry,
                                             int &count) {
  if (!root || root->isLeaf) {
    return root;
  }
  for (int i = 0; i < 8; ++i) {
    root->children[i] = edgeSimplify(root->children[i], threshold, geometry, count);
  }
  for (int i = 0; i < 12; ++i) {
    auto &a = root->children[cellProcFaceMask[i][0]];
    auto &b = root->children[cellProcFaceMask[i][1]];
    const int dir = cellProcFaceMask[i][2];
    edgeCollapse(a,
                 b,
                 dir,
                 threshold,
                 geometry,
                 root->min + min_offset_subdivision(cellProcFaceMask[i][0]) * root->size / 2.f,
                 root->size.x / 2.f,
                 count);
  }
  return root;
  bool nodeCollapsable = true;
  std::shared_ptr<Octree> singleChild = nullptr;
  for (int i = 0; i < 8; ++i) {
    if (root->children[i] != nullptr) {
      if (singleChild == nullptr) {
        singleChild = root->children[i];
      } else if (root->children[i] != singleChild) {
        nodeCollapsable = false;
        break;
      }
    }
  }
  if (nodeCollapsable) {
    if (singleChild) {
      root->isLeaf = singleChild->isLeaf;
      root->qef.set(singleChild->qef);
      root->hermiteP = singleChild->hermiteP;
      root->hermiteN = singleChild->hermiteN;
    } else {
      getSelfQef(root.get(), geometry, root->qef);
      calHermite(root.get(), root->qef, geometry);
    }
  }
  return root;
}

void Octree::edgeCollapse(std::shared_ptr<Octree> &a,
                          std::shared_ptr<Octree> &b,
                          int dir,
                          float threshold,
                          Topology *geometry,
                          vec3 faceMin,
                          float faceSize,
                          int &count) {
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
                     subSize,
                     count
        );
      } else if (a->isLeaf) {
        edgeCollapse(a, b->children[faceProcFaceMask[dir][i][1]], dir, threshold, geometry, subMin, subSize, count);
      } else {
        edgeCollapse(a->children[faceProcFaceMask[dir][i][0]], b, dir, threshold, geometry, subMin, subSize, count);
      }
    }
    return;
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

  if (a == b) {
    return;
  }
  QefSolver edgeQEF;
  edgeQEF.combine(a->qef);
  edgeQEF.combine(b->qef);
  vec3 p, n;
  float error;
  edgeQEF.solve(p, error);
  vec3 surfaceEdge = b->hermiteP - a->hermiteP;

  // vec3 middle = (b->hermiteP * (float)b->qef.pointCount + a->hermiteP * (float)a->qef.pointCount) / (float)(b->qef.pointCount + a->qef.pointCount);
  // vec3 middle = (a->hermiteP + b->hermiteP) / 2.f;
  float roughnessA = glm::dot(glm::normalize(surfaceEdge), glm::normalize(a->qef.averageNormalSum));
  float roughnessB = glm::dot(glm::normalize(surfaceEdge), glm::normalize(b->qef.averageNormalSum));
  if ((roughnessA * roughnessA < threshold && roughnessB * roughnessB < threshold) ||
      (a->qef.roughness < threshold || b->qef.roughness < threshold)) {
    combine(a, b, edgeQEF, p, count);
  }
}

void Octree::compress(std::shared_ptr<Octree> root, float threshold, Topology *geometry, int &count) {
  if (!root) {
    return;
  }
  if (root->isLeaf) {
    return;
  }
  QefSolver sum;
  for (auto &child : root->children) {
    compress(child, threshold, geometry, count);
    if (child) {
      sum.combine(child->qef);
    }
  }
  vec3 tempP;
  sum.solve(tempP, root->error);
  root->qef.set(sum);
  if (root->error < threshold) {
    // getSelfQef(root, geometry, root->qef);
    calHermite(root.get(), sum, geometry);
    count++;
    for (auto &child: root->children) {
      child = nullptr;
    }
    root->isLeaf = true;
  }
}

Mesh *Octree::generateMesh(std::shared_ptr<Octree> root, Topology *geometry, int &count) {
  assert(root);
  auto *mesh = new Mesh();
  generateVertexIndices(root, mesh, geometry);
  contourCell(root, mesh, geometry, count);
  return mesh;
}

void Octree::generateVertexIndices(std::shared_ptr<Octree> node, Mesh *mesh, Topology *geometry) {
  if (!node) {
    return;
  }
  node->vertexIndex = static_cast<unsigned int>(mesh->positions.size());
  mesh->positions.push_back(node->hermiteP);
  mesh->normals.push_back(node->hermiteN);
  for (int i = 0; i < 8; ++i) {
    generateVertexIndices(node->children[i], mesh, geometry);
  }
}

void Octree::contourCell(std::shared_ptr<Octree> root, Mesh *mesh, Topology *geometry, int &count) {
  if (!root || root->isLeaf) {
    return;
  }
  for (int i = 0; i < 8; ++i) {
    contourCell(root->children[i], mesh, geometry, count);
  }
  for (int i = 0; i < 12; ++i) {
    std::shared_ptr<Octree> nodes[2] = {
        root->children[cellProcFaceMask[i][0]],
        root->children[cellProcFaceMask[i][1]],
    };
    contourFace(nodes, cellProcFaceMask[i][2], mesh, geometry, count);
  }
  for (int i = 0; i < 6; ++i) {
    std::shared_ptr<Octree> nodes[4];
    for (int j = 0; j < 4; ++j) {
      nodes[j] = root->children[cellProcEdgeMask[i][j]];
    }
    contourEdge(nodes, cellProcEdgeMask[i][4], mesh, geometry);
  }
}

void Octree::contourFace(std::shared_ptr<Octree> nodes[2], int dir, Mesh *mesh, Topology *geometry, int &count) {
  if (!nodes[0] || !nodes[1]) {
    return;
  }
  if (nodes[0]->isLeaf && nodes[1]->isLeaf) {
    count++;
    return;
  }
  // the subdivision of a face resulting 4 child faces;
  for (int i = 0; i < 4; ++i) {
    std::shared_ptr<Octree> subdivision_face[2] = {nodes[0], nodes[1]};
    for (int j = 0; j < 2; ++j) {
      if (!subdivision_face[j]->isLeaf) {
        subdivision_face[j] = subdivision_face[j]->children[faceProcFaceMask[dir][i][j]];
      }
    }
    contourFace(subdivision_face, faceProcFaceMask[dir][i][2], mesh, geometry, count);
  }
  for (int i = 0; i < 4; ++i) {
    std::shared_ptr<Octree> edge_nodes[4];
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
    contourEdge(edge_nodes, faceProcEdgeMask[dir][i][5], mesh, geometry);
  }
}

void Octree::contourEdge(std::shared_ptr<Octree> nodes[4], int dir, Mesh *mesh, Topology *geometry) {
  if (!nodes[0] || !nodes[1] || !nodes[2] || !nodes[3]) {
    return;
  }

  if (nodes[0]->isLeaf && nodes[1]->isLeaf && nodes[2]->isLeaf && nodes[3]->isLeaf) {
    generateQuad(nodes, dir, mesh, geometry);
    return;
  }

  // the subdivision of a edge resulting 2 child edges;
  for (int i = 0; i < 2; ++i) {
    std::shared_ptr<Octree> subdivision_edge[4];
    for (int j = 0; j < 4; ++j) {
      if (!nodes[j]->isLeaf) {
        subdivision_edge[j] = nodes[j]->children[edgeProcEdgeMask[dir][i][j]];
      } else {
        subdivision_edge[j] = nodes[j];
      }
    }
    contourEdge(subdivision_edge, edgeProcEdgeMask[dir][i][4], mesh, geometry);
  }
}

void Octree::combine(std::shared_ptr<Octree> &a,
                     std::shared_ptr<Octree> &b,
                     const QefSolver &combination,
                     glm::vec3 combinePos,
                     int &count) {

  if (a->hasEdgeCollapse || b->hasEdgeCollapse) { ;
  }

  vec3 amax = a->size + a->min;
  vec3 bmax = a->size + b->min;

  vec3 abmax = glm::max(amax, bmax);
  vec3 abmin = glm::min(a->min, b->min);
  a->min = abmin;
  a->size = abmax - abmin;
  a->qef.set(combination);
  a->hermiteP = combinePos;
  b = a;
  b->hasEdgeCollapse = true;
  count++;
}

bool Octree::intersectWithBrothers(int cornerDir, std::shared_ptr<Octree> node) {
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

bool Octree::findFeatureNodes(std::shared_ptr<Octree> node,
                              std::vector<std::shared_ptr<Octree> > &results,
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

  //float parallelrity = glm::dot(normalize(edgeP - node->hermiteP), normal);
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

void Octree::generateQuad(std::shared_ptr<Octree> nodes[4], int dir, Mesh *mesh, Topology *g) {
  int minNodeIndex = -1;
  int maxDepth = -1;
  for (int i = 0; i < 4; ++i) {
    if (nodes[i]->depth > maxDepth) {
      minNodeIndex = i;
      maxDepth = nodes[i]->depth;
    }
  }

  int differenceEdge = 0;

  for (int i = 0; i < 4; ++i) {
    for (int j = i + 1; j < 4; ++j) {
      if (nodes[i] != nodes[j]) {
        differenceEdge++;
      }
    }
  }
  if (differenceEdge < 5) {
    return;
  }
//  int edgeIndex = processEdgeMask[dir][minNodeIndex];
//  uint8_t corner1 = nodes[minNodeIndex]->cornerSigns[edge_map[edgeIndex][0]];
//  uint8_t corner2 = nodes[minNodeIndex]->cornerSigns[edge_map[edgeIndex][1]];
  bool signDir = true;// (corner1 != corner2);
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      int quadIndex = triangleIndicesFlip[i * 3 + j];
      int adjacentNodeIndices[2] = {
          triangleIndices[i * 3 + (j + 1) % 3],
          triangleIndices[i * 3 + (j + 2) % 3]
      };
      if (signDir) {
        quadIndex = triangleIndicesFlip[i * 3 + j];
        adjacentNodeIndices[0] = triangleIndices[i * 3 + (j + 1) % 3];
        adjacentNodeIndices[1] = triangleIndices[i * 3 + (j + 2) % 3];
      }

      std::shared_ptr<Octree> targetNode = nodes[quadIndex];
      std::shared_ptr<Octree> adjacentNodes[2] = {nodes[adjacentNodeIndices[0]], nodes[adjacentNodeIndices[1]]};
      glm::vec3 offset =
          adjacentNodes[1]->hermiteP - targetNode->hermiteP +
              adjacentNodes[0]->hermiteP - targetNode->hermiteP;
      offset *= 0.05f;
      glm::vec3 normal;
      g->normal(targetNode->hermiteP + offset, normal);
      if (glm::dot(normal, targetNode->hermiteN) < std::cos(glm::radians(15.f))) {
        mesh->indices.push_back(static_cast<unsigned int>(mesh->positions.size()));
        mesh->positions.push_back(targetNode->hermiteP);
        mesh->normals.push_back(normal);
      } else {
        mesh->indices.push_back(targetNode->vertexIndex);
      }
    }
  }
}

void Octree::generatePolygons(std::shared_ptr<Octree> nodes[4], int dir, Mesh *mesh, Topology *g) {
  int minNodeIndex = -1;
  int maxDepth = -1;
  for (int i = 0; i < 4; ++i) {
    if (nodes[i]->depth > maxDepth) {
      minNodeIndex = i;
      maxDepth = nodes[i]->depth;
    }
  }
  int edgeIndex = processEdgeMask[dir][minNodeIndex];

  glm::vec3 p1 = nodes[minNodeIndex]->min + min_offset_subdivision(edge_map[edgeIndex][0]) * nodes[minNodeIndex]->size;
  glm::vec3 p2 = nodes[minNodeIndex]->min + min_offset_subdivision(edge_map[edgeIndex][1]) * nodes[minNodeIndex]->size;
  // uint8_t corner1 = nodes[minNodeIndex]->cornerSigns[edge_map[edgeIndex][0]];
  uint8_t corner2 = nodes[minNodeIndex]->cornerSigns[edge_map[edgeIndex][1]];
  glm::vec3 edgeP, normal;
  g->solve(p1, p2, edgeP);
  g->normal(edgeP, normal);
  std::vector<std::shared_ptr<Octree> > polygons;

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
      mesh->indices.push_back(polygons[0]->vertexIndex);
      mesh->indices.push_back(polygons[i - 1]->vertexIndex);
      mesh->indices.push_back(polygons[i]->vertexIndex);
    }
  }
}

void Octree::collapse(Topology *g) {
  isLeaf = true;
  getSelfQef(this, g, qef);
  calHermite(this, qef, g);
  qef.solve(hermiteP, error);
  for (int i = 0; i < 8; ++i) {
    children[i] = nullptr;
  }
}

void Octree::calHermite(Octree* node, QefSolver &qef, Topology *g) {
  auto &p = node->hermiteP;
  qef.solve(p, node->error);
  auto &min = node->min;
  auto max = node->min + vec3(node->size);
  if (p.x < min.x || p.x > max.x ||
      p.y < min.y || p.y > max.y ||
      p.z < min.z || p.z > max.z) {
    p = qef.massPointSum / (float) qef.pointCount;
  }
  g->normal(p, node->hermiteN);
}