//
// Created by Danielhu on 2018/1/20.
//
#include <glm/glm.hpp>
#include <Mesh.h>
#include "Octree.h"

using namespace glm;

const vec3 min_offset_subdivision[8] = {
    vec3(0.f, 0.f, 0.f),
    vec3(0.f, 0.f, 1.f),
    vec3(0.f, 1.f, 0.f),
    vec3(0.f, 1.f, 1.f),
    vec3(1.f, 0.f, 0.f),
    vec3(1.f, 0.f, 1.f),
    vec3(1.f, 1.f, 0.f),
    vec3(1.f, 1.f, 1.f),
};

// from original dc implementation

const int edge_map[12][2] = {
    {0, 4}, {1, 5}, {2, 6}, {3, 7},    // x-axis
    {0, 2}, {1, 3}, {4, 6}, {5, 7},    // y-axis
    {0, 1}, {2, 3}, {4, 5}, {6, 7}        // z-axis
};

const int faceNodeOrder[2][4] = {{0, 0, 1, 1}, {0, 1, 0, 1}};

const int cellProcFaceMask[12][3] =
    {{0, 4, 0}, {1, 5, 0}, {2, 6, 0}, {3, 7, 0}, {0, 2, 1}, {4, 6, 1}, {1, 3, 1}, {5, 7, 1}, {0, 1, 2}, {2, 3, 2},
     {4, 5, 2}, {6, 7, 2}};
const int cellProcEdgeMask[6][5] =
    {{0, 1, 2, 3, 0}, {4, 5, 6, 7, 0}, {0, 4, 1, 5, 1}, {2, 6, 3, 7, 1}, {0, 2, 4, 6, 2}, {1, 3, 5, 7, 2}};

const int faceProcFaceMask[3][4][3] = {
    {{4, 0, 0}, {5, 1, 0}, {6, 2, 0}, {7, 3, 0}},
    {{2, 0, 1}, {6, 4, 1}, {3, 1, 1}, {7, 5, 1}},
    {{1, 0, 2}, {3, 2, 2}, {5, 4, 2}, {7, 6, 2}}
};

const int faceProcEdgeMask[3][4][6] = {
    {{1, 4, 0, 5, 1, 1}, {1, 6, 2, 7, 3, 1}, {0, 4, 6, 0, 2, 2}, {0, 5, 7, 1, 3, 2}},
    {{0, 2, 3, 0, 1, 0}, {0, 6, 7, 4, 5, 0}, {1, 2, 0, 6, 4, 2}, {1, 3, 1, 7, 5, 2}},
    {{1, 1, 0, 3, 2, 0}, {1, 5, 4, 7, 6, 0}, {0, 1, 5, 0, 4, 1}, {0, 3, 7, 2, 6, 1}}
};

const int edgeProcEdgeMask[3][2][5] = {
    {{3, 2, 1, 0, 0}, {7, 6, 5, 4, 0}},
    {{5, 1, 4, 0, 1}, {7, 3, 6, 2, 1}},
    {{6, 4, 2, 0, 2}, {7, 5, 3, 1, 2}},
};

const int processEdgeMask[3][4] = {{3, 2, 1, 0}, {7, 5, 6, 4}, {11, 10, 9, 8}};

const int triangleIndices[6] = {0, 1, 3, 0, 3, 2};
const int triangleIndicesFlip[6] = {0, 3, 1, 0, 2, 3};

bool Octree::getSelfQef(Octree *node, Topology *g, QefSolver &qef) {
  vec3 corners[8];
  int8_t mtlID = g->getMaterialID();
  for (int i = 0; i < 8; ++i) {
    corners[i] = min_offset_subdivision[i] * node->size + node->min;
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

Octree *Octree::buildWithTopology(
    glm::vec3 min,
    float size,
    int depth,
    Topology *topology,
    int &losslessCut) {
  Octree *root = buildRecursively(min, size, depth, topology);
  root = losslessCompress(root, -1, topology, losslessCut);
  return root;
}

Octree *Octree::buildRecursively(glm::vec3 min, float size, int depth, Topology *topology) {
  auto root = new Octree(min, size, depth);
  assert(depth > 0);
  if (depth == 1) {
    if (!getSelfQef(root, topology, root->qef)) {
      root->internal = true;
      return root;
    }
    calHermite(root, root->qef, topology);
    root->isLeaf = true;
    return root;
  }
  for (int i = 0; i < 8; ++i) {
    root->children[i] =
        buildRecursively(min + min_offset_subdivision[i] * size / 2.f, size / 2.f, depth - 1, topology);
    root->cornerSigns[i] = root->children[i]->cornerSigns[i];
  }
  root->isLeaf = false;
  return root;
}

Octree *Octree::losslessCompress(Octree *root, float threshold, Topology *topology, int &count) {
  if (!root) {
    return nullptr;
  }
  if (root->isLeaf) {
    if (root->internal) {
      count++;
      delete root;
      return nullptr;
    }
    return root;
  }

  bool internal = true;

  for (auto &child : root->children) {
    child = losslessCompress(child, threshold, topology, count);
    internal = internal && (child == nullptr);
  }

  if (internal) {
    count++;
    delete root;
    return nullptr;
  }

  return root;
}

void Octree::uniformSimplify(Octree *root, float threshold, Topology *geometry, int &count) {
  if (!root) {
    return;
  }
  if (root->isLeaf) {
    return;
  }
  QefSolver sum;
  for (auto &child : root->children) {
    uniformSimplify(child, threshold, geometry, count);
    if (child) {
      sum.combine(child->qef);
    }
  }
  vec3 tempP;
  float sumError;
  sum.solve(tempP, sumError);
  root->qef.set(sum);
  if (sumError > threshold) {
  } else {
    // getSelfQef(root, geometry, root->qef);
    calHermite(root, sum, geometry);
    count++;
    for (auto &child: root->children) {
      delete child;
      child = nullptr;
    }
    root->isLeaf = true;
  }
}

Mesh *Octree::generateMesh(Octree *root) {
  assert(root);
  auto *mesh = new Mesh();
  unsigned int vertexCount = 0;
  generateVertexIndices(root, vertexCount, mesh);
  contourCell(root, mesh);
  return mesh;
}

void Octree::generateVertexIndices(Octree *root, unsigned int &count, Mesh *mesh) {
  if (!root) {
    return;
  }
  if (root->isLeaf) {
    root->vertexIndex = count++;
    mesh->positions.push_back(root->hermiteP);
    mesh->normals.push_back(root->hermiteN);
    return;
  }
  for (int i = 0; i < 8; ++i) {
    generateVertexIndices(root->children[i], count, mesh);
  }
}

void Octree::contourCell(Octree *root, Mesh *mesh) {
  if (!root || root->isLeaf) {
    return;
  }
  for (int i = 0; i < 8; ++i) {
    contourCell(root->children[i], mesh);
  }
  for (int i = 0; i < 12; ++i) {
    Octree *nodes[2] = {
        root->children[cellProcFaceMask[i][0]],
        root->children[cellProcFaceMask[i][1]],
    };
    contourFace(nodes, cellProcFaceMask[i][2], mesh);
  }
  for (int i = 0; i < 6; ++i) {
    Octree *nodes[4];
    for (int j = 0; j < 4; ++j) {
      nodes[j] = root->children[cellProcEdgeMask[i][j]];
    }
    contourEdge(nodes, cellProcEdgeMask[i][4], mesh);
  }
}

void Octree::contourFace(Octree *nodes[2], int dir, Mesh *mesh) {
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
    contourFace(subdivision_face, faceProcFaceMask[dir][i][2], mesh);
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
    const int *order = faceNodeOrder[faceProcEdgeMask[dir][i][0]];
    for (int j = 0; j < 4; ++j) {
      if (nodes[order[j]]->isLeaf) {
        edge_nodes[j] = nodes[order[j]];
      } else {
        edge_nodes[j] = nodes[order[j]]->children[c[j]];
      }
    }
    contourEdge(edge_nodes, faceProcEdgeMask[dir][i][5], mesh);
  }
}

void Octree::contourEdge(Octree *nodes[4], int dir, Mesh *mesh) {
  if (!nodes[0] || !nodes[1] || !nodes[2] || !nodes[3]) {
    return;
  }
  if (nodes[0]->isLeaf && nodes[1]->isLeaf && nodes[2]->isLeaf && nodes[3]->isLeaf) {
    int vertexCount = 0;
    generateQuad(nodes, vertexCount, mesh);
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
    contourEdge(subdivision_edge, edgeProcEdgeMask[dir][i][4], mesh);
  }
}

void Octree::generateQuad(Octree *nodes[4], int dir, Mesh *mesh) {
  int minNodeIndex = -1;
  int maxDepth = -1;
  for (int i = 0; i < 4; ++i) {
    if (nodes[i]->depth > maxDepth) {
      minNodeIndex = i;
      maxDepth = nodes[i]->depth;
    }
  }
  int edgeIndex = processEdgeMask[dir][minNodeIndex];
  uint8_t corner1 = nodes[minNodeIndex]->cornerSigns[edge_map[edgeIndex][0]];
  uint8_t corner2 = nodes[minNodeIndex]->cornerSigns[edge_map[edgeIndex][1]];
  bool signDir = (corner1 != corner2);

  if (signDir) {
    for (int i = 0; i < 6; ++i) {
      mesh->indices.push_back(nodes[triangleIndices[i]]->vertexIndex);
    }
  } else {
    for (int i = 0; i < 6; ++i) {
      mesh->indices.push_back(nodes[triangleIndicesFlip[i]]->vertexIndex);
    }
  }

}

void Octree::collapse(Topology *g) {
  isLeaf = true;
  getSelfQef(this, g, qef);
  calHermite(this, qef, g);
  qef.solve(hermiteP, error);
  for (int i = 0; i < 8; ++i) {
    delete children[i];
    children[i] = nullptr;
  }
}

void Octree::calHermite(Octree* node, QefSolver& qef, Topology* g) {
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