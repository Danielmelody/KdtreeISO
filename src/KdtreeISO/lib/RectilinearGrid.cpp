//
// Created by Danielhu on 2018/5/10.
//

#define GLM_ENABLE_EXPERIMENTAL
#define GLM_FORCE_CTOR_INIT
#define GLM_FORCE_EXPLICIT_CTOR

#include <set>
#include <Mesh.h>
#include <map>
#include <glm/ext.hpp>
#include <glm/gtx/intersect.hpp>
#include "Indicators.h"
#include "RectilinearGrid.h"

float RectilinearGrid::unitSize = 0.5f;

void RectilinearGrid::setUnitSize(float size) {
  unitSize = size;
}
float RectilinearGrid::getUnitSize() {
  return unitSize;
}

void RectilinearGrid::solveComponent(int i) {
  solve(components[i], vertices[i]);
}

void RectilinearGrid::solve(QefSolver &qef, Vertex &v) {
  auto &p = v.hermiteP;
  qef.solve(p, v.error);
  auto extends = codeToPos(maxCode - minCode, RectilinearGrid::getUnitSize()) * 0.5f;
  const auto min = codeToPos(minCode, RectilinearGrid::getUnitSize()) - extends;
  const auto max = codeToPos(maxCode, RectilinearGrid::getUnitSize()) + extends;
  if (p.x < min.x || p.x > max.x ||
      p.y < min.y || p.y > max.y ||
      p.z < min.z || p.z > max.z) {
    p = qef.massPointSum / (float)qef.pointCount;
  }
}

void RectilinearGrid::assignSign(ScalarField *t) {
  auto sizeCode = PositionCode(
    maxCode.x - minCode.x,
    maxCode.y - minCode.y,
    maxCode.z - minCode.z);
  int8_t mtlID = t->getMaterialID();
  for (int i = 0; i < 8; ++i) {
    PositionCode code = decodeCell(i);
    float val = t->index(minCode + sizeCode * code);
    cornerSigns[i] = (uint8_t)(val >= 0. ? 0 : mtlID);
  }
  isSigned = !((cornerSigns[0] == cornerSigns[1]) &&
               (cornerSigns[1] == cornerSigns[2]) &&
               (cornerSigns[2] == cornerSigns[3]) &&
               (cornerSigns[3] == cornerSigns[4]) &&
               (cornerSigns[4] == cornerSigns[5]) &&
               (cornerSigns[5] == cornerSigns[6]) &&
               (cornerSigns[6] == cornerSigns[7]));
}

void RectilinearGrid::calCornerComponents() {
  assert(components.empty());
  std::set<int> clusters[8];
  for (int i = 0; i < 8; ++i) {
    if (cornerSigns[i] != 0) {
      clusters[i].insert({i});
      componentIndices[i] = static_cast<uint8_t>(i);
    }
  }
  for (int i = 0; i < 12; ++i) {
    int c1 = cellProcFaceMask[i][0];
    int c2 = cellProcFaceMask[i][1];
    if (cornerSigns[c1] == cornerSigns[c2] && cornerSigns[c2] != 0) {
      int co1 = componentIndices[c1];
      int co2 = componentIndices[c2];
      auto &c2Components = clusters[co2];
      for (auto comp : c2Components) {
        clusters[co1].insert(comp);
      }
      for (auto comp : clusters[co1]) {
        componentIndices[comp] = static_cast<uint8_t>(co1);
      }
    }
  }
  int reorderMap[8]{0};
  for (int i = 0; i < 8; ++i) {
    reorderMap[i] = -1;
  }
  int new_order = 0;
  for (int i = 0; i < 8; ++i) {
    if (reorderMap[componentIndices[i]] == -1 && cornerSigns[i] != 0) {
      reorderMap[componentIndices[i]] = new_order++;
    }
  }
  for (int i = 0; i < 8; ++i) {
    componentIndices[i] = static_cast<uint8_t>(reorderMap[componentIndices[i]]);
  }
  vertices.resize(static_cast<unsigned long>(new_order));
  components.resize(static_cast<unsigned long>(new_order));
}

bool RectilinearGrid::sampleQef(ScalarField *t, bool all) {
  calCornerComponents();
  const auto min = codeToPos(minCode, RectilinearGrid::getUnitSize());

  // auto minX = codeToPos(minCode, RectilinearGrid::getUnitSize()).x;
  // assert(!isinf(minX));
  auto isize = maxCode - minCode;
  auto size = codeToPos(isize, RectilinearGrid::getUnitSize());
  assert(!isnan(size.x));
  // size = codeToPos(isize, RectilinearGrid::getUnitSize());

  fvec3 cornerPositions[8];
  for (int i = 0; i < 8; ++i) {
    cornerPositions[i] = min + size * min_offset_subdivision(i);
  }
  for (int i = 0; i < 12; ++i) {
    fvec3 p1 = cornerPositions[edge_map[i][0]];
    fvec3 p2 = cornerPositions[edge_map[i][1]];
    if (cornerSigns[edge_map[i][0]] != cornerSigns[edge_map[i][1]]) {
      fvec3 p, n;
      if (t->solve(p1, p2, p)) {
        t->normal(p, n);
        int qefIndex = edgeComponentIndex(edge_map[i][0], edge_map[i][1]);
        components.at(static_cast<unsigned long>(qefIndex)).add(p, n);
        if (all) {
          allQef.add(p, n);
        }
      }
    }
  }
  for (int i = 0; i < components.size(); ++i) {
    if (components[i].pointCount == 0 || components[i].pointCount >= 12) {
      return false;
    }
    t->normal(vertices[i].hermiteP, vertices[i].hermiteN);
  }
  return allQef.pointCount > 0;
}

void RectilinearGrid::draw(Mesh *mesh) {
  fvec3 size = codeToPos(maxCode - minCode, RectilinearGrid::getUnitSize());
  fvec3 min = codeToPos(minCode, RectilinearGrid::getUnitSize());
  for (int i = 0; i < 12; ++i) {
    auto a = min_offset_subdivision(cellProcFaceMask[i][0]) * size + min;
    auto b = min_offset_subdivision(cellProcFaceMask[i][1]) * size + min;
    auto na = normalize(min_offset_subdivision(cellProcFaceMask[i][0]) - fvec3(0.5f));
    auto nb = normalize(min_offset_subdivision(cellProcFaceMask[i][1]) - fvec3(0.5f));
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
}
bool RectilinearGrid::isInterFreeCondition2Faild(const std::vector<Vertex *> &polygons,
                                                 const glm::fvec3 &p1,
                                                 const glm::fvec3 &p2) {
  int anotherV = 3;
  bool interSupportingEdge = false;

  for (int i = 2; i < polygons.size(); ++i) {
    fvec2 baryPos;
    float distance;
    bool isInter = glm::intersectRayTriangle(p1,
                                             p2 - p1,
                                             polygons[0]->hermiteP,
                                             polygons[i - 1]->hermiteP,
                                             polygons[i]->hermiteP,
                                             baryPos,
                                             distance);
    isInter = isInter && (distance > 0.f && distance < 1.f);
    if (isInter) {
      interSupportingEdge = true;
      anotherV = i % 3 + 1;
    }
  }
  if (polygons.size() == 3) {
    return !interSupportingEdge;
  }
  else {
    fvec2 baryPos;
    float distance;
    bool interTetrahedron = glm::intersectRayTriangle(polygons[0]->hermiteP,
                                                      polygons[2]->hermiteP - polygons[0]->hermiteP,
                                                      p1,
                                                      p2,
                                                      polygons[anotherV]->hermiteP,
                                                      baryPos,
                                                      distance);
    interTetrahedron = interTetrahedron && (distance > 0.f && distance < 1.f);
    return !(interTetrahedron && interSupportingEdge);
  }
}
bool RectilinearGrid::calClusterability(RectilinearGrid *left,
                                        RectilinearGrid *right,
                                        int dir,
                                        const PositionCode &minCode,
                                        const PositionCode &maxCode,
                                        ScalarField *s) {
  if (!left && !right) {
    return true;
  }
  int clusterCornerSigns[8];
  for (int i = 0; i < 8; ++i) {
    clusterCornerSigns[i] = s->index(minCode + (maxCode - minCode) * decodeCell(i)) >= 0 ? 0 : s->getMaterialID();
  }
  bool homogeneous = true;
  for (int i = 1; i < 8; ++i) {
    if (clusterCornerSigns[i] != clusterCornerSigns[0]) {
      homogeneous = false;
    }
  }
  if (homogeneous) {
    return false;
  }

  if (!(left && right)) {
    return true;
  }

  RectilinearGrid *params[2] = {left, right};

  for (int i = 0; i < 4; ++i) {
    int edgeMinIndex = cellProcFaceMask[dir * 4 + i][0];
    int edgeMaxIndex = cellProcFaceMask[dir * 4 + i][1];
    int signChanges = 0;
    for (int j = 0; j < 2; ++j) {
      if (params[j]->cornerSigns[edgeMinIndex] != params[j]->cornerSigns[edgeMaxIndex]) {
        signChanges++;
      }
    }
    if (signChanges > 1) {
      return false;
    }
  }
  return true;
}
void RectilinearGrid::combineAAGrid(RectilinearGrid *left,
                                    RectilinearGrid *right,
                                    int dir,
                                    RectilinearGrid *out) {
  out->calCornerComponents();
  if (!left && !right) {
    return;
  }
  std::map<int, int> combineMaps[2];
  RectilinearGrid *grids[2] = {left, right};
  for (int i = 0; i < 4; ++i) {
    int c = -1;
    for (int j = 0; j < 2; ++j) {
      if (out->cornerSigns[cellProcFaceMask[dir * 4 + i][j]] != 0) {
        c = out->componentIndices[cellProcFaceMask[dir * 4 + i][j]];
        break;
      }
    }
    if (c == -1) {
      continue;
    }
    for (int j = 0; j < 2; ++j) {
      auto child = grids[j];
      if (child) {
        for (int k = 0; k < 2; ++k) {
          if (child->cornerSigns[cellProcFaceMask[dir * 4 + i][k]] != 0) {
            int childC = child->componentIndices[cellProcFaceMask[dir * 4 + i][k]];
            assert(child->components[childC].pointCount > 0);
            combineMaps[j][c] = childC;
            break;
          }
        }
      }
    }
  }
  for (int i = 0; i < 2; ++i) {
    for (auto p : combineMaps[i]) {
      out->components.at(p.first).combine(grids[i]->components.at(p.second));
      grids[i]->vertices.at(p.second).parent = &out->vertices.at(p.first);
    }
  }
  int count = 0;
  for (auto c : out->components) {
    count += c.pointCount;
  }
}
