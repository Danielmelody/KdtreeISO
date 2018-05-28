//
// Created by Danielhu on 2018/5/10.
//

#include <set>
#include <Mesh.h>
#include <map>
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
  assert(components[i].pointCount);
  components[i].solve(vertices[i].hermiteP, errors[i]);
  auto &p = vertices[i].hermiteP;
  const auto min = codeToPos(minCode, RectilinearGrid::getUnitSize()) - fvec3(0.01);
  const auto max = codeToPos(maxCode, RectilinearGrid::getUnitSize()) + fvec3(0.01);
  if (p.x < min.x || p.x > max.x ||
      p.y < min.y || p.y > max.y ||
      p.z < min.z || p.z > max.z) {
//    vertices[i].hermiteP = components[i].massPointSum / (float) components[i].pointCount;
//    if (p.x < min.x || p.x > max.x ||
//        p.y < min.y || p.y > max.y ||
//        p.z < min.z || p.z > max.z) {
      p = glm::min(p, max);
      p = glm::max(p, min);
//    }
    errors[i] = components[i].getError(p);
  }
}

void RectilinearGrid::assignSign(Topology *t) {
  auto min = codeToPos(minCode, RectilinearGrid::getUnitSize());
  auto size = codeToPos(maxCode - minCode, RectilinearGrid::getUnitSize());
  int8_t mtlID = t->getMaterialID();
  for (int i = 0; i < 8; ++i) {
    float val = t->value(min + size * min_offset_subdivision(i));
    cornerSigns[i] = (uint8_t) (val > 0. ? 0 : mtlID);
  }
}

void RectilinearGrid::calCornerComponents() {
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
    if (cornerSigns[c1] == cornerSigns[c2]
        && cornerSigns[c2] != 0) {
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
  errors.resize(static_cast<unsigned long>(new_order));
}

void RectilinearGrid::sampleQef(Topology *t) {
  calCornerComponents();
  auto min = codeToPos(minCode, RectilinearGrid::getUnitSize());
  auto size = codeToPos(maxCode - minCode, RectilinearGrid::getUnitSize());
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
      }
    }
  }
  for (int i = 0; i < components.size(); ++i) {
    assert(components[i].pointCount > 0 && components[i].pointCount < 12);
    t->normal(vertices[i].hermiteP, vertices[i].hermiteN);
  }
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
