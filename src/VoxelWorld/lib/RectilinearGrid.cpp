//
// Created by Danielhu on 2018/5/10.
//

#include <set>
#include <Mesh.h>
#include "Indicators.h"
#include "RectilinearGrid.h"

float RectilinearGrid::unitSize = 0.5f;

void RectilinearGrid::setUnitSize(float size) {
  unitSize = size;
}
float RectilinearGrid::getUnitSize() {
  return unitSize;
}

void RectilinearGrid::assignSign(Topology *t) {
  auto min = codeToPos(minCode, RectilinearGrid::getUnitSize());
  auto size = codeToPos(maxCode - minCode, RectilinearGrid::getUnitSize());
  int8_t mtlID = t->getMaterialID();
  for (int i = 0; i < 8; ++i) {
    float val = t->value(min + size * min_offset_subdivision(i));
    cornerSigns[i] = (uint8_t) (val > 0. ? mtlID : 0);
  }
}

void RectilinearGrid::calCornerComponents() {
  std::set<int> clusters[8];
  for (int i = 0; i < 8; ++i) {
    clusters[i].insert({i});
    componentIndices[i] = static_cast<uint8_t>(i);
  }
  for (int i = 0; i < 12; ++i) {
    if (cornerSigns[cellProcFaceMask[i][0]] == cornerSigns[cellProcFaceMask[i][1]]
        && cornerSigns[cellProcFaceMask[i][1]] != 0) {
      for (auto i : clusters[componentIndices[cellProcFaceMask[i][1]]]) {
        clusters[componentIndices[cellProcFaceMask[i][0]]].insert(i);
      }
      componentIndices[cellProcFaceMask[i][1]] = static_cast<uint8_t>(cellProcFaceMask[i][0]);
    }
  }
  int reorderMap[8];
  for (int i = 0; i < 8; ++i) {
    reorderMap[i] = -1;
  }
  int new_order = 0;
  for (int i = 0; i < 8; ++i) {
    if (reorderMap[componentIndices[i]] == -1) {
      reorderMap[componentIndices[i]] = new_order++;
    }
  }
  for (int i = 0; i < 8; ++i) {
    componentIndices[i] = static_cast<uint8_t>(reorderMap[componentIndices[i]]);
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