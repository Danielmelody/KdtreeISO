//
// Created by Danielhu on 2018/5/9.
//

#ifndef VOXELWORLD_RECTILINEARGRID_H
#define VOXELWORLD_RECTILINEARGRID_H

#include <vector>
#include "Qef.h"
#include "Mesh.h"
#include "Topology.h"
#include "Utils.h"
#include "Vertex.h"

struct RectilinearGrid {
  PositionCode minCode;
  PositionCode maxCode;
  QefSolver allQef;
  float error{0.f};
  std::vector<QefSolver> components;
  std::vector<Vertex> vertices;
  uint8_t cornerSigns[8]{0};
  uint8_t componentIndices[8]{0};

  RectilinearGrid(PositionCode minCode, PositionCode maxCode, QefSolver sum = QefSolver())
      : minCode(minCode), maxCode(maxCode), allQef(sum) {
    vertices.resize(1);
    allQef.solve(vertices[0].hermiteP, error);
  }

  void assignSign(Topology *t);
  void calCornerComponents();
  void draw(Mesh *mesh);
  static void setUnitSize(float size);
  static float getUnitSize();
private:
  static float unitSize;
};

#endif //VOXELWORLD_RECTILINEARGRID_H
