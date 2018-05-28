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
#include "Indicators.h"

struct RectilinearGrid {
  PositionCode minCode;
  PositionCode maxCode;
  QefSolver allQef;
  float error{0.f};
  std::vector<QefSolver> components;
  std::vector<Vertex> vertices;
  std::vector<float> errors;
  glm::vec3 approximate;
  uint8_t cornerSigns[8]{0};
  int8_t componentIndices[8]{0};

  RectilinearGrid(PositionCode minCode, PositionCode maxCode, QefSolver sum = QefSolver())
      : minCode(minCode), maxCode(maxCode), allQef(sum) {
    allQef.solve(approximate, error);
  }

  void solveComponent(int i);
  void assignSign(Topology *t);
  void calCornerComponents();
  void sampleQef(Topology *t);
  void draw(Mesh *mesh);
  inline int edgeComponentIndex(int corner1, int corner2) {
    if (cornerSigns[corner1] == cornerSigns[corner2]) {
      return -1;
    }
    if (cornerSigns[corner1] != 0) {
      return componentIndices[corner1];
    }
    return componentIndices[corner2];
  }
  inline int faceComponentIndex(int dir, int side) {
    int signChanges = 0;
    int component = -1;
    for (int i = 0; i < 4; ++i) {
      ivec3 code;
      code[dir] = side;
      code[(dir + 1) % 3] = i / 2;
      code[(dir + 2) % 3] = i % 2;
      int corner = encodeCell(code);
      if (cornerSigns[corner] > 0) {
        if (component != componentIndices[corner]) {
          signChanges++;
        }
        component = componentIndices[corner];
      }
    }
    // assert(signChanges == 1);
    if (signChanges > 1) {
      return -1;
    }
    return component;
  }
  static void setUnitSize(float size);
  static float getUnitSize();
  template<class GridHolder>
  static void generateQuad(const std::array<GridHolder, 4> &nodes,
                           int quadDir1,
                           int quadDir2,
                           Mesh *mesh,
                           Topology *t);
private:
  static float unitSize;
};

template<class GridHolder>
void RectilinearGrid::generateQuad(const std::array<GridHolder, 4> &nodes,
                                   int quadDir1,
                                   int quadDir2,
                                   Mesh *mesh,
                                   Topology *t) {
  for (auto &n : nodes) {
    for (int i = 0; i < n->grid.vertices.size(); ++i) {
      // assert(n->grid.vertices[i].hermiteP[2] < 0.3);
    }
  }

  std::vector<Vertex *> polygons;

  int componentIndices[4];
  for (int i = 0; i < 4; ++i) {
    if (nodes[i] != nodes[oppositeQuadIndex(i)]) {
      int c1, c2;
      quadIndex(quadDir1, quadDir2, symmetryQuadIndex(i), c1, c2);
      componentIndices[i] = nodes[i]->grid.edgeComponentIndex(c1, c2);
    } else {
      componentIndices[i] = nodes[i]->grid.faceComponentIndex(quadDir2, 1 - i / 2);
    }
    if (componentIndices[i] == -1) {
      return;
    }
  }

  polygons.push_back(&nodes[0]->grid.vertices.at(componentIndices[0]));
  if (nodes[0] != nodes[1]) {
    polygons.push_back(&nodes[1]->grid.vertices.at(componentIndices[1]));
  }
  polygons.push_back(&nodes[3]->grid.vertices.at(componentIndices[3]));
  if (nodes[2] != nodes[3]) {
    polygons.push_back(&nodes[2]->grid.vertices.at(componentIndices[2]));
  }
  for (int i = 0; i < polygons.size() - 2; ++i) {
    Vertex *triangles[] = {polygons[0], polygons[i + 1], polygons[i + 2]};
    mesh->addTriangle(triangles, t);
  }
}

#endif //VOXELWORLD_RECTILINEARGRID_H
