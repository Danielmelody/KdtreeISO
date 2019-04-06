//
// Created by Danielhu on 2018/5/9.
//

#ifndef VOXELWORLD_RECTILINEARGRID_H
#define VOXELWORLD_RECTILINEARGRID_H

#include <vector>
#include <map>
#include <set>
#include "Qef.h"
#include "Mesh.h"
#include "Topology.h"
#include "Utils.h"
#include "Vertex.h"
#include "Indicators.h"
#include "AxisAlignedLine.h"

struct RectilinearGrid {
  PositionCode minCode;
  PositionCode maxCode;
  QefSolver allQef;
  std::vector<QefSolver> components;
  std::vector<Vertex> vertices;
  Vertex approximate;
  uint8_t cornerSigns[8]{0};
  int8_t componentIndices[8]{0};
  bool isSigned = false;
  std::map<RectilinearGrid *, Vertex *> faceVertices;

  explicit RectilinearGrid(PositionCode minCode = PositionCode(0, 0, 0),
                           PositionCode maxCode = PositionCode(0, 0, 0),
                           QefSolver sum = QefSolver())
    : minCode(minCode), maxCode(maxCode), allQef(sum) {
    solve(allQef, approximate);
  }

  void solveComponent(int i);
  void solve(QefSolver &qef, Vertex &v);
  void assignSign(ScalarField *t);
  void calCornerComponents();
  bool sampleQef(ScalarField *t, bool all);
  void draw(Mesh *mesh);
  inline glm::fvec3 cornerPos(int i) {
    return min_offset_subdivision(i) * codeToPos(maxCode - minCode, RectilinearGrid::getUnitSize()) + codeToPos(minCode, RectilinearGrid::getUnitSize());
  }

  inline int edgeComponentIndex(int corner1, int corner2) {
    //    assert(cornerSigns[corner1] != cornerSigns[corner2]);
    if (cornerSigns[corner1] != 0) {
      return componentIndices[corner1];
    }
    return componentIndices[corner2];
  }
  inline int faceComponentIndex(int faceDir, int edgeDir, int faceSide, int edgeSide) {
    int component = -1;
    int dir = 3 - faceDir - edgeDir;
    for (int i = 0; i < 2; ++i) {
      ivec3 code;
      code[faceDir] = faceSide;
      code[edgeDir] = edgeSide;
      code[dir] = i;
      int corner = encodeCell(code);
      if (cornerSigns[corner] > 0) {
        component = componentIndices[corner];
      }
    }
    if (component != -1) {
      return component;
    }
    for (int i = 0; i < 2; ++i) {
      ivec3 code;
      code[faceDir] = faceSide;
      code[edgeDir] = 1 - edgeSide;
      code[dir] = i;
      int corner = encodeCell(code);
      if (cornerSigns[corner] > 0) {
        component = componentIndices[corner];
      }
    }
    return component;
  }
  static void setUnitSize(float size);
  static float getUnitSize();
  static bool calClusterability(RectilinearGrid *left,
                                RectilinearGrid *right,
                                int dir,
                                const PositionCode &minCode,
                                const PositionCode &maxCode,
                                ScalarField *s);
  static void combineAAGrid(RectilinearGrid *left,
                            RectilinearGrid *right,
                            int dir,
                            RectilinearGrid *out);
  static bool isInterFreeCondition2Faild(const std::vector<Vertex *> &polygons,
                                         const glm::fvec3 &p1,
                                         const glm::fvec3 &p2);
  template <class GridHolder>
  static bool checkSign(const std::array<GridHolder *, 4> &nodes,
                        int quadDir1,
                        int quadDir2,
                        ScalarField *s,
                        int &side,
                        PositionCode &minEnd,
                        PositionCode &maxEnd);
  template <class GridHolder>
  static void generateQuad(const std::array<GridHolder, 4> &nodes,
                           int quadDir1,
                           int quadDir2,
                           Mesh *mesh,
                           ScalarField *t,
                           float threshold);

  private:
  static float unitSize;
};

template <class GridHolder>
bool RectilinearGrid::checkSign(const std::array<GridHolder *, 4> &nodes,
                                int quadDir1,
                                int quadDir2,
                                ScalarField *s,
                                int &side,
                                PositionCode &minEnd,
                                PositionCode &maxEnd) {
  int dir = 3 - quadDir1 - quadDir2;
  if (nodes[0] != nodes[1]) {
    maxEnd = minEnd = nodes[0]->grid.maxCode;
  }
  else {
    maxEnd = minEnd = nodes[3]->grid.minCode;
  }
  maxEnd[dir] = std::min(
    std::min(nodes[0]->grid.maxCode[dir], nodes[1]->grid.maxCode[dir]),
    std::min(nodes[2]->grid.maxCode[dir], nodes[3]->grid.maxCode[dir]));
  minEnd[dir] = std::max(
    std::max(nodes[0]->grid.minCode[dir], nodes[1]->grid.minCode[dir]),
    std::max(nodes[2]->grid.minCode[dir], nodes[3]->grid.minCode[dir]));
  if (minEnd[dir] >= maxEnd[dir]) {
    return false;
  }
  float v1 = s->index(minEnd);
  float v2 = s->index(maxEnd);
  if ((v1 >= 0 && v2 >= 0) || (v1 < 0 && v2 < 0)) {
    return false;
  }

  if (v2 >= 0 && v1 <= 0) {
    side = 0;
  }
  else {
    side = 1;
  }

  //  for (int i = 0; i < 4; ++i) {
  //    if (nodes[i] != nodes[oppositeQuadIndex(i)]) {
  //      minEnd[dir] = nodes[i]->grid.minEnd[dir];
  //      maxEnd[dir] = nodes[i]->grid.maxEnd[dir];
  //      v1 = s->index(minEnd);
  //      v2 = s->index(maxEnd);
  //      if ((v1 > 0 && v2 > 0) || (v1 < 0 && v2 < 0)) {
  //        return false;
  //      }
  //    }
  //  }
  return true;
}

template <class GridHolder>
void RectilinearGrid::generateQuad(const std::array<GridHolder, 4> &nodes,
                                   int quadDir1,
                                   int quadDir2,
                                   Mesh *mesh,
                                   ScalarField *t,
                                   float) {
  int edgeSide;
  PositionCode minEnd, maxEnd;
  if (!RectilinearGrid::checkSign(nodes, quadDir1, quadDir2, t, edgeSide, minEnd, maxEnd)) {
    return;
  }
  std::vector<Vertex *> polygons;
  int lineDir = 3 - quadDir1 - quadDir2;
  int componentIndices[4];
  for (int i = 0; i < 4; ++i) {
    if (nodes[i] != nodes[oppositeQuadIndex(i)]) {
      int c1, c2;
      quadIndex(quadDir1, quadDir2, symmetryQuadIndex(i), c1, c2);
      componentIndices[i] = nodes[i]->grid.edgeComponentIndex(c1, c2);
    }
    else {
      componentIndices[i] = nodes[i]->grid.faceComponentIndex(quadDir2, lineDir, 1 - i / 2, edgeSide);
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
  std::set<Vertex *> identicals;
  for (auto v : polygons) {
    identicals.insert(v);
  }
  if (identicals.size() < 3) {
    return;
  }

  bool condition1Failed = false;
  int firstConcaveFaceVertex = 0;
  if (false) {
    int sameCellIndex[2] = {2, 3};
    for (int i = 0; i < 4; ++i) {
      int testDir = (lineDir + i / 2 + 1) % 3;
      int edgeAdjacentCellIndexA = edgeTestNodeOrder[i][0];
      int edgeAdjacentCellIndexB = edgeTestNodeOrder[i][1];
      RectilinearGrid *a = &nodes[edgeAdjacentCellIndexA]->grid;
      RectilinearGrid *b = &nodes[edgeAdjacentCellIndexB]->grid;
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
          fvec3 cornerA = a->cornerPos(subIndexA);
          fvec3 cornerB = b->cornerPos(subIndexB);
          faceMinA = glm::min(cornerA, faceMinA);
          faceMinB = glm::min(cornerB, faceMinB);
          faceMaxA = glm::max(cornerA, faceMaxA);
          faceMaxB = glm::max(cornerB, faceMaxB);
        }
        fvec3 faceMin = glm::max(faceMinA, faceMinB);
        fvec3 faceMax = glm::min(faceMaxA, faceMaxB);
        if (!segmentFaceIntersection(a->vertices[componentIndices[edgeAdjacentCellIndexA]].hermiteP,
                                     b->vertices[componentIndices[edgeAdjacentCellIndexB]].hermiteP,
                                     faceMin,
                                     faceMax,
                                     testDir)) {
          fvec3 minEndDir = faceMin + directionMap(lineDir) * (faceMax - faceMin);
          fvec3 maxEndDir = faceMax - directionMap(lineDir) * (faceMax - faceMin);
          glm::fvec3 points[4] = {faceMin, minEndDir, faceMax, maxEndDir};
          fvec3 massPointSum(0.f);
          int pointCount = 0;
          for (int k = 0; k < 4; ++k) {
            float v1 = t->value(points[k]);
            float v2 = t->value(points[(k + 1) % 4]);
            if ((v1 >= 0 && v2 < 0) || (v1 < 0 && v2 >= 0)) {
              fvec3 x;
              t->solve(points[k], points[(k + 1) % 4], x);
              massPointSum += x;
              pointCount++;
            }
          }
          if (pointCount > 0) {
            firstConcaveFaceVertex = i;
            auto faceV = new Vertex(massPointSum / (float)pointCount);
            mesh->addVertex(faceV, t);
            a->faceVertices[b] = faceV;
            b->faceVertices[a] = faceV;
            condition1Failed = true;
          }
        }
      }
      else {
        sameCellIndex[0] = edgeAdjacentCellIndexA;
        sameCellIndex[1] = edgeAdjacentCellIndexB;
      }
    }

    int minCellIndex = 0;
    for (int i = 0; i < 4; ++i) {
      int edgeAdjacentCellIndexA = edgeTestNodeOrder[i][0];
      int edgeAdjacentCellIndexB = edgeTestNodeOrder[i][1];
      if (edgeAdjacentCellIndexA != sameCellIndex[0] && edgeAdjacentCellIndexA != sameCellIndex[1] && edgeAdjacentCellIndexB != sameCellIndex[0] && edgeAdjacentCellIndexB != sameCellIndex[1]) {
        minCellIndex = edgeAdjacentCellIndexA;
      }
    }
  }

  fvec3 p1 = codeToPos(minEnd, RectilinearGrid::getUnitSize());
  fvec3 p2 = codeToPos(maxEnd, RectilinearGrid::getUnitSize());

  bool condition2Failed = isInterFreeCondition2Faild(polygons, p1, p2);
  if (polygons.size() > 3) {
    std::vector<Vertex *> reversePolygons = {polygons[1], polygons[2], polygons[3], polygons[0]};
    bool reverseCondition2Failed = isInterFreeCondition2Faild(reversePolygons, p1, p2);
    if (!reverseCondition2Failed) {
      /// NOTE: the swap here happens whether intersection-free or not
      polygons.swap(reversePolygons);
    }
    condition2Failed = condition2Failed && reverseCondition2Failed;
  }
#ifdef INTERSECTION_FREE
  if (condition1Failed || condition2Failed) {
    GridHolder circle[4] = {nodes[0], nodes[1], nodes[3], nodes[2]};
    polygons.clear();
    if (!condition2Failed) {
      std::vector<int> concaveFlags;
      std::vector<Vertex *> convexPart;
      int concaveCount = 0;
      for (int i = 0; i < 4; ++i) {
        int index = (i + firstConcaveFaceVertex) % 4;
        auto faceIter = circle[index]->grid.faceVertices.find(&circle[(index + 1) % 4]->grid);
        auto cellVertex = &(circle[(index + 1) % 4]->grid.vertices[componentIndices[(index + 1) % 4]]);
        if (faceIter != circle[index]->grid.faceVertices.end()) {
          polygons.push_back(faceIter->second);
          concaveFlags.push_back(1);
          convexPart.push_back(faceIter->second);
          concaveCount++;
        }
        polygons.push_back(cellVertex);
        concaveFlags.push_back(0);
      }
      for (int i = 0; i < polygons.size() - 2; ++i) {
        Vertex *triangle[3] = {
          polygons[0], polygons[i + 1], polygons[i + 2]};
        mesh->addTriangle(triangle, t);
      }
    }
    else {
      Vertex edgeVertex;
      t->solve(p1, p2, edgeVertex.hermiteP);
      mesh->addVertex(&edgeVertex, t);
      for (int i = 0; i < 4; ++i) {
        RectilinearGrid *a = &circle[i]->grid;
        RectilinearGrid *b = &circle[(i + 1) % 4]->grid;
        if (a != b) {
          polygons.push_back(&a->vertices[componentIndices[i]]);
          auto faceVIter = a->faceVertices.find(b);
          if (faceVIter != a->faceVertices.end()) {
            polygons.push_back(faceVIter->second);
            polygons.push_back(faceVIter->second);
          }
          polygons.push_back(&b->vertices[componentIndices[(i + 1) % 4]]);
        }
      }
      for (int i = 0; i < polygons.size() / 2; ++i) {
        Vertex *triangle[3] = {
          &edgeVertex, polygons[i * 2], polygons[i * 2 + 1]};
        mesh->addTriangle(triangle, t);
      }
    }
  }
  else {
#endif
    for (int i = 2; i < polygons.size(); ++i) {
      Vertex *triangle[3] = {
        polygons[0], polygons[i - 1], polygons[i]};
      mesh->addTriangle(triangle, t);
    }
#ifdef INTERSECTION_FREE
  }
#endif
}

#endif //VOXELWORLD_RECTILINEARGRID_H
