//
// Created by Danielhu on 2018/1/16.
//

#ifndef VOXELWORLD_QEF_H
#define VOXELWORLD_QEF_H

#include <glm/glm.hpp>

struct QefSolver {
  glm::mat3x3 ATA;
  glm::vec3 ATb;
  float btb;
  glm::vec3 massPointSum;
  int pointCount;

  void set(const QefSolver& other);
  void combine(const QefSolver& other);
  void add(const glm::vec3& p, const glm::vec3& n);
  void solve(glm::vec3 &hermiteP, float& error);
  QefSolver():ATA(glm::mat4(0.0)), ATb(glm::vec3(0.0)), btb(0.f), massPointSum(glm::vec3(0.f)), pointCount(0) {}
};

#endif //VOXELWORLD_QEF_H
