//
// Created by Danielhu on 2018/1/16.
//

#ifndef VOXELWORLD_QEF_H
#define VOXELWORLD_QEF_H

#include <glm/glm.hpp>

struct QefSolver {
  glm::mat3x3 ATA;
  glm::fvec3 ATb;
  float btb;
  glm::fvec3 massPointSum;
  glm::fvec3 averageNormalSum;
  float roughness;
  int pointCount;

  void reset();
  void set(const QefSolver &other);
  void combine(const QefSolver &other);
  void separate(const QefSolver &other);
  void add(const glm::fvec3 &p, const glm::fvec3 &n);
  void solve(glm::fvec3 &hermiteP, float &error);
  void calRoughness();
  float getError(const glm::fvec3& p);
  QefSolver()
      : ATA(glm::mat4(0.0)),
        ATb(glm::fvec3(0.0)),
        btb(0.f),
        massPointSum(glm::fvec3(0.f)),
        averageNormalSum(glm::fvec3(0.f)),
        roughness(0.f),
        pointCount(0) {}
};

#endif //VOXELWORLD_QEF_H
