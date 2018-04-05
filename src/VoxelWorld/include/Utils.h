//
// Created by Danielhu on 2018/4/2.
//

#ifndef VOXELWORLD_UTILS_H
#define VOXELWORLD_UTILS_H

#include <glm/glm.hpp>

using namespace glm;

bool segmentFaceIntersection(const vec3 &va, const vec3 &vb, const vec3 &min, const vec3& max, int dir) {
  float l = (vb - va)[dir];
  vec3 p = (min - va)[dir] / l * vb + (vb - min)[dir] / l * va;
  for (int i = 0; i < 3; ++i) {
    if (dir != i) {
      if (p[i] < min[i] || p[i] > max[i]) {
        return false;
      }
    }
  }
  return true;
}

#endif //VOXELWORLD_UTILS_H
