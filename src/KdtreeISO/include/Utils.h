//
// Created by Danielhu on 2018/4/2.
//

#ifndef VOXELWORLD_UTILS_H
#define VOXELWORLD_UTILS_H

#include "cmath"
#include <glm/glm.hpp>

using glm::fvec3;

using PositionCode = glm::tvec3<int>;

inline bool segmentFaceIntersection(const fvec3 &va, const fvec3 &vb, const fvec3 &min, const fvec3 &max, int dir) {
  float l = (vb - va)[dir];
  fvec3 p = (min - va)[dir] / l * vb + (vb - min)[dir] / l * va;
  for (int i = 0; i < 3; ++i) {
    if (dir != i) {
      if (p[i] < min[i] || p[i] > max[i]) {
        return false;
      }
    }
  }
  return true;
}

template <class T>
inline void hash_combine(std::size_t &seed, const T &v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template <typename C>
struct ContainerHasher {
  typedef typename C::value_type value_type;
  inline size_t operator()(const C &c) const {
    size_t seed = 0;
    for (typename C::const_iterator it = c.begin(), end = c.end(); it != end; ++it) {
      hash_combine<value_type>(seed, *it);
    }
    return seed;
  }
};

inline glm::fvec3 codeToPos(PositionCode code, float cellSize) {
  return fvec3((float)code.x * cellSize, (float)code.y * cellSize, (float)code.z * cellSize);
}

inline PositionCode posToCode(glm::fvec3 pos, float cellSize) {
  return PositionCode(std::round(pos.x / cellSize), std::round(pos.y / cellSize), std::round(pos.z / cellSize));
}

inline PositionCode posToCodeFloor(glm::fvec3 pos, float cellSize) {
  return PositionCode(pos.x / cellSize, pos.y / cellSize, pos.z / cellSize);
}

#endif //VOXELWORLD_UTILS_H
