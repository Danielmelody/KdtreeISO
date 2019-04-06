//
// Created by Danielhu on 2018/4/2.
//

#ifndef VOXELWORLD_UTILS_H
#define VOXELWORLD_UTILS_H

#define GLM_FORCE_EXPLICIT_CTOR

#include <glm/glm.hpp>
#include <cmath>
#include <iostream>

using glm::fvec3;

using PositionCode = glm::ivec3;

// for strange reason the binary operators not work in darwin 10.14 clang debug mode
#define CodeBinaryOp(a, b, op) \
  PositionCode(a.x op b.x, a.y op b.y, a.z op b.z)

#ifndef NDEBUG
#define LOGV(v) \
  std::cout << #v << " " << v.x << " " << v.y << " " << v.z << " " << std::endl;

#define LOGF(v) \
  std::cout << #v << " " << v << std::endl;

#define LOGV4(v) \
  std::cout << #v << " " << v.x << " " << v.y << " " << v.z << " " << v.w << std::endl;

#else
#define LOGV(v)
#define LOGV4(v)
#define LOGF(v)
#endif

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

inline fvec3 codeToPos(const PositionCode &code, float cellSize) {
  // LOGV(code)
  auto result = fvec3(static_cast<float>(code.x) * cellSize, static_cast<float>(code.y) * cellSize, static_cast<float>(code.z) * cellSize);
  return result;
}

inline PositionCode posToCode(const glm::fvec3 &pos, float cellSize) {
  return PositionCode(std::round(pos.x / cellSize), std::round(pos.y / cellSize), std::round(pos.z / cellSize));
}

inline PositionCode posToCodeFloor(const glm::fvec3 &pos, float cellSize) {
  return PositionCode(pos.x / cellSize, pos.y / cellSize, pos.z / cellSize);
}

#endif //VOXELWORLD_UTILS_H
