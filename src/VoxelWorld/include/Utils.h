//
// Created by Danielhu on 2018/4/2.
//

#ifndef VOXELWORLD_UTILS_H
#define VOXELWORLD_UTILS_H

#include <glm/glm.hpp>

using glm::vec3;

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

template <class T>
inline void hash_combine(std::size_t & seed, const T & v)
{
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template <typename C> struct ContainerHasher
{
  typedef typename C::value_type value_type;
  inline size_t operator()(const C & c) const
  {
    size_t seed = 0;
    for (typename C::const_iterator it = c.begin(), end = c.end(); it != end; ++it)
    {
      hash_combine<value_type>(seed, *it);
    }
    return seed;
  }
};

#endif //VOXELWORLD_UTILS_H
