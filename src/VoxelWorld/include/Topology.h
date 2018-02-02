//
// Created by Danielhu on 2018/1/13.
//

#ifndef VOXELWORLD_GENERATORS_H
#define VOXELWORLD_GENERATORS_H

#include <algorithm>
#include <glm/glm.hpp>

class Topology {
  uint8_t materialID;
public:
  virtual float value(const glm::vec3 &p) = 0;
  virtual bool solve(const glm::vec3& p1, const glm::vec3& p2, glm::vec3& out);
  void normal(const glm::vec3 &p, glm::vec3& out);
  uint8_t getMaterialID();
  Topology():materialID(1){}
  virtual ~Topology() {}
};

class Union : public Topology {
  Topology* l;
  Topology* r;
public:
  Union(Topology* l, Topology* r):l(l), r(r) {}
  float value(const glm::vec3 &p) { return std::min(l->value(p), r->value(p));}
  ~Union() {}
};

class Difference : public Topology {
  Topology* l;
  Topology* r;
public:
  Difference(Topology* l, Topology* r): l(l), r(r) {}
  float value(const glm::vec3 &p) { return std::max(l->value(p), -r->value(p));}
  ~Difference() {}
};

class Intersection : public Topology {
  Topology* l;
  Topology* r;
public:
  Intersection(Topology* l, Topology* r):l(l), r(r) {}
  float value(const glm::vec3 &p) { return std::max(l->value(p), r->value(p));}
  ~Intersection() {}
};

class Sphere : public Topology {
  float radius;
  glm::vec3 center;
public:
  Sphere(float radius, glm::vec3 center):radius(radius), center(center) {}
  ~Sphere() {}
  float value(const glm::vec3 &p);
  bool solve(const glm::vec3& p1, const glm::vec3& p2, glm::vec3& out);
};

class AABB : public Topology {
  glm::vec3 min_;
  glm::vec3 max_;
public:
  float value(const glm::vec3 &p);
  AABB(glm::vec3 min_, glm::vec3 max_):min_(min_), max_(max_) {};
  ~AABB() {}
};

class Heart: public Topology {
  float scale;
  glm::vec3 center;
public:
  Heart(float scale, glm::vec3 center):scale(scale), center(center) {}
  ~Heart() {}
  float value(const glm::vec3 &p);
};

#endif //VOXELWORLD_GENERATORS_H
