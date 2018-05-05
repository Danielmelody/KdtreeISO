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
  virtual bool solve(const glm::vec3 &p1, const glm::vec3 &p2, glm::vec3 &out);
  void normal(const glm::vec3 &p, glm::vec3 &out);
  void gradient(const glm::vec3 &p, glm::vec3& out);
  glm::vec3 gradient(const glm::vec3 &p);
  float laplaceOperator(const glm::vec3 &p);
  uint8_t getMaterialID();
  Topology() : materialID(1) {}
  virtual ~Topology() = default;
};

class Union : public Topology {
  Topology *l;
  Topology *r;
public:
  Union(Topology *l, Topology *r) : l(l), r(r) {}
  float value(const glm::vec3 &p) override { return std::min(l->value(p), r->value(p)); }
  ~Union() override {
    delete l;
    delete r;
  }
};

class ExpUnion : public Topology {
  Topology *l;
  Topology *r;
  float k;
public:
  ExpUnion(Topology *l, Topology *r, float k = 32.f) : l(l), r(r), k(k) {}
  float value(const glm::vec3 &p) override {
    float res = exp(-k * l->value(p)) + exp(-k * r->value(p));
    return -log(std::max(0.0001f, res)) / k;
  }
  ~ExpUnion() override {
    delete l;
    delete r;
  }
};

class Difference : public Topology {
  Topology *l;
  Topology *r;
public:
  Difference(Topology *l, Topology *r) : l(l), r(r) {}
  float value(const glm::vec3 &p) override { return std::max(l->value(p), -r->value(p)); }
  ~Difference() override {
    delete l;
    delete r;
  }
};

class Intersection : public Topology {
  Topology *l;
  Topology *r;
public:
  Intersection(Topology *l, Topology *r) : l(l), r(r) {}
  float value(const glm::vec3 &p) override { return std::max(l->value(p), r->value(p)); }
  ~Intersection() override {
    delete l;
    delete r;
  }
};

class Transform : public Topology {
  glm::mat4 trans;
  Topology *inner;
public:
  Transform(const glm::mat4 &trans, Topology *inner) : trans(trans), inner(inner) {}
  ~Transform() override { delete inner; }
  float value(const glm::vec3 &root) override;
};

class Sphere : public Topology {
  float radius;
  glm::vec3 center;
public:
  explicit Sphere(float radius, glm::vec3 center = glm::vec3(0)) : radius(radius), center(center) {}
  ~Sphere() override {}
  float value(const glm::vec3 &p) override;
  bool solve(const glm::vec3 &p1, const glm::vec3 &p2, glm::vec3 &out) override;
};

class AABB : public Topology {
  glm::vec3 min_;
  glm::vec3 max_;
public:
  float value(const glm::vec3 &p) override;
  AABB(glm::vec3 min_, glm::vec3 max_) : min_(min_), max_(max_) {};
  ~AABB() override {}
};

class Torus : public Topology {
  float r1;
  float r2;
public:
  Torus(float r1, float r2) : r1(r1), r2(r2) {}
  float value(const glm::vec3 &p) override {
    glm::vec2 q = glm::vec2(glm::length(glm::vec2(p.x, p.z)) - r1, p.y);
    return length(q) - r2;
  }
};

class Cylinder : public Topology {
  glm::vec3 c;
public:
  explicit Cylinder(const glm::vec3 &c) : c(c) {}
  float value(const glm::vec3 &p) override { return glm::length(glm::vec2(p.x, p.z) - glm::vec2(c.x, c.y)) - c.z; }
};

class Capsule : public Topology {
  glm::vec3 a;
  glm::vec3 b;
  float r;
public:
  Capsule(const glm::vec3 &a, const glm::vec3 &b, float r) : a(a), b(b), r(r) {}
  float value(const glm::vec3 &p) override {
    glm::vec3 pa = p - a, ba = b - a;
    float h = glm::clamp(dot(pa, ba) / dot(ba, ba), 0.f, 1.f);
    return length(pa - ba * h) - r;
  }
};

class Heart : public Topology {
  float scale;
  glm::vec3 center;
public:
  explicit Heart(float scale, glm::vec3 center = glm::vec3(0)) : scale(scale), center(center) {}
  ~Heart() override {}
  float value(const glm::vec3 &p) override;
};

#endif //VOXELWORLD_GENERATORS_H
