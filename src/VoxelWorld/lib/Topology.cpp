//
// Created by Danielhu on 2018/1/13.
//
#include <algorithm>
#include "Topology.h"

using namespace glm;

float gradient_offset = 0.003f;
float divergence_offset = 0.01f;

bool Topology::solve(const glm::vec3 &p1, const glm::vec3 &p2, glm::vec3 &out) {
  auto offset = p2 - p1;
  float min = 0.f;
  float max = 1.f;
  float mid = (min + max) / 2.f;

  for (int i = 0; i < 16; ++i) {
    float l = value(p1 + offset * min);
    mid = (min + max) / 2.f;
    float midsign = value(p1 + offset * mid);
    if (l * midsign < 0.) {
      max = mid;
    } else {
      min = mid;
    }
  }
  out = p1 + offset * mid;
  return true;
}

void Topology::normal(const glm::vec3 &p, glm::vec3& out) {

  float nx = value(p + vec3(gradient_offset, 0.f, 0.f)) - value(p - vec3(gradient_offset, 0.f, 0.f));
  float ny = value(p + vec3(0.f, gradient_offset, 0.f)) - value(p - vec3(0.f, gradient_offset, 0.f));
  float nz = value(p + vec3(0.f, 0.f, gradient_offset)) - value(p - vec3(0.f, 0.f, gradient_offset));

  out = normalize(vec3(nx, ny, nz));
}

glm::vec3 Topology::gradient(const glm::vec3 &p) {

  float nx = value(p + vec3(gradient_offset, 0.f, 0.f)) - value(p - vec3(gradient_offset, 0.f, 0.f));
  float ny = value(p + vec3(0.f, gradient_offset, 0.f)) - value(p - vec3(0.f, gradient_offset, 0.f));
  float nz = value(p + vec3(0.f, 0.f, gradient_offset)) - value(p - vec3(0.f, 0.f, gradient_offset));

  return vec3(nx, ny, nz) / gradient_offset;
}

float Topology::laplaceOperator(const glm::vec3 p) {
  float lx = gradient(p + vec3(divergence_offset, 0.f, 0.f)).x - gradient(p - vec3(divergence_offset, 0.f, 0.f)).x;
  float ly = gradient(p + vec3(0.f, divergence_offset, 0.f)).y - gradient(p - vec3(0.f, divergence_offset, 0.f)).y;
  float lz = gradient(p + vec3(0.f, 0.f, divergence_offset)).z - gradient(p - vec3(0.f, 0.f, divergence_offset)).z;
  return (lx + ly + lz) / divergence_offset;
}

uint8_t Topology::getMaterialID() {
  return materialID;
}

float Sphere::value(const glm::vec3 &p) {
  return glm::length(p - center) - radius;
}

bool Sphere::solve(const glm::vec3 &p1, const glm::vec3 &p2, glm::vec3 &out){
  vec3 p1p2 = p1 - p2;
  float a = dot(p1p2, p1p2);
  float b = 2 * dot(p2 - center, p1 - p2);
  vec3 p2c = p2 - center;
  float c = dot(p2c, p2c) - dot(radius, radius);
  float delta = b * b - 4.f * a * c;
  if (delta < 0) {
    return false;
  }
  float sqrt_delta = sqrt(delta);
  float t1 = (-b + sqrt_delta) / (2 * a);
  float t2 = (-b - sqrt_delta) / (2 * a);
  if (t1 >= 0.f && t1 <= 1.f) {
    out = t1 * p1p2 + p2;
    return true;
  }
  if(t2 >= 0.f && t2 <= 1.f) {
    out = t2 * p1p2 + p2;
    return true;
  }
  return false;
}

float AABB::value(const glm::vec3 &p) {
  vec3 offset = glm::abs(p - (min_ + max_) / 2.f);
  offset -= (max_ - min_) / 2.f;
  return min(length(offset), max(offset.x, max(offset.y, offset.z)));
}

float Heart::value(const glm::vec3 &p) {
  glm::vec3 offset = (p - center) / scale;

  float x = offset.x, y = offset.z, z = offset.y;
  float a = x * x + 9.0f / 4.0f * y * y + z * z - 1;
  return a * a * a - x * x * z * z * z - 9.0f / 80.0f * y * y * z * z * z;
}
