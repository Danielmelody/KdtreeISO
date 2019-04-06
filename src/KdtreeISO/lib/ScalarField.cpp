//
// Created by Danielhu on 2018/5/30.
//

#include <glm/glm.hpp>

#include "RectilinearGrid.h"
#include "ScalarField.h"

fvec3 ScalarField::normal_f1(const fvec3 &p) {
  float nx = value(p + fvec3(gradientOffset(), 0.f, 0.f)) - value(p - fvec3(gradientOffset(), 0.f, 0.f));
  float ny = value(p + fvec3(0.f, gradientOffset(), 0.f)) - value(p - fvec3(0.f, gradientOffset(), 0.f));
  float nz = value(p + fvec3(0.f, 0.f, gradientOffset())) - value(p - fvec3(0.f, 0.f, gradientOffset()));

  // auto g = fvec3(nx, ny, nz) / gradientOffset() / 2.f;
  if (nx == 0.f && ny == 0.f && nz == 0.f) {
    return glm::normalize(fvec3(1));
  }
  return glm::normalize(fvec3(nx, ny, nz));
}

void ScalarField::normal(const fvec3 &p, fvec3 &out) {
  float nx = value(p + fvec3(gradientOffset(), 0.f, 0.f)) - value(p - fvec3(gradientOffset(), 0.f, 0.f));
  float ny = value(p + fvec3(0.f, gradientOffset(), 0.f)) - value(p - fvec3(0.f, gradientOffset(), 0.f));
  float nz = value(p + fvec3(0.f, 0.f, gradientOffset())) - value(p - fvec3(0.f, 0.f, gradientOffset()));

  // auto g = fvec3(nx, ny, nz) / gradientOffset() / 2.f;
  if (nx == 0.f && ny == 0.f && nz == 0.f) {
    out = glm::normalize(fvec3(1));
    return;
  }
  out = glm::normalize(fvec3(nx, ny, nz));

  //  constexpr int filterSize = 5;
  //  float l = gradientOffset();
  //  for (int x = 0; x < filterSize; ++x) {
  //    for (int y = 0; y < filterSize; ++y) {
  //      for (int z = 0; z < filterSize; ++z) {
  //        out += p + fvec3((x - filterSize / 2) * l, (y - filterSize / 2) * l, (z - filterSize / 2) * l);
  //      }
  //    }
  //  }
  //  if (out.x == 0.f && out.y == 0.f && out.z == 0.f) {
  //    out = glm::normalize(fvec3(1));
  //  }
  //  out = glm::normalize(out);
  assert(!isnan(out.x));
}
glm::fvec3 ScalarField::gradient(const glm::fvec3 &p) {
  float nx = value(p + fvec3(gradientOffset(), 0.f, 0.f)) - value(p - fvec3(gradientOffset(), 0.f, 0.f));
  float ny = value(p + fvec3(0.f, gradientOffset(), 0.f)) - value(p - fvec3(0.f, gradientOffset(), 0.f));
  float nz = value(p + fvec3(0.f, 0.f, gradientOffset())) - value(p - fvec3(0.f, 0.f, gradientOffset()));

  // auto g = fvec3(nx, ny, nz) / gradientOffset() / 2.f;
  return fvec3(nx, ny, nz) / gradientOffset() / 2.f;
}