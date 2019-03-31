//
// Created by Danielhu on 2018/5/29.
//

#ifndef VOXELWORLD_SCALARFIELD_H
#define VOXELWORLD_SCALARFIELD_H

#include <cstdint>
#include <glm/glm.hpp>
#include "Utils.h"

class ScalarField {
  protected:
  uint8_t materialID;

  public:
  virtual float value(const glm::fvec3 &p) = 0;
  virtual float index(const PositionCode &code) = 0;
  virtual bool solve(const glm::fvec3 &p1, const glm::fvec3 &p2, glm::fvec3 &out) = 0;
  virtual float gradientOffset() = 0;
  virtual void normal(const glm::fvec3 &p, glm::fvec3 &out);
  virtual fvec3 normal_f1(const glm::fvec3 &p);
  virtual glm::fvec3 gradient(const glm::fvec3 &p);
  uint8_t getMaterialID() { return materialID; }
  ScalarField() : materialID(1) {}
  virtual ~ScalarField() = default;
};

#endif //VOXELWORLD_SCALARFIELD_H
