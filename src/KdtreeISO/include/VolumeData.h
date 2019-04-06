//
// Created by Danielhu on 2018/5/29.
//

#ifndef VOXELWORLD_VOLUME_DATA_H
#define VOXELWORLD_VOLUME_DATA_H

extern "C" {
#include "tiffio.h"
}
#include <cstdint>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iomanip>
#include <utility>
#include "ScalarField.h"
#include "RectilinearGrid.h"
#include "Utils.h"

class VolumeData : public ScalarField {
  static constexpr float ISO_VAL = 105.f;
  std::string wildcard;
  const int levels;
  const int width;
  const int height;
  uint8_t *data{nullptr};
  PositionCode minCode;
  PositionCode scale;
  float index(const PositionCode &code) override;

  public:
  VolumeData(std::string wildcard, int levels, int width, int height, const PositionCode &minCode, const PositionCode &scale)
    : wildcard(std::move(wildcard)),
      levels(levels),
      width(width),
      height(height),
      minCode(minCode),
      scale(scale) {}
  inline int codeToOffset(const PositionCode &code) {
    return code.z * width * height + code.y * width + code.x;
  }
  float value(const glm::fvec3 &p) override;
  float gradientOffset() override { return RectilinearGrid::getUnitSize(); }
  bool solve(const glm::fvec3 &p1, const glm::fvec3 &p2, glm::fvec3 &out) override;
  void readTIFF();
};

#endif //VOXELWORLD_VOLUME_DATA_H
