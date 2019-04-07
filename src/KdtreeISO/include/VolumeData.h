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
  std::string pathToTiffs;
  int levels;
  int width = 0;
  int height = 0;
  std::vector<uint8_t> data;
  PositionCode minCode;
  PositionCode scale;
  float index(const PositionCode &code) override;

  public:
  VolumeData(const std::string &pathToTiffs, int levels, const PositionCode &minCode, const PositionCode &scale)
    : pathToTiffs(pathToTiffs),
      levels(levels),
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
