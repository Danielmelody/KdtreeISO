//
// Created by Danielhu on 2018/5/29.
//

#include "VolumeData.h"
#include "Indicators.h"
#include "RectilinearGrid.h"
#include <glm/gtc/type_precision.hpp>

void VolumeData::readTIFF() {
  data = static_cast<uint8_t *>(_TIFFmalloc(width * height * levels));
  uint8_t *p = data;
  for (int i = 1; i < levels + 1; ++i) {
    std::stringstream namess;
    namess << std::setfill('0') << std::setw(3) << i;
    auto name = (wildcard + namess.str() + ".tif");
    TIFF *file = TIFFOpen(name.c_str(), "r");
    for (int h = 0; h < height; ++h) {
      TIFFReadScanline(file, p, h);
      p += width;
    }
    TIFFClose(file);
  }
}

float VolumeData::index(const PositionCode &code) {
  auto offset = codeToOffset((code - minCode) * scale);
  if (offset >= width * height * levels || offset < 0) {
    return ISO_VAL;
  }
  uint8_t *ptr = data + offset;
  return ISO_VAL - (*ptr);
}

float VolumeData::value(const glm::fvec3 &p) {
  float l = RectilinearGrid::getUnitSize();
  return index(posToCode(p, l));
  /*PositionCode samples[8];
  float values[8];
  for (int i = 0; i < 8; ++i) {
    samples[i] = posToCodeFloor(p + l * min_offset_subdivision(i), l);
    values[i] = index(samples[i]);
  }
  fvec3 d = (p - codeToPos(samples[0], l)) / l;
  d = glm::max(fvec3(0), glm::min(fvec3(1), d));
  float c00 = values[0b000] * (1 - d.x) + values[0b100] * d.x;
  float c01 = values[0b001] * (1 - d.x) + values[0b101] * d.x;
  float c10 = values[0b010] * (1 - d.x) + values[0b110] * d.x;
  float c11 = values[0b011] * (1 - d.x) + values[0b111] * d.x;
  float c0 = c00 * (1 - d.y) + c10 * d.y;
  float c1 = c01 * (1 - d.y) + c11 * d.y;
  float c = c0 * (1 - d.z) + c1 * d.z;
  return c;*/
}

bool VolumeData::solve(const glm::fvec3 &p1, const glm::fvec3 &p2,
                       glm::fvec3 &out) {
  float v1 = value(p1);
  float v2 = value(p2);
  if (v2 - v1 == 0.f) {
    out = (p1 + p2) / 2.f;
  }
  else {
    out = p1 - (p2 - p1) * v1 / (v2 - v1);
  }
  return true;
}
