//
// Created by Danielhu on 2018/5/29.
//

#include "VolumeData.h"
#include "Indicators.h"
#include "RectilinearGrid.h"
#include <glm/gtc/type_precision.hpp>

void VolumeData::readTIFF() {
  TIFF *firstFile = TIFFOpen((pathToTiffs + "001.tif").c_str(), "r");
  if (!firstFile)
    throw "no .tif file found";
  TIFFGetField(firstFile, TIFFTAG_IMAGEWIDTH, &width);
  TIFFGetField(firstFile, TIFFTAG_IMAGELENGTH, &height);
  data.resize(width * height * levels);
  uint8_t *p = data.data();
  for (int i = 0; i < levels; ++i) {
    std::stringstream namess;
    namess << std::setfill('0') << std::setw(3) << i + 1;
    auto name = (pathToTiffs + namess.str() + ".tif");
    if (TIFF *file = TIFFOpen(name.c_str(), "r")) {
      for (int h = 0; h < height; ++h) {
        TIFFReadScanline(file, p, h);
        p += width;
      }
      TIFFClose(file);
    }
  }
}

float VolumeData::index(const PositionCode &code) {
  auto offset = codeToOffset((code - minCode) * scale);
  if (offset >= width * height * levels || offset < 0) {
    return isovalue;
  }

  // return ISO_VAL - (float)data[codeToOffset((code - minCode) * scale)];

  float result = 0;
  for (int x = 0; x < scale.x; ++x)
    for (int y = 0; y < scale.y; ++y)
      for (int z = 0; z < scale.z; ++z) {
        result += (float)data[codeToOffset((code - minCode) * scale + PositionCode(x, y, z))];
      }
  return result / (scale.x * scale.y * scale.z) - isovalue;
}

float VolumeData::value(const glm::fvec3 &p) {
  float l = RectilinearGrid::getUnitSize();
  return index(posToCode(p, l));
  /* PositionCode samples[8];
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
