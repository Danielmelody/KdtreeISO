//
// Created by Danielhu on 2018/5/2.
//

#ifndef VOXELWORLD_AXISALIGNEDLINE_H
#define VOXELWORLD_AXISALIGNEDLINE_H

#include <glm/glm.hpp>
#include "Utils.h"

struct AALine {
  PositionCode point;
  int dir;
  bool operator==(const AALine &other) {
    return point[(dir + 1) % 3] == other.point[(dir + 1) % 3] && point[(dir + 2) % 3] == other.point[(dir + 2) % 3];
  }
  AALine(const PositionCode &axes, int dotPlane) : point(axes), dir(dotPlane) {}
  AALine() = default;
};

#endif //VOXELWORLD_AXISALIGNEDLINE_H
