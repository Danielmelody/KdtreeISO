//
// Created by Danielhu on 2017/12/25.
//

#ifndef VOXELWORLD_MESH_H
#define VOXELWORLD_MESH_H

#include <vector>
#include <glm/glm.hpp>


class Mesh {
  public:
  std::vector<unsigned int> indices;
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;

  void generateSharpNormals();
};

#endif //VOXELWORLD_MESH_H
