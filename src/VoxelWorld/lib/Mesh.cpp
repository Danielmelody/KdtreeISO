//
// Created by Danielhu on 2018/1/22.
//

#include <Topology.h>
#include "Mesh.h"

void Mesh::generateFlatNormals() {
  normals.resize(positions.size());
  std::vector<glm::vec3> flat_positions;
  std::vector<glm::vec3> flat_normals;
  std::vector<unsigned int> flat_indices;
  for (unsigned int i = 0; i < indices.size() / 3; ++i) {
    glm::vec3 normal = glm::normalize(
        normals[indices[i * 3 + 0]] +
            normals[indices[i * 3 + 1]] +
            normals[indices[i * 3 + 2]]
    );
    for (unsigned int j = 0; j < 3; ++j) {
      flat_normals.push_back(normal);
      flat_positions.push_back(positions[indices[i * 3 + j]]);
      flat_indices.push_back(3 * i + j);
    }
  }
  positions = flat_positions;
  normals = flat_normals;
  indices = flat_indices;
}