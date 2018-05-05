//
// Created by Danielhu on 2017/12/25.
//

#ifndef VOXELWORLD_MESH_H
#define VOXELWORLD_MESH_H

#include <vector>
#include <glm/glm.hpp>
#include "Topology.h"
#include "Vertex.h"

struct Mesh {
  std::vector<unsigned int> indices;
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  void addTriangle(Vertex **vertices, Topology *g);
  void addVertex(Vertex *v, Topology *g);
  void drawAABBDebug(glm::vec3 min, glm::vec3 max);
  void generateFlatNormals();
};

#endif //VOXELWORLD_MESH_H
