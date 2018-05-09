//
// Created by Danielhu on 2018/1/22.
//

#include "Mesh.h"
#include "Indicators.h"

void Mesh::generateFlatNormals() {
  normals.resize(positions.size());
  std::vector<glm::fvec3> flat_positions;
  std::vector<glm::fvec3> flat_normals;
  std::vector<unsigned int> flat_indices;
  for (unsigned int i = 0; i < indices.size() / 3; ++i) {
    glm::fvec3 normal = glm::normalize(
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

void Mesh::addVertex(Vertex *v, Topology *g) {
  g->normal(v->hermiteP, v->hermiteN);
  v->vertexIndex = static_cast<unsigned int>(positions.size());
  positions.push_back(v->hermiteP);
  normals.push_back(v->hermiteN);
}

void Mesh::addTriangle(Vertex **vertices, Topology *g) {
  for (int j = 0; j < 3; ++j) {
    auto targetVert = vertices[j];
    Vertex *adjacentVerts[2] = {vertices[(j + 1) % 3], vertices[(j + 2) % 3]};
    glm::fvec3 offset =
        adjacentVerts[1]->hermiteP - targetVert->hermiteP +
            adjacentVerts[0]->hermiteP - targetVert->hermiteP;
    offset *= 0.05f;
    glm::fvec3 normal;
    g->normal(targetVert->hermiteP + offset, normal);
    if (glm::dot(normal, targetVert->hermiteN) < std::cos(glm::radians(15.f))) {
      indices.push_back(static_cast<unsigned int>(positions.size()));
      positions.push_back(targetVert->hermiteP);
      normals.push_back(normal);
    } else {
      indices.push_back(targetVert->vertexIndex);
    }
  }
  
}
void Mesh::drawAABBDebug(glm::fvec3 min, glm::fvec3 max) {
  auto offset = max - min;
  for (int i = 0; i < 3; ++i) {
    for(int j = 0; j < 2; ++j) {
      fvec3 quad[4];
      for (int k = 0; k < 4; ++k) {
        quad[k] = min + offset * min_offset_subdivision(cellProcFaceMask[i * 4 + k][j]);
      }
      int quadIndices[] = {0, 1, 2, 1, 2, 3};
      for (auto quadIndex : quadIndices) {
        positions.push_back(quad[quadIndex]);
        normals.push_back(glm::normalize(quad[quadIndex] - (min + max) / 2.f));
        indices.push_back(static_cast<unsigned int &&>(indices.size()));
      }
    }
  }
}
