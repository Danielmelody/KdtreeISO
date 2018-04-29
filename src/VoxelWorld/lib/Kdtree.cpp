//
// Created by Danielhu on 2018/4/22.
//

#include "Kdtree.h"
#include "Octree.h"
#include "Indicators.h"
#include "Mesh.h"

void Kdtree::drawKdtree(Kdtree *root, Mesh *mesh) {
  if (!root) {
    return;
  }
  if (root->depth == 7) {
    vec3 size = codeToPos(root->maxCode - root->minCode, Octree::cellSize);
    vec3 min = codeToPos(root->minCode, Octree::cellSize);
    for (int i = 0; i < 12; ++i) {
      auto a = min_offset_subdivision(cellProcFaceMask[i][0]) * size + min;
      auto b = min_offset_subdivision(cellProcFaceMask[i][1]) * size + min;

      auto na = normalize(min_offset_subdivision(cellProcFaceMask[i][0]) - vec3(0.5f));
      auto nb = normalize(min_offset_subdivision(cellProcFaceMask[i][1]) - vec3(0.5f));

      mesh->positions.push_back(a);
      mesh->positions.push_back(a);
      mesh->positions.push_back(b);
      mesh->normals.push_back(na);
      mesh->normals.push_back(na);
      mesh->normals.push_back(nb);
      mesh->indices.push_back(static_cast<unsigned int &&>(mesh->indices.size()));
      mesh->indices.push_back(static_cast<unsigned int &&>(mesh->indices.size()));
      mesh->indices.push_back(static_cast<unsigned int &&>(mesh->indices.size()));
    }
    return;
  }
  drawKdtree(root->left, mesh);
  drawKdtree(root->right, mesh);
}