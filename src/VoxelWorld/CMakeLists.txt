include_directories(include)

set(SOURCE_FILES lib/Topology.cpp lib/Octree.cpp lib/Kdtree.cpp lib/Qef.cpp lib/Mesh.cpp lib/RectilinearGrid.cpp)

add_library(VoxelTerrian SHARED ${SOURCE_FILES})
