include_directories(include)

set(SOURCE_FILES lib/ScalarField.cpp lib/Topology.cpp lib/Octree.cpp lib/Kdtree.cpp lib/Qef.cpp lib/Mesh.cpp lib/RectilinearGrid.cpp lib/VolumeData.cpp)

add_library(KdtreeISO SHARED ${SOURCE_FILES})

target_link_libraries(KdtreeISO -ltiff)
