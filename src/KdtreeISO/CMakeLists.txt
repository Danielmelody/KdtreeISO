include_directories(KdtreeISO include)

set(SOURCE_FILES lib/ScalarField.cpp lib/Topology.cpp lib/Octree.cpp lib/Kdtree.cpp lib/Qef.cpp lib/Mesh.cpp lib/RectilinearGrid.cpp lib/VolumeData.cpp)

find_package(glm REQUIRED)
find_package(tiff REQUIRED)

add_library(KdtreeISO STATIC ${SOURCE_FILES})


target_link_libraries(KdtreeISO glm ${TIFF_LIBRARIES})
