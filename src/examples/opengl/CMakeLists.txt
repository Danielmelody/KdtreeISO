add_executable(OPENGL_EXAMPLE main.cpp program.cpp)
target_link_libraries(OPENGL_EXAMPLE LINK_PUBLIC VoxelTerrian glfw glew ${OPENGL_LIBRARY})
