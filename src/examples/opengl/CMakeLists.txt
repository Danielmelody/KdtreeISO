find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glew REQUIRED)

add_executable(OPENGL_EXAMPLE main.cpp program.cpp)
target_link_libraries(OPENGL_EXAMPLE LINK_PUBLIC VoxelTerrian glfw glew ${OPENGL_LIBRARY})
