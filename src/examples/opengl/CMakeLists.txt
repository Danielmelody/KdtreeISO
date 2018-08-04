find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

add_executable(opengl_viewer main.cpp program.cpp)
target_link_libraries(opengl_viewer LINK_PUBLIC KdtreeISO glfw glew ${OPENGL_LIBRARY})