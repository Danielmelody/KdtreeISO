find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

add_executable(opengl_viewer main.cpp program.cpp)
target_link_libraries(opengl_viewer LINK_PUBLIC KdtreeISO glfw ${GLEW_LIBRARY} ${OPENGL_LIBRARY})