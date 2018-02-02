cmake_minimum_required(VERSION 3.7)
project(VoxelWorld)

set(CMAKE_CXX_STANDARD 11)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -pedantic")

include_directories(src/VoxelWorld/include)

add_subdirectory(src/VoxelWorld)
add_subdirectory(src/examples/opengl)

