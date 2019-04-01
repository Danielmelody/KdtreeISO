cmake_minimum_required(VERSION 3.7)
project(Discrete_Kdtree_ISOSURFACING)

set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -pedantic -Wno-nested-anon-types -Wno-return-std-move -Wno-gnu-anonymous-struct")

include_directories(src/KdtreeISO/include)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


add_subdirectory(src/KdtreeISO)
add_subdirectory(src/examples/opengl)
