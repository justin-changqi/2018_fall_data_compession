cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project( hw4-2_image_sharpning )

# set(CMAKE_CXX_FLAGS "-g -O0")
find_package(PCL 1.7 REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_octree src/pcl_octree.cpp)
target_link_libraries( pcl_octree ${PCL_LIBRARIES})

# add_executable( huffman_code src/huffman_code.cpp )
# target_link_libraries( huffman_code)