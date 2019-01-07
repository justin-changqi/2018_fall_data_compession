cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project( hw4-2_image_sharpning )

set(CMAKE_CXX_FLAGS "-g -O0")

include_directories(include)

add_executable( huffman_code src/huffman_code.cpp )
target_link_libraries( huffman_code)