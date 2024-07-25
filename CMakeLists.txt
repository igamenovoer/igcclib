cmake_minimum_required(VERSION 3.20)
project(igcclib-project)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

# core library
add_library(igcclib-core INTERFACE)
target_include_directories(igcclib-core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(test)