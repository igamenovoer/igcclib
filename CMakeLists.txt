cmake_minimum_required(VERSION 3.20)
project(igcclib-project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_DEBUG_POSTFIX d)
set(IGCCLIB_OUTPUT_PREFIX "igcclib-")

add_subdirectory(src)
add_subdirectory(test)