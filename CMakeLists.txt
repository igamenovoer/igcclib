cmake_minimum_required(VERSION 3.20)
project(igcclib VERSION 0.1.0 LANGUAGES CXX)

set(project_name ${CMAKE_PROJECT_NAME})

# Set the master name of the project
set(master_name "igcclib")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_DEBUG_POSTFIX d)
set(IGCCLIB_OUTPUT_PREFIX "igcclib-")

# add ./cmake to module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_subdirectory(src)
add_subdirectory(test)