cmake_minimum_required(VERSION 3.20)
project(igcclib VERSION 0.1.0 LANGUAGES CXX)

set(project_name ${CMAKE_PROJECT_NAME})

# Set the master name of the project
set(master_name "igcclib")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_DEBUG_POSTFIX d)
set(IGCCLIB_OUTPUT_PREFIX "${master_name}-")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# for catching fetch content
set(IGCCLIB_FETCHCONTENT_BASE_DIR "${CMAKE_CURRENT_LIST_DIR}/.cmake-fetch")

# if BUILD_SHARED_LIBS is not defined, add an option to define it
# otherwise, use the value of BUILD_SHARED_LIBS
if(NOT DEFINED BUILD_SHARED_LIBS)
    option(BUILD_SHARED_LIBS "Build shared libraries")
endif()

if(BUILD_SHARED_LIBS)
    add_definitions(-DIGCCLIB_EXPORTING)
endif()

# main library
add_subdirectory(src)

set(IGCCLIB_TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")
set(IGCCLIB_TEST_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tmp")

option(WITH_TESTS "Build tests" OFF)
option(WITH_EXAMPLES "Build examples" OFF)

if(WITH_TESTS)
    add_subdirectory(tests)
endif()

if(WITH_EXAMPLES)
    add_subdirectory(examples)
endif()