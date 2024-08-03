cmake_minimum_required(VERSION 3.20)
project(igcclib-examples)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# check if IGCCLIB_TEST_DATA_DIR is defined and non empty, otherwise raise an error
if(NOT DEFINED IGCCLIB_TEST_DATA_DIR OR "${IGCCLIB_TEST_DATA_DIR}" STREQUAL "")
  message(FATAL_ERROR "IGCCLIB_TEST_DATA_DIR is not defined or empty")
endif()

# check if IGCCLIB_TEST_OUTPUT_DIR is defined and non empty, otherwise raise an error
if(NOT DEFINED IGCCLIB_TEST_OUTPUT_DIR OR "${IGCCLIB_TEST_OUTPUT_DIR}" STREQUAL "")
  message(FATAL_ERROR "IGCCLIB_TEST_OUTPUT_DIR is not defined or empty")
endif()

# is this the top level project? using PROJECT_IS_TOP_LEVEL
# if not, find_package(igcclib) should be called before
if(PROJECT_IS_TOP_LEVEL)
  message(STATUS "Test project is building as a standalone project")
  message(STATUS "We need to find igcclib")
  find_package(igcclib COMPONENTS core vision io graph geometry extern REQUIRED)
  # find_package(igcclib COMPONENTS vision io REQUIRED)
endif()


# fetch catch2
include(FetchContent)

# do you have IGCCLIB_FETCHCONTENT_BASE_DIR defined and non empty?
# if yes, set it to FETCHCONTENT_BASE_DIR
if(DEFINED IGCCLIB_FETCHCONTENT_BASE_DIR AND NOT "${IGCCLIB_FETCHCONTENT_BASE_DIR}" STREQUAL "")
  set(FETCHCONTENT_BASE_DIR "${IGCCLIB_FETCHCONTENT_BASE_DIR}")
else()
  set(FETCHCONTENT_BASE_DIR "${PROJECT_SOURCE_DIR}/.cmake-fetch")
endif()

message(STATUS "fetch content will be cached at ${FETCHCONTENT_BASE_DIR}")

# find spdlog
# find_package(spdlog REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(nanoflann REQUIRED)

add_definitions(-DIGCCLIB_TEST_DATA_DIR="${IGCCLIB_TEST_DATA_DIR}")
add_definitions(-DIGCCLIB_TEST_OUTPUT_DIR="${IGCCLIB_TEST_OUTPUT_DIR}")

include_directories(${PROJECT_SOURCE_DIR}/include)
set(src_dir ${PROJECT_SOURCE_DIR}/src)
add_executable(imgproc_resize_image ${src_dir}/imgproc_resize_image.cpp)
# target_link_libraries(imgproc_resize_image PRIVATE igcclib::core igcclib::vision 
#     ${OpenCV_LIBS} spdlog::spdlog Eigen3::Eigen)
target_link_libraries(imgproc_resize_image PRIVATE igcclib::vision)

