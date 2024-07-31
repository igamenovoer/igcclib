cmake_minimum_required(VERSION 3.20)
project(igcclib-test)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# is this the top level project? using PROJECT_IS_TOP_LEVEL
# if not, find_package(igcclib) should be called before
if(PROJECT_IS_TOP_LEVEL)
  message(STATUS "Test project is building as a standalone project")
  message(STATUS "We need to find igcclib")
  find_package(igcclib COMPONENTS core vision io graph geometry REQUIRED)
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

# avoid using build location to avoid fetch every time
# a new build directory is created
message(STATUS "Fetching Catch2 ...")
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.6.0
)
FetchContent_MakeAvailable(Catch2)
message(STATUS "Catch2 source dir: ${catch2_SOURCE_DIR}")

# find spdlog
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(IGCCLIB_DATA_DIR "${PROJECT_SOURCE_DIR}/../data")

add_definitions(-DIGCCLIB_DATA_DIR="${IGCCLIB_DATA_DIR}")

add_executable(utest-core utest-core.cpp)
target_link_libraries(utest-core PRIVATE 
  igcclib::core Catch2::Catch2WithMain spdlog::spdlog Eigen3::Eigen)

add_executable(utest-vision utest-vision.cpp)
target_link_libraries(utest-vision PRIVATE igcclib::vision
  Catch2::Catch2WithMain spdlog::spdlog Eigen3::Eigen ${OpenCV_LIBS})

# let catch2 handle the test discovery
include(CTest)
include(Catch)
catch_discover_tests(utest-core)
catch_discover_tests(utest-vision)
