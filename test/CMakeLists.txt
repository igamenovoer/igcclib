cmake_minimum_required(VERSION 3.20)
project(igcclib-test)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find spdlog
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the test executable
# include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(test-core test-core.cpp)
target_link_libraries(test-core core spdlog::spdlog Eigen3::Eigen)

add_executable(test-vision test-vision.cpp)
target_link_libraries(test-vision vision core spdlog::spdlog Eigen3::Eigen)