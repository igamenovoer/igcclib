cmake_minimum_required(VERSION 3.20)
project(igcclib-test)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find spdlog
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the test executable
add_executable(test-core test-core.cpp)
target_link_libraries(test-core igcclib-core spdlog::spdlog Eigen3::Eigen)