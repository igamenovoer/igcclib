cmake_minimum_required(VERSION 3.20)

add_subdirectory(core)
add_subdirectory(extern)

# find_package(Eigen3 REQUIRED)
# find_package(Boost)
# find_package(OpenCV)

# # global links
# include_directories(${CMAKE_SOURCE_DIR}/include)
# link_libraries(Eigen3::Eigen)

# # core target
# add_library(core INTERFACE)
# target_include_directories(core PUBLIC INTERFACE ${CMAKE_SOURCE_DIR}/include)

# # vision target
# add_library(vision vision/igcclib_opencv.cpp)
# target_include_directories(vision PUBLIC ${OpenCV_INCLUDE_DIRS})
# target_link_libraries(vision ${OpenCV_LIBS} Eigen3::Eigen)
# set_target_properties(vision PROPERTIES PREFIX ${IGCCLIB_OUTPUT_PREFIX})

# # geometry target
# set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
# set(CGAL_DATA_DIR "/tmp")
# find_package(CGAL COMPONENTS Core)
# add_library(geometry geometry/igcclib_cgal.cpp)
# set_target_properties(geometry PROPERTIES PREFIX ${IGCCLIB_OUTPUT_PREFIX})
# target_link_libraries(geometry PUBLIC CGAL::CGAL CGAL::CGAL_Core Boost::boost Eigen3::Eigen)