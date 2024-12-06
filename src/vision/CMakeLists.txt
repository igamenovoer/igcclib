include(${PROJECT_SOURCE_DIR}/cmake/igcclib-component-helpers.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/igcclib-dependency.cmake)
include(FindPackageHandleStandardArgs)

set(component vision)

set(src_files ${CMAKE_CURRENT_LIST_DIR}/CameraModel.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/igcclib_image_processing.cpp
    ${CMAKE_CURRENT_LIST_DIR}/igcclib_opencv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoReader.cpp
)
set_source_files_properties(${src_files} PROPERTIES LANGUAGE CXX)

create_component_library(COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    SRC_FILES ${src_files})

# setup component linking
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nanoflann REQUIRED)
set(dep_libs Eigen3::Eigen ${OpenCV_LIBS} nanoflann::nanoflann)

target_link_libraries(${component} PUBLIC 
    ${master_name}::core ${master_name}::extern ${dep_libs}
)

# create install rules for the component
# create_component_install_rules(${component} ${master_name} "core;extern")

# create install rules for the component
set(required_comps ${${component}_REQUIRED_COMPONENTS})
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_LIBRARIES OpenCV Eigen3 nanoflann
    REQUIRED_COMPONENTS ${required_comps}
)