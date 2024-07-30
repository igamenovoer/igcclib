include(${CMAKE_SOURCE_DIR}/cmake/component-helpers.cmake)
include(FindPackageHandleStandardArgs)

set(component vision)

set(src_files  
    ${CMAKE_CURRENT_LIST_DIR}/CameraModel.cpp
    ${CMAKE_CURRENT_LIST_DIR}/igcclib_image_processing.cpp
    ${CMAKE_CURRENT_LIST_DIR}/igcclib_opencv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoReader.cpp
)
set_source_files_properties(${src_files} PROPERTIES LANGUAGE CXX)

create_component_library(${component} ${master_name} ${src_files})

# setup component linking
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
set(dep_libs Eigen3::Eigen ${OpenCV_LIBS})

target_link_libraries(${component} PUBLIC 
    ${master_name}::core ${master_name}::extern ${dep_libs}
)

# create install rules for the component
# create_component_install_rules(${component} ${master_name} "core;extern")

# create install rules for the component
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_LIBRARIES OpenCV Eigen3
    REQUIRED_COMPONENTS "core" "extern"
)