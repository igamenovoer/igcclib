# the style follows this guide:
# https://stackoverflow.com/questions/54702582/how-to-configure-project-with-components-in-cmake

# include(${CMAKE_SOURCE_DIR}/cmake/igcclib-utils.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/component-helpers.cmake)
set(component extern)

# core library is header only
set(src_files  
    ${CMAKE_CURRENT_LIST_DIR}/splinter/cinterface/bspline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/cinterface/bsplinebuilder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/cinterface/cinterface.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/cinterface/datatable.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/cinterface/utilities.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/bspline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/bsplinebasis.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/bsplinebasis1d.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/bsplinebuilder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/datapoint.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/datatable.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/function.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/knots.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/mykroneckerproduct.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/serializer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/utilities.cpp
    ${CMAKE_CURRENT_LIST_DIR}/miniz.c
    ${CMAKE_CURRENT_LIST_DIR}/voxelizer.cpp
)
set_source_files_properties(${src_files} PROPERTIES LANGUAGE CXX)

create_component_library(${component} ${master_name} ${src_files})

# setup component linking
find_package(Eigen3 REQUIRED)
target_link_libraries(${component} PUBLIC Eigen3::Eigen)

# create install rules for the component
# create_component_install_rules(${component} ${master_name} "")

# create install rules for the component
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_LIBRARIES Eigen3)