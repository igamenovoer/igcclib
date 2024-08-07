# the style follows this guide:
# https://stackoverflow.com/questions/54702582/how-to-configure-project-with-components-in-cmake

# include(${PROJECT_SOURCE_DIR}/cmake/igcclib-utils.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/igcclib-component-helpers.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/igcclib-dependency.cmake)
set(component geometry)

# core library is header only
set(src_files  
    ${CMAKE_CURRENT_LIST_DIR}/igcclib_cgal.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MeshSearcher.cpp
    ${CMAKE_CURRENT_LIST_DIR}/SpatialQuery_2.cpp
)
set_source_files_properties(${src_files} PROPERTIES LANGUAGE CXX)

create_component_library(COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    SRC_FILES ${src_files})

# setup component linking
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
target_link_libraries(${component} PUBLIC Eigen3::Eigen CGAL::CGAL ${master_name}::core)

# create install rules for the component
# create_component_install_rules(${component} ${master_name} "core")

# create install rules for the component
set(required_comps ${${component}_REQUIRED_COMPONENTS})
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_LIBRARIES Eigen3 CGAL
    REQUIRED_COMPONENTS ${required_comps}
)