include(${PROJECT_SOURCE_DIR}/cmake/igcclib-component-helpers.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/igcclib-dependency.cmake)
set(component compression)

# core library is header only
set(src_files  
    ${CMAKE_CURRENT_LIST_DIR}/compression.cpp
)
set_source_files_properties(${src_files} PROPERTIES LANGUAGE CXX)

create_component_library(COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    SRC_FILES ${src_files})

# setup component linking
# target_link_libraries(${component} PUBLIC Eigen3::Eigen CGAL::CGAL ${master_name}::core)

# create install rules for the component
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_COMPONENTS ${${component}_REQUIRED_COMPONENTS})