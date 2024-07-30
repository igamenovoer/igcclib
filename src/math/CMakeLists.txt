include(${CMAKE_SOURCE_DIR}/cmake/component-helpers.cmake)
set(component math)

create_component_interface(${component} ${master_name})

# find dependencies
# require eigen3, spdlog, nanoflann
find_package(Boost REQUIRED)

target_link_libraries(${component} INTERFACE
    ${master_name}::core Boost::boost
)

# create_component_install_rules(${component} ${master_name} "extern;core")

# create install rules for the component
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_COMPONENTS "core" "extern"
    REQUIRED_LIBRARIES Boost
)