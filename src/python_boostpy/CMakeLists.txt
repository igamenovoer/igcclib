include(${CMAKE_SOURCE_DIR}/cmake/component-helpers.cmake)
set(component python_boostpy)

create_component_interface(${component} ${master_name})

# find dependencies
# require eigen3, spdlog, nanoflann
find_package(Boost REQUIRED COMPONENTS python)

# find python libraries
find_package(PythonLibs REQUIRED)

target_link_libraries(${component} INTERFACE
    ${master_name}::core Boost::boost Boost::python Python::Python
)

# create_component_install_rules(${component} ${master_name} "core")

# create install rules for the component
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_LIBRARIES PythonLibs Boost::python
    REQUIRED_COMPONENTS "core"
)