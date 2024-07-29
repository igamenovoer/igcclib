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

create_component_install_rules(${component} ${master_name} "core")