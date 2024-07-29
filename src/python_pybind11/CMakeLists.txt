include(${CMAKE_SOURCE_DIR}/cmake/component-helpers.cmake)
set(component python_pybind11)

create_component_interface(${component} ${master_name})

# find pybind11
find_package(pybind11 REQUIRED)

# find python libraries
find_package(PythonLibs REQUIRED)

target_link_libraries(${component} INTERFACE
    ${master_name}::core Boost::boost Python::Python pybind11::pybind11
)

create_component_install_rules(${component} ${master_name} "core")