include(${PROJECT_SOURCE_DIR}/cmake/igcclib-component-helpers.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/igcclib-dependency.cmake)
set(component python_pybind11)

create_component_interface(${component} ${master_name})

# find pybind11
find_package(pybind11 REQUIRED)

# find python libraries
find_package(PythonLibs REQUIRED)

target_link_libraries(${component} INTERFACE
    ${master_name}::core Python::Python pybind11::pybind11
)

# create_component_install_rules(${component} ${master_name} "core")

# create install rules for the component
set(required_comps ${${component}_REQUIRED_COMPONENTS})
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_LIBRARIES pybind11 PythonLibs
    REQUIRED_COMPONENTS ${required_comps}
)