# component dependency information

set(core_REQUIRED_COMPONENTS "")
set(compression_REQUIRED_COMPONENTS core)
set(crypto_REQUIRED_COMPONENTS "")
set(device_REQUIRED_COMPONENTS core vision io)
set(extern_REQUIRED_COMPONENTS "")
set(geometry_REQUIRED_COMPONENTS core extern)
set(graph_REQUIRED_COMPONENTS core)
set(io_REQUIRED_COMPONENTS core extern)
set(math_REQUIRED_COMPONENTS core extern)
set(python_boostpy_REQUIRED_COMPONENTS core)
set(python_pybind11_REQUIRED_COMPONENTS core)
set(vision_REQUIRED_COMPONENTS core extern)
set(visualization_REQUIRED_COMPONENTS core extern geometry)

# collect all variables with pattern *_REQUIRED_COMPONENTS into all_REQ_COMPS
# * means a component name, collect all component names to all_components
get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    if (_variableName MATCHES ".*_REQUIRED_COMPONENTS$")
        string(REGEX REPLACE "_REQUIRED_COMPONENTS$" "" _component ${_variableName})
        list(APPEND all_components ${_component})
        list(APPEND all_REQ_COMPS ${${_variableName}})
    endif()
endforeach()

# for all variables in all_REQ_COMPS, copy them to parent scope
foreach (_component ${all_components})
    list(REMOVE_DUPLICATES ${_component}_REQUIRED_COMPONENTS)
    set(${_component}_REQUIRED_COMPONENTS ${${_component}_REQUIRED_COMPONENTS})
endforeach()

set(IGCCLIB_ALL_COMPONENTS ${all_components})

# given a list of components, return a list of all dependencies
function(get_igcclib_component_dependencies)
    set(oneValueArgs OUTPUT_VAR)
    set(multiValueArgs COMPONENTS)
    cmake_parse_arguments(GET_IGCCLIB_COMPONENT_DEPENDENCIES "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    set(component_asked ${GET_IGCCLIB_COMPONENT_DEPENDENCIES_COMPONENTS})
    set(output_var ${GET_IGCCLIB_COMPONENT_DEPENDENCIES_OUTPUT_VAR})

    set(comps_required ${component_asked})

    # for each component in comps_required, add its dependencies to comps_required
    foreach (comp ${component_asked})
        list(APPEND comps_required ${${comp}_REQUIRED_COMPONENTS})
    endforeach()

    # remove duplicates
    list(REMOVE_DUPLICATES comps_required)

    # if core is in the list, move it to the top
    if("core" IN_LIST comps_required)
        list(REMOVE_ITEM comps_required "core")
        list(INSERT comps_required 0 "core")
    endif()

    # if extern is in the list, move it to the top
    if("extern" IN_LIST comps_required)
        list(REMOVE_ITEM comps_required "extern")
        list(INSERT comps_required 0 "extern")
    endif()

    set(${output_var} ${comps_required} PARENT_SCOPE)
endfunction()
