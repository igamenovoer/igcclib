# component dependency information

set(core_REQUIRED_COMPONENTS "" PARENT_SCOPE)
set(compression_REQUIRED_COMPONENTS core PARENT_SCOPE)
set(crypto_REQUIRED_COMPONENTS "" PARENT_SCOPE)
set(device_REQUIRED_COMPONENTS core vision io PARENT_SCOPE)
set(extern_REQUIRED_COMPONENTS "" PARENT_SCOPE)
set(geometry_REQUIRED_COMPONENTS core PARENT_SCOPE)
set(graph_REQUIRED_COMPONENTS core PARENT_SCOPE)
set(io_REQUIRED_COMPONENTS core PARENT_SCOPE)
set(math_REQUIRED_COMPONENTS core extern PARENT_SCOPE)
set(python_boostpy_REQUIRED_COMPONENTS core PARENT_SCOPE)
set(python_pybind11_REQUIRED_COMPONENTS core PARENT_SCOPE)
set(vision_REQUIRED_COMPONENTS core extern PARENT_SCOPE)
set(visualization_REQUIRED_COMPONENTS core extern geometry PARENT_SCOPE)

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
    set(${_component}_REQUIRED_COMPONENTS ${${_component}_REQUIRED_COMPONENTS} PARENT_SCOPE)
endforeach()

set(IGCCLIB_ALL_COMPONENTS ${all_components} PARENT_SCOPE)