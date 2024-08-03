# component dependency information

set(core_REQUIRED_COMPONENTS "")
set(compression_REQUIRED_COMPONENTS core)
set(crypto_REQUIRED_COMPONENTS "")
set(device_REQUIRED_COMPONENTS core vision io)
set(extern_REQUIRED_COMPONENTS "")
set(geometry_REQUIRED_COMPONENTS core extern)
set(graph_REQUIRED_COMPONENTS core)
set(io_REQUIRED_COMPONENTS core extern vision)
set(math_REQUIRED_COMPONENTS core extern)
set(python_boostpy_REQUIRED_COMPONENTS core)
set(python_pybind11_REQUIRED_COMPONENTS core)
set(vision_REQUIRED_COMPONENTS core extern)
set(visualization_REQUIRED_COMPONENTS core extern geometry)

# IGCCLIB_ALL_COMPONENTS lists all components, in the order of their dependencies
# you should load the components in this order
set(IGCCLIB_ALL_COMPONENTS core extern graph math compression geometry crypto vision io device python_pybind11 visualization)

# do we have duplicate?
set(IGCCLIB_ALL_COMPONENTS_TMP ${IGCCLIB_ALL_COMPONENTS})
list(REMOVE_DUPLICATES IGCCLIB_ALL_COMPONENTS_TMP)
if(NOT IGCCLIB_ALL_COMPONENTS STREQUAL IGCCLIB_ALL_COMPONENTS_TMP)
    message(FATAL_ERROR "Duplicate components in IGCCLIB_ALL_COMPONENTS")
endif()

# for all variables in all_REQ_COMPS, copy them to parent scope
foreach (_component ${IGCCLIB_ALL_COMPONENTS})
    list(REMOVE_DUPLICATES ${_component}_REQUIRED_COMPONENTS)
    set(${_component}_REQUIRED_COMPONENTS ${${_component}_REQUIRED_COMPONENTS})
endforeach()

# given a list of components, return a list of all dependencies
# including the input components themselves
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

    # create a new list containing all the components in comps_required, but in the order of their dependencies
    set(comps_required_ordered "")
    foreach (comp ${IGCCLIB_ALL_COMPONENTS})
        if("${comp}" IN_LIST comps_required)
            list(APPEND comps_required_ordered ${comp})
        endif()
    endforeach()

    set(${output_var} ${comps_required_ordered} PARENT_SCOPE)

    # # if core is in the list, move it to the top
    # if("core" IN_LIST comps_required)
    #     list(REMOVE_ITEM comps_required "core")
    #     list(INSERT comps_required 0 "core")
    # endif()

    # # if extern is in the list, move it to the top
    # if("extern" IN_LIST comps_required)
    #     list(REMOVE_ITEM comps_required "extern")
    #     list(INSERT comps_required 0 "extern")
    # endif()

    # set(${output_var} ${comps_required} PARENT_SCOPE)
endfunction()
