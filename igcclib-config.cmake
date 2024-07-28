foreach(component ${@project_name@_FIND_COMPONENTS})
#   set(igcclib_FIND_COMPONENTS_STRING "${igcclib_FIND_COMPONENTS_STRING} ${component}")
    include(${CMAKE_CURRENT_LIST_DIR}/igcclib-config-${component}.cmake)
endforeach()