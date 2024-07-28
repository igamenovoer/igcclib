# define a function that
# given component name, configuration name, generate target name as component_name-config_name-targets
# and return the export target name
function(igcclib_get_export_target_cmake_filename component_name output_name)
    set(${output_name} ${component_name}-${CMAKE_BUILD_TYPE}-targets.cmake PARENT_SCOPE)
endfunction()

# get all supported configs
# including Release, Debug, MinSizeRel, RelWithDebInfo
function(igcclib_get_supported_configs output_name)
    set(${output_name} "Release;Debug;MinSizeRel;RelWithDebInfo" PARENT_SCOPE)
endfunction()

# get .cmake file output dir for component, relative to CMAKE_BINARY_DIR
function(igcclib_get_component_cmake_output_dir component_name output_name)
    set(${output_name} "lib/cmake/component" PARENT_SCOPE)
endfunction()
    


