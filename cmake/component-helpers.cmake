
function(get_component_export_target_filename component output_var)
    set(${output_var} ${component}-${CMAKE_BUILD_TYPE}-targets.cmake PARENT_SCOPE)
    # set(${output_var} ${component}-$<CONFIG>-targets.cmake PARENT_SCOPE)
endfunction()

# get .cmake file output dir for component, relative to CMAKE_BINARY_DIR
function(get_component_cmake_output_dir component output_var)
    set(${output_var} "lib/cmake/component" PARENT_SCOPE)
endfunction()

# create library target for component, named ${component}
# setup its include directories as well, but linking is not done here
# you need to link the component manually
macro(create_component_library component master_name src_files)

    add_library(${component} ${src_files})
    add_library(${master_name}::${component} ALIAS ${component})

    # include directories
    # during build: CMAKE_SOURCE_DIR/include
    # after install: include
    target_include_directories(${component} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

endmacro()

# create interface target for component, named ${component}, this is used for header-only libraries
# setup its include directories as well, but linking is not done here
# you need to link the component manually
macro(create_component_interface component master_name)

    add_library(${component} INTERFACE)
    add_library(${master_name}::${component} ALIAS ${component})

    # include directories
    # during build: CMAKE_SOURCE_DIR/include
    # after install: include
    target_include_directories(${component} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
endmacro()


# assuming the component represents a library target, like this:
# add_library(${component} ...)
macro(create_component_install_rules component master_name depends_on_components)

    # component .cmake output dir relative to CMAKE_BINARY_DIR
    get_component_cmake_output_dir(${component} component_cmake_output_dir)
    get_component_export_target_filename(${component} export_target_cmake_filename)

    # copy compiled files to install directory, but we have nothing to copy
    # this is just used to add the include directory to INTERFACE_INCLUDE_DIRECTORIES to the install(EXPORT) target
    install(TARGETS ${component}
        EXPORT ${component}-targets   # this affects target in install(EXPORT)
        COMPONENT ${component}
        INCLUDES DESTINATION include
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )

    # create cmake files for the target
    install(EXPORT ${component}-targets
        FILE ${export_target_cmake_filename}
        NAMESPACE ${master_name}::
        DESTINATION ${component_cmake_output_dir}
        COMPONENT ${component}
    )

    # create version file
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        ${component}-config-version.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
    )

    # copy version files to install directory
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${component}-config-version.cmake
        DESTINATION ${component_cmake_output_dir}
        COMPONENT ${component}
    )

    # finally, copy include files
    set(include_base_dir ${CMAKE_SOURCE_DIR}/include/${master_name})
    set(include_dirs ${include_base_dir}/${component})

    # also add depends_on_components include directories
    foreach(dep_component ${depends_on_components})
        set(dep_include_dirs ${include_base_dir}/${dep_component})
        list(APPEND include_dirs ${dep_include_dirs})
    endforeach()

    install(DIRECTORY ${include_dirs}
        DESTINATION include/${master_name}
        COMPONENT ${component}
    )

endmacro()