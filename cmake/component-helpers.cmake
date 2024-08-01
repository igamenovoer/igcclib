include(CMakePackageConfigHelpers)

function(get_component_export_target_filename component output_var)
    set(${output_var} ${component}-targets.cmake PARENT_SCOPE)
endfunction()

function(get_library_cmake_output_dir output_var)
    set(${output_var} "lib/cmake" PARENT_SCOPE)
endfunction()

# get .cmake file output dir for component, relative to CMAKE_BINARY_DIR
function(get_component_cmake_output_dir component output_var)
    get_library_cmake_output_dir(OUTPUT_DIR)
    set(${output_var} "${OUTPUT_DIR}/component" PARENT_SCOPE)
endfunction()

function(get_component_config_filename component output_var)
    set(${output_var} "${component}-config.cmake" PARENT_SCOPE)
endfunction()

# create library target for component, named ${component}
# setup its include directories as well, but linking is not done here
# you need to link the component manually
# function(create_component_library component master_name src_files)
# use named parameters because src_files is a list
function(create_component_library)
    set(oneValueArgs COMPONENT MASTER_NAME)
    set(multiValueArgs SRC_FILES)
    cmake_parse_arguments(CREATE_COMPONENT_LIBRARY "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(component ${CREATE_COMPONENT_LIBRARY_COMPONENT})
    set(master_name ${CREATE_COMPONENT_LIBRARY_MASTER_NAME})
    set(src_files ${CREATE_COMPONENT_LIBRARY_SRC_FILES})

    add_library(${component} ${src_files})
    # message(STATUS "Creating component ${component} with sources ${src_files}")
    
    add_library(${master_name}::${component} ALIAS ${component})

    # include directories
    # during build: PROJECT_SOURCE_DIR/include
    # after install: include
    target_include_directories(${component} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        # $<INSTALL_INTERFACE:include> # leave it for install(EXPORT)
    )

endfunction()

# create interface target for component, named ${component}, this is used for header-only libraries
# setup its include directories as well, but linking is not done here
# you need to link the component manually
macro(create_component_interface component master_name)

    add_library(${component} INTERFACE)
    add_library(${master_name}::${component} ALIAS ${component})

    # include directories
    # during build: PROJECT_SOURCE_DIR/include
    # after install: include
    target_include_directories(${component} INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        # $<INSTALL_INTERFACE:include> # leave it for install(EXPORT)
    )
endmacro()


# create_component_install_rules(COMPONENT <component> MASTER_NAME <master_name> REQUIRED_COMPONENTS <depends_on_components> REQUIRED_LIBRARIES <depends_on_libraries>)
# COMPONENT: the name of the component, there must exists a target with the same name
# MASTER_NAME: the name of the master project
# REQUIRED_COMPONENTS: the components that this component depends on
# REQUIRED_LIBRARIES: the libraries that this component depends on
# COPY_INCLUDES: ON/OFF, copy include files to install directory, default is OFF, leave this part to parent
macro(create_component_install_rules)

    set(oneValueArgs COMPONENT MASTER_NAME COPY_INCLUDES)
    set(multiValueArgs REQUIRED_COMPONENTS REQUIRED_LIBRARIES)
    cmake_parse_arguments(CREATE_COMPONENT_INSTALL_RULES "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(component ${CREATE_COMPONENT_INSTALL_RULES_COMPONENT})
    message(STATUS "Creating install rules for component: ${component}")
    set(master_name ${CREATE_COMPONENT_INSTALL_RULES_MASTER_NAME})
    set(depends_on_components ${CREATE_COMPONENT_INSTALL_RULES_REQUIRED_COMPONENTS})
    set(depends_on_libraries ${CREATE_COMPONENT_INSTALL_RULES_REQUIRED_LIBRARIES})
    set(copy_includes ${CREATE_COMPONENT_INSTALL_RULES_COPY_INCLUDES})

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

    # create cmake target files for the target
    install(EXPORT ${component}-targets
        FILE ${export_target_cmake_filename}
        NAMESPACE ${master_name}::
        DESTINATION ${component_cmake_output_dir}
        COMPONENT ${component}
    )

    # create config file
    set(ConfigComponentName ${component})
    set(ConfigMasterName ${master_name})
    set(ConfigDependsLibrary ${depends_on_libraries})
    
    get_component_config_filename(${component} component_config_filename)
    configure_package_config_file(
        ${PROJECT_SOURCE_DIR}/cmake/config-component.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${component_config_filename}
        INSTALL_DESTINATION ${component_cmake_output_dir}
    )
    # configure_file(
    #     ${PROJECT_SOURCE_DIR}/cmake/config-component.cmake.in
    #     ${CMAKE_CURRENT_BINARY_DIR}/${component_config_filename}
    #     @ONLY
    # )
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${component_config_filename}
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

    if(copy_includes)
        # copy include files
        set(include_base_dir ${PROJECT_SOURCE_DIR}/include/${master_name})
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
    endif()

endmacro()