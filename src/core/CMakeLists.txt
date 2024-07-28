# the style follows this guide:
# https://stackoverflow.com/questions/54702582/how-to-configure-project-with-components-in-cmake

# what is the use for COMPONENT keyword in install command?
# it can be used to group targets into component, and use cmake -DCOMPONENT=xxx to install only xxx component
# it has nothing to do with find_package, but only with install command
# https://stackoverflow.com/questions/9190098/for-cmakes-install-command-what-can-the-component-argument-do

# for multi configuration build system:
# https://discourse.cmake.org/t/install-with-export-and-multiple-configurations/1292

include(${CMAKE_SOURCE_DIR}/cmake/igcclib-utils.cmake)
set(component core)

# core library is header only
add_library(${component} INTERFACE)
add_library(${master_name}::${component} ALIAS ${component})

# find dependencies
# require eigen3, spdlog, nanoflann
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(nanoflann REQUIRED)

# component .cmake output dir relative to CMAKE_BINARY_DIR
igcclib_get_component_cmake_output_dir(${component} component_cmake_output_dir)
igcclib_get_export_target_cmake_filename(${component} export_target_cmake_filename)

# internal dependencies, only some header-only libraries in extern, no need to link target
# target_link_libraries(${component} INTERFACE ${master_name}::extern)

# include directories
# during build: CMAKE_SOURCE_DIR/include
# after install: include
target_include_directories(${component} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# get export target name

# copy compiled files to install directory, but we have nothing to copy
# this is just used to add the include directory to INTERFACE_INCLUDE_DIRECTORIES to the install(EXPORT) target
install(TARGETS ${component}
    EXPORT ${component}-targets   # this affects target in install(EXPORT)
    COMPONENT ${component}
    INCLUDES DESTINATION include
    # LIBRARY DESTINATION lib
    # ARCHIVE DESTINATION lib
    # RUNTIME DESTINATION bin
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
# this is actually useless, because component version cannot be used in find_package
# just for completeness
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${component}-config-version.cmake
    DESTINATION ${component_cmake_output_dir}
    COMPONENT ${component}
)

# finally, copy include files, but this is left to the master project
set(include_base_dir ${CMAKE_SOURCE_DIR}/include/${master_name})
set(include_dirs ${include_base_dir}/core ${include_base_dir}/extern)
install(DIRECTORY ${include_dirs}
    DESTINATION include/${master_name}
    COMPONENT ${component}
)