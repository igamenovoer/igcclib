# the style follows this guide:
# https://stackoverflow.com/questions/54702582/how-to-configure-project-with-components-in-cmake

include(${CMAKE_SOURCE_DIR}/cmake/igcclib-utils.cmake)
set(component extern)

# core library is header only
set(src_files  
    ${CMAKE_CURRENT_LIST_DIR}/splinter/cinterface/bspline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/cinterface/bsplinebuilder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/cinterface/cinterface.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/cinterface/datatable.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/cinterface/utilities.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/bspline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/bsplinebasis.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/bsplinebasis1d.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/bsplinebuilder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/datapoint.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/datatable.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/function.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/knots.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/mykroneckerproduct.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/serializer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/splinter/utilities.cpp
    ${CMAKE_CURRENT_LIST_DIR}/miniz.c
    ${CMAKE_CURRENT_LIST_DIR}/voxelizer.cpp
)
set_source_files_properties(${src_files} PROPERTIES LANGUAGE CXX)

add_library(${component} ${src_files})
add_library(${master_name}::${component} ALIAS ${component})

# find dependencies
# require eigen3, spdlog, nanoflann
find_package(Eigen3 REQUIRED)
target_link_libraries(${component} PUBLIC Eigen3::Eigen)

# include directories
# during build: CMAKE_SOURCE_DIR/include
# after install: include
target_include_directories(${component} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# component .cmake output dir relative to CMAKE_BINARY_DIR
igcclib_get_component_cmake_output_dir(${component} component_cmake_output_dir)
igcclib_get_export_target_cmake_filename(${component} export_target_cmake_filename)

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
set(include_dirs ${include_base_dir}/extern)
install(DIRECTORY ${include_dirs}
    DESTINATION include/${master_name}
    COMPONENT ${component}
)