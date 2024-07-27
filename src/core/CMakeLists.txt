# the style follows this guide:
# https://stackoverflow.com/questions/54702582/how-to-configure-project-with-components-in-cmake

set(component core)

# core library is header only
add_library(${component} INTERFACE)
add_library(${CMAKE_PROJECT_NAME}::${component} ALIAS ${component})

# find dependencies
# require eigen3, spdlog, nanoflann
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(nanoflann REQUIRED)

# internal dependencies
# required targets: extern
target_link_libraries(${component} INTERFACE ${CMAKE_PROJECT_NAME}::extern)

# include directories
# during build: CMAKE_SOURCE_DIR/include
# after install: include
target_include_directories(${component} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# copy compiled files to install directory, but we have nothing to copy
# this is just used to add the include directory to INTERFACE_INCLUDE_DIRECTORIES to the install(EXPORT) target
install(TARGETS ${component}
    EXPORT ${CMAKE_PROJECT_NAME}-${component}-targets   # this affects target in install(EXPORT)
    COMPONENT ${component}
    INCLUDES DESTINATION include
    # LIBRARY DESTINATION lib
    # ARCHIVE DESTINATION lib
    # RUNTIME DESTINATION bin
)

# create cmake files for the target
install(EXPORT ${CMAKE_PROJECT_NAME}-${component}-targets
    FILE ${CMAKE_PROJECT_NAME}-${component}-targets.cmake
    NAMESPACE ${CMAKE_PROJECT_NAME}::
    DESTINATION lib/cmake/${CMAKE_PROJECT_NAME}
    COMPONENT ${component}
)

# create version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_PROJECT_NAME}-${component}-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# copy version files to install directory
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-${component}-config-version.cmake
    DESTINATION lib/cmake/${CMAKE_PROJECT_NAME}
    COMPONENT ${component}
)

# finally, copy include files, but this is left to the master project
# install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
#     DESTINATION include
#     COMPONENT ${component}
# )