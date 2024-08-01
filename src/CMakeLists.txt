include(GNUInstallDirs)
include(${PROJECT_SOURCE_DIR}/cmake/component-helpers.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/igcclib-dependency.cmake)

option(WITH_ALL_COMPONENTS "Build all components" ON)
option(WITH_GEOMETRY "Build geometry module" OFF)
option(WITH_COMPRESSION "Build compression module" OFF)
option(WITH_CRYPTO "Build crypto module" OFF)
option(WITH_IO "Build io module" OFF)
option(WITH_VISION "Build vision module" OFF)
option(WITH_VISUALIZATION "Build visualization module" OFF)
option(WITH_GRAPH "Build graph module" OFF)
option(WITH_MATH "Build math module" OFF)
option(WITH_PYTHON_BOOSTPY "Build python boostpy module" OFF)
option(WITH_PYTHON_PYBIND11 "Build python pybind11 module" OFF)

# these components are always built
set(WITH_CORE ON)
set(WITH_EXTERN ON)

# turn on all components if WITH_ALL_COMPONENTS is on
if(WITH_ALL_COMPONENTS)
    set(WITH_GEOMETRY ON)
    set(WITH_COMPRESSION ON)
    set(WITH_CRYPTO ON)
    set(WITH_IO ON)
    set(WITH_VISION ON)
    set(WITH_VISUALIZATION ON)
    set(WITH_GRAPH ON)
    set(WITH_MATH ON)
    set(WITH_PYTHON_BOOSTPY ON)
    set(WITH_PYTHON_PYBIND11 ON)
endif()

# resolve dependencies
# for each component in IGCCLIB_ALL_COMPONENTS, check its dependencies
# if it depends on a component xx, then WITH_XX must be ON
foreach(component ${IGCCLIB_ALL_COMPONENTS})
    set(dependencies ${${component}_REQUIRED_COMPONENTS})
    foreach(dep ${dependencies})
        # convert dep to upper case
        string(TOUPPER ${dep} dep_upper)

        if(NOT WITH_${dep_upper})
            message(FATAL_ERROR "Component ${component} depends on ${dep}, but WITH_${dep_upper} is OFF")
        endif()
        
    endforeach()
endforeach()

add_subdirectory(core)
add_subdirectory(extern)

if(WITH_GEOMETRY)
    add_subdirectory(geometry)
endif()

if(WITH_GRAPH)
    add_subdirectory(graph)
endif()

if(WITH_VISUALIZATION)
    add_subdirectory(visualization)
endif()

if(WITH_VISION)
    add_subdirectory(vision)
endif()

if(WITH_COMPRESSION)
    add_subdirectory(compression)
endif()

if(WITH_CRYPTO)
    add_subdirectory(crypto)
endif()

if(WITH_IO)
    add_subdirectory(io)
endif()

if(WITH_MATH)
    add_subdirectory(math)
endif()

if(WITH_PYTHON_BOOSTPY)
    add_subdirectory(python_boostpy)
endif()

if(WITH_PYTHON_PYBIND11)
    add_subdirectory(python_pybind11)
endif()

# create config file
include(CMakePackageConfigHelpers)

set(ConfigMasterName ${master_name})
get_library_cmake_output_dir(OUTPUT_DIR_FOR_CONFIG)
configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/config-root.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/${master_name}-config.cmake
    INSTALL_DESTINATION ${OUTPUT_DIR_FOR_CONFIG}
)

# write version file
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/${master_name}-config-version.cmake
    VERSION ${master_version}
    COMPATIBILITY AnyNewerVersion
)

# copy version and config file to install dir
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/${master_name}-config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/${master_name}-config-version.cmake
    DESTINATION ${OUTPUT_DIR_FOR_CONFIG}
)

# copy project/cmake files to install/cmake dir
install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/cmake
    DESTINATION .
    FILES_MATCHING PATTERN "*.cmake"
)

# copy project/include files to install/include dir
install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/include/${master_name}
    DESTINATION include
)