include(${PROJECT_SOURCE_DIR}/cmake/component-helpers.cmake)
include(FindPackageHandleStandardArgs)

# ====== create header-only target ==========
# set(component_header_only io_header_only)
# create_component_interface(${component_header_only} ${master_name})

# find_package(Boost REQUIRED COMPONENTS filesystem system)
# target_link_libraries(${component_header_only} INTERFACE 
#     ${master_name}::core Boost::filesystem Boost::system)
# create_component_install_rules(${component_header_only} ${master_name} "extern;core")

# ========== create compiled target ==============
set(component io)

set(src_files  
    ${CMAKE_CURRENT_LIST_DIR}/igcclib_io_compression.cpp 
)
set_source_files_properties(${src_files} PROPERTIES LANGUAGE CXX)

create_component_library(${component} ${master_name} ${src_files})

# setup component linking
find_package(nlohmann_json REQUIRED)
set(dep_libs nlohmann_json::nlohmann_json)

# use pkg-config to find zstd
# if pkg-config is not found, try to find zstd library using find_path and find_library
find_package(ZSTD REQUIRED)

# you will then have target:
# ZSTD::ZSTD

# message(STATUS "Using pkg-config to find zstd, if you are on Windows, you may need to install pkgconf using vcpkg")
# find_package(PkgConfig QUIET)
# if (NOT PkgConfig_FOUND)
#     message(STATUS "pkg-config not found, trying to find zstd.h and libzstd directly")
#     find_path(ZSTD_INCLUDE_DIR NAMES zstd.h)
#     find_library(ZSTD_LIBRARY NAMES zstd)
#     if (ZSTD_INCLUDE_DIR AND ZSTD_LIBRARY)
#         message(STATUS "Found zstd: ${ZSTD_LIBRARY}")
#         add_library(PkgConfig::ZSTD INTERFACE IMPORTED)
#         target_include_directories(PkgConfig::ZSTD INTERFACE ${ZSTD_INCLUDE_DIR})
#         target_link_libraries(PkgConfig::ZSTD INTERFACE ${ZSTD_LIBRARY})
#     else()
#         message(FATAL_ERROR "zstd not found")
#     endif()
# else()
#     message(STATUS "Using pkg-config to find zstd")
#     find_package(PkgConfig REQUIRED)
#     pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)
#     list(APPEND dep_libs PkgConfig::ZSTD)
# endif()

# message(STATUS "Using pkg-config to find zstd")
# message(STATUS "If you are on Windows, you may need to install pkgconf using vcpkg")
# find_package(PkgConfig QUIET)
# if (NOT PkgConfig_FOUND)
#     message(FATAL_ERROR "pkg-config not found")
# pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

# add libzstd to the list of dependencies
list(APPEND dep_libs ZSTD::ZSTD)

target_link_libraries(${component} PUBLIC 
    ${master_name}::core ${master_name}::vision 
    ${master_name}::extern ${dep_libs}
)
# target_include_directories(${component} PRIVATE ${ZSTD_INCLUDE_DIR})
# target_link_libraries(${component} PRIVATE ${ZSTD_LIBRARY})

# create install rules for the component
# create_component_install_rules(${component} ${master_name} "extern;core;vision")

# create install rules for the component
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_LIBRARIES Boost::graph ZSTD
    REQUIRED_COMPONENTS "core"
)