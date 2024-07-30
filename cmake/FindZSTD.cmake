# see https://github.com/facebook/folly/blob/main/CMake/FindZstd.cmake

# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# - Try to find Facebook zstd library
# This will define
# ZSTD_FOUND
# ZSTD_INCLUDE_DIR
# ZSTD_LIBRARY
#

find_path(ZSTD_INCLUDE_DIR NAMES zstd.h)

find_library(ZSTD_LIBRARY_DEBUG NAMES zstdd zstd_staticd)
find_library(ZSTD_LIBRARY_RELEASE NAMES zstd zstd_static)

# found zstd? if not, try to use pkg-config
if (NOT ZSTD_LIBRARY_DEBUG AND NOT ZSTD_LIBRARY_RELEASE)
    message(STATUS "zstd library not found, trying pkg-config")
    find_package(PkgConfig)
    if (PKG_CONFIG_FOUND)
        pkg_check_modules(ZSTD_PKG libzstd)
        if (ZSTD_PKG_FOUND)
            set(ZSTD_INCLUDE_DIR ${ZSTD_PKG_INCLUDE_DIRS})
            set(ZSTD_LIBRARY ${ZSTD_PKG_LIBRARIES})
        endif()
    endif()
endif()

include(SelectLibraryConfigurations)
SELECT_LIBRARY_CONFIGURATIONS(ZSTD)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
    ZSTD DEFAULT_MSG
    ZSTD_LIBRARY ZSTD_INCLUDE_DIR
)

if (ZSTD_FOUND)
    message(STATUS "Found ZSTD: ${ZSTD_LIBRARY}")
endif()

mark_as_advanced(ZSTD_INCLUDE_DIR ZSTD_LIBRARY)

# if debug is not found, use release as debug
if (NOT ZSTD_LIBRARY_DEBUG)
    set(ZSTD_LIBRARY_DEBUG ${ZSTD_LIBRARY_RELEASE})
endif()

# also create imported target zstd::zstd
add_library(ZSTD::ZSTD UNKNOWN IMPORTED)
set_target_properties(ZSTD::ZSTD PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${ZSTD_INCLUDE_DIR}"
    IMPORTED_LOCATION_DEBUG "${ZSTD_LIBRARY_DEBUG}"
    IMPORTED_LOCATION_RELEASE "${ZSTD_LIBRARY_RELEASE}"
    IMPORTED_LOCATION_RELWITHDEBINFO "${ZSTD_LIBRARY_RELEASE}"
    IMPORTED_LOCATION_MINSIZEREL "${ZSTD_LIBRARY_RELEASE}"
)