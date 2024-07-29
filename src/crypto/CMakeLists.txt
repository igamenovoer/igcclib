# the style follows this guide:
# https://stackoverflow.com/questions/54702582/how-to-configure-project-with-components-in-cmake

# include(${CMAKE_SOURCE_DIR}/cmake/igcclib-utils.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/component-helpers.cmake)
set(component crypto)

# core library is header only
set(src_files  
    ${CMAKE_CURRENT_LIST_DIR}/AES_ECB_Cipher.cpp
    ${CMAKE_CURRENT_LIST_DIR}/RSA_Cipher.cpp
)
set_source_files_properties(${src_files} PROPERTIES LANGUAGE CXX)

create_component_library(${component} ${master_name} ${src_files})

# setup component linking
find_package(OpenSSL REQUIRED)
target_link_libraries(${component} PRIVATE OpenSSL::Crypto)

# create install rules for the component
create_component_install_rules(${component} ${master_name} "")