# the style follows this guide:
# https://stackoverflow.com/questions/54702582/how-to-configure-project-with-components-in-cmake

# what is the use for COMPONENT keyword in install command?
# it can be used to group targets into component, and use cmake -DCOMPONENT=xxx to install only xxx component
# it has nothing to do with find_package, but only with install command
# https://stackoverflow.com/questions/9190098/for-cmakes-install-command-what-can-the-component-argument-do

# for multi configuration build system:
# https://discourse.cmake.org/t/install-with-export-and-multiple-configurations/1292

include(${PROJECT_SOURCE_DIR}/cmake/igcclib-component-helpers.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/igcclib-dependency.cmake)
set(component core)

create_component_interface(${component} ${master_name})

# find dependencies
# require eigen3, spdlog, nanoflann
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(nanoflann REQUIRED)

target_link_libraries(${component} INTERFACE
    Eigen3::Eigen
    spdlog::spdlog
    nanoflann::nanoflann
)

set(required_comps ${${component}_REQUIRED_COMPONENTS})
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_COMPONENTS ${required_comps}
    REQUIRED_LIBRARIES Eigen3 spdlog nanoflann
)