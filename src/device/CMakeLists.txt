include(${PROJECT_SOURCE_DIR}/cmake/igcclib-component-helpers.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/igcclib-dependency.cmake)
set(component device)

# core library is header only
set(src_files  
    ${CMAKE_CURRENT_LIST_DIR}/DefaultFrameReader.cpp
    ${CMAKE_CURRENT_LIST_DIR}/FrameRecorder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/GyroAccelSensor.cpp
)
set_source_files_properties(${src_files} PROPERTIES LANGUAGE CXX)

create_component_library(COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    SRC_FILES ${src_files})

# setup component linking
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
target_link_libraries(${component} PUBLIC 
    ${master_name}::core ${master_name}::vision ${master_name}::io
)

# create install rules for the component
set(required_comps ${${component}_REQUIRED_COMPONENTS})
create_component_install_rules(
    COMPONENT ${component} 
    MASTER_NAME ${master_name} 
    REQUIRED_COMPONENTS ${required_comps}
)