include(${CMAKE_SOURCE_DIR}/cmake/component-helpers.cmake)
include(FindPackageHandleStandardArgs)

set(component visualization)

set(src_files  
    ${CMAKE_CURRENT_LIST_DIR}/soft_renderer/softlit/AnalysisShader.cpp
    ${CMAKE_CURRENT_LIST_DIR}/soft_renderer/softlit/Display.cpp
    ${CMAKE_CURRENT_LIST_DIR}/soft_renderer/softlit/Primitive.cpp
    ${CMAKE_CURRENT_LIST_DIR}/soft_renderer/softlit/Rasterizer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/soft_renderer/softlit/Texture.cpp
    ${CMAKE_CURRENT_LIST_DIR}/soft_renderer/softlit.cpp
)
set_source_files_properties(${src_files} PROPERTIES LANGUAGE CXX)

create_component_library(${component} ${master_name} ${src_files})

# setup component linking
# find_package(Eigen3 REQUIRED)
# find_package(OpenCV REQUIRED)
# set(dep_libs Eigen3::Eigen ${OpenCV_LIBS})

target_link_libraries(${component} PUBLIC 
    ${master_name}::core ${master_name}::extern ${master_name}::geometry
)

# create install rules for the component
create_component_install_rules(${component} ${master_name} "core;extern;geometry")