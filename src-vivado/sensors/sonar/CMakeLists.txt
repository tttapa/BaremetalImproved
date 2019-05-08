add_library(sonar-vivado INTERFACE)

target_include_directories(sonar-vivado
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)