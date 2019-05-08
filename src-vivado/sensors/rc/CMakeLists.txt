add_library(rc-vivado INTERFACE)

target_include_directories(rc-vivado
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)