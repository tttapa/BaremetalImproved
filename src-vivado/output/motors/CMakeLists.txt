add_library(motors-vivado INTERFACE)

target_include_directories(motors-vivado
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)