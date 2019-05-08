add_library(buzzer-vivado INTERFACE)

target_include_directories(buzzer-vivado
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)