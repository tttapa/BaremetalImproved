add_library(main-vivado INTERFACE)

target_include_directories(main-vivado 
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)