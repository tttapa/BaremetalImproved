# Create target and set properties
add_library(src-vivado INTERFACE)

# Set target properties
target_include_directories(src-vivado
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS src-vivado
        EXPORT src-vivado
        INCLUDES DESTINATION include)
install(EXPORT src-vivado DESTINATION /tmp)