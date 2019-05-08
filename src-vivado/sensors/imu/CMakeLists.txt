add_library(imu-vivado INTERFACE)

target_include_directories(imu-vivado
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS imu-vivado EXPORT imu-vivado) # TODO
install(EXPORT imu-vivado DESTINATION /tmp)