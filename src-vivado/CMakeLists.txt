# Create target and set properties
add_library(src-vivado INTERFACE)

# Set target properties
target_include_directories(src-vivado
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(src-vivado INTERFACE cxx_std_17)


# Link the library with its dependencies.
#   This manages both the include directories of the dependencies, as well as  
#   the actual linking of the other libraries.
#   Libraries that are used in the public header files should be public,
#   libraries that are only used in the implementation files should be private.
target_link_libraries(src-vivado
    PUBLIC
        src-shared
)

if(NOT CMAKE_CROSSCOMPILING)
add_subdirectory(test)
endif()

install(TARGETS src-vivado
        EXPORT src-vivado
        INCLUDES DESTINATION include)
install(EXPORT src-vivado DESTINATION lib)
install(
FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PublicHardwareConstants.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/output/Buzzer.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/output/Motors.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/output/WPT.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/platform/AxiGpio.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/platform/Interrupt.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/platform/Platform.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sensors/AHRS.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sensors/IMU.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sensors/RC.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sensors/Sonar.hpp
DESTINATION include)