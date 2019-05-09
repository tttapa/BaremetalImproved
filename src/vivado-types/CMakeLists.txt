# Create target and set properties
add_library(vivado-types INTERFACE)

# Set target properties
target_include_directories(vivado-types
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(vivado-types INTERFACE cxx_std_17)

install(TARGETS vivado-types
        EXPORT vivado-types
        INCLUDES DESTINATION include)
install(EXPORT vivado-types DESTINATION lib)
install(
FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/OutputTypes.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/SensorTypes.hpp
DESTINATION include)