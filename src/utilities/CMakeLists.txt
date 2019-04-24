# Create target and set properties
add_library(utilities INTERFACE)

# Set target properties
target_include_directories(utilities
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(utilities INTERFACE cxx_std_17)

if(NOT CMAKE_CROSSCOMPILING)
add_subdirectory(test)
endif()