# # Create library target and specify the implementation files to compile.
# add_library(utilites

# )

# Set target include directories 
#   These are the folders where the compiler will search for included header 
#   files.
#   The include files in the `include` folder are part of the public API of the
#   library, users of the library need to be able to include them in their code. 
#   Any header files in the `src` directory are private, they are only required 
#   to compile the implementation files of the library, users of the library
#   should not include them.
target_include_directories(utilities
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Tell CMake to compile this library with C++17 features enabled
# target_compile_features(utilities PUBLIC cxx_std_17)

# install(TARGETS utilities
#         EXPORT utilities
#         ARCHIVE DESTINATION lib
#         INCLUDES DESTINATION include)
# install(EXPORT utilities DESTINATION lib)
install(
FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/EulerAngles.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Quaternion.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Square.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Vector.hpp
DESTINATION include)