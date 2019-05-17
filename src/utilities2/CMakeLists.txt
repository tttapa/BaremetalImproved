# Create library target and specify the implementation files to compile.
add_library(utilities2 INTERFACE)

# Set target include directories 
#   These are the folders where the compiler will search for included header 
#   files.
#   The include files in the `include` folder are part of the public API of the
#   library, users of the library need to be able to include them in their code. 
#   Any header files in the `src` directory are private, they are only required 
#   to compile the implementation files of the library, users of the library
#   should not include them.
target_include_directories(utilities2 INTERFACE
$<INSTALL_INTERFACE:include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS utilities2
EXPORT utilities2
INCLUDES DESTINATION include)
install(EXPORT utilities2 DESTINATION /tmp)