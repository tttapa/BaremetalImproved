# Create library target and specify the implementation files to compile.
add_library(time 
    src/Time.cpp
)

# Set target include directories 
#   These are the folders where the compiler will search for included header 
#   files.
#   The include files in the `include` folder are part of the public API of the
#   library, users of the library need to be able to include them in their code. 
#   Any header files in the `src` directory are private, they are only required 
#   to compile the implementation files of the library, users of the library
#   should not include them.
target_include_directories(time
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Tell CMake to compile this library with C++17 features enabled
# target_compile_features(time PUBLIC cxx_std_17)

# Link the library with its dependencies.
#   This manages both the include directories of the dependencies, as well as  
#   the actual linking of the other libraries.
#   Libraries that are used in the public header files should be public,
#   libraries that are only used in the implementation files should be private.
target_link_libraries(time 
    PUBLIC 
        real_t
    PRIVATE
        src-vivado
)

# Run CMake again in the `test` folder, to discover the CMakeLists.txt file that
# adds the tests.
# add_subdirectory(test) 

install(TARGETS time
        EXPORT time
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include)
install(EXPORT time DESTINATION lib)
install(
FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Time.hpp
DESTINATION include)