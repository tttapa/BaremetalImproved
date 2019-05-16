# Create library target and specify the implementation files to compile.
add_library(communication
    src/GetLogData.cpp)

# Set target include directories 
#   These are the folders where the compiler will search for included header 
#   files.
#   The include files in the `include` folder are part of the public API of the
#   library, users of the library need to be able to include them in their code. 
#   Any header files in the `src` directory are private, they are only required 
#   to compile the implementation files of the library, users of the library
#   should not include them.
target_include_directories(shared
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Tell CMake to compile this library with C++17 features enabled
# target_compile_features(shared PUBLIC cxx_std_17)

# Link the library with its dependencies.
#   This manages both the include directories of the dependencies, as well as  
#   the actual linking of the other libraries.
#   Libraries that are used in the public header files should be public,
#   libraries that are only used in the implementation files should be private.
target_link_libraries(shared 
    PRIVATE
        instances
        misc
        time
)

# if(NOT CMAKE_CROSSCOMPILING)
#     # Run CMake again in the `test` folder, to discover the CMakeLists.txt file 
#     # that adds the tests.
#     add_subdirectory(test) 
# endif()

install(TARGETS shared
        EXPORT shared
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include)
install(EXPORT shared DESTINATION lib)
install(
FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Array.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ArrayHelper.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/BaremetalCommunicationDef.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/EulerAngles.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/GetLogData.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/HouseholderQR.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LeastSquares.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LoggerStructs.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Matrix.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/MatrixHelpers.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Quaternion.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/real_t.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/SharedStruct.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Square.hpp
DESTINATION include)