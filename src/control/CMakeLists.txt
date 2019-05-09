execute_process(COMMAND which matlab
                OUTPUT_VARIABLE MATLAB_EXEC)

if (NOT MATLAB_EXEC STREQUAL "")

file(GLOB MATLAB_CODEGEN_SCRIPTS_DEP ${CMAKE_CURRENT_SOURCE_DIR}/codegen/*.m)

add_custom_command(
    OUTPUT
        src-generated/AttitudeCodegen.cpp
        src-generated/AltitudeCodegen.cpp
        src-generated/PositionCodegen.cpp
    COMMAND
        ${CMAKE_CURRENT_SOURCE_DIR}/codegen/Codegen.sh
    DEPENDS
        ${MATLAB_CODEGEN_SCRIPTS_DEP}
        templates/AttitudeCodegenTemplate.cpp
        templates/AltitudeCodegenTemplate.cpp
        templates/PositionCodegenTemplate.cpp
)

else()
message(WARNING "MATLAB is not available. Control codegen will not be run")
add_custom_command(
    OUTPUT
        src-generated/AttitudeCodegen.cpp
        src-generated/AltitudeCodegen.cpp
        src-generated/PositionCodegen.cpp
    COMMAND
        make -C ${CMAKE_CURRENT_SOURCE_DIR}/codegen || (cmake -E cmake_echo_color --red --bold "Error: Codegen out of date" && exit 1)
    DEPENDS
        ${MATLAB_CODEGEN_SCRIPTS_DEP}
        templates/AttitudeCodegenTemplate.cpp
        templates/AltitudeCodegenTemplate.cpp
        templates/PositionCodegenTemplate.cpp
)
endif()

# Create library target and specify the implementation files to compile.
add_library(control
    src/Attitude.cpp
    src-generated/AttitudeCodegen.cpp  
    src/Altitude.cpp
    src-generated/AltitudeCodegen.cpp  
    src/Position.cpp
    src-generated/PositionCodegen.cpp
    src/TiltCorrection.cpp
    src/Autonomous.cpp
    src/InputBias/InputBias.cpp
)

# Set target include directories 
#   These are the folders where the compiler will search for included header 
#   files.
#   The include files in the `include` folder are part of the public API of the
#   library, users of the library need to be able to include them in their code. 
#   Any header files in the `src` directory are private, they are only required 
#   to compile the implementation files of the library, users of the library
#   should not include them.
target_include_directories(control
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Tell CMake to compile this library with C++17 features enabled
# target_compile_features(control PUBLIC cxx_std_17)

# Link the library with its dependencies.
#   This manages both the include directories of the dependencies, as well as  
#   the actual linking of the other libraries.
#   Libraries that are used in the public header files should be public,
#   libraries that are only used in the implementation files should be private.
target_link_libraries(control 
    PUBLIC
        misc
        quaternion
        utilities
        vivado-types
    PRIVATE
        communication
        instances
        time
        src-vivado
)

if(NOT CMAKE_CROSSCOMPILING)
    # Run CMake again in the `test` folder, to discover the CMakeLists.txt file 
    # that adds the tests.
    add_subdirectory(test) 
endif()

install(TARGETS control
        EXPORT control
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include)
install(EXPORT control DESTINATION lib)
install(
FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Altitude.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Attitude.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Autonomous.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Position.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/TiltCorrection.hpp
DESTINATION include)
install(
FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/InputBias/InputBias.hpp
DESTINATION include/InputBias)