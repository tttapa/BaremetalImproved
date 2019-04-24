# Create library target and specify the implementation files to compile.
add_library(control
    src/AltitudeCodegen.cpp  
    src/AttitudeCodegen.cpp  
    src/Autonomous.cpp
    src/Position.cpp
    src/TiltCorrection.cpp
    src/Altitude.cpp
    src/Attitude.cpp
    src/PositionCodegen.cpp
    src/QRFSM.cpp
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
target_compile_features(control PUBLIC cxx_std_17)

# Link the library with its dependencies.
#   This manages both the include directories of the dependencies, as well as  
#   the actual linking of the other libraries.
#   Libraries that are used in the public header files should be public,
#   libraries that are only used in the implementation files should be private.
target_link_libraries(control 
    PUBLIC 
        quaternion
        utilities
    PRIVATE
        configuration
        globals
        time
)

if(NOT CMAKE_CROSSCOMPILING)
    # Run CMake again in the `test` folder, to discover the CMakeLists.txt file 
    # that adds the tests.
    add_subdirectory(test) 
endif()

# oké, dus ik heb gewoon de CMakelists van greet gekopieerd
# ik ga nu alle files voor control toevoegen, en een nieuw target maken
# 
# welke dependencies heeft control allemaal? Quaternion?
# quaternion, matrix, real_t, globals/SoftwareConstants.hpp, globals/Globals.h
# and time/time.hpp
# 
# oké, nu moeten we dus alle includes zien te fiksen :p normaal doe je dat
# terwijl je programmeert, dan moet je dat achteraf niet allemaal gaan zoeken :D
#
# oh shiit, that one lemme check if it's implemented at all...
# yeah you need misc/configuration/Configuration.hpp too!
# Do I rename it? It's lowercase 
#
# all files should be upper case except for real_t - oké