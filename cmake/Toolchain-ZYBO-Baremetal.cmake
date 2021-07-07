# cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-ZYBO-Baremetal.cmake ..

# The Generic system name is used for embedded targets (targets without OS) in
# CMake
SET(CMAKE_SYSTEM_NAME          Generic)
SET(CMAKE_SYSTEM_PROCESSOR     ARM)

# Specify the cross compiler
SET(CMAKE_C_COMPILER   arm-eabi-gcc)
SET(CMAKE_CXX_COMPILER arm-eabi-g++)

# Search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Set the CMake C and C++ flags (which should also be used by the assembler!)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpu=vfpv3" )
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=hard" )
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-a9" )
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections" )
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-use-cxa-atexit" )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=vfpv3" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfloat-abi=hard" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a9" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-use-cxa-atexit" )

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "" FORCE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" CACHE STRING "" FORCE)

SET(CMAKE_EXE_LINKER_FLAGS "-specs=${CMAKE_SOURCE_DIR}/src-vivado/Xilinx.spec"
CACHE STRING "" FORCE)