# Bug in ARM cross-compiler: 
# https://bugs.launchpad.net/gcc-arm-embedded/+bug/1698539
set(CMAKE_NO_SYSTEM_FROM_IMPORTED TRUE)

include(${CMAKE_SOURCE_DIR}/shared-linux/cmake/shared-linux.cmake)

add_subdirectory(control)
add_subdirectory(instances)
add_subdirectory(logger)
add_subdirectory(main)
add_subdirectory(misc)
add_subdirectory(time)
add_subdirectory(utilities)
