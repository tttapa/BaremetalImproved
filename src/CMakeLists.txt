# add_subdirectory(utilities)
# add_subdirectory(matrix)
# add_subdirectory(quaternion)
# add_subdirectory(tilt-correction)
# add_subdirectory(init)
# add_subdirectory(control)
# add_subdirectory(communication)

# Hello World! example
add_subdirectory(greet)
add_subdirectory(test-vivado)

add_subdirectory(matrix)
add_subdirectory(utilities)
add_subdirectory(quaternion)
add_subdirectory(misc)
add_subdirectory(control)
add_subdirectory(globals)
add_subdirectory(time)

add_custom_target(vivado-link
        COMMAND ar x $<TARGET_FILE:greet>
        COMMAND ar x $<TARGET_FILE:test-vivado>
        COMMAND ar qc libvivado-link.a *.obj
        COMMAND rm *.obj
        WORKING_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
        DEPENDS greet test-vivado
)

## C doesn't have namespaces. You can't include C++ headers in a C implementation file
# ah in vivado? Globals.c it includes RC.hpp, which has namespaces in it
# so it should be Globals.cpp then? , yeah, that's probably the easies way
# you 