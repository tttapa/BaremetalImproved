# add_subdirectory(utilities)
# add_subdirectory(matrix)
# add_subdirectory(quaternion)
# add_subdirectory(tilt-correction)
# add_subdirectory(init)
# add_subdirectory(control)
# add_subdirectory(communication)

# Hello World! example
add_subdirectory(greet) # TODO
add_subdirectory(test-vivado) # TODO

include(${CMAKE_SOURCE_DIR}/shared-linux/cmake/shared-linux.cmake)

# add_subdirectory(matrix)
# add_subdirectory(utilities)
# add_subdirectory(quaternion)
# add_subdirectory(communication)
add_subdirectory(misc)
add_subdirectory(control)
add_subdirectory(instances)
add_subdirectory(main)
add_subdirectory(misc)
add_subdirectory(shared)
add_subdirectory(time)

# add_custom_target(vivado-link
#         COMMAND ar x $<TARGET_FILE:greet>
#         COMMAND ar x $<TARGET_FILE:test-vivado>
#         COMMAND ar qc libvivado-link.a *.obj
#         COMMAND rm *.obj
#         WORKING_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
#         DEPENDS greet test-vivado
# )
