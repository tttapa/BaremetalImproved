#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "time" for configuration ""
set_property(TARGET time APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(time PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libtime.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS time )
list(APPEND _IMPORT_CHECK_FILES_FOR_time "${_IMPORT_PREFIX}/lib/libtime.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
