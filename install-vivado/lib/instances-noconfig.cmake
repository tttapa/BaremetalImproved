#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "instances" for configuration ""
set_property(TARGET instances APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(instances PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libinstances.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS instances )
list(APPEND _IMPORT_CHECK_FILES_FOR_instances "${_IMPORT_PREFIX}/lib/libinstances.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
