#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rover_msgs::lib_exec" for configuration ""
set_property(TARGET rover_msgs::lib_exec APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rover_msgs::lib_exec PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblib_exec.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS rover_msgs::lib_exec )
list(APPEND _IMPORT_CHECK_FILES_FOR_rover_msgs::lib_exec "${_IMPORT_PREFIX}/lib/liblib_exec.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
