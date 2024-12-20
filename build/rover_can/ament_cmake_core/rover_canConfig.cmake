# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rover_can_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rover_can_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rover_can_FOUND FALSE)
  elseif(NOT rover_can_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rover_can_FOUND FALSE)
  endif()
  return()
endif()
set(_rover_can_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rover_can_FIND_QUIETLY)
  message(STATUS "Found rover_can: 0.0.0 (${rover_can_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rover_can' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rover_can_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rover_can_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rover_can_DIR}/${_extra}")
endforeach()
