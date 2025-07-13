# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_myrobot_utils_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED myrobot_utils_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(myrobot_utils_FOUND FALSE)
  elseif(NOT myrobot_utils_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(myrobot_utils_FOUND FALSE)
  endif()
  return()
endif()
set(_myrobot_utils_CONFIG_INCLUDED TRUE)

# output package information
if(NOT myrobot_utils_FIND_QUIETLY)
  message(STATUS "Found myrobot_utils: 0.0.0 (${myrobot_utils_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'myrobot_utils' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${myrobot_utils_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(myrobot_utils_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${myrobot_utils_DIR}/${_extra}")
endforeach()
