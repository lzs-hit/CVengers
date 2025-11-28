# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cvengers_challenge_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cvengers_challenge_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cvengers_challenge_FOUND FALSE)
  elseif(NOT cvengers_challenge_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cvengers_challenge_FOUND FALSE)
  endif()
  return()
endif()
set(_cvengers_challenge_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cvengers_challenge_FIND_QUIETLY)
  message(STATUS "Found cvengers_challenge: 0.0.0 (${cvengers_challenge_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cvengers_challenge' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cvengers_challenge_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cvengers_challenge_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${cvengers_challenge_DIR}/${_extra}")
endforeach()
