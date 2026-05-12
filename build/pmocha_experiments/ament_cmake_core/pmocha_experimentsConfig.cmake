# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pmocha_experiments_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pmocha_experiments_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pmocha_experiments_FOUND FALSE)
  elseif(NOT pmocha_experiments_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pmocha_experiments_FOUND FALSE)
  endif()
  return()
endif()
set(_pmocha_experiments_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pmocha_experiments_FIND_QUIETLY)
  message(STATUS "Found pmocha_experiments: 0.1.0 (${pmocha_experiments_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pmocha_experiments' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pmocha_experiments_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pmocha_experiments_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pmocha_experiments_DIR}/${_extra}")
endforeach()
