#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rclc_parameter::rclc_parameter" for configuration ""
set_property(TARGET rclc_parameter::rclc_parameter APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rclc_parameter::rclc_parameter PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librclc_parameter.a"
  )

list(APPEND _cmake_import_check_targets rclc_parameter::rclc_parameter )
list(APPEND _cmake_import_check_files_for_rclc_parameter::rclc_parameter "${_IMPORT_PREFIX}/lib/librclc_parameter.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
