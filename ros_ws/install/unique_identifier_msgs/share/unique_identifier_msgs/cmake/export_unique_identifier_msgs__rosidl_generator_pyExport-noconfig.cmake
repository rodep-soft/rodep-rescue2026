#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "unique_identifier_msgs::unique_identifier_msgs__rosidl_generator_py" for configuration ""
set_property(TARGET unique_identifier_msgs::unique_identifier_msgs__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(unique_identifier_msgs::unique_identifier_msgs__rosidl_generator_py PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "unique_identifier_msgs::unique_identifier_msgs__rosidl_generator_c;Python3::Python;unique_identifier_msgs::unique_identifier_msgs__rosidl_typesupport_c"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libunique_identifier_msgs__rosidl_generator_py.so"
  IMPORTED_SONAME_NOCONFIG "libunique_identifier_msgs__rosidl_generator_py.so"
  )

list(APPEND _cmake_import_check_targets unique_identifier_msgs::unique_identifier_msgs__rosidl_generator_py )
list(APPEND _cmake_import_check_files_for_unique_identifier_msgs::unique_identifier_msgs__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libunique_identifier_msgs__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
