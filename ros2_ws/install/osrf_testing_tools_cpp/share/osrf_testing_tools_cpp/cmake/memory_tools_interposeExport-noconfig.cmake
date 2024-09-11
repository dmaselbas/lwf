#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "osrf_testing_tools_cpp::memory_tools_interpose" for configuration ""
set_property(TARGET osrf_testing_tools_cpp::memory_tools_interpose APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(osrf_testing_tools_cpp::memory_tools_interpose PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmemory_tools_interpose.dylib"
  IMPORTED_SONAME_NOCONFIG "@rpath/libmemory_tools_interpose.dylib"
  )

list(APPEND _cmake_import_check_targets osrf_testing_tools_cpp::memory_tools_interpose )
list(APPEND _cmake_import_check_files_for_osrf_testing_tools_cpp::memory_tools_interpose "${_IMPORT_PREFIX}/lib/libmemory_tools_interpose.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
