#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "performance_test_fixture::performance_test_fixture" for configuration ""
set_property(TARGET performance_test_fixture::performance_test_fixture APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(performance_test_fixture::performance_test_fixture PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libperformance_test_fixture.dylib"
  IMPORTED_SONAME_NOCONFIG "@rpath/libperformance_test_fixture.dylib"
  )

list(APPEND _cmake_import_check_targets performance_test_fixture::performance_test_fixture )
list(APPEND _cmake_import_check_files_for_performance_test_fixture::performance_test_fixture "${_IMPORT_PREFIX}/lib/libperformance_test_fixture.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
