#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mcap_vendor::mcap" for configuration ""
set_property(TARGET mcap_vendor::mcap APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(mcap_vendor::mcap PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmcap.dylib"
  IMPORTED_SONAME_NOCONFIG "@rpath/libmcap.dylib"
  )

list(APPEND _cmake_import_check_targets mcap_vendor::mcap )
list(APPEND _cmake_import_check_files_for_mcap_vendor::mcap "${_IMPORT_PREFIX}/lib/libmcap.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
