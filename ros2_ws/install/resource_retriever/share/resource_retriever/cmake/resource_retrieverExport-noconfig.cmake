#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "resource_retriever::resource_retriever" for configuration ""
set_property(TARGET resource_retriever::resource_retriever APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(resource_retriever::resource_retriever PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "ament_index_cpp::ament_index_cpp"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libresource_retriever.dylib"
  IMPORTED_SONAME_NOCONFIG "@rpath/libresource_retriever.dylib"
  )

list(APPEND _cmake_import_check_targets resource_retriever::resource_retriever )
list(APPEND _cmake_import_check_files_for_resource_retriever::resource_retriever "${_IMPORT_PREFIX}/lib/libresource_retriever.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
