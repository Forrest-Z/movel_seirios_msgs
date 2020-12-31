#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mongo::mongoc_shared" for configuration "None"
set_property(TARGET mongo::mongoc_shared APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(mongo::mongoc_shared PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/libmongoc-1.0.so.0.0.0"
  IMPORTED_SONAME_NONE "libmongoc-1.0.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS mongo::mongoc_shared )
list(APPEND _IMPORT_CHECK_FILES_FOR_mongo::mongoc_shared "${_IMPORT_PREFIX}/lib/libmongoc-1.0.so.0.0.0" )

# Import target "mongo::mongoc_static" for configuration "None"
set_property(TARGET mongo::mongoc_static APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(mongo::mongoc_static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NONE "C"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/libmongoc-static-1.0.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS mongo::mongoc_static )
list(APPEND _IMPORT_CHECK_FILES_FOR_mongo::mongoc_static "${_IMPORT_PREFIX}/lib/libmongoc-static-1.0.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
