#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "g3log" for configuration "None"
set_property(TARGET g3log APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(g3log PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/libg3log.so.2.1.2-122"
  IMPORTED_SONAME_NONE "libg3log.so.2.1.2-122"
  )

list(APPEND _IMPORT_CHECK_TARGETS g3log )
list(APPEND _IMPORT_CHECK_FILES_FOR_g3log "${_IMPORT_PREFIX}/lib/libg3log.so.2.1.2-122" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
