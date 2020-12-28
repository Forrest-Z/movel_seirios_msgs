if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND "${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 6.0.0)
  message(FATAL_ERROR "sml requires GCC >= 6.0.0")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND "${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 3.5.0)
  message(FATAL_ERROR "sml requires Clang >= 3.5.0")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.0)
  message(FATAL_ERROR "sml requires Visual Studio 14 2015 at least")
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++14 HAS_CXX14_FLAG)
check_cxx_compiler_flag(-std=c++17 HAS_CXX17_FLAG)
check_cxx_compiler_flag(-std=c++2a HAS_CXX20_FLAG)

if(HAS_CXX20_FLAG)
  set(CMAKE_CXX_STANDARD 20)
elseif(HAS_CXX17_FLAG)
  set(CMAKE_CXX_STANDARD 17)
elseif(HAS_CXX14_FLAG)
  set(CMAKE_CXX_STANDARD 14)
else()
  message(FATAL_ERROR "sml requires c++14")
endif()

if(NOT (DEFINED CMAKE_CXX_STANDARD) OR CMAKE_CXX_STANDARD STREQUAL "" OR CMAKE_CXX_STANDARD LESS 14)
    message(FATAL_ERROR "sml requires c++14")
endif()

set(IS_COMPILER_GCC_LIKE FALSE)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )

  set(IS_COMPILER_GCC_LIKE TRUE)
endif()
