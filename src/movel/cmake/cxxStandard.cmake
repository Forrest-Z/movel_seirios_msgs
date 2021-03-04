if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND "${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 6.0.0)
  message(FATAL_ERROR "${PROJECT_NAME} requires GCC >= 6.0.0")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND "${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 3.5.0)
  message(FATAL_ERROR "${PROJECT_NAME} requires Clang >= 3.5.0")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.0)
  message(FATAL_ERROR "${PROJECT_NAME} requires Visual Studio 14 2015 at least")
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
    message(FATAL_ERROR "${PROJECT_NAME} requires c++14")
endif()

if(NOT (DEFINED CMAKE_CXX_STANDARD) OR CMAKE_CXX_STANDARD STREQUAL "" OR CMAKE_CXX_STANDARD LESS 14)
    message(FATAL_ERROR "${PROJECT_NAME} requires c++14")
endif()

set(IS_COMPILER_GCC_LIKE FALSE)
set(CXX_STANDARD_REQUIRED ON)
message("${BoldCyan}CMAKE_CXX_STANDARD set to ${CMAKE_CXX_STANDARD}\n\n${ColourReset}")

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Release by default
# Turn on Debug with "-DCMAKE_BUILD_TYPE=Debug"
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set compiler specific settings
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
	set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
	set(CMAKE_CXX_FLAGS_RELEASE "-O3")
	set(CMAKE_C_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
	set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wno-deprecated-register -Qunused-arguments -fcolor-diagnostics")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-register -Qunused-arguments -fcolor-diagnostics")
  	set(IS_COMPILER_GCC_LIKE TRUE)
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
	set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
   	set(CMAKE_CXX_FLAGS_RELEASE "-O3")
	set(CMAKE_C_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
	set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wno-deprecated-declarations -ftemplate-backtrace-limit=0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -ftemplate-backtrace-limit=0")
   	set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_DEBUG} --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
   	set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --coverage")
   	set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} --coverage")
  	set(IS_COMPILER_GCC_LIKE TRUE)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "^MSVC$")
   	add_definition("-D _USE_MATH_DEFINES /bigobj /wd4305 /wd4244 /MP")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
  	set(IS_COMPILER_GCC_LIKE TRUE)
endif()
