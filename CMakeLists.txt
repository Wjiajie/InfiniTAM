##########################################
# Top-level CMakeLists.txt for InfiniTAM #
##########################################

cmake_minimum_required(VERSION 3.8)

project(InfiniTAM)

#################################
# CMake policies #
#################################
if (POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW)
endif ()
if (POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif ()

#################################
# Enforce C++ language standards
#################################
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#################################
# Specify the CMake module path #
#################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

#################################
# Add additional compiler flags #
#################################

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
endif ()


if (NOT MSVC_IDE)
    set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
    set(CMAKE_C_FLAGS_RELEASE "-O3 -march=native ${CFLAGS_WARN} ${CMAKE_C_FLAGS_RELEASE}")
    set(CMAKE_C_FLAGS_DEBUG "-g -march=native ${CFLAGS_WARN} ${CMAKE_C_FLAGS_DEBUG}")
    #SET(CMAKE_CXX_FLAGS "-g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
endif ()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#########################
# Suppress auto-linking #
#########################

if (MSVC_IDE)
    add_definitions(-DUSING_CMAKE=1)
    add_compile_options("/MP")
endif ()


file(GLOB_RECURSE CMAKE_SOURCES "cmake/*")
add_custom_target(cmake SOURCES ${CMAKE_SOURCES})
source_group(CMake FILES ${CMAKE_SOURCES})

######################
# Add subdirectories #
######################

add_subdirectory(Apps)
add_subdirectory(FernRelocLib)
add_subdirectory(InputSource)
add_subdirectory(ITMLib)
add_subdirectory(MiniSlamGraphLib)
add_subdirectory(ORUtils)
add_subdirectory(Tests)