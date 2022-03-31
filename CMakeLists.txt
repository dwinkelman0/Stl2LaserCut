# Copyright 2022 by Daniel Winkelman. All rights reserved.

cmake_minimum_required (VERSION 3.16)

set (CMAKE_BUILD_TYPE Debug)
set (CMAKE_CXX_STANDARD 20)

project (Stl2LaserCut CXX)
set (Target_lib stl2lasercut-lib)
set (Target_client stl2lasercut-client)
# set (Target_tests stl2lasercut-tests)

include (FetchContent)
FetchContent_Declare (googletest URL https://github.com/google/googletest/archive/refs/tags/release-1.11.0.zip)
FetchContent_MakeAvailable (googletest)
enable_testing ()

include_directories (src)
include_directories (${googletest_SOURCE_DIR}/googletest/include)

file (GLOB LIB_SOURCES src/*.cc)
add_library (${Target_lib} ${LIB_SOURCES})

file (GLOB CLIENT_SOURCES client/*.cc)
add_executable (${Target_client} ${CLIENT_SOURCES})
target_link_libraries (${Target_client} ${Target_lib})

# file (GLOB TESTS_SOURCES tests/*.cc)
# add_executable (${Target_tests} ${TESTS_SOURCES})
# target_link_libraries (${Target_tests} ${Target_lib} gtest_main)

# include (GoogleTest)
# gtest_discover_tests (${Target_tests})
