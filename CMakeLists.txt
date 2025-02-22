cmake_minimum_required(VERSION 3.25)

project(test)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

option(enable_chrono_tests "Enable tests for Chrono" ON)
option(enable_cgal_tests "Enable tests for CGAL" ON)

if(enable_cgal_tests)
    add_subdirectory(test_cgal)
endif(enable_cgal_tests)

if(enable_chrono_tests)
    add_subdirectory(test_chrono)
endif(enable_chrono_tests)
