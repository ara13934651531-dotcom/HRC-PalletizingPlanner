cmake_minimum_required(VERSION 3.14)
project(HRC-PalletizingPlanner LANGUAGES CXX)

add_compile_options(-fPIC -Wall -Wextra -Wno-unused-parameter -Wno-attributes)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# 输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin) 

# Eigen3
find_package(Eigen3 3.3 QUIET)
if(NOT EIGEN3_FOUND)
    # 回退: 直接设置 include 路径
    set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
    if(EXISTS ${EIGEN3_INCLUDE_DIR})
        message(STATUS "Eigen3: using fallback path ${EIGEN3_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "Eigen3 not found! Install with: sudo apt install libeigen3-dev")
    endif()
endif()

# CTest
enable_testing()

add_subdirectory(test)

