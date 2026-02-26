cmake_minimum_required(VERSION 3.14)
project(HRC-PalletizingPlanner LANGUAGES CXX)

add_compile_options(-fPIC -Wall -Wextra -Wno-unused-parameter -Wno-attributes)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# 文档的输出结果
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin) 

add_subdirectory(test)





