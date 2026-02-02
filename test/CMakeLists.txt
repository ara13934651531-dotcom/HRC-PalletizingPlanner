cmake_minimum_required(VERSION 3.0.2)
project(testHansAlgorithm)

add_compile_options(-fPIC)

# 使用C++17标准以支持optional等现代特性
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR}/HRCInterface)
include_directories(${CMAKE_SOURCE_DIR}/include)

include_directories(
  /usr/include/eigen3
)

link_directories(${CMAKE_SOURCE_DIR}/lib)

aux_source_directory(./src/kinematic srcs)

# 原有的C++测试程序 - 碰撞检测性能测试
add_executable(testCollisionDetectionTime
  ${CMAKE_SOURCE_DIR}/test/testCollisionDetectionTime.cpp
)
target_link_libraries(testCollisionDetectionTime
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
)

# 码垛运动规划器测试程序
add_executable(testPalletizingPlanner
  ${CMAKE_SOURCE_DIR}/test/testPalletizingPlanner.cpp
)
target_link_libraries(testPalletizingPlanner
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
)

# 高性能规划器测试程序
add_executable(testHighPerformance
  ${CMAKE_SOURCE_DIR}/test/testHighPerformance.cpp
)
target_link_libraries(testHighPerformance
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
  pthread
)

# 性能基准测试程序
add_executable(testPerformanceBenchmark
  ${CMAKE_SOURCE_DIR}/test/testPerformanceBenchmark.cpp
)
target_link_libraries(testPerformanceBenchmark
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
  pthread
)

# 鲁棒性和泛化测试程序
add_executable(testRobustnessValidation
  ${CMAKE_SOURCE_DIR}/test/testRobustnessValidation.cpp
)
target_link_libraries(testRobustnessValidation
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
  pthread
)