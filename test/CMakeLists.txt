cmake_minimum_required(VERSION 3.14)
project(HRC-PalletizingPlanner-Tests)

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

# ============================================================================
# S50 碰撞仿真 (libCmpRML + HRC碰撞检测)
# ============================================================================

# RML API探针测试 (CoDeSys函数块验证)
add_executable(testRMLProbe
  ${CMAKE_SOURCE_DIR}/test/testRMLProbe.cpp
)
target_include_directories(testRMLProbe PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(testRMLProbe
  ${CMAKE_SOURCE_DIR}/lib/libCmpHansFreeDriveMotion.so
  ${CMAKE_SOURCE_DIR}/lib/libCmpHansAlgorithmLib.so
  ${CMAKE_SOURCE_DIR}/lib/libwl.so
  ${CMAKE_SOURCE_DIR}/libCmpRML.so
  stdc++
  m
  dl
)

# 碰撞仿真测试 (内置S曲线轨迹 + HRC实时碰撞检测)
# 注: libCmpRML.so需CoDeSys PLC运行时,此处使用功能等价的TimeParameterization.hpp
add_executable(testCollisionSimulation
  ${CMAKE_SOURCE_DIR}/test/testCollisionSimulation.cpp
)
target_link_libraries(testCollisionSimulation
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
  pthread
)

# 码垛场景仿真 (内置S曲线 + 碰撞检测 + TSP优化 + TCP记录)
add_executable(testPalletizingScenarioRML
  ${CMAKE_SOURCE_DIR}/test/testPalletizingScenarioRML.cpp
)
target_link_libraries(testPalletizingScenarioRML
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
  pthread
)

# S曲线碰撞仿真 (内置S曲线轨迹 + HRC碰撞检测 + STL可视化数据)
add_executable(testS50CollisionRML
  ${CMAKE_SOURCE_DIR}/test/testS50CollisionRML.cpp
)
target_include_directories(testS50CollisionRML PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(testS50CollisionRML
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
  pthread
)

# S曲线码垛场景仿真 (内置S曲线 + 碰撞检测 + TSP + 12位码垛)
add_executable(testS50PalletizingRML
  ${CMAKE_SOURCE_DIR}/test/testS50PalletizingRML.cpp
)
target_include_directories(testS50PalletizingRML PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(testS50PalletizingRML
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
  pthread
)

# ============================================================================
# S50 v3.0 — libHRCInterface.so (独立碰撞库, dlopen动态加载)
# ============================================================================

# 碰撞仿真 v3.0 (.so碰撞 + S曲线 + 分层计时)
add_executable(testS50CollisionSO
  ${CMAKE_SOURCE_DIR}/test/testS50CollisionSO.cpp
)
target_include_directories(testS50CollisionSO PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
# 注: 不链接静态HRC库! 使用dlopen动态加载 libHRCInterface.so
target_link_libraries(testS50CollisionSO
  stdc++
  m
  pthread
  dl
)

# 码垛仿真 v3.0 (.so碰撞 + Free-TCP RRT* + 分层计时)
add_executable(testS50PalletizingSO
  ${CMAKE_SOURCE_DIR}/test/testS50PalletizingSO.cpp
)
target_include_directories(testS50PalletizingSO PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(testS50PalletizingSO
  stdc++
  m
  pthread
  dl
)

# 系统性轨迹最优性测试 (Free-TCP, 全面验证)
add_executable(testTrajectoryOptimality
  ${CMAKE_SOURCE_DIR}/test/testTrajectoryOptimality.cpp
)
target_include_directories(testTrajectoryOptimality PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(testTrajectoryOptimality
  stdc++
  m
  pthread
  dl
)

# ============================================================================
# Examples
# ============================================================================

# 基础规划示例
add_executable(basicPlanningExample
  ${CMAKE_SOURCE_DIR}/examples/basic_planning_example.cpp
)
target_link_libraries(basicPlanningExample
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
)

# 码垛示例
add_executable(palletizingExample
  ${CMAKE_SOURCE_DIR}/examples/palletizing_example.cpp
)
target_link_libraries(palletizingExample
  libHRCInterface.a
  libCmpAgu.a
  libhansKinematics.a
  stdc++
  m
)