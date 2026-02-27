cmake_minimum_required(VERSION 3.14)
project(HRC-PalletizingPlanner-Tests)

add_compile_options(-fPIC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# 公共设置
# ============================================================================

include_directories(${CMAKE_SOURCE_DIR}/include)

# Eigen3
if(EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIRS})
else()
    include_directories(/usr/include/eigen3)
endif()

# SO栈链接库 (dlopen动态加载, 不链接静态HRC库)
set(SO_STACK_LIBS stdc++ m pthread dl)

# ============================================================================
# SO栈测试 (主架构)
# ============================================================================

# 碰撞仿真 (.so碰撞 + S曲线 + 分层计时)
add_executable(testS50CollisionSO
  ${CMAKE_SOURCE_DIR}/test/testS50CollisionSO.cpp
)
target_include_directories(testS50CollisionSO PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(testS50CollisionSO ${SO_STACK_LIBS})
add_test(NAME testS50CollisionSO COMMAND testS50CollisionSO)

# 码垛仿真 (.so碰撞 + Free-TCP RRT* + IK + 完整环境碰撞)
add_executable(testS50PalletizingSO
  ${CMAKE_SOURCE_DIR}/test/testS50PalletizingSO.cpp
)
target_include_directories(testS50PalletizingSO PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(testS50PalletizingSO ${SO_STACK_LIBS})
add_test(NAME testS50PalletizingSO COMMAND testS50PalletizingSO)

# 系统性轨迹最优性测试 (6套测试: A-F)
add_executable(testTrajectoryOptimality
  ${CMAKE_SOURCE_DIR}/test/testTrajectoryOptimality.cpp
)
target_include_directories(testTrajectoryOptimality PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(testTrajectoryOptimality ${SO_STACK_LIBS})
add_test(NAME testTrajectoryOptimality COMMAND testTrajectoryOptimality)

# ============================================================================
# SO栈示例
# ============================================================================

# SO栈基础规划示例
if(EXISTS ${CMAKE_SOURCE_DIR}/examples/basic_planning_example_so.cpp)
  add_executable(basicPlanningExampleSO
    ${CMAKE_SOURCE_DIR}/examples/basic_planning_example_so.cpp
  )
  target_include_directories(basicPlanningExampleSO PRIVATE ${CMAKE_SOURCE_DIR}/include)
  target_link_libraries(basicPlanningExampleSO ${SO_STACK_LIBS})
endif()

# SO栈码垛示例
if(EXISTS ${CMAKE_SOURCE_DIR}/examples/palletizing_example_so.cpp)
  add_executable(palletizingExampleSO
    ${CMAKE_SOURCE_DIR}/examples/palletizing_example_so.cpp
  )
  target_include_directories(palletizingExampleSO PRIVATE ${CMAKE_SOURCE_DIR}/include)
  target_link_libraries(palletizingExampleSO ${SO_STACK_LIBS})
endif()

# ============================================================================
# CTest 环境设置
# ============================================================================
set_tests_properties(testS50CollisionSO testS50PalletizingSO testTrajectoryOptimality
  PROPERTIES
    ENVIRONMENT "HRC_LIB_PATH=${CMAKE_SOURCE_DIR}/lib/libHRCInterface.so"
    TIMEOUT 120
)