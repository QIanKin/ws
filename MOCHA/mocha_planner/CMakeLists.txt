cmake_minimum_required(VERSION 3.16)
project(mocha_planner LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

add_library(mocha_core STATIC
  src/core/ackermann_bilayer_optimizer.cpp
  src/core/esdf_map.cpp
  src/core/grid_astar_planner.cpp
  src/core/mco_optimizer.cpp
  src/core/perception_simulator.cpp
  src/core/reference_selector.cpp
  src/core/risk_corridor.cpp
  src/core/risk_field_builder.cpp
  src/core/risk_grid.cpp
  src/core/trajectory_utils.cpp
)

target_include_directories(mocha_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mocha_core PUBLIC
  Eigen3::Eigen
)

add_executable(risk_mocha_v1_demo
  src/risk_mocha_v1_demo.cpp
)
target_link_libraries(risk_mocha_v1_demo PRIVATE
  mocha_core
)

option(BUILD_RVIZ_VIS "Build the ROS2 RViz visualization node for Risk-Aware MOCHA V1" OFF)
if(BUILD_RVIZ_VIS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)

  add_executable(risk_mocha_v1_rviz_node
    src/ros/risk_mocha_v1_rviz_node.cpp
  )
  target_link_libraries(risk_mocha_v1_rviz_node PRIVATE
    mocha_core
  )
  ament_target_dependencies(risk_mocha_v1_rviz_node
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
  )
endif()

option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  enable_testing()

  add_executable(test_grid_astar_planner
    test/test_grid_astar_planner.cpp
  )
  target_link_libraries(test_grid_astar_planner PRIVATE
    mocha_core
  )
  add_test(NAME test_grid_astar_planner COMMAND test_grid_astar_planner)

  add_executable(test_risk_aware_astar
    test/test_risk_aware_astar.cpp
  )
  target_link_libraries(test_risk_aware_astar PRIVATE
    mocha_core
  )
  add_test(NAME test_risk_aware_astar COMMAND test_risk_aware_astar)

  add_executable(test_risk_grid
    test/test_risk_grid.cpp
  )
  target_link_libraries(test_risk_grid PRIVATE
    mocha_core
  )
  add_test(NAME test_risk_grid COMMAND test_risk_grid)

  add_executable(test_mco_risk_penalty
    test/test_mco_risk_penalty.cpp
  )
  target_link_libraries(test_mco_risk_penalty PRIVATE
    mocha_core
  )
  add_test(NAME test_mco_risk_penalty COMMAND test_mco_risk_penalty)

  add_executable(test_perception_risk_builder
    test/test_perception_risk_builder.cpp
  )
  target_link_libraries(test_perception_risk_builder PRIVATE
    mocha_core
  )
  add_test(NAME test_perception_risk_builder COMMAND test_perception_risk_builder)

  add_executable(test_risk_corridor
    test/test_risk_corridor.cpp
  )
  target_link_libraries(test_risk_corridor PRIVATE
    mocha_core
  )
  add_test(NAME test_risk_corridor COMMAND test_risk_corridor)
endif()

install(TARGETS mocha_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS risk_mocha_v1_demo
  RUNTIME DESTINATION bin
)

if(BUILD_RVIZ_VIS)
  install(TARGETS risk_mocha_v1_rviz_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

install(DIRECTORY third_party/
  DESTINATION include/mocha_planner/third_party
)

if(BUILD_RVIZ_VIS)
  ament_package()
endif()
