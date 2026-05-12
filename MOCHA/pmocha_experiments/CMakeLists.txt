cmake_minimum_required(VERSION 3.16)
project(pmocha_experiments LANGUAGES CXX)

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
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(nav_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(tf2 QUIET)
find_package(tf2_ros QUIET)
find_package(tf2_eigen QUIET)
find_package(visualization_msgs QUIET)

add_subdirectory(../mocha_planner ${CMAKE_CURRENT_BINARY_DIR}/mocha_planner)

add_library(pmocha_experiment_core STATIC
  src/occupancy_grid.cpp
  src/esdf_grid.cpp
  src/single_astar.cpp
  src/single_corridor.cpp
  src/trajectory_evaluator.cpp
)

target_include_directories(pmocha_experiment_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(pmocha_experiment_core PUBLIC
  Eigen3::Eigen
  mocha_core
)

install(TARGETS pmocha_experiment_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch urdf worlds config meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/teleop.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(rclcpp_FOUND AND sensor_msgs_FOUND AND nav_msgs_FOUND AND tf2_ros_FOUND AND tf2_eigen_FOUND)
  add_executable(perception_grid_node src/perception_grid_node.cpp)
  ament_target_dependencies(perception_grid_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_eigen
  )
  target_link_libraries(perception_grid_node pmocha_experiment_core)
  install(TARGETS perception_grid_node
    DESTINATION lib/${PROJECT_NAME}
  )

  if(visualization_msgs_FOUND)
    add_executable(planner_node src/planner_node.cpp)
    ament_target_dependencies(planner_node
      rclcpp
      sensor_msgs
      nav_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(planner_node pmocha_experiment_core)
    install(TARGETS planner_node
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(WARNING "visualization_msgs not found; skipping planner_node")
  endif()
else()
  message(WARNING "ROS2 dependencies not found; skipping ROS nodes")
endif()

option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  enable_testing()

  add_executable(test_grid_corridor_pipeline
    test/test_grid_corridor_pipeline.cpp
  )
  target_link_libraries(test_grid_corridor_pipeline PRIVATE
    pmocha_experiment_core
  )
  add_test(NAME test_grid_corridor_pipeline COMMAND test_grid_corridor_pipeline)

  add_executable(test_mocha_single_corridor_pipeline
    test/test_mocha_single_corridor_pipeline.cpp
  )
  target_link_libraries(test_mocha_single_corridor_pipeline PRIVATE
    pmocha_experiment_core
  )
  add_test(NAME test_mocha_single_corridor_pipeline COMMAND test_mocha_single_corridor_pipeline)
endif()

if(ament_cmake_FOUND)
  ament_package()
endif()
