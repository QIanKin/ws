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
  src/core/mco_optimizer.cpp
)

target_include_directories(mocha_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mocha_core PUBLIC
  Eigen3::Eigen
)

option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  enable_testing()

  add_executable(test_core_mco_optimizer
    test/test_core_mco_optimizer.cpp
  )
  target_link_libraries(test_core_mco_optimizer PRIVATE
    mocha_core
  )
  add_test(NAME test_core_mco_optimizer COMMAND test_core_mco_optimizer)

  add_executable(test_core_ackermann_bilayer_optimizer
    test/test_core_ackermann_bilayer_optimizer.cpp
  )
  target_link_libraries(test_core_ackermann_bilayer_optimizer PRIVATE
    mocha_core
  )
  add_test(NAME test_core_ackermann_bilayer_optimizer COMMAND test_core_ackermann_bilayer_optimizer)
endif()

install(TARGETS mocha_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY third_party/
  DESTINATION include/mocha_planner/third_party
)
