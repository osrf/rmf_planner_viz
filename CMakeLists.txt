cmake_minimum_required(VERSION 3.5.0)

project(rmf_planner_viz)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(SFML COMPONENTS graphics window system REQUIRED)
find_package(SFGUI)

include(GNUInstallDirs)

add_library(
  rmf_planning_viz
    src/rmf_planner_viz/draw/ColorPicker.cpp
    src/rmf_planner_viz/draw/Fit.cpp
    src/rmf_planner_viz/draw/Graph.cpp
    src/rmf_planner_viz/draw/Capsule.cpp
    src/rmf_planner_viz/draw/Schedule.cpp
    src/rmf_planner_viz/draw/Trajectory.cpp
)

target_link_libraries(
  rmf_planning_viz
  PUBLIC
    rmf_traffic::rmf_traffic
    sfml-graphics
    SFGUI::SFGUI
)

target_include_directories(
  rmf_planning_viz
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_executable(simple_test test/simple_test.cpp)
target_link_libraries(
  simple_test
  PUBLIC
    rmf_planning_viz
)

add_executable(test_trajectory test/test_trajectory.cpp)
target_link_libraries(
  test_trajectory
  PUBLIC
    rmf_planning_viz
)
