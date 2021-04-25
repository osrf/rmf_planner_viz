cmake_minimum_required(VERSION 3.5.0)

project(rmf_planner_viz)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(SFML COMPONENTS graphics window system REQUIRED)
find_package(ImGui-SFML)
find_package(rmf_performance_tests)
find_package(rmf_freespace_planner)
find_package(rmf_probabilistic_road_map)

include(GNUInstallDirs)

add_library(
  rmf_planning_viz SHARED
    src/rmf_planner_viz/draw/ColorPicker.cpp
    src/rmf_planner_viz/draw/Fit.cpp
    src/rmf_planner_viz/draw/Graph.cpp
    src/rmf_planner_viz/draw/Capsule.cpp
    src/rmf_planner_viz/draw/Schedule.cpp
    src/rmf_planner_viz/draw/Trajectory.cpp
    src/rmf_planner_viz/draw/IMDraw.cpp
    src/rmf_planner_viz/draw/Camera.cpp
)

target_link_libraries(
  rmf_planning_viz
  PUBLIC
    rmf_traffic::rmf_traffic
    sfml-graphics
)

target_include_directories(
  rmf_planning_viz
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_executable(simple_test test/simple_test.cpp test/planner_debug.cpp)
target_link_libraries(
  simple_test
  PUBLIC
    rmf_planning_viz
    rmf_fleet_adapter::rmf_fleet_adapter
    ImGui-SFML::ImGui-SFML
)

target_include_directories(
  simple_test
  PUBLIC
    rmf_fleet_adapter::rmf_fleet_adapter
)

if (rmf_performance_tests_FOUND)
  add_executable(performance_test
    test/performance_test.cpp
    test/planner_debug.cpp
  )

  target_link_libraries(
    performance_test
    PUBLIC
      rmf_planning_viz
      rmf_fleet_adapter::rmf_fleet_adapter
      ImGui-SFML::ImGui-SFML
      rmf_performance_tests::rmf_performance_tests
  )

  target_include_directories(
    performance_test
    PUBLIC
      rmf_fleet_adapter::rmf_fleet_adapter
      rmf_performance_tests::rmf_performance_tests
  )
endif()

add_executable(test_trajectory test/test_trajectory.cpp)
target_link_libraries(
  test_trajectory
  PUBLIC
    rmf_planning_viz
    ImGui-SFML::ImGui-SFML
)

add_executable(performance_test_trajectory test/performance_test_trajectory.cpp)
target_link_libraries(
  performance_test_trajectory
  PUBLIC
    rmf_planning_viz
    ImGui-SFML::ImGui-SFML
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_performance_tests::rmf_performance_tests
)
target_include_directories(
  performance_test_trajectory
  PUBLIC
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_performance_tests::rmf_performance_tests
)

add_executable(test_freespace_planner test/test_freespace_planner.cpp)
target_link_libraries(
  test_freespace_planner
  PUBLIC
    rmf_planning_viz
    ImGui-SFML::ImGui-SFML
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_performance_tests::rmf_performance_tests
    rmf_freespace_planner::rmf_freespace_planner
)
target_include_directories(
  test_freespace_planner
  PUBLIC
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_performance_tests::rmf_performance_tests
    rmf_freespace_planner::rmf_freespace_planner
)

add_executable(test_probabilistic_road_map
  test/test_probabilistic_road_map.cpp
  test/planner_debug.cpp
)
target_link_libraries(
  test_probabilistic_road_map
  PUBLIC
    rmf_planning_viz
    ImGui-SFML::ImGui-SFML
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_performance_tests::rmf_performance_tests
    rmf_probabilistic_road_map::rmf_probabilistic_road_map
)
target_include_directories(
  test_probabilistic_road_map
  PUBLIC
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_performance_tests::rmf_performance_tests
    rmf_probabilistic_road_map::rmf_probabilistic_road_map
)

add_executable(test_trajectory_probabilistic_road_map
  test/test_trajectory_probabilistic_road_map.cpp
)
target_link_libraries(
  test_trajectory_probabilistic_road_map
  PUBLIC
    rmf_planning_viz
    ImGui-SFML::ImGui-SFML
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_performance_tests::rmf_performance_tests
    rmf_probabilistic_road_map::rmf_probabilistic_road_map
)
target_include_directories(
  test_trajectory_probabilistic_road_map
  PUBLIC
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_performance_tests::rmf_performance_tests
    rmf_probabilistic_road_map::rmf_probabilistic_road_map
)

add_executable(test_spline
  test/test_spline.cpp
  )

target_link_libraries(
  test_spline
  PUBLIC
    rmf_planning_viz
    ImGui-SFML::ImGui-SFML
)

######## FCL test ##########

find_package(fcl 0.6 QUIET)
if(fcl_FOUND)
  set(FCL_LIBRARIES fcl)
  message(STATUS "Using FCL version: ${FCL_VERSION}")

  add_executable(test_fcl_spline
    test/test_fcl_spline.cpp
  )

  target_link_libraries(
    test_fcl_spline
    PUBLIC
      rmf_planning_viz
      ImGui-SFML::ImGui-SFML
      fcl
  )

  add_executable(test_fcl_spline_offset
    test/test_fcl_spline_offset.cpp
    test/spline_offset_utils.cpp
  )

  target_link_libraries(
    test_fcl_spline_offset
    PUBLIC
      rmf_planning_viz
      ImGui-SFML::ImGui-SFML
      fcl
  )

  add_executable(test_sidecar
    test/test_sidecar.cpp
    test/spline_offset_utils.cpp
    test/test_sidecar_utils.cpp
  )

  target_link_libraries(test_sidecar
    rmf_planning_viz
    ImGui-SFML::ImGui-SFML
    fcl
  )

  add_executable(test_fcl_bvh
    test/test_fcl_bvh.cpp
  )

  target_link_libraries(test_fcl_bvh
    rmf_planning_viz
    ImGui-SFML::ImGui-SFML
    fcl
  )
endif()

## copy font files
install(
  DIRECTORY fonts/
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/fonts/
)
