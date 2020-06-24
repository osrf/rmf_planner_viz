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

add_executable(simple_test test/simple_test.cpp)
target_link_libraries(
  simple_test
  PUBLIC
    sfml-graphics
    SFGUI::SFGUI
)
