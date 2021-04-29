# Tools for visualizing the `rmf_core` planner

To build:

1. Create a new colcon workspace
2. Clone https://github.com/osrf/rmf_core
3. Clone https://github.com/osrf/rmf_planner_viz
4. Clone https://github.com/SFML/SFML
6. Clone https://github.com/eliasdaler/imgui-sfml.git
7. Clone the version of https://github.com/ocornut/imgui.git mentioned in the previous step (at the time of this writing, v1.80)
8. go into your imgui directory and do `touch COLCON_IGNORE`
9. get back to your colcon workspace and source your foxy installation
10. `colcon build --cmake-args -DBUILD_SHARED_LIBS=ON -DIMGUI_DIR=<your colcon workspace>/src/imgui`

Then you can start running interactive tests in the `./build/rmf_planner_viz/` directory

----

Test Programs:
- simple_test: Visual AStar planner debugger
- test_trajectory, test_spline: spline testing utilities
- performance_test: Visualizer for [`rmf_performance_tests`](https://github.com/osrf/rmf_performance_tests)
- performance_test_trajectory: Trajectory Visualizer for [`rmf_performance_tests`](https://github.com/osrf/rmf_performance_tests)
- test_freespace_planner: Visualizer for [`rmf_freespace_planner`](https://github.com/open-rmf/rmf_freespace_planner)
- test_probabilistic_road_map: Visualizer for [`rmf_probabilistic_road_map`](https://github.com/shiveshkhaitan/rmf_probabilistic_road_map)
- test_trajectory_probabilistic_road_map: Trajectory Visualizer for [`rmf_probabilistic_road_map`](https://github.com/shiveshkhaitan/rmf_probabilistic_road_map)


(Requires fcl 0.6)

- test_fcl_spline: spline drawing using fcl SplineMotion parameters
- test_fcl_spline_offset: Spline catmullrom approximation
- test_sidecar: CCD with bilateral advancement algorithm
- test_fcl_bvh: Collision detection via adding shapes to bounding volume hierarchy. Crashes with issue https://github.com/flexible-collision-library/fcl/issues/512

--
Optionally, you can build and run fcl 0.6 demos by cloning 

https://github.com/flexible-collision-library/fcl version 0.6

----

Using the Visual AStar planner debugger

Running ./build/simple_test will use a default nav graph.

To use other nav graphs,
- Import https://github.com/osrf/rmf_demos and https://github.com/osrf/traffic_editor into your colcon workspace and colcon build it.
- This will generate the navgraph .yaml files in the build/rmf_demo_maps/maps/<map name>/nav_graphs/*.yaml
- Run `./build/simple_test ./build/rmf_demo_maps/maps/<map name>/nav_graphs/*.yaml` to load the nav graph

To run the performance tests:
```asm
cd {ros2_ws}
./src/rmf/rmf_planner_viz/build/performance_test {SCENARIO_NAME}
```

To run the performance trajectory tests:
```asm
cd {ros2_ws}
./src/rmf/rmf_planner_viz/build/performance_test_trajectory {SCENARIO_NAME} {MAP_NAME} {TIME_OFFSET_IN_MILLISECONDS} {TIME_INCREMENT_IN_MILLISECONDS}
```

To run the freespace_planner tests:
```asm
cd {ros2_ws}
./src/rmf/rmf_planner_viz/build/test_freespace_planner {SCENARIO_NAME} {MAP_NAME} {TIME_OFFSET_IN_MILLISECONDS} {TIME_INCREMENT_IN_MILLISECONDS}
```

To run the probabilistic_road_map tests:
```asm
cd {ros2_ws}
./src/rmf/rmf_planner_viz/build/test_probabilistic_road_map {SCENARIO_NAME} {MAP_NAME} {TIME_OFFSET_IN_MILLISECONDS} {TIME_INCREMENT_IN_MILLISECONDS}
```
or
```asm
cd {ros2_ws}
./src/rmf/rmf_planner_viz/build/test_trajectory_probabilistic_road_map {SCENARIO_NAME} {MAP_NAME} {TIME_OFFSET_IN_MILLISECONDS} {TIME_INCREMENT_IN_MILLISECONDS}
```
