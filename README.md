# Tools for visualizing the `rmf_core` planner

To build:

1. Create a new colcon workspace
2. Clone https://github.com/osrf/rmf_core
3. Clone https://github.com/osrf/rmf_planner_viz
4. Clone https://github.com/SFML/SFML
5. Clone https://github.com/ocornut/imgui.git
6. Clone https://github.com/eliasdaler/imgui-sfml.git
7. go into your imgui directory and do `touch COLCON_IGNORE`
8. get back to your colcon workspace and source your foxy installation
9. `colcon build --cmake-args -DBUILD_SHARED_LIBS=ON -DIMGUI_DIR=<your colcon workspace>/src/imgui`

Then you can start running interactive tests in the `./build/rmf_planner_viz/` directory

----

Test Programs:
- simple_test: Visual AStar planner debugger
- test_trajectory, test_spline: spline testing utilities

(Requires fcl 0.6)

- test_fcl_spline: spline drawing using fcl SplineMotion parameters
- test_fcl_spline_offset: Spline approximation experiments
- test_sidecar: CCD with bilateral advancement algorithm, also experimental triangle mesh ccd solution.

--
Optionally, you can build and run test_fcl_spline, test_fcl_spline_offset, test_sidecar by cloning 

https://github.com/ddengster/fcl.git with branch 0.6.1-fix-conservative-adv into your workspace

alternatively, you can use the latest version of https://github.com/flexible-collision-library/fcl 

----

Using the Visual AStar planner debugger

Running ./build/simple_test will use a default nav graph.

To use other nav graphs,
- Import https://github.com/osrf/rmf_demos and https://github.com/osrf/traffic_editor into your colcon workspace and colcon build it.
- This will generate the navgraph .yaml files in the build/rmf_demo_maps/maps/<map name>/nav_graphs/*.yaml
- Run `./build/simple_test ./build/rmf_demo_maps/maps/<map name>/nav_graphs/*.yaml` to load the nav graph
