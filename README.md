Tools for visualizing the `rmf_core` planner

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
- test_trajectory, test_spline: spline testing utilitis
- test_fcl_spline: spline drawing using fcl SplineMotion parameters
- test_fcl_spline_offset: Sidecar collision test using modified ccd algorithm to account for an offset transform, Experimental.
- test_sidecar: Triangle mesh ccd solution for sidecar. Experimental.

--
Optionally, you can build and run test_fcl_spline by cloning 

https://github.com/ddengster/fcl.git with branch 0.6.1-fix-conservative-adv into your workspace

alternatively, you can use the latest version of https://github.com/flexible-collision-library/fcl 
