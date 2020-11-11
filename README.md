Tools for visualizing the `rmf_core` planner

To build:

1. Create a new colcon workspace
2. Clone https://github.com/osrf/rmf_core
3. Clone https://github.com/osrf/rmf_planner_viz
4. Clone https://github.com/SFML/SFML
5. Clone https://github.com/TankOs/SFGUI
6. Clone https://github.com/ocornut/imgui.git
7. Clone https://github.com/eliasdaler/imgui-sfml.git
8. go into your imgui directory and do `touch COLCON_IGNORE`
9. get back to your colcon workspace and source your foxy installation
10. `colcon build --cmake-args -DBUILD_SHARED_LIBS=ON -DIMGUI_DIR=<your colcon workspace>/src/imgui`

Then you can start running interactive tests in the `./build/rmf_planner_viz/` directory
