/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <SFML/Graphics.hpp>
#include <imgui.h>

#include <iostream>

#include <rmf_planner_viz/draw/Graph.hpp>
#include <rmf_planner_viz/draw/Schedule.hpp>
#include <rmf_planner_viz/draw/IMDraw.hpp>
#include <rmf_planner_viz/draw/Trajectory.hpp>
#include <rmf_planner_viz/draw/Camera.hpp>

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/debug_Planner.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include "imgui-SFML.h"
#include "planner_debug.hpp"

int main(int argc, char* argv[])
{
  sf::Font font;
  if (!font.loadFromFile("./build/rmf_planner_viz/fonts/OpenSans-Bold.ttf"))
  {
    std::cout << "Failed to load font. Make sure you run the executable from the colcon directory" << std::endl;
    return -1;
  }
  
  const rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  rmf_traffic::agv::Graph graph_0;

  const std::string test_map_name = "test_map";
  if (argc == 2)
  {
    std::cout << "Loading nav_graph file " << argv[1] << std::endl;
    graph_0 = rmf_fleet_adapter::agv::parse_graph(
      argv[1], 
      traits);
  }
  else // default graph
  {
    std::cout << "Using default map" << std::endl;
    /*            0<------------1<------------2
    *                                        ^
    *                                        |
    *  12------------->10----->11            |
    *                   |      |             |
    *                   |      v             |
    *                   8------9             |
    *                   |      |             |
    *                   |      |             |
    *     3------4------5------6------7      3
    *                   |      |
    *                   |      |
    *                   1------2
    *                   |
    *                   |
    *                   0
    **/
    graph_0.add_waypoint(test_map_name, {0.0, -10.0}); // 0
    graph_0.add_waypoint(test_map_name, {0.0, -5.0});  // 1
    graph_0.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true);  // 2
    graph_0.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
    graph_0.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
    graph_0.add_waypoint(test_map_name, {0.0, 0.0}); // 5
    graph_0.add_waypoint(test_map_name, {5.0, 0.0}); // 6
    graph_0.add_waypoint(test_map_name, {10.0, 0.0}); // 7
    graph_0.add_waypoint(test_map_name, {0.0, 5.0}); // 8
    graph_0.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
    graph_0.add_waypoint(test_map_name, {0.0, 10.0}); // 10
    graph_0.add_waypoint(test_map_name, {5.0, 10.0}); // 11
    graph_0.add_waypoint(test_map_name, {-12.0, 10.0}); // 12
    graph_0.add_key("Interesting Waypoint", 0);

    auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph_0.add_lane(w0, w1);
      graph_0.add_lane(w1, w0);
    };

    add_bidir_lane(0, 1);
    add_bidir_lane(1, 2);
    add_bidir_lane(1, 5);
    add_bidir_lane(2, 6);
    add_bidir_lane(3, 4);
    add_bidir_lane(4, 5);
    add_bidir_lane(5, 6);
    add_bidir_lane(6, 7);
    add_bidir_lane(5, 8);
    add_bidir_lane(6, 9);
    add_bidir_lane(8, 9);
    add_bidir_lane(8, 10);
    graph_0.add_lane(10, 11);
    graph_0.add_lane(11, 9);
    graph_0.add_lane(12, 10);
  }

  rmf_planner_viz::draw::Graph graph_0_drawable(graph_0, 1.0, font);
  std::vector<std::string> map_names = graph_0_drawable.get_map_names();
  std::string chosen_map;
  if (graph_0_drawable.current_map())
    chosen_map = *graph_0_drawable.current_map();
  
  std::shared_ptr<rmf_traffic::schedule::Database> database =
      std::make_shared<rmf_traffic::schedule::Database>();

  rmf_traffic::agv::Planner planner_0(
        rmf_traffic::agv::Planner::Configuration(graph_0, traits),
        rmf_traffic::agv::Planner::Options(nullptr));

  /// Setup participants
  auto p0 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_0",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto p1 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_1",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto pa = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_0",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto pb = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_0",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

//  auto pc = rmf_traffic::schedule::make_participant(
//        rmf_traffic::schedule::ParticipantDescription{
//          "participant_0",
//          "simple_test",
//          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
//          profile
//        },
//        database);

  auto p2 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_2",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);


  // set plans for the participants
  using namespace std::chrono_literals;
  auto plan_start_timing = std::chrono::steady_clock::now();

  std::vector<rmf_traffic::agv::Planner::Start> starts;
  starts.emplace_back(plan_start_timing, 0, 0.0);
  // starts.emplace_back(now, 12, 0.0);
  // starts.emplace_back(now, 10, 0.0);

  // p0.set(planner_0.plan(starts[0], 3)->get_itinerary());
  // p1.set(planner_0.plan(starts[1], 2)->get_itinerary());
  // p2.set(planner_0.plan(starts[2], 7,
  //   rmf_traffic::agv::Plan::Options(
  //   rmf_traffic::agv::ScheduleRouteValidator::make(
  //       database, p2.id(), p2.description().profile())))->get_itinerary());

  rmf_traffic::agv::Planner::Goal goal(0);
  rmf_traffic::agv::Planner::Debug planner_debug(planner_0);
  rmf_traffic::agv::Planner::Debug::Progress progress =
    planner_debug.begin(starts, goal, planner_0.get_default_options());

  rmf_planner_viz::draw::Schedule schedule_drawable(
        database, 0.25, test_map_name, plan_start_timing);

  rmf_planner_viz::draw::Fit fit(
    {graph_0_drawable.bounds()}, 0.02);

  sf::RenderWindow app_window(
        sf::VideoMode(1250, 1028),
        "Simple Test",
        sf::Style::Default);

  app_window.resetGLStates();
  app_window.setFramerateLimit(60);

  sf::View max_cam_view = app_window.getView();
  sf::Vector2f sz = max_cam_view.getSize();

  int textsz = 24;

  auto bounds = fit.get_bounds();
  auto s = bounds.max - bounds.min;
  //printf("bounds sz: %f %f\n", s.x(), s.y());
  double max_length_side = s.x() < s.y() ? s.y() : s.x();
  if (max_length_side < 20.0)
  {
    textsz = 12;
    graph_0_drawable.set_text_size(textsz);
  }
    
  rmf_planner_viz::draw::Camera camera(sz);
  
  ImGui::SFML::Init(app_window);

  sf::Clock deltaClock;
  while (app_window.isOpen())
  {
    sf::Event event;
    while (app_window.pollEvent(event))
    {
      ImGui::SFML::ProcessEvent(event);

      if (event.type == sf::Event::Closed)
      {
        return 0;
      }

      if (event.type == sf::Event::Resized)
      {
        sf::FloatRect visibleArea(0, 0, event.size.width, event.size.height);
        app_window.setView(sf::View(visibleArea));
      }

      if (event.type == sf::Event::MouseButtonPressed)
      {
        const sf::Vector2f p =
            fit.compute_transform(app_window.getSize()).getInverse()
            * sf::Vector2f(event.mouseButton.x, event.mouseButton.y);

        const auto pick = graph_0_drawable.pick(p.x, p.y);
        if (pick)
          graph_0_drawable.select(*pick);
        else
          camera.on_mouse_button_pressed(event.mouseButton.x, event.mouseButton.y);
      }

      if (event.type == sf::Event::KeyReleased)
      {
        if (event.key.code == sf::Keyboard::F1)
        {
          for (uint i=0; i<map_names.size(); ++i)
          {
            if (map_names[i] != chosen_map)
              continue;
            if (i > 0)
              chosen_map = map_names[i - 1];
            else
              chosen_map = map_names[map_names.size() - 1];
            break;
          }
        }
        if (event.key.code == sf::Keyboard::F2)
        {
          for (uint i=0; i<map_names.size(); ++i)
          {
            if (map_names[i] != chosen_map)
              continue;
            if (i < (map_names.size() - 1))
              chosen_map = map_names[i + 1];
            else
              chosen_map = map_names[0];
            break;
          }
        }
      }

      if (event.type == sf::Event::MouseButtonPressed)
        camera.on_mouse_button_pressed(event.mouseButton.x, event.mouseButton.y);

      if (event.type == sf::Event::MouseButtonReleased)
        camera.on_mouse_button_released();

      if (event.type == sf::Event::MouseWheelScrolled)
        camera.on_mouse_wheel_scrolled(event.mouseWheelScroll);
    }

    auto& io = ImGui::GetIO();
    if (!io.WantCaptureMouse)
      camera.update(deltaClock.getElapsedTime().asSeconds(), app_window);

    ImGui::SFML::Update(app_window, deltaClock.restart());

    bool force_replan = false;
    if (ImGui::BeginMainMenuBar())
    {
      if (ImGui::BeginMenu("UI"))
      {
        if (ImGui::InputInt("Text size", &textsz))
        {
          if (textsz > 0)
            graph_0_drawable.set_text_size((uint)textsz);
        }
        ImGui::Separator();
        ImGui::NewLine();

        ImGui::Text("Maps (Use keys F1/F2 to iterate through)");

        for (uint i=0; i<map_names.size(); ++i)
        {
          bool activated = map_names[i] == chosen_map;
          if (ImGui::RadioButton(map_names[i].c_str(), activated))
            chosen_map = map_names[i];
        }

        ImGui::NewLine();

        ImGui::Text("WASD/mousedrag to move camera");
        ImGui::Text("MouseWheel scroll to zoom in/out");
        ImGui::Text("Z to reset");

        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    graph_0_drawable.choose_map(chosen_map);
    
    static bool show_node_trajectories = true;
    static std::vector<rmf_planner_viz::draw::Trajectory> trajectories_to_render;

    bool startgoal_force_replan = rmf_planner_viz::draw::do_planner_presets(starts, goal, plan_start_timing);
    force_replan |= startgoal_force_replan;

    rmf_planner_viz::draw::do_planner_debug(
      profile, chosen_map,
      planner_0, starts, goal, graph_0.num_waypoints(), planner_debug, progress, plan_start_timing,
      force_replan, show_node_trajectories, trajectories_to_render);
    
    ImGui::EndFrame();

    /*** drawing ***/
    app_window.clear();

    schedule_drawable.timespan(std::chrono::steady_clock::now());

    sf::RenderStates states;
    fit.apply_transform(states.transform, app_window.getSize());
    app_window.draw(graph_0_drawable, states);
    app_window.draw(schedule_drawable, states);
    if (show_node_trajectories)
    {
      for (const auto& trajectory : trajectories_to_render)
        app_window.draw(trajectory, states);
    }

    sf::Transform ident;
    rmf_planner_viz::draw::IMDraw::flush_and_render(app_window, ident);
    

    ImGui::SFML::Render(app_window);
    app_window.display();
  }
}
