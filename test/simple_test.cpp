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

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/Planner.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include "imgui-SFML.h"
#include "planner_debug.hpp"

int main()
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

  using namespace std::chrono_literals;

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {-5, -5}).set_passthrough_point(true); // 0
  graph.add_waypoint(test_map_name, { 0, -5}).set_passthrough_point(true); // 1
  graph.add_waypoint(test_map_name, { 5, -5}).set_passthrough_point(true); // 2
  graph.add_waypoint(test_map_name, {10, -5}).set_passthrough_point(true); // 3
  graph.add_waypoint(test_map_name, {-5, 0}); // 4
  graph.add_waypoint(test_map_name, { 0, 0}); // 5
  graph.add_waypoint(test_map_name, { 5, 0}); // 6
  graph.add_waypoint(test_map_name, {10, 0}).set_passthrough_point(true); // 7
  graph.add_waypoint(test_map_name, {10, 4}).set_passthrough_point(true); // 8
  graph.add_waypoint(test_map_name, { 0, 8}).set_passthrough_point(true); // 9
  graph.add_waypoint(test_map_name, { 5, 8}).set_passthrough_point(true); // 10
  graph.add_waypoint(test_map_name, {10, 12}).set_passthrough_point(true); // 11
  graph.add_waypoint(test_map_name, {12, 12}).set_passthrough_point(true); // 12

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(2, 3);
  add_bidir_lane(1, 5);
  add_bidir_lane(3, 7);
  add_bidir_lane(4, 5);
  add_bidir_lane(6, 10);
  add_bidir_lane(7, 8);
  add_bidir_lane(9, 10);
  add_bidir_lane(10, 11);

  std::shared_ptr<rmf_traffic::schedule::Database> database =
      std::make_shared<rmf_traffic::schedule::Database>();

  /// Setup participants
  auto p0 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_0",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto p_obstacle = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "obstacle",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  const auto default_options = rmf_traffic::agv::Planner::Options{
    rmf_traffic::agv::ScheduleRouteValidator::make(database, p0.id(), profile)};

  rmf_traffic::agv::Planner planner{
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_options
  };

  const auto now = std::chrono::steady_clock::now();

  // GIVEN("Goal from 2->12 and obstacle from 9->1")
  {
    add_bidir_lane(5, 9);
//    const auto start = rmf_traffic::agv::Planner::Start(now, 2, 0.0);
//    const auto goal = rmf_traffic::agv::Planner::Goal(12);

    std::vector<rmf_traffic::Trajectory> obstacles;

    rmf_traffic::Trajectory obstacle;
    obstacle.insert(
      now + 24s,
      {0.0, 8.0, 0.0},
      {0.0, 0.0, 0.0});
    obstacle.insert(
      now + 50s,
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0});
    obstacle.insert(
      now + 70s,
      {0.0, -5.0, 0.0},
      {0.0, 0.0, 0.0});

    p_obstacle.set({{test_map_name, std::move(obstacle)}});

    // WHEN("Docking must be at 180-degrees")
    {
      using namespace rmf_traffic::agv;
      graph.add_lane(11, {12, Graph::OrientationConstraint::make({M_PI})});
      graph.add_lane({12, Graph::OrientationConstraint::make({M_PI})}, 11);

      planner = rmf_traffic::agv::Planner{
        rmf_traffic::agv::Planner::Configuration{graph, traits},
        default_options
      };
    }
  }

  rmf_planner_viz::draw::Graph graph_drawable(graph, 1.0, font);

  std::vector<rmf_traffic::agv::Planner::Start> start;
  start.push_back({now, 2, 0.0});
  auto goal = rmf_traffic::agv::Planner::Goal(12);
  rmf_traffic::agv::Planner::Debug planner_debug(planner);
  rmf_traffic::agv::Planner::Debug::Progress progress =
    planner_debug.begin({start}, goal, planner.get_default_options());

  rmf_planner_viz::draw::Schedule schedule_drawable(
        database, 0.25, test_map_name, now);

  rmf_planner_viz::draw::Fit fit({graph_drawable.bounds()}, 0.02);

  sf::RenderWindow app_window(
        sf::VideoMode(1250, 1028),
        "Simple Test",
        sf::Style::Default);

  app_window.resetGLStates();

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

        const auto pick = graph_drawable.pick(p.x, p.y);
        if (pick)
          graph_drawable.select(*pick);
      }
    }

    ImGui::SFML::Update(app_window, deltaClock.restart());
    
    static bool show_node_trajectories = true;
    static std::vector<rmf_planner_viz::draw::Trajectory> trajectories_to_render;

    rmf_planner_viz::draw::do_planner_debug(
      profile, planner, {start}, goal, planner_debug, progress, now,
      show_node_trajectories, trajectories_to_render, schedule_drawable);
    
    ImGui::EndFrame();

    /*** drawing ***/
    app_window.clear();

    sf::RenderStates states;
    fit.apply_transform(states.transform, app_window.getSize());
    app_window.draw(graph_drawable, states);
    app_window.draw(schedule_drawable, states);
    if (show_node_trajectories)
    {
      for (const auto& trajectory : trajectories_to_render)
        app_window.draw(trajectory, states);
    }

    rmf_planner_viz::draw::IMDraw::flush_and_render(app_window, states.transform);
    

    ImGui::SFML::Render(app_window);
    app_window.display();
  }
}
