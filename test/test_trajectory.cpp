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

#include <rmf_planner_viz/draw/Schedule.hpp>

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <Eigen/Geometry>

#include <iostream>

#include "imgui-SFML.h"

int main()
{
  const auto database = std::make_shared<rmf_traffic::schedule::Database>();

  const rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(1.0)
  };

  auto p0 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_0",
          "test_trajectory",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto p1 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_1",
          "test_trajectory",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  const std::string test_map_name = "test_map";

  using namespace std::chrono_literals;
  const auto now = std::chrono::steady_clock::now();
  const auto soon = now + 5s;

  const Eigen::Vector3d xi = {0.0, 0.0, 0.0};
  const Eigen::Vector3d xf = {10.0, 0.0, 0.0};
  const Eigen::Vector3d vi = {0.0, 1.0, 0.0};
  const Eigen::Vector3d vf = {0.0, -1.0, 0.0};
  const auto duration = 50s;

  rmf_traffic::Trajectory t_long;
  t_long.insert(soon, xi, vi);
  t_long.insert(soon + duration, xf, vf);

  const double multiplier = 20.0;
  const double time_scaling = 1.0/multiplier;
  const auto scaled_duration =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        time_scaling * duration);

  rmf_traffic::Trajectory t_quick;
  t_quick.insert(soon, xi, multiplier*vi);
  t_quick.insert(soon + scaled_duration, xf, multiplier*vf);

  p0.set({{test_map_name, t_long}});
  p1.set({{test_map_name, t_quick}});

  rmf_planner_viz::draw::Schedule schedule_drawable(
        database, 0.25, test_map_name, soon);

  sf::RenderWindow app_window(
        sf::VideoMode(1250, 1028),
        "Test Trajectory",
        sf::Style::Default);

  app_window.resetGLStates();
  
  const auto bounds = schedule_drawable.bounds();

  rmf_planner_viz::draw::Fit fit({bounds}, 0.02);
  std::cout << "initial bounds:\n"
            << " -- min: " << bounds.min.transpose()
            << "\n -- max: " << bounds.max.transpose() << std::endl;

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
    }

    ImGui::SFML::Update(app_window, deltaClock.restart());

    app_window.clear();

    schedule_drawable.timespan(std::chrono::steady_clock::now());

    sf::RenderStates states;
    fit.apply_transform(states.transform, app_window.getSize());
    app_window.draw(schedule_drawable, states);


    app_window.display();
  }
}
