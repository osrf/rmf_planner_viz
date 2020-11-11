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
#include <math.h>

#include <rmf_planner_viz/draw/Graph.hpp>
#include <rmf_planner_viz/draw/Schedule.hpp>
#include <rmf_planner_viz/draw/IMDraw.hpp>

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/Motion.hpp>

#include "imgui-SFML.h"

sf::Vector2f sample_trajectory(const rmf_traffic::Trajectory& trajectory, rmf_traffic::Time time)
{
  const auto motion = rmf_traffic::Motion::compute_cubic_splines(trajectory);
  const Eigen::Vector2d p =
      motion->compute_position(time).block<2,1>(0, 0);
  return sf::Vector2f(p.x(), p.y());
}

int main()
{
  const auto circle_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(0.5);
  const auto circle_shape_ex = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(0.6);
    
  rmf_traffic::Profile profile_circle { circle_shape };
  rmf_traffic::Profile profile_circle_with_circle_offset { circle_shape };
  
  // square window to avoid stretching
  sf::RenderWindow app_window(
        sf::VideoMode(1024, 1024),
        "Test_Spline",
        sf::Style::Default);
  app_window.setFramerateLimit(60);
  app_window.resetGLStates();

  //view centered at origin, with width/height
  sf::View view(sf::Vector2f(0.f, 0.f), sf::Vector2f(10.f, 10.f));
  app_window.setView(view);

  sf::Transformable vqs;
  vqs.setScale(1.f, -1.f);
  auto tx_flipped_2d = vqs.getTransform();

  ImGui::SFML::Init(app_window);

  sf::Clock deltaClock;
  while (app_window.isOpen())
  {
    sf::Event event;
    while (app_window.pollEvent(event))
    {
      //window->HandleEvent(event);
      ImGui::SFML::ProcessEvent(event);

      if (event.type == sf::Event::Closed)
        return 0;

      if (event.type == sf::Event::Resized)
      {
        sf::FloatRect visibleArea(0, 0, event.size.width, event.size.height);
        app_window.setView(sf::View(visibleArea));
      }
    }

    ImGui::SFML::Update(app_window, deltaClock.restart());
    
    ImGui::SetWindowSize(ImVec2(600, 200));
    ImGui::Begin("Spline control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    static bool enable_duration_scale = true;
    ImGui::Checkbox("Enable velocity scale by duration", &enable_duration_scale);

    static double duration = 10.0;
    ImGui::InputDouble("Red Spline Time duration", &duration, 0.1);
    
    static double red_spline_t_val = 0.0;
    ImGui::InputDouble("Red Spline Time-value", &red_spline_t_val, 0.1);
    ImGui::Separator();
    
    
    using namespace std::chrono_literals;
    using namespace rmf_planner_viz::draw;

    auto now = std::chrono::steady_clock::now();
    rmf_traffic::Trajectory t1;
    rmf_traffic::Trajectory t2;

    {
      auto duration_chrono = rmf_traffic::time::from_seconds(duration);
      auto end = now + duration_chrono;
      double duration_scale = 1.0f;
      if (enable_duration_scale)
        duration_scale = duration;

      Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
      Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
      
      t1.insert(now, pos, vel);
      t1.insert(end, pos, vel);
      
      t2.insert(now, Eigen::Vector3d(-2, 2, 0), Eigen::Vector3d(0, -2 / duration_scale, 0));
      t2.insert(end, Eigen::Vector3d(2, 2, 0), Eigen::Vector3d(0, 2 / duration_scale, 0));

      auto collide_timing = rmf_traffic::DetectConflict::between(profile_circle, t1, profile_circle_with_circle_offset, t2);
      ImGui::TextColored(ImVec4(0, 1, 0, 1), "rmf_traffic::DetectConflict::between results: ");
      if (collide_timing)
        ImGui::Text("Collided at %f!", rmf_traffic::time::to_seconds(*collide_timing - now));
      else
        ImGui::Text("Not collided!");

      rmf_planner_viz::draw::IMDraw::draw_trajectory(t1, sf::Color::Green);
      rmf_planner_viz::draw::IMDraw::draw_trajectory(t2, sf::Color::Red);
    }
    ImGui::End();
    ImGui::EndFrame();

    // draw robot shapes
    {
      IMDraw::draw_circle(sf::Vector2f(0, 0), circle_shape->get_characteristic_length(), sf::Color::Green);

      sf::Vector2f pt = sample_trajectory(t2, now + rmf_traffic::time::from_seconds(red_spline_t_val));
      IMDraw::draw_circle(pt, circle_shape->get_characteristic_length(), sf::Color::Red);
    }
    
    IMDraw::draw_axis();
    
    app_window.clear();
    app_window.setView(view);

    IMDraw::flush_and_render(app_window, tx_flipped_2d);

    ImGui::SFML::Render(app_window);
    app_window.display();
  }
}
