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

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/Motion.hpp>

#include "imgui-SFML.h"

std::vector<sf::VertexArray> g_vertexarrays;

void DrawAxis()
{
  // x-axis
  {
    sf::VertexArray arr(sf::Lines);
    sf::Vertex v;
    v.color = sf::Color(255, 0, 0, 255);

    v.position = sf::Vector2f(0, 0);
    arr.append(v);

    v.position = sf::Vector2f(1, 0);
    arr.append(v);

    g_vertexarrays.push_back(arr);
  }
  // y-axis
  {
    sf::VertexArray arr(sf::Lines);
    sf::Vertex v;
    v.color = sf::Color(0, 255, 0, 255);

    v.position = sf::Vector2f(0, 0);
    arr.append(v);

    v.position = sf::Vector2f(0, 1);
    arr.append(v);

    g_vertexarrays.push_back(arr);
  }
}

void DrawCircle(const sf::Vector2f& center, double radius, const sf::Color& color, uint slices = 16)
{
  sf::VertexArray arr(sf::Lines);

  double rotation_per_slice = (2.0 * M_PI) / (double)slices;
  for (uint i=0; i<slices; ++i)
  {
    sf::Vertex v;
    v.color = color;

    double rot = rotation_per_slice * (double)i;
    v.position = center + sf::Vector2f(radius * cos(rot), radius * sin(rot));
    arr.append(v);

    rot = rotation_per_slice * (double)(i + 1);
    v.position = center + sf::Vector2f(radius * cos(rot), radius * sin(rot));
    arr.append(v);
  }
  g_vertexarrays.push_back(arr);
}

void DrawTrajectory(const rmf_traffic::Trajectory& trajectory, const sf::Color& color = sf::Color(255, 255, 255, 255))
{
  //lifted from Trajectory::Implementation::accurate_curve_drawing

  const auto motion = rmf_traffic::Motion::compute_cubic_splines(trajectory);

  const auto step = std::chrono::milliseconds(100);
  // const auto end = duration ?
  //       start + duration.value() : motion->finish_time();
  const auto end = motion->finish_time();
  const auto begin = motion->start_time();

  /*if (motion->start_time() <= start)
  {
    const Eigen::Vector3d p = motion->compute_position(start);
    configure_arrow(p, profile.footprint()->get_characteristic_length(), color);
    configure_footprint(p, profile.footprint()->get_characteristic_length());
    configure_vicinity(p, profile.vicinity()->get_characteristic_length());
  }*/
  sf::VertexArray vtx_arr(sf::Lines);

  for (auto time=begin; time <= end; time += step)
  {
    const auto next_time = std::min(time + step, end);

    const Eigen::Vector2d p =
        motion->compute_position(time).block<2,1>(0, 0);
    const Eigen::Vector2d pn =
        motion->compute_position(next_time).block<2,1>(0, 0);

    sf::Vertex v;
    v.color = color;

    v.position = sf::Vector2f(p.x(), p.y());
    vtx_arr.append(v);
    
    v.position = sf::Vector2f(pn.x(), pn.y());
    vtx_arr.append(v);
  }

  g_vertexarrays.push_back(vtx_arr);
}

int main()
{
  // const auto box_shape = rmf_traffic::geometry::make_final_convex<
  //     rmf_traffic::geometry::Box>(1.0, 1.0);
  const auto circle_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(0.5);
  const auto circle_shape_ex = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(0.6);
    
  rmf_traffic::Profile profile_circle { circle_shape };
  rmf_traffic::Profile profile_circle_with_circle_offset { circle_shape };
  // profile_circle_with_circle_offset.addFootPrintShape(
  //   circle_shape_ex, Eigen::Vector3d(0, -1.0, 0));
  
  // const rmf_traffic::agv::VehicleTraits traits{
  //   {0.7, 0.3},
  //   {1.0, 0.45},
  //   profile
  // };

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
    static double duration = 10.0;
    ImGui::InputDouble("Red Spline Time duration", &duration, 0.1);
    
    static double red_spline_t_val = 0.0;
    ImGui::InputDouble("Red Spline T-value", &red_spline_t_val, 0.1);
    ImGui::InputDouble("Red Spline Time-value", &red_spline_t_val, 0.1);
    ImGui::Separator();
    
    {
      using namespace std::chrono_literals;
      auto now = std::chrono::steady_clock::now();

      auto duration_chrono = rmf_traffic::time::from_seconds(duration);
      auto end = now + duration_chrono;

      Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
      Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
      rmf_traffic::Trajectory t1;
      t1.insert(now, pos, vel);
      t1.insert(end, pos, vel);

      rmf_traffic::Trajectory t2;
      t2.insert(now, Eigen::Vector3d(-2, 2, 0), Eigen::Vector3d(0, -2, 0));
      t2.insert(end, Eigen::Vector3d(2, 2, 0), Eigen::Vector3d(0, 2, 0));

      auto collide_timing = rmf_traffic::DetectConflict::between(profile_circle, t1, profile_circle_with_circle_offset, t2);
      ImGui::TextColored(ImVec4(0, 1, 0, 1), "rmf_traffic::DetectConflict::between results: ");
      if (collide_timing)
        ImGui::Text("Collided at %f!", rmf_traffic::time::to_seconds(*collide_timing - now));
      else
        ImGui::Text("Not collided!");

      DrawTrajectory(t1, sf::Color::Green);
      DrawTrajectory(t2, sf::Color::Red);
    }
    ImGui::End();
    ImGui::EndFrame();

    DrawCircle(sf::Vector2f(0, 0), circle_shape->get_characteristic_length(), sf::Color::Green);
    DrawAxis();
    
    app_window.clear();
    app_window.setView(view);


    for (auto& single_array : g_vertexarrays)
      app_window.draw(single_array, tx_flipped_2d);
    g_vertexarrays.clear();

    ImGui::SFML::Render(app_window);
    app_window.display();
  }
}
