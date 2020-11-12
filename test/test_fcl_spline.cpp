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

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/Motion.hpp>

#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/math/motion/spline_motion.h>

#include "imgui-SFML.h"

void draw_fcl_splinemotion(const fcl::SplineMotion<double>& motion, const sf::Color& color = sf::Color(255, 255, 255, 255))
{
  const uint steps = 100;
  double interp_per_step = 1.0f / (double)steps;
  fcl::Transform3d tf, tf2;

  for (uint i=0; i<steps; ++i)
  {
    double interp = interp_per_step * (double)i;
    double interp_next = interp_per_step * (double)(i + 1);
    
    motion.integrate(interp);
    motion.getCurrentTransform(tf);

    tf.linear();
    auto translate1 = tf.translation();    
    motion.integrate(interp_next);

    motion.getCurrentTransform(tf2);
    auto translate2 = tf2.translation();
    
    using namespace rmf_planner_viz::draw;

    IMDraw::draw_line(sf::Vector2f(translate1.x(), translate1.y()), sf::Vector2f(translate2.x(), translate2.y()), color);
  }
  motion.integrate(0.0); //reset
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
        "Test_FCL_Spline",
        sf::Style::Default);
  app_window.setFramerateLimit(60);
  app_window.resetGLStates();

  //view centered at origin, with width/height
  sf::View view(sf::Vector2f(0.f, 0.f), sf::Vector2f(20.f, 20.f));
  app_window.setView(view);

  sf::Transformable vqs;
  vqs.setScale(1.f, -1.f);
  auto tx_flipped_2d = vqs.getTransform();

  ImGui::SFML::Init(app_window);
  
  static float t[4][3];
  static float r[4][3];
  memset(t, 0, sizeof(t));
  memset(r, 0, sizeof(r));

  //some defaults
  t[0][0] = 7.5; t[0][1] = 8;
  t[1][0] = 4.2; t[1][1] = 8;
  t[2][0] = 0.8; t[2][1] = 8;
  t[3][0] = -2.5; t[3][1] = 8;

  r[0][2] = r[1][2] = r[2][2] = r[3][2] = M_PI;

  sf::Clock deltaClock;
  while (app_window.isOpen())
  {
    sf::Event event;
    while (app_window.pollEvent(event))
    {
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
    ImGui::Begin("FCL Spline control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    
    using namespace std::chrono_literals;
    using namespace rmf_planner_viz::draw;

    {
      ImGui::InputFloat3("T0", t[0]);
      ImGui::InputFloat3("T1", t[1]);
      ImGui::InputFloat3("T2", t[2]);
      ImGui::InputFloat3("T3", t[3]);

      ImGui::InputFloat3("R0", r[0]);
      ImGui::InputFloat3("R1", r[1]);
      ImGui::InputFloat3("R2", r[2]);
      ImGui::InputFloat3("R3", r[3]);
      
      ImGui::Separator();

      fcl::Vector3d t0 { t[0][0], t[0][1], t[0][2] };
      fcl::Vector3d t1 { t[1][0], t[1][1], t[1][2] };
      fcl::Vector3d t2 { t[2][0], t[2][1], t[2][2] };
      fcl::Vector3d t3 { t[3][0], t[3][1], t[3][2] };

      fcl::Vector3d r0 { r[0][0], r[0][1], r[0][2] };
      fcl::Vector3d r1 { r[1][0], r[1][1], r[1][2] };
      fcl::Vector3d r2 { r[2][0], r[2][1], r[2][2] };
      fcl::Vector3d r3 { r[3][0], r[3][1], r[3][2] };

      fcl::SplineMotion<double> motion(t0, t1, t2, t3, r0, r1, r2, r3);
      draw_fcl_splinemotion(motion, sf::Color(255, 255, 255, 255));

      t0 = fcl::Vector3d(0.0, 8.0, 0.0);
      t1 = fcl::Vector3d(1.25, 8.0, 0.0);
      t2 = fcl::Vector3d(3.0, 8.0, 0.0);
      t3 = fcl::Vector3d(4.6, 8.0, 0.0);

      r0 = fcl::Vector3d(0.0, 0.0, 0.0);
      r1 = fcl::Vector3d(0.0, 0.0, 0.0);
      r2 = fcl::Vector3d(0.0, 0.0, 0.0);
      r3 = fcl::Vector3d(0.0, 0.0, 0.0);
      
      fcl::SplineMotion<double> motion2(t0, t1, t2, t3, r0, r1, r2, r3);
      draw_fcl_splinemotion(motion2, sf::Color(255, 255, 0, 255));

      // draw robot circle on motion
      static float interp = 0.0f;
      ImGui::InputFloat("interp", &interp, 0.01);

      motion.integrate(interp);
      fcl::Transform3d tf;
      motion.getCurrentTransform(tf);
      auto pt = tf.translation();
      IMDraw::draw_circle(sf::Vector2f(pt.x(), pt.y()), circle_shape->get_characteristic_length(), sf::Color::Green);
      auto linear = tf.linear();

      auto column1 = linear.block<1, 3>(0,0);
      ImGui::Text("forward: %f %f", column1.x(), column1.y());
      //auto column2 = linear.block<1, 3>(1,0);
      auto pt_end = fcl::Vector3d(pt.x() + column1.x(), pt.y() + column1.y(), 0.0);
      IMDraw::draw_arrow(sf::Vector2f(pt.x(), pt.y()), sf::Vector2f(pt_end.x(), pt_end.y()), sf::Color::Green);
    }
    ImGui::End();
    ImGui::EndFrame();

    IMDraw::draw_axis();
    
    app_window.clear();
    app_window.setView(view);

    IMDraw::flush_and_render(app_window, tx_flipped_2d);

    ImGui::SFML::Render(app_window);
    app_window.display();
  }
}
