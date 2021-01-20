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
#include <vector>
#include <chrono>

#include <rmf_planner_viz/draw/Graph.hpp>
#include <rmf_planner_viz/draw/Schedule.hpp>
#include <rmf_planner_viz/draw/IMDraw.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/Motion.hpp>

#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/math/motion/spline_motion.h>
#include <fcl/geometry/bvh/BVH_internal.h>
#include <fcl/geometry/geometric_shape_to_BVH_model.h>

#include "imgui-SFML.h"
#include "test_sidecar_utils.hpp"
#include "spline_offset_utils.hpp"

//#define PROFILING_USE_RDTSC 1
#ifdef PROFILING_USE_RDTSC
#include <cpuid.h>
#include <x86intrin.h>
#endif

void draw_fcl_motion(fcl::MotionBase<double>* motion, const sf::Color& color = sf::Color(255, 255, 255, 255))
{
  const uint steps = 100;
  double interp_per_step = 1.0f / (double)steps;
  fcl::Transform3d tf, tf2;

  for (uint i=0; i<steps; ++i)
  {
    double interp = interp_per_step * (double)i;
    double interp_next = interp_per_step * (double)(i + 1);
    
    motion->integrate(interp);
    motion->getCurrentTransform(tf);

    tf.linear();
    auto translate1 = tf.translation();    
    motion->integrate(interp_next);

    motion->getCurrentTransform(tf2);
    auto translate2 = tf2.translation();
    
    using namespace rmf_planner_viz::draw;

    IMDraw::draw_line(sf::Vector2f(translate1.x(), translate1.y()), sf::Vector2f(translate2.x(), translate2.y()), color);
  }
  motion->integrate(0.0); //reset
}

void draw_robot_on_spline(fcl::MotionBase<double>* motion, double interp, 
  const std::vector<rmf_planner_viz::draw::ModelSpaceShape>& shapes,
  const sf::Color& color = sf::Color(255, 255, 255, 255))
{
  if (shapes.empty())
    return;
  motion->integrate(interp);
  fcl::Transform3d tf;
  motion->getCurrentTransform(tf);

  for (uint i=0; i<shapes.size(); ++i)
  {
    auto tx = tf * shapes[i]._transform;
    auto pt = tx.translation();
    rmf_planner_viz::draw::IMDraw::draw_circle(sf::Vector2f(pt.x(), pt.y()), 
      shapes[i]._radius, color);
    
    if (i == 0)
    {
      auto linear = tx.linear();
      auto column1 = linear.block<3, 1>(0,0);
      //ImGui::Text("forward: %f %f", column1.x(), column1.y());
      auto pt_end = fcl::Vector3d(pt.x() + column1.x(), pt.y() + column1.y(), 0.0);
      rmf_planner_viz::draw::IMDraw::draw_arrow(sf::Vector2f(pt.x(), pt.y()), sf::Vector2f(pt_end.x(), pt_end.y()), color);
    }
  }
}

int main()
{
  // square window to avoid stretching
  sf::RenderWindow app_window(
        sf::VideoMode(1024, 1024),
        "Test_Sidecar",
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

  /// Setup the collision scenario with robot offsets
  const auto circle_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(0.5);
  const auto circle_shape_ex = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(0.6);
    
  rmf_traffic::Profile profile_circle { circle_shape };
  rmf_traffic::Profile profile_circle_with_circle_offset { circle_shape };

  double tolerance = 0.01;
  std::vector<rmf_planner_viz::draw::Preset> presets;
  presets = rmf_planner_viz::draw::setup_presets();

  // Test collision with unit spheres vs 
  auto shape_a = std::make_shared<fcl::Sphere<double>>(0.5);
  
  // auto shape_b = std::make_shared<fcl::Sphere<double>>(0.5);
  // auto shape_b2 = std::make_shared<fcl::Sphere<double>>(0.6);
  
  // interp motion parameters
  Eigen::Vector3d a_start(0,0,0), a_end(0,0,0);

  Eigen::Vector3d b_start(0,0,0), b_end(0,0,0);

  Eigen::Vector3d b_start_pos(0,0,0);
  Eigen::Vector3d b_end_pos(0,0,0);

  fcl::Transform3<double> identity;
  identity.setIdentity();
  bool b_start_override = false;
  bool b_end_override = false;
  bool preset_changed = true;

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

      auto& io = ImGui::GetIO();
      if (!io.WantCaptureMouse)
      {
        if (event.type == sf::Event::MouseButtonPressed)
        {
          sf::Vector2f px = app_window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
          if (event.mouseButton.button == sf::Mouse::Button::Left)
          {
            b_start_override = true;
            b_start_pos = Eigen::Vector3d((double)px.x, (double)px.y * -1.f, 0.0);
            preset_changed = true;
          }
          if (event.mouseButton.button == sf::Mouse::Button::Right)
          {
            b_end_override = true;
            b_end_pos = Eigen::Vector3d((double)px.x, (double)px.y * -1.f, 0.0);
            preset_changed = true;
          }
        }
      }
    }

    ImGui::SFML::Update(app_window, deltaClock.restart());
    
    ImGui::SetWindowSize(ImVec2(800, 200));
    ImGui::Begin("Sidecar control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("Use LMB/RMB to adjust start/end positions");
    
    using namespace std::chrono_literals;
    using namespace rmf_planner_viz::draw;

    {
      static std::shared_ptr<fcl::MotionBase<double>> motion_a, motion_b;
      static std::vector<ModelSpaceShape> a_shapes;
      static std::vector<ModelSpaceShape> b_shapes;
      PRESET_TYPE preset_type = PRESET_SPLINEMOTION;
      static int current_preset = 0;

      std::string preview_val = std::to_string(current_preset);
      if (current_preset != -1 && current_preset < (int)presets.size())
        preview_val = "#" + std::to_string(current_preset) + " (" + presets[current_preset]._description + ")";

      if (ImGui::BeginCombo("Preset", preview_val.c_str()))
      {
        for (int i=0; i<(int)presets.size(); ++i)
        {
          std::string btn_text = "#" + std::to_string(i) + " (" + presets[i]._description + ")";
          if (ImGui::Selectable(btn_text.c_str(), current_preset == i))
          {
            current_preset = i;
            preset_changed = true;
            b_start_override = false;
            b_end_override = false;
          }
        }
        ImGui::EndCombo();
      }

      if (preset_changed)
      {
        if (current_preset != -1 && current_preset < (int)presets.size())
        {
          const auto& preset = presets[current_preset];

          a_shapes.clear();
          b_shapes.clear();
          a_shapes = preset.a_shapes;
          b_shapes = preset.b_shapes;

          Eigen::Vector3d a_start = preset.a_start;
          Eigen::Vector3d a_end = preset.a_end;

          Eigen::Vector3d b_start = preset.b_start;
          Eigen::Vector3d b_end = preset.b_end;

          if (b_start_override)
          {
            b_start.x() = b_start_pos.x();
            b_start.y() = b_start_pos.y();
          }
          if (b_end_override)
          {
            b_end.x() = b_end_pos.x();
            b_end.y() = b_end_pos.y();
          }

          Eigen::Vector3d b_vel = preset.b_vel;
          Eigen::Vector3d zero(0,0,0);

          tolerance = preset.tolerance;

          auto knots_a =
            rmf_planner_viz::draw::compute_knots(a_start, a_end, zero, zero);
          auto knots_b =
            rmf_planner_viz::draw::compute_knots(b_start, b_end, b_vel, -b_vel);

          motion_a = std::make_shared<fcl::SplineMotion<double>>(to_fcl(knots_a));
          motion_b = std::make_shared<fcl::SplineMotion<double>>(to_fcl(knots_b));

          preset_type = preset._type;
        }
        preset_changed = false;
      }
    
      ImGui::Separator();
      ImGui::Text("Preset: %d", current_preset);
      ImGui::Separator();
      
      static bool tolerance_override = false;
      static float tol = 0.01;
      ImGui::Checkbox("Override Tolerance", &tolerance_override);
      ImGui::InputFloat("Tolerance", &tol, 0.001);
      if (tolerance_override)
        tolerance = (double)tol;
      ImGui::Text("Tolerance: %f", tolerance);
      ImGui::Separator();

      static bool draw_toi_shapes = true;
      ImGui::Checkbox("Draw TOI shapes", &draw_toi_shapes);

      sf::Color toi_green_color(3, 125, 88);
      sf::Color toi_red_color(178, 34, 34);

      // collision
      static bool use_fcl = false;
      if (presets.size() && current_preset == (presets.size() - 1))
        ImGui::Checkbox("Use FCL only", &use_fcl);
      else
        use_fcl = false;

      if (use_fcl && presets.size() && current_preset == (presets.size() - 1))
      {
        auto& preset = presets[current_preset];

        auto shape_a = std::make_shared<fcl::Sphered>(preset.a_shapes[0]._radius);
        auto shape_b = std::make_shared<fcl::Sphered>(preset.b_shapes[0]._radius);
        
        const auto obj_a = fcl::ContinuousCollisionObjectd(shape_a, motion_a);
        const auto obj_b = fcl::ContinuousCollisionObjectd(shape_b, motion_b);

        fcl::ContinuousCollisionRequestd request;
        request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
        request.gjk_solver_type = fcl::GST_LIBCCD;
#ifdef PROFILING_USE_RDTSC
        int v = 0;
        __cpuid(v,v,v,v,v);
        uint64_t start = __rdtsc();
#else
        auto start = std::chrono::high_resolution_clock::now();
#endif 
        fcl::ContinuousCollisionResultd result;
        fcl::collide(&obj_a, &obj_b, request, result);
#ifdef PROFILING_USE_RDTSC
        uint64_t end = __rdtsc();
#else
        auto end = std::chrono::high_resolution_clock::now();
#endif
        if (result.is_collide)
        {
          ImGui::Text("Collide! TOI: %f", result.time_of_contact);
          //if (draw_toi_shapes)
          { 
            draw_robot_on_spline(motion_a.get(), result.time_of_contact, a_shapes, toi_red_color);
            draw_robot_on_spline(motion_b.get(), result.time_of_contact, b_shapes, toi_green_color);
          }
        }
        else
          ImGui::Text("No collision");
#ifdef PROFILING_USE_RDTSC
        ImGui::Text("Clock cycles: %010llu", end - start);
#else
        auto duration = end - start;
        
        //std::chrono::duration<double, std::nano> dur = end - start;
        //std::chrono::duration<double, std::micro> dur = end - start;
        std::chrono::duration<double, std::milli> dur = end - start;
        double val = dur.count();
        ImGui::Text("Time taken (ms): %.10g", val);
#endif
      }
      else if (preset_type == PRESET_SPLINEMOTION)
      {
#ifdef PROFILING_USE_RDTSC
        int v = 0;
        __cpuid(v,v,v,v,v);
        uint64_t start = __rdtsc();
#else
        auto start = std::chrono::high_resolution_clock::now();
#endif

        double toi = 0.0;
        uint dist_checks = 0;
        bool collide = collide_seperable_circles(
          *(fcl::SplineMotion<double>*)motion_a.get(),
          *(fcl::SplineMotion<double>*)motion_b.get(),
          a_shapes, b_shapes,
          toi, dist_checks, 120, (double)tolerance);
#ifdef PROFILING_USE_RDTSC
        uint64_t end = __rdtsc();
#else
        auto end = std::chrono::high_resolution_clock::now();
#endif
        if (collide)
        {
          ImGui::Text("Collide! TOI: %f", toi);
          if (draw_toi_shapes)
          {
            draw_robot_on_spline(motion_a.get(), toi, a_shapes, toi_red_color);
            draw_robot_on_spline(motion_b.get(), toi, b_shapes, toi_green_color);
          }
        }
        else
          ImGui::Text("No collision");
#ifdef PROFILING_USE_RDTSC
        ImGui::Text("Clock cycles: %010llu", end - start);
#else
        auto duration = end - start;
        
        //std::chrono::duration<double, std::nano> dur = end - start;
        //std::chrono::duration<double, std::micro> dur = end - start;
        std::chrono::duration<double, std::milli> dur = end - start;
        double val = dur.count();
        ImGui::Text("Time taken (ms): %.10g", val);
#endif
        ImGui::Text("Distance checks: %d", dist_checks);
      }

      // reset the motions
      motion_a->integrate(0.0);
      motion_b->integrate(0.0);

      ImGui::Separator();

      // draw robot circle on motion
      static float interp = 0.0f;
      ImGui::SliderFloat("interp", &interp, 0.0f, 1.0f);
      
      // draw motions of both splines
      {
        draw_fcl_motion(motion_a.get(), sf::Color::Red);
        draw_fcl_motion(motion_b.get(), sf::Color::Green);

        draw_robot_on_spline(motion_a.get(), interp, a_shapes, sf::Color::Red);
        draw_robot_on_spline(motion_b.get(), interp, b_shapes, sf::Color::Green);
      }
    }

    static bool draw_axis = false;
    ImGui::Checkbox("draw axis", &draw_axis);

    ImGui::End();
    ImGui::EndFrame();

    if (draw_axis)
      IMDraw::draw_axis();
    
    app_window.clear();
    app_window.setView(view);

    IMDraw::flush_and_render(app_window, tx_flipped_2d);

    ImGui::SFML::Render(app_window);
    app_window.display();
  }
}
