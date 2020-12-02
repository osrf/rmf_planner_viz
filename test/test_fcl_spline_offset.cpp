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
#include "spline_offset_utils.hpp"

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

void draw_robot_on_spline(const fcl::SplineMotion<double>& motion, double interp, double radius, const sf::Color& color = sf::Color(255, 255, 255, 255))
{
  motion.integrate(interp);
  fcl::Transform3d tf;
  motion.getCurrentTransform(tf);

  auto pt = tf.translation();
  rmf_planner_viz::draw::IMDraw::draw_circle(sf::Vector2f(pt.x(), pt.y()), radius, color);
  auto linear = tf.linear();

  auto column1 = linear.block<3, 1>(0,0);
  //ImGui::Text("forward: %f %f", column1.x(), column1.y());
  auto pt_end = fcl::Vector3d(pt.x() + column1.x(), pt.y() + column1.y(), 0.0);
  rmf_planner_viz::draw::IMDraw::draw_arrow(sf::Vector2f(pt.x(), pt.y()), sf::Vector2f(pt_end.x(), pt_end.y()), color);
}

void draw_sidecar_motion(const fcl::SplineMotion<double>& original_motion,
  const fcl::Transform3d& tx_offset, const sf::Color& color = sf::Color(255, 255, 255, 255))
{
  const uint steps = 100;
  double interp_per_step = 1.0f / (double)steps;
  fcl::Transform3d tf, tf2;

  for (uint i=0; i<steps; ++i)
  {
    double interp = interp_per_step * (double)i;
    double interp_next = interp_per_step * (double)(i + 1);
    
    original_motion.integrate(interp);
    original_motion.getCurrentTransform(tf);

    auto tf_offset = tf * tx_offset;
    auto translate1 = tf_offset.translation();

    original_motion.integrate(interp_next);
    original_motion.getCurrentTransform(tf2);

    auto tf2_offset = tf2 * tx_offset;
    auto translate2 = tf2_offset.translation();
    
    using namespace rmf_planner_viz::draw;

    IMDraw::draw_line(sf::Vector2f(translate1.x(), translate1.y()), sf::Vector2f(translate2.x(), translate2.y()), color);
  }
  original_motion.integrate(0.0); //reset
}

void draw_catmull_rom(
  Eigen::Vector3d p0,
  Eigen::Vector3d p1,
  Eigen::Vector3d p2,
  Eigen::Vector3d p3,
  const sf::Color& color = sf::Color(255, 255, 255, 255))
{
  const uint steps = 100;
  double t_per_step = 1.0f / (double)steps;
  
  for (uint i=0; i<steps; ++i)
  {
    auto point_on_catmullrom_spline = [&](double t)
    {
      double t_sq = t * t;
      double t_cube = t * t * t;

      Eigen::Vector3d p = (2.0 * p1) + (-p0 + p2) * t + 
        (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t_sq +
        (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t_cube;
      p = 0.5 * p;
      return p;
    };
    double t = t_per_step * (double)i;
    auto start = point_on_catmullrom_spline(t);

    double t_next = t_per_step * (double)(i + 1);
    auto end = point_on_catmullrom_spline(t_next);

    using namespace rmf_planner_viz::draw;

    IMDraw::draw_line(sf::Vector2f(start.x(), start.y()), sf::Vector2f(end.x(), end.y()), color);
  }
}

void draw_offset_sidecar_on_spline(const fcl::SplineMotion<double>& motion, double interp, double radius,
  const fcl::Transform3d& tx_offset, const sf::Color& color = sf::Color(255, 255, 255, 255))
{
  motion.integrate(interp);
  fcl::Transform3d tf;
  motion.getCurrentTransform(tf);

  auto sidecar_tx = tf * tx_offset;
  auto sidecar_pt = sidecar_tx.translation();
  rmf_planner_viz::draw::IMDraw::draw_circle(sf::Vector2f(sidecar_pt.x(), sidecar_pt.y()), radius, color);
}

int main()
{
  // square window to avoid stretching
  sf::RenderWindow app_window(
        sf::VideoMode(1024, 1024),
        "Test_FCL_SplineOffset",
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

  auto to_fcl = [](const std::array<Eigen::Vector3d, 4>& knots) {
    std::array<Eigen::Vector3d, 4> Td;
    std::array<Eigen::Vector3d, 4> Rd;

    for (std::size_t i = 0; i < 4; ++i) {
      const Eigen::Vector3d p = knots[i];
      Td[i] = Eigen::Vector3d(p[0], p[1], 0.0);
      Rd[i] = Eigen::Vector3d(0.0, 0.0, p[2]);
    }

    return fcl::SplineMotion<double>(Td[0], Td[1], Td[2], Td[3], Rd[0], Rd[1], Rd[2],
                             Rd[3]);
  };

  // Test collision with unit spheres
  auto shape_a = std::make_shared<fcl::Sphere<double>>(0.5);
  auto shape_b = std::make_shared<fcl::Sphere<double>>(0.5);
  auto shape_b2 = std::make_shared<fcl::Sphere<double>>(0.6);

  fcl::Transform3d shape_b2_offset;
  shape_b2_offset.setIdentity();
  shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));

  sf::Color dark_green_color(75, 111, 68);

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
      // ImGui::Text("knots_b[0]: %f %f %f", knots_b[0][0], knots_b[0][1], knots_b[0][2]);
      // ImGui::Text("knots_b[1]: %f %f %f", knots_b[1][0], knots_b[1][1], knots_b[1][2]);
      // ImGui::Text("knots_b[2]: %f %f %f", knots_b[2][0], knots_b[2][1], knots_b[2][2]);
      // ImGui::Text("knots_b[3]: %f %f %f", knots_b[3][0], knots_b[3][1], knots_b[3][2]);
      std::shared_ptr<fcl::SplineMotion<double>> motion_a, motion_b;
      static int current_preset = 0;
      if (ImGui::Button("Preset #0"))
        current_preset = 0;
      if (ImGui::Button("Preset #1"))
        current_preset = 1;
      if (ImGui::Button("Preset #2"))
        current_preset = 2;
      
      ImGui::Separator();

      if (current_preset == 0)
      {
        auto knots_a =
          rmf_planner_viz::draw::compute_knots(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0),
                        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

        auto knots_b =
            rmf_planner_viz::draw::compute_knots(Eigen::Vector3d(-2, 0, 0), Eigen::Vector3d(-2, 0, EIGEN_PI / 2.0),
                          Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

        motion_a = std::make_shared<fcl::SplineMotion<double>>(to_fcl(knots_a));
        motion_b = std::make_shared<fcl::SplineMotion<double>>(to_fcl(knots_b));
      }
      else if (current_preset == 1)
      {
        auto knots_a =
          rmf_planner_viz::draw::compute_knots(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0),
                        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

        auto knots_b =
            rmf_planner_viz::draw::compute_knots(Eigen::Vector3d(-5 - 0.15, 0, 0), Eigen::Vector3d(-2 + 0.15, 0, EIGEN_PI / 2.0),
                          Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

        motion_a = std::make_shared<fcl::SplineMotion<double>>(to_fcl(knots_a));
        motion_b = std::make_shared<fcl::SplineMotion<double>>(to_fcl(knots_b));
      }
      else if (current_preset == 2)
      {
        auto knots_a =
          rmf_planner_viz::draw::compute_knots(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0),
                        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

        auto knots_b =
            rmf_planner_viz::draw::compute_knots(Eigen::Vector3d(-5 - 0.15, 0, 0), Eigen::Vector3d(-2 + 0.15, 0, EIGEN_PI / 2.0),
                          Eigen::Vector3d(0, 16, 0), Eigen::Vector3d(0, -16, 0));

        motion_a = std::make_shared<fcl::SplineMotion<double>>(to_fcl(knots_a));
        motion_b = std::make_shared<fcl::SplineMotion<double>>(to_fcl(knots_b));
      }
      
      {
        const auto obj_a = fcl::ContinuousCollisionObject<double>(shape_a, motion_a);
        const auto obj_b = fcl::ContinuousCollisionObject<double>(shape_b, motion_b);
        const auto obj_b2 = fcl::ContinuousCollisionObject<double>(shape_b2, motion_b);

        fcl::ContinuousCollisionRequest<double> request;
        request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
        request.gjk_solver_type = fcl::GST_INDEP;

        // reset the motions
        motion_a->integrate(0.0);
        motion_b->integrate(0.0);

        // test for collision
        fcl::Transform3d identity_offset;
        identity_offset.setIdentity();
        fcl::ContinuousCollisionResultd result;
        collide_shapes_with_offset<double>(
          obj_a.collisionGeometry().get(), obj_a.getMotion(), identity_offset,
          obj_b2.collisionGeometry().get(), obj_b2.getMotion(), shape_b2_offset,
          result);
          
        ImGui::Text("collide_shapes_with_offset results:");
        if (result.is_collide)
          ImGui::Text("Collide! TOI: %f", result.time_of_contact);
        else
          ImGui::Text("No collision");
        ImGui::Separator();

        // draw motions of both splines
        draw_fcl_splinemotion(*motion_a, sf::Color::Red);
        draw_fcl_splinemotion(*motion_b, dark_green_color);

        // draw robot circle on motion
        static float interp = 0.0f;
        ImGui::InputFloat("interp", &interp, 0.01);

        draw_robot_on_spline(*motion_a, interp, circle_shape->get_characteristic_length(), sf::Color::Red);
        draw_robot_on_spline(*motion_b, interp, circle_shape->get_characteristic_length(), dark_green_color);
        
        draw_offset_sidecar_on_spline(*motion_b, interp, circle_shape_ex->get_characteristic_length(), shape_b2_offset, sf::Color::Green);
      }
      
      // approximated
      {
        const int steps = 3;
        const int point_count = steps + 1;
        fcl::Vector3d points[point_count];
        float interp = 0.0f;

        //draw points and multiply in shape_b2_offset
        for (uint i=0; i<=steps; ++i)
        {
          interp = (float)i * (1.0f / (float)steps);
          motion_b->integrate(interp);
          fcl::Transform3d tf;
          motion_b->getCurrentTransform(tf);

          // apply offset
          auto sidecar_tx = tf * shape_b2_offset;
          auto sidecar_pt = sidecar_tx.translation();
          points[i] = sidecar_pt;

          IMDraw::draw_circle(sf::Vector2f(sidecar_pt.x(), sidecar_pt.y()), 0.0625f, sf::Color::White);
        }

        static bool show_sidecar_motion = true;
        ImGui::Checkbox("Show sampled sidecar motion", &show_sidecar_motion);
        if (show_sidecar_motion)
          draw_sidecar_motion(*motion_b, shape_b2_offset, sf::Color::Green);

        static bool show_approximated_sidecar_motion = true;
        ImGui::Checkbox("Show sampled sidecar catmull rom", &show_approximated_sidecar_motion);
        if (show_approximated_sidecar_motion)
          draw_catmull_rom(points[0], points[1], points[2], points[3], sf::Color(128,128,128));

        static bool show_control_poly = false;
        auto convert = [show_control_poly](Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) -> fcl::SplineMotion<double> 
        {
          // @reference
          // https://computergraphics.stackexchange.com/questions/8267/conversion-from-cubic-catmull-rom-spline-to-cubic-b-spline
          // the topright and botleft values of the matrix is wrong, should be negative

          Eigen::Matrix4d mtx_catmullrom;
          mtx_catmullrom << 
            -1.0,  3.0, -3.0,  1.0,
             2.0, -5.0,  4.0, -1.0,
            -1.0,  0.0,  1.0,  0.0,
             0.0,  2.0,  0.0,  0.0;
          mtx_catmullrom *= 0.5;

          Eigen::Matrix4d mtx_bspline;
          mtx_bspline << 
            -1.0,  3.0, -3.0,  1.0,
             3.0, -6.0,  3.0,  0.0,
            -3.0,  0.0,  3.0,  0.0,
             1.0,  4.0,  1.0,  0.0;
          mtx_bspline /= 6.0;
          Eigen::Matrix4d mtx_bspline_inv = mtx_bspline.inverse();

          Eigen::Matrix<double, 4, 3> input;
          input << 
            p0[0], p0[1], p0[2],
            p1[0], p1[1], p1[2],
            p2[0], p2[1], p2[2],
            p3[0], p3[1], p3[2];

          auto result = mtx_bspline_inv * mtx_catmullrom * input; // 4x3 matrix
          Eigen::Vector3d bspline_p0(result(0, 0), result(0, 1), result(0, 2));
          Eigen::Vector3d bspline_p1(result(1, 0), result(1, 1), result(1, 2));
          Eigen::Vector3d bspline_p2(result(2, 0), result(2, 1), result(2, 2));
          Eigen::Vector3d bspline_p3(result(3, 0), result(3, 1), result(3, 2));

          if (show_control_poly)
          {
            IMDraw::draw_circle(sf::Vector2f(bspline_p0.x(), bspline_p0.y()), 0.0625f, sf::Color(128,128,128));
            IMDraw::draw_circle(sf::Vector2f(bspline_p1.x(), bspline_p1.y()), 0.0625f, sf::Color(128,128,128));
            IMDraw::draw_circle(sf::Vector2f(bspline_p2.x(), bspline_p2.y()), 0.0625f, sf::Color(128,128,128));
            IMDraw::draw_circle(sf::Vector2f(bspline_p3.x(), bspline_p3.y()), 0.0625f, sf::Color(128,128,128));

            IMDraw::draw_line(sf::Vector2f(bspline_p0.x(), bspline_p0.y()), sf::Vector2f(bspline_p1.x(), bspline_p1.y()), sf::Color(128,128,128));
            IMDraw::draw_line(sf::Vector2f(bspline_p1.x(), bspline_p1.y()), sf::Vector2f(bspline_p2.x(), bspline_p2.y()), sf::Color(128,128,128));
            IMDraw::draw_line(sf::Vector2f(bspline_p2.x(), bspline_p2.y()), sf::Vector2f(bspline_p3.x(), bspline_p3.y()), sf::Color(128,128,128));
          }

          const Eigen::Vector3d zero = Eigen::Vector3d(0,0,0);
          return fcl::SplineMotion<double>(
            bspline_p0, bspline_p1, bspline_p2, bspline_p3,
            zero, zero, zero, zero);
        };

        auto motion_b2_approximated = std::make_shared<fcl::SplineMotion<double>>(
          convert(points[0], points[1], points[2], points[3]));

        static bool show_fcl_sidecar_motion = true;
        ImGui::Checkbox("Show fcl sidecar motion", &show_fcl_sidecar_motion);
        if (show_fcl_sidecar_motion)
          draw_fcl_splinemotion(*motion_b2_approximated, sf::Color::White);

        ImGui::Checkbox("Show control polygon", &show_control_poly);

        ImGui::Separator();
        {
          fcl::ContinuousCollisionRequest<double> request;
          request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
          request.gjk_solver_type = fcl::GST_INDEP;
          
          fcl::ContinuousCollisionResultd result;
          const auto obj_a = fcl::ContinuousCollisionObjectd(
            shape_a, motion_a);
          const auto obj_b = fcl::ContinuousCollisionObjectd(
            shape_b2, motion_b2_approximated);
          fcl::collide(&obj_a, &obj_b, request, result);

          ImGui::Text("fcl::collide with approximated sidecar spline results:");
          if (result.is_collide)
            ImGui::Text("Collide! TOI: %f", result.time_of_contact);
          else
            ImGui::Text("No collision");
        }

        ImGui::Separator();
      }
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
