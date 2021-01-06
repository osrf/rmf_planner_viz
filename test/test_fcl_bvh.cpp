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
        "Test_Fcl_BVH",
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

  double tolerance = 0.01;

  // Test collision with unit spheres vs 
  auto shape_a = std::make_shared<fcl::Sphere<double>>(0.5);
  fcl::Boxd shape_b(0.5, 0.5, 0.0);
  fcl::Boxd shape_b2(0.6, 0.6, 0.0);
  
  fcl::Transform3d shape_a2_offset, shape_b2_offset;
  shape_a2_offset.setIdentity();
  shape_b2_offset.setIdentity();
  
  auto shape_b_bvh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
  auto box_to_triangle_vertices = [](const fcl::Boxd& box, const fcl::Transform3d& pose, std::vector<fcl::Vector3d>& vertices_out, 
    std::vector<fcl::Triangle>& triangles_out)
  {
    double x_length = box.side[0];
    double y_length = box.side[1];

    vertices_out.resize(4);
    vertices_out[0] = fcl::Vector3d(-0.5 * x_length, -0.5 * y_length, 0.0);
    vertices_out[1] = fcl::Vector3d( 0.5 * x_length, -0.5 * y_length, 0.0);
    vertices_out[2] = fcl::Vector3d(-0.5 * x_length,  0.5 * y_length, 0.0);
    vertices_out[3] = fcl::Vector3d( 0.5 * x_length,  0.5 * y_length, 0.0);

    triangles_out.resize(1);
    triangles_out[0].set(0, 1, 2);
    triangles_out[1].set(1, 3, 2);

    for(unsigned int i = 0; i < vertices_out.size(); ++i)
      vertices_out[i] = pose * vertices_out[i];
  };
  
  // add shape
  {
#if 1 //this block doesnt detect a collision
    shape_b_bvh->beginModel();
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangle_indices;
    
    fcl::Transform3d identity;
    identity.setIdentity();
    box_to_triangle_vertices(shape_b, identity, vertices, triangle_indices);
    int r = shape_b_bvh->addSubModel(vertices, triangle_indices);
    if (r != fcl::BVH_OK)
      printf("failed#1\n");

    // vertices.clear();
    // triangle_indices.clear();
    // box_to_triangle_vertices(shape_b2, shape_b2_offset, vertices, triangle_indices);
    // r = shape_b_bvh->addSubModel(vertices, triangle_indices);
    // if (r != fcl::BVH_OK)
    //   printf("failed#2");
    shape_b_bvh->endModel();
#else // this block results in infinate recursions while doing collision
    fcl::Transform3d ident;
    ident.setIdentity();
    int res = fcl::generateBVHModel(*shape_b_bvh, shape_b, ident, fcl::FinalizeModel::DO);
#endif
  }

  // interp motion parameters
  Eigen::Vector3d a_start(0,0,0), a_end(0,0,0);
  double a_rot_start = 0.0, a_rot_end = 0.0;

  Eigen::Vector3d b_start(0,0,0), b_end(0,0,0);
  double b_rot_start = 0.0, b_rot_end = 0.0;

  fcl::Transform3<double> identity;
  identity.setIdentity();

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
    
    ImGui::SetWindowSize(ImVec2(800, 200));
    ImGui::Begin("Sidecar control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    
    using namespace std::chrono_literals;
    using namespace rmf_planner_viz::draw;

    {
      std::shared_ptr<fcl::MotionBase<double>> motion_a, motion_b;
      std::vector<ModelSpaceShape> a_shapes;
      std::vector<ModelSpaceShape> b_shapes;

      {
        a_shapes.emplace_back(identity, 0.5);
        b_shapes.emplace_back(identity, 0.5);

        shape_b2_offset.setIdentity();
        shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
        b_shapes.emplace_back(shape_b2_offset, 0.6);

        fcl::Transform3<double> b_start;
        b_start.setIdentity();
        b_start.translation() = fcl::Vector3d(-2, 0, 0);

        fcl::Transform3<double> b_end;
        b_end.setIdentity();
        b_end.translation() = fcl::Vector3d(0, 0, 0);

        motion_a = std::make_shared<fcl::InterpMotion<double>>(identity, identity);
        motion_b = std::make_shared<fcl::InterpMotion<double>>(b_start, b_end);
      }
      
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
      {
        const auto obj_a = fcl::ContinuousCollisionObject<double>(shape_a, motion_a);
        const auto obj_b = fcl::ContinuousCollisionObject<double>(shape_b_bvh, motion_b);
        
        fcl::ContinuousCollisionRequest<double> request;
        request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
        request.gjk_solver_type = fcl::GST_LIBCCD;

        // test for collision
        fcl::ContinuousCollisionResultd result;
        fcl::collide(&obj_a, &obj_b, request, result);

        if (result.is_collide)
          ImGui::Text("Collide! TOI: %f", result.time_of_contact);
        else
          ImGui::Text("No collision");
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

        // reset the motions
        motion_a->integrate(0.0);
        motion_b->integrate(0.0);
        
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
