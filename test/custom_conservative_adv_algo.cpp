
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

#include "custom_conservative_adv_algo.hpp"

#include <SFML/Graphics.hpp>
#include <imgui.h>

#include <fcl/math/motion/interp_motion.h>

#include "imgui-SFML.h"

namespace rmf_planner_viz {
namespace draw {

bool CA_collide_seperable_circles(
  Eigen::Vector3d a_start, Eigen::Vector3d a_end, double a_rot_start, double a_rot_end,
  Eigen::Vector3d b_start, Eigen::Vector3d b_end, double b_rot_start, double b_rot_end,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  double& impact_time, double tolerance)
{
  if (a_shapes.empty() || b_shapes.empty())
    return false;

  auto calc_min_dist = [](
    Eigen::Vector3d a, double a_rot,
    Eigen::Vector3d b, double b_rot,
    const std::vector<ModelSpaceShape>& a_shapes,
    const std::vector<ModelSpaceShape>& b_shapes,
    Eigen::Vector3d& d, double& min_dist)
  {
    fcl::Transform3d a_tx, b_tx;
    
    a_tx.setIdentity();
    a_tx.prerotate(fcl::AngleAxis<double>(a_rot, Eigen::Vector3d::UnitZ()));
    a_tx.pretranslate(a);

    b_tx.setIdentity();
    b_tx.prerotate(fcl::AngleAxis<double>(b_rot, Eigen::Vector3d::UnitZ()));
    b_tx.pretranslate(b);
    
    min_dist = DBL_MAX;
    for (const auto& a_shape : a_shapes)
    {
      auto a_shape_tx = a_tx * a_shape._transform;

      for (const auto& b_shape : b_shapes)
      {
        auto b_shape_tx = b_tx * b_shape._transform;
        Eigen::Vector3d b_to_a = a_shape_tx.translation() - b_shape_tx.translation();
        double dist = b_to_a.norm() - (a_shape._radius + b_shape._radius);
        if (dist < min_dist)
        {
          min_dist = dist;
          d = b_to_a;
        }
      }
    }
  };
  auto get_furthest_point_dist = [](
    Eigen::Vector3d start, double rot_start,
    const std::vector<ModelSpaceShape>& shapes)
  {
    fcl::Transform3d tx;
    
    tx.setIdentity();
    tx.prerotate(fcl::AngleAxis<double>(rot_start, Eigen::Vector3d::UnitZ()));
    tx.pretranslate(start);

    double d = 0.0;
    for (const auto& shape : shapes)
    {
      auto shape_tx = tx * shape._transform;

      double furthest_dist = (shape_tx.translation() - start).norm() + shape._radius;
      if (furthest_dist > d)
        d = furthest_dist;
    }
    return d;
  };

  Eigen::Vector3d a_vstep = a_end - a_start;
  Eigen::Vector3d b_vstep = b_end - b_start;
  Eigen::Vector3d v_step = b_vstep - a_vstep;

  double dist_along_d_to_cover = 0.0;
  Eigen::Vector3d d(0,0,0);
  calc_min_dist(a_start, a_rot_start, b_start, b_rot_start, a_shapes, b_shapes,
    d, dist_along_d_to_cover);

  double a_rot_diff = a_rot_end - a_rot_start;
  double a_furthest_pt_dist = get_furthest_point_dist(a_start, a_rot_start, a_shapes);

  double b_rot_diff = b_rot_end - b_rot_start;
  double b_furthest_pt_dist = get_furthest_point_dist(b_start, b_rot_start, b_shapes);
  //printf("a:%f b:%f\n", a_furthest_pt_dist, b_furthest_pt_dist);
  //printf("adiff:%f bdiff:%f\n", a_rot_diff, b_rot_diff);

  double t = 0.0;
  uint iter = 0;
  while (dist_along_d_to_cover > tolerance && t < 1.0)
  {
    //printf("======= iter:%d\n", iter);
    Eigen::Vector3d d_normalized = d.normalized();

    // original conservative adv algorithm for 1 object. 
    // It breaks on 2 equivalent rotation on the spot sidecars
    // ie. the terms (a_furthest_pt_dist * a_rot_diff + b_furthest_pt_dist * b_rot_diff)
    // cancel each other out

    // double vel_bound = d_normalized.dot(v_step) + 
    //   a_furthest_pt_dist * a_rot_diff + b_furthest_pt_dist * b_rot_diff;
    // double delta = abs(dist) / vel_bound;
    // t += delta;

    // bilateral adv
    // since we're using seperable shapes, 
    // we're testing for the smallest dist we can go with a discontinuous function
    auto bilateral_adv = [&](double current_t, const Eigen::Vector3d& d_normalized, double dist_to_cover)
    {
      double lower_t_limit = current_t;
      double upper_t_limit = 1.0f;

      double prev_dist_abs = DBL_MAX;
      double t_at_prev_dist_abs = current_t;
      uint bilateral_adv_iter = 0;
      do
      {
        double sample_t = lower_t_limit + 0.5 * (upper_t_limit - lower_t_limit);
        printf("iter: %d, sample_t: %f\n", bilateral_adv_iter, sample_t);

        Eigen::Vector3d a = a_start + a_vstep * sample_t;
        Eigen::Vector3d b = b_start + b_vstep * sample_t;
        double a_rot = a_rot_start + a_rot_diff * sample_t;
        double b_rot = b_rot_start + b_rot_diff * sample_t;

        double dist_along_vec;
        fcl::Transform3d a_tx, b_tx;
        
        a_tx.setIdentity();
        a_tx.prerotate(fcl::AngleAxis<double>(a_rot, Eigen::Vector3d::UnitZ()));
        a_tx.pretranslate(a);

        b_tx.setIdentity();
        b_tx.prerotate(fcl::AngleAxis<double>(b_rot, Eigen::Vector3d::UnitZ()));
        b_tx.pretranslate(b);
        
        double dist_output = DBL_MAX;
        // our piecewise distance function
        for (const auto& a_shape : a_shapes)
        {
          auto a_shape_tx = a_tx * a_shape._transform;
          for (const auto& b_shape : b_shapes)
          {
            auto b_shape_tx = b_tx * b_shape._transform;
            Eigen::Vector3d b_to_a = a_shape_tx.translation() - b_shape_tx.translation();
            auto b_to_a_norm = b_to_a / b_to_a.norm();

            double dist_along_b_to_a = b_to_a.norm() - (a_shape._radius + b_shape._radius);
            auto v = dist_along_b_to_a * b_to_a_norm;
            double dist_along_d = v.dot(d_normalized);

            if (dist_along_d < dist_output)
              dist_output = dist_along_d;

            if (b_shape._radius == 0.6 && a_shape._radius == 0.6)
              printf("a2b2 dist: %f\n", dist_along_d);
          }
        }

        if (abs(dist_output) > prev_dist_abs)
        {
          printf("abs distance increased from the previous sample, reverting to t_at_prev_dist_abs: %f\n", t_at_prev_dist_abs); 
          return t_at_prev_dist_abs;
        }
        prev_dist_abs = abs(dist_output);
        t_at_prev_dist_abs = sample_t;

        printf("dist_output: %f\n", dist_output);
        if (abs(dist_output) < tolerance)
        {
          printf("minimal dist within tolerance range %f\n", tolerance);
          return sample_t;
        }
        if ((upper_t_limit - lower_t_limit) < tolerance)
        {
          printf("range too small\n");
          return sample_t;
        }
        
        if (dist_output < 0.0)
          upper_t_limit = sample_t;
        else if (dist_output > 0.0)
          lower_t_limit = sample_t;

        ++bilateral_adv_iter;
      } while (1);
    };

    t = bilateral_adv(t, d_normalized, dist_along_d_to_cover);

    Eigen::Vector3d a = a_start + a_vstep * t;
    Eigen::Vector3d b = b_start + b_vstep * t;
    double a_rot = a_rot_start + a_rot_diff * t;
    double b_rot = b_rot_start + b_rot_diff * t;

    calc_min_dist(a, a_rot, b, b_rot, a_shapes, b_shapes,
      d, dist_along_d_to_cover);
    
    // printf("dist_along_d_to_cover: %f\n", dist_along_d_to_cover);
    // printf("vel_bound %f, delta: %f t: %f dist: %f\n", vel_bound, delta, t, dist);
    ++iter;
    // if (iter > 2)
    //   break;
  }
  
  if (t < 1.0)
  {
    impact_time = t;
    return true;
  }
  return false;
}

bool CA_collide_seperable_circles(
  CustomSplineMotion motion_a, double a_radius,
  CustomSplineMotion motion_b, double b_radius,
  fcl::Transform3d b2_offset, double b2_radius,
  double& impact_time, double tolerance)
{
  auto calc_min_dist = [](
    fcl::Transform3d a_tf, double a_radius, 
    fcl::Transform3d b_tf, double b_radius,
    fcl::Transform3d b2_offset, double b2_radius,
    Eigen::Vector3d& d, double& dist)
  {
    Eigen::Vector3d a = a_tf.translation();
    Eigen::Vector3d b = b_tf.translation();
    
    auto b2_tx = b_tf * b2_offset;
    Eigen::Vector3d b2 = b2_tx.translation();
    std::cout << "b2:\n" << b2 << std::endl;
    
    Eigen::Vector3d b_to_a = a - b;
    double b_to_a_dist = b_to_a.norm();
    double d1 = b_to_a_dist - (a_radius + b_radius);

    Eigen::Vector3d b2_to_a = a - b2;
    double b2_to_a_dist = b2_to_a.norm();
    double d2 = b2_to_a_dist - (a_radius + b2_radius);
    if (d1 < d2)
    {
      printf("b is closer (dist: %f)\n", d1);
      dist = d1;
      d = b_to_a;
    }
    else
    {
      printf("b2 is closer (dist: %f)\n", d2);
      dist = d2;
      d = b2_to_a;
    }
  };

  fcl::Transform3d a_start_tf, b_start_tf;
  fcl::Transform3d a_tf, b_tf;

  motion_a.integrate(0.0);
  motion_b.integrate(0.0);
  motion_a.getCurrentTransform(a_start_tf);
  auto a_start = a_start_tf.translation();
  motion_b.getCurrentTransform(b_start_tf);
  auto b_start = b_start_tf.translation();

  motion_a.integrate(1.0);
  motion_b.integrate(1.0);
  motion_a.getCurrentTransform(a_tf);
  auto a_end = a_tf.translation();
  motion_b.getCurrentTransform(b_tf);
  auto b_end = b_tf.translation();

  double dist = 0.0;
  Eigen::Vector3d d(0,0,0);
  calc_min_dist(a_start_tf, a_radius, b_start_tf, b_radius,
    b2_offset, b2_radius, d, dist);

  motion_a.integrate(0.0);
  motion_b.integrate(0.0);
  
  double t = 0.0;
  uint iter = 0;
  while (dist > tolerance && t < 1.0)
  {
    printf("======= iter:%d\n", iter);
    Eigen::Vector3d d_normalized = d.normalized();

    // double vel_bound = d_normalized.dot(v_step) + 
    //   a_furthest_pt_dist * a_rot_diff + b_furthest_pt_dist * b_rot_diff;
    //double vel_bound = motion_b.computeTBound(d_normalized) + a_furthest_pt_dist * a_rot_diff + b_furthest_pt_dist * b_rot_diff;
    //printf("vel_bound: %f\n", vel_bound);

    std::cout << "d_norm: \n" << d_normalized << std::endl;
    double t_bound = 0.0;
    motion_b.computeTBoundBilateralAdv(d_normalized, dist, b2_offset, t_bound);

    // fcl::Transform3d offset_tx;
    // offset_tx.setIdentity();
    //double tbound_mainshape = 0.0;
    // printf("start mainshape\n");
    // motion_b.computeTBoundBilateralAdv(d_normalized, dist, offset_tx, tbound_mainshape);
    // printf("tbound_mainshape: %f\n", tbound_mainshape);

    // printf("start offsetshape\n");
    // double tbound_offsetshape = 0.0;
    // motion_b.computeTBoundBilateralAdv(d_normalized, dist, b2_offset, tbound_offsetshape);
    // printf("tbound_offsetshape: %f\n", tbound_offsetshape);
    
    //t = std::min(tbound_mainshape, tbound_offsetshape);
    t = t_bound;
    printf("t: %f\n", t);

    motion_a.integrate(t);
    motion_b.integrate(t);

    printf("currenttime: %f\n", motion_b.getCurrentTime());

    motion_a.getCurrentTransform(a_tf);
    motion_b.getCurrentTransform(b_tf);

    calc_min_dist(a_tf, a_radius, b_tf, b_radius,
      b2_offset, b2_radius,
      d, dist);
    
    //printf("vel_bound %f, delta: %f t: %f dist: %f\n", vel_bound, delta, t, dist);
    ++iter;
    // if (iter > 2)
    //   break;
  }
  
  if (t < 1.0)
  {
    impact_time = t;
    printf("time of impact: %f\n", t);
    return true;
  }
  printf("no collide\n");
  return false;
}

} // namespace draw
} // namespace rmf_planner_viz