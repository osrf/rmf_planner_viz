
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
  Eigen::Vector3d a_start, Eigen::Vector3d a_end, double a_rot_start, double a_rot_end, double a_radius,
  Eigen::Vector3d b_start, Eigen::Vector3d b_end, double b_rot_start, double b_rot_end, double b_radius,
  fcl::Transform3d b2_offset, double b2_radius,
  double& impact_time, double tolerance)
{
  auto calc_min_dist = [](
    Eigen::Vector3d a, double a_rot, double a_radius, 
    Eigen::Vector3d b, double b_rot, double b_radius,
    fcl::Transform3d b2_offset, double b2_radius,
    Eigen::Vector3d& d, double& dist)
  {
    fcl::Transform3d b_tx;
    b_tx.setIdentity();
    b_tx.prerotate(fcl::AngleAxis<double>(b_rot, Eigen::Vector3d::UnitZ()));
    b_tx.pretranslate(b);
    
    auto b2_tx = b_tx * b2_offset;
    Eigen::Vector3d b2 = b2_tx.translation();
    
    Eigen::Vector3d b_to_a = a - b;
    double b_to_a_dist = b_to_a.norm();
    double d1 = b_to_a_dist - (a_radius + b_radius);

    Eigen::Vector3d b2_to_a = a - b2;
    double b2_to_a_dist = b2_to_a.norm();
    double d2 = b2_to_a_dist - (a_radius + b2_radius);
    if (d1 < d2)
    {
      // printf("b is closer\n");
      dist = d1;
      d = b_to_a;
    }
    else
    {
      // printf("b2 is closer\n");
      dist = d2;
      d = b2_to_a;
    }
  };

  Eigen::Vector3d a_vstep = a_end - a_start;
  Eigen::Vector3d b_vstep = b_end - b_start;
  Eigen::Vector3d v_step = b_vstep - a_vstep;

  double dist = 0.0;
  Eigen::Vector3d d(0,0,0);
  calc_min_dist(a_start, a_rot_start, a_radius, b_start, b_rot_start, b_radius,
    b2_offset, b2_radius, d, dist);

  double a_rot_diff = a_rot_end - a_rot_start;
  double a_furthest_pt_dist = a_radius;

  double b_rot_diff = b_rot_end - b_rot_start;
  Eigen::Vector3d b2_start = b2_offset * b_start;
  double b_furthest_pt_dist = (b2_start - b_start).norm() + b2_radius;
  b_furthest_pt_dist = b_furthest_pt_dist < b_radius ? b_radius : b_furthest_pt_dist;

  double t = 0.0;
  uint iter = 0;
  while (abs(dist) > tolerance && t < 1.0)
  {
    //printf("======= iter:%d\n", iter);
    Eigen::Vector3d d_normalized = d.normalized();

    double vel_bound = d_normalized.dot(v_step) + 
      a_furthest_pt_dist * a_rot_diff + b_furthest_pt_dist * b_rot_diff;
    //printf("vel_bound: %f\n", vel_bound);

    double delta = abs(dist) / vel_bound;
    t += delta;

    Eigen::Vector3d a = a_start + a_vstep * t;
    Eigen::Vector3d b = b_start + b_vstep * t;
    double a_rot = a_rot_start + a_rot_diff * t;
    double b_rot = b_rot_start + b_rot_diff * t;

    calc_min_dist(a, a_rot, a_radius, b, b_rot, b_radius,
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