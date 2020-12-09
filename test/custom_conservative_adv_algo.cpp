
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

//#define DO_LOGGING 1

namespace rmf_planner_viz {
namespace draw {

bool collide_seperable_circles(
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
  double b_rot_diff = b_rot_end - b_rot_start;

  double prev_min_dist = DBL_MAX;
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
      for (;;)
      {
        double sample_t = lower_t_limit + 0.5 * (upper_t_limit - lower_t_limit);
#ifdef DO_LOGGING
        printf("iter: %d, sample_t: %f\n", bilateral_adv_iter, sample_t);
#endif

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
        
        double dist_diff_output = DBL_MAX;
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

            double dist_diff = dist_along_d - dist_to_cover;
            
            if (dist_diff < dist_diff_output)
              dist_diff_output = dist_along_d;
          }
        }

        // printf("dist_output: %f\n", dist_diff_output);
        if (abs(dist_diff_output) < tolerance)
        {
          // printf("minimal dist within tolerance range %f\n", tolerance);
          return sample_t;
        }

        // our window is too small and we're hopping around, so we stop.
        // Also, this is what box2d does.
        if (bilateral_adv_iter >= 25) 
        {
          // printf("range too small\n");
          return sample_t;
        }

        if (dist_diff_output < 0.0)
          upper_t_limit = sample_t;
        else if (dist_diff_output > 0.0)
          lower_t_limit = sample_t;

        ++bilateral_adv_iter;
      }
    };

    t = bilateral_adv(t, d_normalized, dist_along_d_to_cover);

    Eigen::Vector3d a = a_start + a_vstep * t;
    Eigen::Vector3d b = b_start + b_vstep * t;
    double a_rot = a_rot_start + a_rot_diff * t;
    double b_rot = b_rot_start + b_rot_diff * t;

    prev_min_dist = dist_along_d_to_cover;
    calc_min_dist(a, a_rot, b, b_rot, a_shapes, b_shapes,
      d, dist_along_d_to_cover);
    
    // if (prev_min_dist < dist_along_d_to_cover) //ensure an improvement in abs minimum distance
    // {
    //   impact_time = t;
    // }
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

static double max_splinemotion_advancement(double current_t,
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  const Eigen::Vector3d& d_normalized, double max_dist,
  uint& dist_checks, double tolerance)
{
  assert(tolerance >= 0.0);
  
  double lower_t_limit = current_t;
  double upper_t_limit = 1.0;
  uint bilateral_adv_iter = 0;
  
  double s1 = max_dist, s2 = max_dist;
  double sample_t = 0.0;
  for (;;)
  {
#ifdef DO_LOGGING
    if (bilateral_adv_iter < 3)
      printf("#1: (%f,%f) #2: (%f,%f)\n", lower_t_limit, s1, upper_t_limit, s2);
#endif
    
    // alternate between bisection and false position methods
    if (bilateral_adv_iter & 1/* && ((s1 < 0.0 && s2 > 0.0) || (s1 > 0.0 && s2 < 0.0))*/)
    {
      // use false position method
      // solve for t where (t, tolerance) for a line with 
      // endpoints (lower_t_limit, s1) and (upper_t_limit, s2)
      // where s2 is negative and s1 is positive
      double inv_m = (upper_t_limit - lower_t_limit) / (s2 - s1);
      sample_t = lower_t_limit + (tolerance - s1) * inv_m;
    }
    else // bisection method
      sample_t = lower_t_limit + 0.5 * (upper_t_limit - lower_t_limit);
#ifdef DO_LOGGING
    printf("iteration: %d picked t: %f\n", bilateral_adv_iter, sample_t);
#endif

    // integrate
    motion_a.integrate(sample_t);
    motion_b.integrate(sample_t);

    fcl::Transform3d a_tx, b_tx;
    motion_a.getCurrentTransform(a_tx);
    motion_b.getCurrentTransform(b_tx);

    double s = DBL_MAX;
    // compute closest distance between all 4 shapes in direction d
    for (const auto& a_shape : a_shapes)
    {
      auto a_shape_tx = a_tx * a_shape._transform;
      for (const auto& b_shape : b_shapes)
      {
        auto b_shape_tx = b_tx * b_shape._transform;
        Eigen::Vector3d b_to_a = a_shape_tx.translation() - b_shape_tx.translation();
        
        double b_to_a_dist = b_to_a.norm();
        double dist_between_shapes_along_d = 0.0;
        if (b_to_a_dist > 1e-04)
        {
          auto b_to_a_norm = b_to_a / b_to_a_dist;
          double dist_along_b_to_a = b_to_a_dist - (a_shape._radius + b_shape._radius);
          auto v = dist_along_b_to_a * b_to_a_norm;
          dist_between_shapes_along_d = v.dot(d_normalized);
        }
        
        // get the minimum
        if (dist_between_shapes_along_d < s)
          s = dist_between_shapes_along_d;
      }
    }

#ifdef DO_LOGGING
    printf("dist_output: %f\n", s);
#endif
    if (abs(s) < tolerance)
    {
#ifdef DO_LOGGING
      printf("minimal dist %f within tolerance range %f\n", s, tolerance);
#endif
      break;
    }

    // our window is very small and we're hopping around, so we stop.
    // Also, this is what box2d does.
    if (bilateral_adv_iter >= 25) 
    {
#ifdef DO_LOGGING
      printf("range too small\n");
#endif
      break;
    }
    
    if (s < 0.0)
    {
      upper_t_limit = sample_t;
      s2 = s;
    }
    else if (s > 0.0)
    {
      lower_t_limit = sample_t;
      s1 = s;
    }
    ++bilateral_adv_iter;
  }
  dist_checks += bilateral_adv_iter;

  return sample_t;
}

bool collide_seperable_circles(
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  double& impact_time, uint& dist_checks, uint safety_maximum_checks, double tolerance)
{
  auto calc_min_dist = [](
    const fcl::Transform3d& a_tx,
    const fcl::Transform3d& b_tx,
    const std::vector<ModelSpaceShape>& a_shapes,
    const std::vector<ModelSpaceShape>& b_shapes,
    Eigen::Vector3d& d, double& min_dist)
  {
    if (a_shapes.empty() || b_shapes.empty())
      return false;
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

  double dist_along_d_to_cover = 0.0;
  Eigen::Vector3d d(0,0,0);
  calc_min_dist(a_start_tf, b_start_tf, a_shapes, b_shapes,
    d, dist_along_d_to_cover);

  motion_a.integrate(0.0);
  motion_b.integrate(0.0);
  
  double t = 0.0;
  uint iter = 0;
  while (dist_along_d_to_cover > tolerance && t < 1.0)
  {

    Eigen::Vector3d d_normalized = d.normalized();
#ifdef DO_LOGGING
    printf("======= iter:%d\n", iter);
    std::cout << "d_norm: \n" << d_normalized << std::endl;
#endif

    t = max_splinemotion_advancement(t, motion_a, motion_b, a_shapes, b_shapes, 
      d_normalized, dist_along_d_to_cover, dist_checks, tolerance);
#ifdef DO_LOGGING
    printf("max_splinemotion_advancement returns t: %f\n", t);
#endif

    motion_a.integrate(t);
    motion_b.integrate(t);

    motion_a.getCurrentTransform(a_tf);
    motion_b.getCurrentTransform(b_tf);

    calc_min_dist(a_tf, b_tf, a_shapes, b_shapes,
      d, dist_along_d_to_cover);
    
    ++dist_checks;
    ++iter;

    //infinite loop prevention. you should increase safety_maximum_checks if you still want a solution
    if (dist_checks > safety_maximum_checks)
      break;
  }
  
  if (t >= 0.0 && t < 1.0)
  {
    impact_time = t;
#ifdef DO_LOGGING
    printf("time of impact: %f\n", t);
#endif
    return true;
  }
#ifdef DO_LOGGING
  printf("no collide\n");
#endif
  return false;
}

} // namespace draw
} // namespace rmf_planner_viz