
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



#ifndef RMF_PLANNER_VIZ__DRAW__CUSTOM_CONSERVATIVE_ADV_ALGO_HPP
#define RMF_PLANNER_VIZ__DRAW__CUSTOM_CONSERVATIVE_ADV_ALGO_HPP

#include <Eigen/Dense>
#include <fcl/math/motion/spline_motion.h>
#include <rmf_planner_viz/draw/IMDraw.hpp>

namespace rmf_planner_viz {
namespace draw {


class CustomSplineMotion : public fcl::SplineMotion<double>
{
public:
  CustomSplineMotion(
    const fcl::Vector3d& Td0, const fcl::Vector3d& Td1, const fcl::Vector3d& Td2, const fcl::Vector3d& Td3,
    const fcl::Vector3d& Rd0, const fcl::Vector3d& Rd1, const fcl::Vector3d& Rd2, const fcl::Vector3d& Rd3)
    :fcl::SplineMotion<double>(Td0, Td1, Td2, Td3, Rd0, Rd1, Rd2, Rd3)
  {
  }

  bool computeTBound(const fcl::Vector3d& n, double& t_bound) const
  {
    //computing the highest T for the translational part of the spline
    //along a given vector n

    Eigen::Vector3d derivative_coeff_a =
      0.5 * (Td[3] - 3.0 * Td[2] + 3.0 * Td[1] + Td[0]);
    Eigen::Vector3d derivative_coeff_b =
      (Td[0] - 2.0 * Td[1] + Td[2]);
    Eigen::Vector3d derivative_coeff_c =
      (-0.5 * Td[0]) + (0.5 * Td[2]);

    // dotp is associative, so we have a quadratic formula, equate it to 0 and solve for t
    double coeff_a = derivative_coeff_a.dot(n);
    double coeff_b = derivative_coeff_b.dot(n);
    double coeff_c = derivative_coeff_c.dot(n);
    
    // solve for potential t 
    if (coeff_a == 0.0) 
    {
      printf("coeff_a is 0\n");
      return false; //no roots
    }
    
    double b_sq_minus_4ac = coeff_b * coeff_b - 4.0 * coeff_a * coeff_c;
    if (b_sq_minus_4ac < 0)
    {
      printf("imag roots\n");
      return false; //imaginary roots
    }
    
    double t1 = (-coeff_b + sqrt(b_sq_minus_4ac)) / (2.0 * coeff_a);
    double t2 = (-coeff_b - sqrt(b_sq_minus_4ac)) / (2.0 * coeff_a);

    t_bound = t1 < t2 ? t2 : t1;
    
    printf("t1: %f t2: %f\n", t1, t2);
    return true;
  }

  bool computeTBoundBilateralAdv(const fcl::Vector3d& d_norm, double dist_along_d, double& t_bound)
  {
    fcl::Transform3d tf;
    this->getCurrentTransform(tf);

    auto currentpoint = tf.translation();

    //find point (or t) that is <= dist_along_d
    //use bisection method. 
    double lower_t_limit = this->tf_t;
    double upper_t_limit = 1.0f;
    
    Eigen::Vector3d p = currentpoint;
    uint iter = 0;
    do
    {
      double t = lower_t_limit + 0.5 * (upper_t_limit - lower_t_limit);

      p = Td[0] * getWeight0(t) + Td[1] * getWeight1(t) + 
        Td[2] * getWeight2(t) + Td[3] * getWeight3(t);
      //IMDraw::draw_circle(sf::Vector2f(p.x(), p.y()), 0.5);

      auto current_pt_to_point_at_t = p - currentpoint;

      double projected_dist = current_pt_to_point_at_t.dot(d_norm);
      double diff = projected_dist - dist_along_d;
      // printf("projected_dist_sq: %f dist_along_d: %f diff: %f\n",
      //   projected_dist, dist_along_d, diff);
      
      const double tolerance = 0.01;
      if (abs(diff) < tolerance)
      {
        t_bound = t;
        break;
      }

      if (diff < 0.0) //projected dist not enough, search for a higher t
      {
        lower_t_limit = t;
      }
      else if (diff > 0.0) //projected dist too excessive, choose a lower t
      {
        upper_t_limit = t;
      }

      if (t >= 1.0)
      {
        t_bound = 1.0;
        break;
      }

      ++iter;
    } while(1);

    printf("selected t_bound: %f\n", t_bound);
    return false;
  }
};

// Custom Conservative Advancement algorithm that uses seperable shapes 

// this uses straight-line linear interpolation motions
bool CA_collide_seperable_circles(
  Eigen::Vector3d a_start, Eigen::Vector3d a_end, double a_rot_start, double a_rot_end, double a_radius,
  Eigen::Vector3d b_start, Eigen::Vector3d b_end, double b_rot_start, double b_rot_end, double b_radius,
  fcl::Transform3d b2_offset, double b2_radius,
  double& impact_time, double tolerance = 0.001);

// this uses spline motions
bool CA_collide_seperable_circles(
  CustomSplineMotion motion_a, double a_radius,
  CustomSplineMotion motion_b, double b_radius,
  fcl::Transform3d b2_offset, double b2_radius,
  double& impact_time, double tolerance = 0.001);


} // namespace draw
} // namespace rmf_planner_viz

#endif // RMF_PLANNER_VIZ__DRAW__CUSTOM_CONSERVATIVE_ADV_ALGO_HPP


