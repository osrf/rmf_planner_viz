
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
#include <float.h>

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

  bool computeTBoundBilateralAdv(const fcl::Vector3d& d_norm, double dist_along_d, 
    const fcl::Transform3d& offset_shape_tx, double& t_bound)
  {
    double current_t = this->tf_t;

    fcl::Transform3d current_tf;
    this->getCurrentTransform(current_tf);

    // get the current position and rotation
    auto current_point = current_tf.translation();

    //find point (or t) that is <= dist_along_d
    //use bisection method. 
    double lower_t_limit = this->tf_t;
    double upper_t_limit = 1.0f;
    
    double prev_diff_abs = DBL_MAX;
    double prev_t = this->tf_t;
    Eigen::Vector3d p = current_point;
    uint iter = 0;
    do
    {
      double t = lower_t_limit + 0.5 * (upper_t_limit - lower_t_limit);
      printf("picked t: %f\n", t);
      // integrate
      this->integrate(t);

      fcl::Transform3d tf;
      this->getCurrentTransform(tf);
      auto p1 = tf.translation();
      auto p2 = offset_shape_tx * p1;

      // p = Td[0] * getWeight0(t) + Td[1] * getWeight1(t) + 
      //   Td[2] * getWeight2(t) + Td[3] * getWeight3(t);
      //IMDraw::draw_circle(sf::Vector2f(p.x(), p.y()), 0.5);

      auto get_projected_dist = [&current_point, &d_norm](Eigen::Vector3d p)
      {
        auto current_pt_to_p = p - current_point;
        return current_pt_to_p.dot(d_norm);
      };

      double projected_dist_p1 = get_projected_dist(p1);
      double projected_dist_p2 = get_projected_dist(p2);

      // take the furthest distance
      double projected_dist = projected_dist_p1 < projected_dist_p2 ?
        projected_dist_p2 : projected_dist_p1;

      double diff = projected_dist - dist_along_d;
      printf("projected_dist: %f dist_along_d: %f diff: %f\n",
        projected_dist, dist_along_d, diff);
      
      if (abs(diff) > prev_diff_abs)
      {
        t_bound = prev_t; 
        printf("reversion of abs distance, reverting to prev_t: %f\n", prev_t);
        break;
      }


      prev_diff_abs = abs(diff);
      prev_t = t;

      printf("[%.9f, %.9f], range: %f\n", lower_t_limit, upper_t_limit,
        upper_t_limit - lower_t_limit);
      if (lower_t_limit == upper_t_limit)
      {
        t_bound = t;
        printf("range equal\n");
        break;
      }
      const double tolerance = 0.01;
      
      if (abs(diff) < tolerance || (upper_t_limit - lower_t_limit) < tolerance)
      {
        t_bound = t;
        printf("range too small\n");
        break;
      }

      if (diff < 0.0) //projected dist not enough, search for a higher t
        lower_t_limit = t;
      else if (diff > 0.0) //projected dist too excessive, choose a lower t
        upper_t_limit = t;

      if (t >= 1.0)
      {
        t_bound = 1.0;
        break;
      }

      ++iter;
    } while(1);

    printf("selected t_bound: %f\n", t_bound);
    this->integrate(current_t); //reset
    return false;
  }
};

// Custom Bilateral Advancement algorithm that uses seperable shapes 

struct ModelSpaceShape
{
  ModelSpaceShape(const fcl::Transform3d& tx, double r)
    :_transform(tx), _radius(r)
  { }
  fcl::Transform3d _transform;
  double _radius;
};

// this uses straight-line linear interpolation motions
bool CA_collide_seperable_circles(
  Eigen::Vector3d a_start, Eigen::Vector3d a_end, double a_rot_start, double a_rot_end,
  Eigen::Vector3d b_start, Eigen::Vector3d b_end, double b_rot_start, double b_rot_end,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
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


