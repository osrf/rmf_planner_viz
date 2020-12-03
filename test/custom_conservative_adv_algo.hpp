
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
#include <fcl/math/geometry.h>

namespace rmf_planner_viz {
namespace draw {

// Custom Conservative Advancement algorithm that uses seperable shapes 

// this uses interp-style motions without any arcing
bool CA_collide_seperable_circles(
  Eigen::Vector3d a_start, Eigen::Vector3d a_end, double a_rot_start, double a_rot_end, double a_radius,
  Eigen::Vector3d b_start, Eigen::Vector3d b_end, double b_rot_start, double b_rot_end, double b_radius,
  fcl::Transform3d b2_offset, double b2_radius,
  double& impact_time, double tolerance = 0.001);



} // namespace draw
} // namespace rmf_planner_viz

#endif // RMF_PLANNER_VIZ__DRAW__CUSTOM_CONSERVATIVE_ADV_ALGO_HPP


