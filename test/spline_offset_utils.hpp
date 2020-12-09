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


#ifndef RMF_PLANNER_VIZ__DRAW__SPLINEOFFSETUTILS_HPP
#define RMF_PLANNER_VIZ__DRAW__SPLINEOFFSETUTILS_HPP

#include <array>
#include <eigen3/Eigen/Dense>

#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/math/motion/spline_motion.h>

namespace rmf_planner_viz {
namespace draw {

// Lifted functions from so they do not affect the public api. Double check that they sync up

std::array<Eigen::Vector3d, 4> compute_knots(
  Eigen::Vector3d x0,
  Eigen::Vector3d x1,
  Eigen::Vector3d v0,
  Eigen::Vector3d v1);

fcl::SplineMotion<double> convert_catmullrom_to_bspline(
  Eigen::Vector3d p0,
  Eigen::Vector3d p1,
  Eigen::Vector3d p2,
  Eigen::Vector3d p3,
  bool show_control_poly);

} // namespace draw
} // namespace rmf_planner_viz

#endif // RMF_PLANNER_VIZ__DRAW__SPLINEOFFSETUTILS_HPP

