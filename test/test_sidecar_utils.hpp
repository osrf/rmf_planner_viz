
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

// Custom Bilateral Advancement algorithm that uses seperable shapes 

struct ModelSpaceShape
{
  ModelSpaceShape(const fcl::Transform3d& tx, double r)
    :_transform(tx), _radius(r)
  { }
  fcl::Transform3d _transform;
  double _radius;
};

// this uses spline motions
bool collide_seperable_circles(
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  double& impact_time, uint& dist_checks, 
  uint safety_maximum_checks = 120, double tolerance = 0.001);

// Presets
enum PRESET_TYPE
{
  PRESET_LINEAR = 0,
  PRESET_SPLINEMOTION
};

struct Preset
{
  std::string _description;
  PRESET_TYPE _type = PRESET_SPLINEMOTION;
  double tolerance = 0.01;

  using PresetSetupFunc = void (*)(
    bool override_b_start_pos,
    Eigen::Vector3d b_start_o,
    bool override_b_end_pos,
    Eigen::Vector3d b_end_o,
    std::vector<ModelSpaceShape>& a_shapes, 
    std::vector<ModelSpaceShape>& b_shapes, 
    double& tolerance,
    std::shared_ptr<fcl::MotionBase<double>>& motion_a,
    std::shared_ptr<fcl::MotionBase<double>>& motion_b);
  PresetSetupFunc _callback = nullptr;
};

std::vector<Preset> setup_presets();

} // namespace draw
} // namespace rmf_planner_viz

#endif // RMF_PLANNER_VIZ__DRAW__CUSTOM_CONSERVATIVE_ADV_ALGO_HPP
