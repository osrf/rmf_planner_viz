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

#ifndef RMF_PLANNER_VIZ__DRAW__FIT_HPP
#define RMF_PLANNER_VIZ__DRAW__FIT_HPP

#include <SFML/Graphics/Rect.hpp>
#include <SFML/Graphics/Transform.hpp>

#include <vector>

#include <Eigen/Geometry>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Fit
{
public:

  struct Bounds
  {
    Eigen::Vector2f min;
    Eigen::Vector2f max;

    Bounds();

    Bounds(Eigen::Vector2f min, Eigen::Vector2f max);

    Bounds& add_point(Eigen::Vector2f p, float radius=0.0f);
    Bounds& add_bounds(const Bounds& other);
    Bounds& reset();

    bool inside(Eigen::Vector2f p) const;
  };

  Fit(const std::vector<Bounds>& all_bounds, float margin = 0.02);

  Fit& add_bounds(const Bounds& new_bounds);

  /// Clear out all the bounds that have been given to this Fit.
  Fit& reset();

  Fit& left_border(unsigned int p);

  Fit& right_border(unsigned int p);

  Fit& top_border(unsigned int p);

  Fit& bottom_border(unsigned int p);

  void apply_transform(
      sf::Transform& transform,
      sf::Vector2u render_size) const;

  sf::Transform compute_transform(
      sf::Vector2u render_size) const;

  Bounds get_bounds() const;

private:
  float _margin;
  Bounds _bounds;
  sf::Vector2u _left_top_border;
  sf::Vector2u _right_bottom_border;
};

} // namespace draw
} // namespace rmf_planner_viz

#endif // RMF_PLANNER_VIZ__DRAW__FIT_HPP
