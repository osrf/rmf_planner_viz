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

#include <rmf_planner_viz/draw/Fit.hpp>

namespace rmf_planner_viz {
namespace draw {

namespace {
//==============================================================================
const float inf = std::numeric_limits<float>::infinity();
} // anonymous namespace

//==============================================================================
Fit::Fit(const std::vector<Bounds>& all_bounds, float margin)
  : _margin(margin),
    _bounds{{inf, inf}, {-inf, -inf}}
{
  for (const auto& b : all_bounds)
    add_bounds(b);
}

//==============================================================================
Fit& Fit::add_bounds(const Bounds& new_bounds)
{
  for (int i=0; i < 2; ++i)
  {
    if (new_bounds.min[i] < _bounds.min[i])
      _bounds.min[i] = new_bounds.min[i];

    if (_bounds.max[i] < new_bounds.max[i])
      _bounds.max[i] = new_bounds.max[i];
  }

  return *this;
}

//==============================================================================
Fit& Fit::left_border(unsigned int p)
{
  _left_top_border.x = p;
  return *this;
}

//==============================================================================
Fit& Fit::right_border(unsigned int p)
{
  _right_bottom_border.x = p;
  return *this;
}

//==============================================================================
Fit& Fit::top_border(unsigned int p)
{
  _left_top_border.y = p;
  return *this;
}

//==============================================================================
Fit& Fit::bottom_border(unsigned int p)
{
  _right_bottom_border.y = p;
  return *this;
}

//==============================================================================
void Fit::apply_transform(
    sf::Transform& transform,
    const sf::Vector2u render_size) const
{
  const auto dt = render_size - _left_top_border - _right_bottom_border;
  const float full_scale = std::min(
        static_cast<float>(dt.x)/(_bounds.max.x() - _bounds.min.x()),
        static_cast<float>(dt.y)/(_bounds.max.y() - _bounds.min.y()));

  const float m = (1.0 - _margin)*full_scale;
  const float scale = (1.0 - 2.0*_margin) * full_scale;

  const float left = _left_top_border.x;
  const float top = _left_top_border.y;
  const float right = _right_bottom_border.x;
  const float bottom = _right_bottom_border.y;
  const float vx = render_size.x;
  const float vy = render_size.y;

  Eigen::Vector2f center_g = 0.5 * (_bounds.max + _bounds.min);
  center_g[1] = -center_g[1];

  const Eigen::Vector2f center_r(
        (vx - (right+m) + (left+m))/2.0f,
        (vy - (bottom+m) + (top+m))/2.0f);

  const Eigen::Vector2f dx = center_r - scale*center_g;

  transform
      .translate(dx[0], dx[1])
      .scale(scale, -scale);
}

//==============================================================================
sf::Transform Fit::compute_transform(
    const sf::Vector2u render_size) const
{
  sf::Transform output;
  apply_transform(output, render_size);
  return output;
}

} // namespace draw
} // namespace rmf_planner_viz
