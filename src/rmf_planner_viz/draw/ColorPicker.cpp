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

#include <rmf_planner_viz/draw/ColorPicker.hpp>

#include <cmath>
#include <limits>

#include <iostream>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
sf::Color ColorPicker::choose(std::size_t index)
{
  const auto m = index % 12;
  const auto t = m % 3;
  const auto shift = m/3;

  const auto tier = index/12;
  const bool saturated = tier % 2 == 1;

  const float theta = 120.0*t + 15.0*shift;
  const float theta_r = 180.0 < theta ? theta - 360.0 : theta;

  const sf::Uint8 Primary = std::numeric_limits<sf::Uint8>::max();
  const auto Secondary = [&](const float t1, const float t0)
  {
    return Primary * (t1 - t0)/60.0;
  };

  std::cout << index << " --> " << theta << " | " << theta_r << std::endl;

  sf::Color color;
  if (-60.0 <= theta_r && theta_r <= 60.0)
    color.r = Primary;

  if (60.0 <= theta && theta <= 180.0)
    color.g = Primary;

  if (180.0 <= theta && theta <= 300.0)
    color.b = Primary;

  if (0.0 <= theta && theta <= 60.0)
    color.g = Secondary(theta, 0.0);

  if (60.0 <= theta && theta <= 120.0)
    color.r = Secondary(120.0, theta);

  if (120.0 <= theta && theta <= 180.0)
    color.b = Secondary(theta, 120.0);

  if (180.0 <= theta && theta <= 240.0)
    color.g = Secondary(240.0, theta);

  if (240.0 <= theta && theta <= 300.0)
    color.r = Secondary(theta, 240);

  if (300.0 <= theta && theta <= 360.0)
    color.b = Secondary(360.0, theta);

  return color;
}

} // namespace draw
} // namespace rmf_planner_viz
