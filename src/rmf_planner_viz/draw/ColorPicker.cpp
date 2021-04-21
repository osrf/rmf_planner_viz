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
#include <vector>

#include <iostream>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
sf::Color ColorPicker::choose(std::size_t index)
{
//  const std::size_t m = index % 12;
//  const std::size_t t = m % 3;
//  const std::size_t shift = m/3;
//  std::cout << "index: " << index << " | shfit: " << shift << std::endl;

//  const auto tier = index/12;
//  const bool saturated = tier % 2 == 1;

//  const float theta = 120.0*t + 15.0*shift;
//  const float theta_r = 180.0 < theta ? theta - 360.0 : theta;

//  const sf::Uint8 Max = std::numeric_limits<sf::Uint8>::max();
//  const sf::Uint8 Primary = (4-index/3)*Max/4;
//  const auto Secondary = [&](const float t1, const float t0)
//  {
//    return Max * (t1 - t0)/30.0;
//  };

////  std::cout << index << " --> " << theta << " | " << theta_r << std::endl;

//  sf::Color color;
//  if (-60.0 <= theta_r && theta_r <= 60.0)
//    color.g = Primary;

//  if (60.0 <= theta && theta <= 180.0)
//    color.r = Primary;

//  if (180.0 <= theta && theta <= 300.0)
//    color.b = Primary;

//  if (0.0 <= theta && theta <= 60.0)
//    color.r = Secondary(theta, 0.0);

//  if (60.0 <= theta && theta <= 120.0)
//    color.g = Secondary(120.0, theta);

//  if (120.0 <= theta && theta <= 180.0)
//    color.b = Secondary(theta, 120.0);

//  if (180.0 <= theta && theta <= 240.0)
//    color.r = Secondary(240.0, theta);

//  if (240.0 <= theta && theta <= 300.0)
//    color.g = Secondary(theta, 240);

//  if (300.0 <= theta && theta <= 360.0)
//    color.b = Secondary(360.0, theta);

//  std::cout << "Color: (" << (int)color.r << ", " << (int)color.g << ", " << (int)color.b
//            << ")" << std::endl;

//  return color;

  static const std::vector<sf::Color> colors = {
    sf::Color::Green,
    sf::Color::Blue,
    sf::Color::Red,
    sf::Color::Magenta,
    sf::Color::Cyan
  };

  return colors[index % colors.size()];
}

} // namespace draw
} // namespace rmf_planner_viz
