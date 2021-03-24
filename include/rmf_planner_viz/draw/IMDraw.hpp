
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

#ifndef RMF_PLANNER_VIZ__DRAW__IMDRAW_HPP
#define RMF_PLANNER_VIZ__DRAW__IMDRAW_HPP

#include <SFML/Graphics.hpp>

#include <rmf_traffic/Trajectory.hpp>

namespace rmf_planner_viz {
namespace draw {

/// Series of immediate-mode drawing functions. Note that this is done for iteration times and quick access, and may not be profile
class IMDraw
{
public:

  static void draw_circle(
    const sf::Vector2f& center, double radius,
    const sf::Color& color = sf::Color(255, 255, 255, 255),
    uint slices = 16);

  static void draw_axis(float size = 1.0f);

  static void draw_trajectory(
    const rmf_traffic::Trajectory& trajectory,
    const sf::Color& color = sf::Color(255, 255, 255, 255));

  static void draw_line(const sf::Vector2f& start, const sf::Vector2f& end, const sf::Color& color = sf::Color(255, 255, 255, 255));

  static void draw_arrow(const sf::Vector2f& start, const sf::Vector2f& end, const sf::Color& color = sf::Color(255, 255, 255, 255));

  static void draw_aabb(const sf::Vector2f& box_min, const sf::Vector2f& box_max, const sf::Color& color = sf::Color(255, 255, 255, 255));

  static void draw_box(const sf::Vector2f& center,
    const sf::Vector2f& box_x, const sf::Vector2f& box_y, 
    const sf::Color& color = sf::Color(255, 255, 255, 255));

  // render all objects and flush the internal vertex array buffer
  static void flush_and_render(sf::RenderWindow& app_window, const sf::Transform& tx_flipped_2d);
};


} // namespace draw
} // namespace rmf_planner_viz

#endif // RMF_PLANNER_VIZ__DRAW__IMDRAW_HPP
