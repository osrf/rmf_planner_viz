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

#include <rmf_planner_viz/draw/Capsule.hpp>

#include <SFML/Graphics/VertexArray.hpp>
#include <SFML/Graphics/RenderTarget.hpp>

#include <Eigen/Geometry>
#include <cmath>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Capsule::Implementation
{
public:

  Implementation(
      const sf::Vertex& v0,
      const sf::Vertex& v1,
      float radius,
      std::size_t resolution)
    : cap_0(sf::TriangleFan, resolution+1),
      cap_1(sf::TriangleFan, resolution+1),
      center(sf::Triangles, 6),
      resolution(resolution)
  {
    const auto& p0 = v0.position;
    this->p0 = Eigen::Vector2f(p0.x, p0.y);
    const auto& p1 = v1.position;
    this->p1 = Eigen::Vector2f(p1.x, p1.y);
    this->radius = radius;

    const sf::Vector2f dp = p1 - p0;
    const float length = std::sqrt(dp.x*dp.x + dp.y*dp.y);
    const sf::Vector2f cross = sf::Vector2f(-dp.y, dp.x)/length*radius;

    const sf::Vector2f c0a = p0 + cross;
    const sf::Vector2f c0b = p0 - cross;
    const sf::Vector2f c1a = p1 + cross;
    const sf::Vector2f c1b = p1 - cross;

    center[0].position = c0a;
    center[1].position = c0b;
    center[2].position = c1a;

    center[3].position = c1a;
    center[4].position = c1b;
    center[5].position = c0b;

    const auto rotate = [](const sf::Vector2f& p, float theta) -> sf::Vector2f
    {
      const float ct = std::cos(theta);
      const float st = std::sin(theta);
      return sf::Vector2f(p.x*ct - p.y*st, p.x*st + p.y*ct);
    };

    cap_0[0].position = p0;

    cap_1[0].position = p1;

    for (std::size_t i=0; i < resolution; ++i)
    {
      const float s = static_cast<float>(i)/static_cast<float>(resolution-1);
      const float theta = s*M_PI;
      cap_0[i+1].position = p0 + rotate(cross, theta);
      cap_1[i+1].position = p1 - rotate(cross, theta);
    }

    set_start_color(v0.color);
    set_end_color(v1.color);
  }

  void set_start_color(const sf::Color& color)
  {
    center[0].color = color;
    center[1].color = color;
    center[5].color = color;

    cap_0[0].color = color;
    for (std::size_t i = 0; i < resolution; ++i)
      cap_0[i+1].color = color;
  }

  void set_end_color(const sf::Color& color)
  {
    center[2].color = color;
    center[3].color = color;
    center[4].color = color;

    cap_1[0].color = color;
    for (std::size_t i = 0; i < resolution; ++i)
      cap_1[i+1].color = color;
  }

  sf::VertexArray cap_0;
  sf::VertexArray cap_1;
  sf::VertexArray center;

  Eigen::Vector2f p0;
  Eigen::Vector2f p1;
  float radius;
  std::size_t resolution;
};

//==============================================================================
Capsule::Capsule(
    const sf::Vertex& v0,
    const sf::Vertex& v1,
    float radius,
    std::size_t resolution)
  : _pimpl(rmf_utils::make_impl<Implementation>(v0, v1, radius, resolution))
{
  // Do nothing
}

//==============================================================================
void Capsule::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
  target.draw(_pimpl->center, states);
  target.draw(_pimpl->cap_0, states);
  target.draw(_pimpl->cap_1, states);
}

//==============================================================================
bool Capsule::pick(float x, float y) const
{
  const Eigen::Vector2f p(x, y);
  const Eigen::Vector2f p0 = _pimpl->p0;
  const Eigen::Vector2f p1 = _pimpl->p1;
  const float radius = _pimpl->radius;

  if ((p0-p).norm() <= radius)
    return true;

  if ((p1-p).norm() <= radius)
    return true;

  const float lane_length = (p1 - p0).norm();
  if (lane_length < 1e-8)
    return false;

  const Eigen::Vector2f pn = (p1 - p0)/lane_length;
  const Eigen::Vector2f p_l = p - p0;
  const float p_l_projection = p_l.dot(pn);

  if (p_l_projection < 0.0)
    return false;

  if (lane_length < p_l_projection)
    return false;

  const double lane_dist = (p_l - p_l_projection*pn).norm();
  if (lane_dist <= radius)
    return true;

  return false;
}

//==============================================================================
Capsule& Capsule::set_start_color(const sf::Color& color)
{
  _pimpl->set_start_color(color);
  return *this;
}

//==============================================================================
Capsule& Capsule::set_end_color(const sf::Color& color)
{
  _pimpl->set_end_color(color);
  return *this;
}

} // namespace draw
} // namespace rmf_planner_viz
