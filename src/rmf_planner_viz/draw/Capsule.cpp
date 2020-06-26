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
      center(sf::Triangles, 6)
  {
    const auto& p0 = v0.position;
    const auto& p1 = v1.position;
    const sf::Vector2f dp = p1 - p0;
    const float length = std::sqrt(dp.x*dp.x + dp.y*dp.y);
    const sf::Vector2f cross = sf::Vector2f(-dp.y, dp.x)/length*radius;

    const sf::Vector2f c0a = p0 + cross;
    const sf::Vector2f c0b = p0 - cross;
    const sf::Vector2f c1a = p1 + cross;
    const sf::Vector2f c1b = p1 - cross;

    center[0].position = c0a;
    center[0].color = v0.color;
    center[1].position = c0b;
    center[1].color = v0.color;
    center[2].position = c1a;
    center[2].color = v1.color;

    center[3].position = c1a;
    center[3].color = v1.color;
    center[4].position = c1b;
    center[4].color = v1.color;
    center[5].position = c0b;
    center[5].color = v0.color;

    const auto rotate = [](const sf::Vector2f& p, float theta) -> sf::Vector2f
    {
      const float ct = std::cos(theta);
      const float st = std::sin(theta);
      return sf::Vector2f(p.x*ct - p.y*st, p.x*st + p.y*ct);
    };

    cap_0[0].position = p0;
    cap_0[0].color = v0.color;

    cap_1[0].position = p1;
    cap_1[0].color = v1.color;

    for (std::size_t i=0; i < resolution; ++i)
    {
      const float s = static_cast<float>(i)/static_cast<float>(resolution-1);
      const float theta = s*M_PI;

      cap_0[i+1].position = p0 + rotate(cross, theta);
      cap_0[i+1].color = v0.color;

      cap_1[i+1].position = p1 - rotate(cross, theta);
      cap_1[i+1].color = v1.color;
    }
  }

  sf::VertexArray cap_0;
  sf::VertexArray cap_1;
  sf::VertexArray center;
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

} // namespace draw
} // namespace rmf_planner_viz
