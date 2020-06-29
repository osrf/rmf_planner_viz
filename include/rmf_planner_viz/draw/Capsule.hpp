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

#ifndef RMF_PLANNER_VIZ__DRAW__CAPSULE_HPP
#define RMF_PLANNER_VIZ__DRAW__CAPSULE_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <SFML/Graphics/Drawable.hpp>
#include <SFML/Graphics/Vertex.hpp>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Capsule : public sf::Drawable
{
public:

  Capsule(
      const sf::Vertex& v0,
      const sf::Vertex& v1,
      float radius,
      std::size_t resolution = 15);

  /// Returns true if the coordinates of (x, y) are touching this capsule
  bool pick(float x, float y) const;

  Capsule& set_start_color(const sf::Color& color);

  Capsule& set_end_color(const sf::Color& color);

  class Implementation;
protected:
  void draw(sf::RenderTarget& target, sf::RenderStates states) const final;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace draw
} // namespace capsule

#endif // RMF_PLANNER_VIZ__DRAW__CAPSULE_HPP
