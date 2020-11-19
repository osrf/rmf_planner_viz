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

#ifndef RMF_PLANNER_VIZ__DRAW__GRAPH_HPP
#define RMF_PLANNER_VIZ__DRAW__GRAPH_HPP

#include <rmf_traffic/agv/Graph.hpp>

#include <SFML/Graphics/Drawable.hpp>
#include <SFML/Graphics/Shape.hpp>
#include <SFML/Graphics/Vertex.hpp>
#include <SFML/Graphics/Font.hpp>

#include <rmf_utils/optional.hpp>

#include <rmf_planner_viz/draw/Fit.hpp>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Graph : public sf::Drawable
{
public:

  Graph(const rmf_traffic::agv::Graph& graph, float lane_width, const sf::Font& font);

  bool choose_map(const std::string& name);

  const std::string* current_map() const;

  /// Get the scaling factor for this graph that will allow it to fit into the
  /// given view size
  const Fit::Bounds& bounds() const;

  enum class ElementType
  {
    Waypoint,
    Lane
  };

  struct Pick
  {
    ElementType type;
    std::size_t index;
  };

  rmf_utils::optional<Pick> pick(float x, float y) const;

  void select(Pick chosen);

  void deselect();

  rmf_utils::optional<Pick> selected() const;

protected:

  void draw(sf::RenderTarget& target, sf::RenderStates states) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace draw
} // namespace rmf_planner_viz

#endif // RMF_PLANNER_VIZ__DRAW__GRAPH_HPP
