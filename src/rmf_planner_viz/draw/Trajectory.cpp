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

#include <rmf_planner_viz/draw/Trajectory.hpp>
#include <rmf_planner_viz/draw/Capsule.hpp>

#include <rmf_traffic/Motion.hpp>

#include <SFML/Graphics/RenderTarget.hpp>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Trajectory::Implementation
{
public:

  std::vector<Capsule> capsules;
  float radius;
  Fit::Bounds bounds;

  Implementation(
      const rmf_traffic::Trajectory& trajectory,
      rmf_traffic::Time start,
      rmf_utils::optional<rmf_traffic::Duration> duration,
      sf::Color color,
      Eigen::Vector2d offset,
      float width)
    : radius(width/2.0)
  {
    if (trajectory.size() == 0)
      return;

    const auto begin_it = trajectory.lower_bound(start);

    const auto interpolate_it = [&]()
    {
      if (start < begin_it->time() && begin_it != trajectory.begin())
        --rmf_traffic::Trajectory::const_iterator(begin_it);

      return begin_it;
    }();

    const auto end_it = duration?
          trajectory.lower_bound(start + *duration) : trajectory.end();

    if (interpolate_it == end_it)
      return;

    const auto motion =
        rmf_traffic::Motion::compute_cubic_splines(interpolate_it, end_it);

    const Eigen::Vector2d p_interp =
        motion->compute_position(start).block<2,1>(0,0) + offset;
    const Eigen::Vector2d p_begin =
        begin_it->position().block<2,1>(0,0) + offset;

    capsules.reserve(trajectory.size());
    capsules.push_back(
          Capsule(
            {sf::Vector2f(p_interp.x(), p_interp.y()), color},
            {sf::Vector2f(p_begin.x(), p_begin.y()), color},
            radius));

    bounds.add_point(p_interp.block<2,1>(0,0).cast<float>(), radius);
    bounds.add_point(p_begin.block<2,1>(0,0).cast<float>(), radius);

    auto it_next = ++rmf_traffic::Trajectory::const_iterator(begin_it);
    for (auto it = begin_it; it_next != trajectory.end(); ++it, ++it_next)
    {
      const Eigen::Vector2d p = it->position().block<2,1>(0,0) + offset;
      const Eigen::Vector2d pn = it_next->position().block<2,1>(0,0) + offset;
      capsules.push_back(
        Capsule(
          {sf::Vector2f(p.x(), p.y()), color},
          {sf::Vector2f(pn.x(), pn.y()), color},
          radius));

      bounds.add_point(pn.block<2,1>(0,0).cast<float>(), radius);
    }
  }
};

//==============================================================================
Trajectory::Trajectory(
    const rmf_traffic::Trajectory& trajectory,
    rmf_traffic::Time start,
    rmf_utils::optional<rmf_traffic::Duration> duration,
    sf::Color color,
    Eigen::Vector2d offset,
    float width)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             trajectory, start, duration, color, offset, width))
{
  // Do nothing
}

//==============================================================================
const Fit::Bounds& Trajectory::bounds() const
{
  return _pimpl->bounds;
}

//==============================================================================
bool Trajectory::pick(float x, float y) const
{
  if (!_pimpl->bounds.inside({x, y}))
    return false;

  for (const auto& capsule : _pimpl->capsules)
  {
    if (capsule.pick(x, y))
      return true;
  }

  return false;
}

//==============================================================================
void Trajectory::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
  for (const auto& capsule : _pimpl->capsules)
    target.draw(capsule, states);
}

} // namespace draw
} // namespace rmf_planner_viz
