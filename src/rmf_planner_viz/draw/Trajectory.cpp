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

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Trajectory::Implementation
{
public:

  std::vector<Capsule> capsules;

  Implementation(
      const rmf_traffic::Trajectory& trajectory,
      rmf_traffic::Time start,
      rmf_utils::optional<rmf_traffic::Duration> duration,
      float width)
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

    const Eigen::Vector3d p_interp = motion->compute_position(start);
    const Eigen::Vector3d p_begin = begin_it->position();

    capsules.reserve(trajectory.size());
    capsules.push_back(
          Capsule(
            {sf::Vector2f(p_interp.x(), p_interp.y()), sf::Color::Magenta},
            {sf::Vector2f(p_begin.x(), p_begin.y()), sf::Color::Magenta},
            width/2.0));


  }

};

//==============================================================================
Trajectory::Trajectory(
    const rmf_traffic::Trajectory& trajectory,
    rmf_traffic::Time start,
    rmf_utils::optional<rmf_traffic::Duration> duration,
    float width)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             trajectory, start, duration, width))
{
  // Do nothing
}

} // namespace draw
} // namespace rmf_planner_viz
