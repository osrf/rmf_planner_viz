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

#ifndef RMF_PLANNER_VIZ__DRAW__TRAJECTORY_HPP
#define RMF_PLANNER_VIZ__DRAW__TRAJECTORY_HPP

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/Profile.hpp>

#include <SFML/Graphics/Drawable.hpp>
#include <SFML/Graphics/Color.hpp>

#include <rmf_planner_viz/draw/Fit.hpp>

#include <rmf_utils/optional.hpp>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Trajectory : public sf::Drawable
{
public:
  Trajectory(
      const rmf_traffic::Trajectory& trajectory,
      const rmf_traffic::Profile& profile,
      rmf_traffic::Time start,
      rmf_utils::optional<rmf_traffic::Duration> duration,
      sf::Color color,
      Eigen::Vector2d offset,
      float projection_width,
      std::optional<rmf_traffic::Time> now = std::nullopt);

  const Fit::Bounds& bounds() const;

  bool pick(float x, float y) const;

protected:

  void draw(sf::RenderTarget& target, sf::RenderStates states) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace draw
} // namespace rmf_planner_viz


#endif // RMF_PLANNER_VIZ__DRAW__TRAJECTORY_HPP
