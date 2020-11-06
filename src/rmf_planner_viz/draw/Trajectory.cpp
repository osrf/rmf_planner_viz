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
#include <SFML/Graphics/CircleShape.hpp>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Trajectory::Implementation
{
public:

  sf::CircleShape arrow;
  sf::CircleShape footprint;
  sf::CircleShape vicinity;
  std::vector<Capsule> capsules;
  float radius;
  Fit::Bounds bounds;

  void configure_circle(
      sf::CircleShape& circle,
      const Eigen::Vector3d& p,
      float R,
      const sf::Color& color,
      std::size_t point_count = 30,
      float s_x = 1.0)
  {
    circle.setRadius(R);
    circle.setPointCount(point_count);
    circle.setFillColor(color);
    circle.setOrigin(R, R);
    circle.setPosition(p[0], p[1]);
    circle.scale(s_x, 1.0);
    circle.rotate(90.0);
    circle.rotate(p[2]*180.0/M_PI);
    circle.setOutlineColor(sf::Color::Black);
    circle.setOutlineThickness(radius/3.0);
  }

  void configure_arrow(
      const Eigen::Vector3d& p,
      float R,
      const sf::Color& color)
  {
    configure_circle(arrow, p, R, color, 3, 0.7);
  }

  void configure_footprint(const Eigen::Vector3d& p, float R)
  {
    configure_circle(footprint, p, R, sf::Color::White);
  }

  void configure_vicinity(const Eigen::Vector3d& p, float R)
  {
    auto color = sf::Color::Yellow;
    color.a = 255/5;
    configure_circle(vicinity, p, R, color);
  }

  void efficient_straight_line_drawing(
      const rmf_traffic::Trajectory& trajectory,
      const rmf_traffic::Profile& profile,
      const rmf_traffic::Time start,
      std::optional<rmf_traffic::Duration> duration,
      const sf::Color color,
      const Eigen::Vector2d offset)
  {
    const auto begin_it = trajectory.lower_bound(start);

    if (begin_it == trajectory.end())
      return;

    const auto interpolate_it = [&]()
    {
      if (start < begin_it->time() && begin_it != trajectory.begin())
        return --rmf_traffic::Trajectory::const_iterator(begin_it);

      return begin_it;
    }();

    const auto end_it = duration?
          trajectory.lower_bound(start + *duration) : trajectory.end();

    if (interpolate_it == end_it)
      return;

    capsules.reserve(trajectory.size());

    Eigen::Vector3d p;
    if (interpolate_it != begin_it)
    {
      const auto motion =
          rmf_traffic::Motion::compute_cubic_splines(
            interpolate_it,
            ++rmf_traffic::Trajectory::const_iterator(begin_it));

      p = motion->compute_position(start);
      const Eigen::Vector2d p_interp = p.block<2,1>(0,0) + offset;
      const Eigen::Vector2d p_begin =
          begin_it->position().block<2,1>(0,0) + offset;

      capsules.push_back(
            Capsule(
              {sf::Vector2f(p_interp.x(), p_interp.y()), color},
              {sf::Vector2f(p_begin.x(), p_begin.y()), color},
              radius));

      bounds.add_point(p_interp.block<2,1>(0,0).cast<float>(), radius);
      bounds.add_point(p_begin.block<2,1>(0,0).cast<float>(), radius);
    }
    else
    {
      p = begin_it->position();
    }

    configure_arrow(p, profile.footprint()->get_characteristic_length(), color);
    configure_footprint(p, profile.footprint()->get_characteristic_length());
    configure_vicinity(p, profile.vicinity()->get_characteristic_length());


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

  void accurate_curve_drawing(
      const rmf_traffic::Trajectory& trajectory,
      const rmf_traffic::Profile& profile,
      const rmf_traffic::Time start,
      std::optional<rmf_traffic::Duration> duration,
      const sf::Color color,
      const Eigen::Vector2d offset)
  {
    const auto motion = rmf_traffic::Motion::compute_cubic_splines(trajectory);

    const auto step = std::chrono::milliseconds(100);
    const auto end = duration ?
          start + duration.value() : motion->finish_time();
    const auto begin = motion->start_time();

    if (motion->start_time() <= start)
    {
      const Eigen::Vector3d p = motion->compute_position(start);
      configure_arrow(p, profile.footprint()->get_characteristic_length(), color);
      configure_footprint(p, profile.footprint()->get_characteristic_length());
      configure_vicinity(p, profile.vicinity()->get_characteristic_length());
    }

    for (auto time=std::max(begin, start); time <= end; time += step)
    {
      const auto next_time = std::min(time + step, end);

      const Eigen::Vector2d p =
          motion->compute_position(time).block<2,1>(0, 0) + offset;
      const Eigen::Vector2d pn =
          motion->compute_position(next_time).block<2,1>(0, 0) + offset;

      capsules.push_back(
        Capsule(
          {sf::Vector2f(p.x(), p.y()), color},
          {sf::Vector2f(pn.x(), pn.y()), color},
          radius));

      bounds.add_point(p.block<2,1>(0,0).cast<float>(), radius);
      bounds.add_point(pn.block<2,1>(0,0).cast<float>(), radius);
    }
  }

  Implementation(
      const rmf_traffic::Trajectory& trajectory,
      const rmf_traffic::Profile& profile,
      rmf_traffic::Time start,
      rmf_utils::optional<rmf_traffic::Duration> duration,
      sf::Color color,
      Eigen::Vector2d offset,
      float width)
    : radius(width/2.0)
  {
    if (trajectory.size() == 0)
      return;

    // This is very efficient but only works for straight-line trajectories
//    efficient_straight_line_drawing(
//          trajectory, profile, start, duration, color, offset);

    accurate_curve_drawing(
          trajectory, profile, start, duration, color, offset);
  }
};

//==============================================================================
Trajectory::Trajectory(
    const rmf_traffic::Trajectory& trajectory,
    const rmf_traffic::Profile& profile,
    rmf_traffic::Time start,
    rmf_utils::optional<rmf_traffic::Duration> duration,
    sf::Color color,
    Eigen::Vector2d offset,
    float projection_width)
  : _pimpl(rmf_utils::make_impl<Implementation>(
      trajectory, profile, start, duration, color, offset, projection_width))
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

  // TODO(MXG): Add picking for the vehicle shape
  return false;
}

//==============================================================================
void Trajectory::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
  target.draw(_pimpl->vicinity, states);
  for (const auto& capsule : _pimpl->capsules)
    target.draw(capsule, states);

  target.draw(_pimpl->footprint, states);
  target.draw(_pimpl->arrow, states);
}

} // namespace draw
} // namespace rmf_planner_viz
