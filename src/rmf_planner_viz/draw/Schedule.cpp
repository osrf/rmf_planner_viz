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

#include <rmf_planner_viz/draw/Schedule.hpp>
#include <rmf_planner_viz/draw/Trajectory.hpp>

#include <SFML/Graphics/RenderTarget.hpp>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Schedule::Implementation
{
public:

  std::shared_ptr<rmf_traffic::schedule::Viewer> viewer;
  rmf_traffic::schedule::Query::Participants participants;
  rmf_traffic::schedule::Query::Spacetime spacetime;
  float width;

  struct RenderData
  {
    rmf_traffic::schedule::ParticipantId participant;
    rmf_traffic::RouteId route_id;
    Trajectory trajectory;
  };

  mutable std::vector<RenderData> data;
  mutable Fit::Bounds bounds;

  mutable bool dirty = true;

  Implementation(
      std::shared_ptr<rmf_traffic::schedule::Viewer> viewer_,
      rmf_traffic::schedule::Query::Participants participants_,
      std::string map_,
      rmf_traffic::Time start_time_,
      rmf_utils::optional<rmf_traffic::Duration> duration_,
      float width_)
    : viewer(std::move(viewer_)),
      participants(participants_),
      width(width_)
  {
    if (duration_)
    {
      spacetime = rmf_traffic::schedule::Query::Spacetime(
        {map_}, start_time_, start_time_ + *duration_);
    }
    else
    {
      spacetime = rmf_traffic::schedule::Query::Spacetime({map_}, start_time_);
    }
  }

  sf::Color compute_color(rmf_traffic::schedule::ParticipantId) const
  {
    return sf::Color::Magenta;
  }

  Eigen::Vector2d compute_offset(rmf_traffic::schedule::ParticipantId id) const
  {
    if (id == 0)
      return Eigen::Vector2d::Zero();

    if (id % 2 == 0)
      return Eigen::Vector2d::Constant(-id/2 * width);

    return Eigen::Vector2d::Constant((id+1)/2 * width);
  }

  void prepare() const
  {
    if (!dirty)
      return;

    dirty = false;
    assert(spacetime.timespan());
    assert(spacetime.timespan()->get_lower_time_bound());
    const rmf_traffic::Time start_time =
        *spacetime.timespan()->get_lower_time_bound();

    rmf_utils::optional<rmf_traffic::Duration> duration;
    if (spacetime.timespan()->get_upper_time_bound())
    {
      duration =
          *spacetime.timespan()->get_upper_time_bound()
          - *spacetime.timespan()->get_lower_time_bound();
    }

    data.clear();
    const auto view = viewer->query(spacetime, participants);
    data.reserve(view.size());

    for (const auto& v : view)
    {
      data.push_back(
            RenderData{
              v.participant,
              v.route_id,
              Trajectory(
                v.route.trajectory(),
                start_time,
                duration,
                compute_color(v.participant),
                compute_offset(v.participant),
                width)
            });

      bounds.add_bounds(data.back().trajectory.bounds());
    }
  }
};

//==============================================================================
Schedule::Schedule(
    std::shared_ptr<rmf_traffic::schedule::Viewer> viewer,
    float width,
    std::string map,
    rmf_traffic::Time start_time,
    rmf_utils::optional<rmf_traffic::Duration> duration,
    rmf_traffic::schedule::Query::Participants participants)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(viewer),
               std::move(participants),
               std::move(map),
               start_time,
               duration,
               width
             }))
{
  // Do nothing
}

//==============================================================================
Schedule& Schedule::participants(
    const rmf_traffic::schedule::Query::Participants& participants)
{
  _pimpl->participants = participants;
  _pimpl->dirty = true;
  return *this;
}

//==============================================================================
Schedule& Schedule::choose_map(const std::string& name)
{
  if (_pimpl->spacetime.timespan()->maps().count(name))
    return *this;

  _pimpl->spacetime.timespan()->clear_maps().add_map(name);
  _pimpl->dirty = true;
  return *this;
}

//==============================================================================
const std::string& Schedule::current_map() const
{
  assert(!_pimpl->spacetime.timespan()->maps().empty());
  return *_pimpl->spacetime.timespan()->maps().begin();
}

//==============================================================================
Schedule& Schedule::timespan(
    rmf_traffic::Time start,
    rmf_utils::optional<rmf_traffic::Duration> duration)
{
  _pimpl->spacetime.timespan()->set_lower_time_bound(start);

  if (duration)
    _pimpl->spacetime.timespan()->set_upper_time_bound(start + *duration);
  else
    _pimpl->spacetime.timespan()->remove_upper_time_bound();

  _pimpl->dirty = true;
  return *this;
}

//==============================================================================
const Fit::Bounds& Schedule::bounds() const
{
  _pimpl->prepare();
  return _pimpl->bounds;
}

//==============================================================================
rmf_utils::optional<Schedule::Pick> Schedule::pick(float x, float y) const
{
  for (const auto& t : _pimpl->data)
  {
    if (t.trajectory.pick(x, y))
      return Pick{t.participant, t.route_id};
  }

  return rmf_utils::nullopt;
}

//==============================================================================
void Schedule::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
  _pimpl->prepare();
  for (const auto& t : _pimpl->data)
    target.draw(t.trajectory, states);
}

} // namespace draw
} // namespace rmf_planner_viz
