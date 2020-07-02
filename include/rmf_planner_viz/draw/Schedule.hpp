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

#ifndef RMF_PLANNER_VIZ__DRAW__SCHEDULE_HPP
#define RMF_PLANNER_VIZ__DRAW__SCHEDULE_HPP

#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Writer.hpp>

#include <rmf_planner_viz/draw/Fit.hpp>

#include <SFML/Graphics/Drawable.hpp>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Schedule : public sf::Drawable
{
public:

  Schedule(
      std::shared_ptr<rmf_traffic::schedule::Viewer> viewer,
      float width,
      std::string map,
      rmf_traffic::Time start_time,
      rmf_utils::optional<rmf_traffic::Duration> duration = rmf_utils::nullopt,
      rmf_traffic::schedule::Query::Participants participants =
          rmf_traffic::schedule::Query::Participants::make_all());

  Schedule& participants(
      const rmf_traffic::schedule::Query::Participants& participants);

  Schedule& choose_map(const std::string& name);

  const std::string& current_map() const;

  Schedule& timespan(
      rmf_traffic::Time start,
      rmf_utils::optional<rmf_traffic::Duration> duration);

  const Fit::Bounds& bounds() const;

  struct Pick
  {
    rmf_traffic::schedule::ParticipantId participant;
    rmf_traffic::RouteId route_id;
  };

  rmf_utils::optional<Pick> pick(float x, float y) const;


protected:

  void draw(sf::RenderTarget& target, sf::RenderStates states) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace draw
} // namespace rmf_planner_viz

#endif // RMF_PLANNER_VIZ__DRAW__SCHEDULE_HPP
