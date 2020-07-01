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

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Schedule::Implementation
{
public:

  std::shared_ptr<rmf_traffic::schedule::Viewer> viewer;
  rmf_traffic::schedule::Query::Participants participants;
  std::string current_map;
  rmf_traffic::Time start_time;
  rmf_utils::optional<rmf_traffic::Duration> duration;
  float width;

};

//==============================================================================
Schedule::Schedule(
    std::shared_ptr<rmf_traffic::schedule::Viewer> viewer,
    rmf_traffic::schedule::Query::Participants participants,
    std::string map,
    rmf_traffic::Time start_time,
    rmf_utils::optional<rmf_traffic::Duration> duration,
    float width)
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
const Fit::Bounds& Schedule::bounds() const
{

}

} // namespace draw
} // namespace rmf_planner_viz
