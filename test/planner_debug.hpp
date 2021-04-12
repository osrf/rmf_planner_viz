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


#ifndef RMF_PLANNER_VIZ__DRAW__PLANNERDEBUG_HPP
#define RMF_PLANNER_VIZ__DRAW__PLANNERDEBUG_HPP

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/debug_Planner.hpp>

#include <rmf_planner_viz/draw/Trajectory.hpp>
#include <rmf_planner_viz/draw/Schedule.hpp>

#include <queue>

namespace rmf_planner_viz {
namespace draw {

// Hacky method to access the underlying container of std::priority_queue
// normally we'll derive off std::priority_queue and add a get_container function
// but this will suffice for the debugger's purposes
// Returned container will not provide any guarantees about order
template <class T, class S, class C>
S& get_priority_queue_container(std::priority_queue<T, S, C>& queue)
{
  struct HackedQueue : private std::priority_queue<T, S, C> 
  {
    static S& Container(std::priority_queue<T, S, C>& queue) 
    {
      return queue.*&HackedQueue::c;
    }
  };
  return HackedQueue::Container(queue);
}

void do_planner_debug(
  const rmf_traffic::Profile& profile,
  const std::string& chosen_map,
  rmf_traffic::agv::Planner& planner,
  std::vector<rmf_traffic::agv::Planner::Start>& starts,
  rmf_traffic::agv::Planner::Goal& goal,
  std::size_t graph_num_waypoints,
  rmf_traffic::agv::Planner::Debug& debug,
  rmf_traffic::agv::Planner::Debug::Progress& progress,
  const std::chrono::steady_clock::time_point& plan_start_timing, // earliest time the timeline starts from
  bool force_replan,
  bool& show_node_trajectories,
  std::vector<rmf_planner_viz::draw::Trajectory>& trajectories_to_render,
  rmf_planner_viz::draw::Schedule& schedule);

bool do_planner_presets(
  std::vector<rmf_traffic::agv::Planner::Start>& starts,
  rmf_traffic::agv::Planner::Goal& goal,
  const std::chrono::steady_clock::time_point& plan_start_timing);

} // namespace draw
} // namespace rmf_planner_viz

#endif // RMF_PLANNER_VIZ__DRAW__PLANNERDEBUG_HPP
