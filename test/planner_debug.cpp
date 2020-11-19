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

#include "planner_debug.hpp"

#include <SFML/Graphics.hpp>
#include <imgui.h>

#include <rmf_planner_viz/draw/Trajectory.hpp>

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/Planner.hpp>

#include "imgui-SFML.h"

namespace rmf_planner_viz {
namespace draw {

void do_planner_debug(
  const rmf_traffic::Profile& profile, 
  rmf_traffic::agv::Planner& planner,
  std::vector<rmf_traffic::agv::Planner::Start>& starts,
  rmf_traffic::agv::Planner::Goal& goal,
  rmf_traffic::agv::Planner::Debug& debug,
  rmf_traffic::agv::Planner::Debug::Progress& progress,
  const std::chrono::steady_clock::time_point& plan_start_timing,
  bool& show_node_trajectories,
  std::vector<rmf_planner_viz::draw::Trajectory>& trajectories_to_render)
{
  static rmf_utils::optional<rmf_traffic::agv::Plan> current_plan;
  static float timeline_control_value = 0.0f;
  static int steps = 0;
  static int selected_node_idx = -1;

  ImGui::SetWindowPos(ImVec2(800, 200));
  ImGui::SetWindowSize(ImVec2(600, 800));
  
  ImGui::Begin("Planner AStar Debug", nullptr, ImGuiWindowFlags_HorizontalScrollbar);
  
  ImGui::Checkbox("Show node trajectories", &show_node_trajectories);

  ImGui::Separator();
  
  /// Presets, edit you plans here as you please
  if (ImGui::TreeNode("Presets"))
  {
    bool preset_triggered = false;
    if (ImGui::Button("Preset #0"))
    {
      starts.clear();
      starts.emplace_back(plan_start_timing, 11, 0.0);
      goal = rmf_traffic::agv::Planner::Goal(3);
      preset_triggered = true;
    }
    if (ImGui::Button("Preset #1"))
    {
      starts.clear();
      starts.emplace_back(plan_start_timing, 10, 0.0);
      goal = rmf_traffic::agv::Planner::Goal(3);
      preset_triggered = true;
    }
    if (ImGui::Button("Preset #2"))
    {
      starts.clear();
      starts.emplace_back(plan_start_timing, 9, 0.0);
      starts.emplace_back(plan_start_timing, 11, 0.0);
      goal = rmf_traffic::agv::Planner::Goal(3);
      preset_triggered = true;
    }
    if (preset_triggered)
    {
      progress = debug.begin(starts, goal, planner.get_default_options());
      current_plan.reset();
      steps = 0;
      selected_node_idx = -1;
    }
    ImGui::TreePop();
  }
  
  ImGui::Separator();

  /// Current plan details
  if (ImGui::TreeNode("Current Plan"))
  {
    ImGui::Text("Starts: ");
    for (uint i=0; i<starts.size(); ++i)
    {
      ImGui::Text("#%d", i);
      ImGui::Text("Time: %ld", starts[i].time().time_since_epoch().count());
      ImGui::Text("Waypoint: %lu", starts[i].waypoint());
      ImGui::Text("Orientation: %f", starts[i].orientation());
    }

    //goals
    ImGui::NewLine();
    ImGui::Text("Goal waypoint: %lu", goal.waypoint());
    if (goal.orientation())
      ImGui::Text("Goal orientation: %f", *goal.orientation());
    else
      ImGui::Text("No chosen goal orientation");
    ImGui::TreePop();
  }
  ImGui::Separator();

  /// AStar plan control
  ImGui::TextColored(ImVec4(0, 1, 0, 1), "AStar plan generation");
  ImGui::TextColored(ImVec4(0, 1, 0, 1), "Steps taken: %d", steps);
  if (ImGui::Button("Step forward"))
  {
    current_plan = progress.step();
    ++steps;
  }
  if (ImGui::Button("Step forward until valid plan.."))
  {
    current_plan = progress.step();
    ++steps;
    while (!current_plan)
    {
      current_plan = progress.step();
      ++steps;
    }
  }
  
  if (ImGui::TreeNode("Reset/Jump to.."))
  {
    static int steps_jump = 0;
    ImGui::InputInt("Jump steps", &steps_jump);
    if (steps_jump < 0)
      steps_jump = 0;
    char reset_label[32] = { 0 };
    snprintf(reset_label, sizeof(reset_label), "Reset to %d steps", steps_jump);
    if (ImGui::Button(reset_label))
    {
      progress = debug.begin(starts, goal, planner.get_default_options());

      current_plan.reset();
      for (int i=0; i<steps_jump; ++i)
        current_plan = progress.step();
      steps = steps_jump;
    }
    ImGui::TreePop();
  }
  
  ImGui::Separator();

  trajectories_to_render.clear();
  /// AStar Node details  
  if (current_plan)
  {      
    auto searchqueue = progress.queue();
    auto& container = get_priority_queue_container(searchqueue);

    ImGui::TextColored(ImVec4(0, 1, 0, 1), "AStar Node Count: %lu", container.size());
    ImGui::NewLine();

    if (ImGui::ListBoxHeader("AStar Nodes"))
    {
      for (uint i=0; i<container.size(); ++i)
      {
        auto& node = container[i];
        char node_name[32] = { 0 };
        snprintf(node_name, sizeof(node_name), "Node %d (score: %f)",
          i, node->current_cost + node->remaining_cost_estimate);
        if (ImGui::Selectable(node_name, (int)i == selected_node_idx))
          selected_node_idx = i;
      }
      ImGui::ListBoxFooter();
    }
    
    ImGui::NewLine();
    ImGui::Separator();
    if (selected_node_idx != -1 && selected_node_idx < (int)container.size())
    {
      ImGui::TextColored(ImVec4(0, 1, 0, 1), "Node #%d Inspection", selected_node_idx);
      auto selected_node = container[selected_node_idx];
      ImGui::Text("Current Cost: %f", selected_node->current_cost);
      ImGui::Text("Remaining Cost Estimate: %f", selected_node->remaining_cost_estimate);
      if (selected_node->waypoint)
        ImGui::Text("Waypoint: %lu", *selected_node->waypoint);
      if (selected_node->start_set_index)
        ImGui::Text("start_set_index: %lu", *selected_node->start_set_index);
      
      const rmf_traffic::Route& route = selected_node->route_from_parent;
      if (route.trajectory().start_time())
        ImGui::Text("Node Traj start time: %ld",  route.trajectory().start_time()->time_since_epoch().count());
      if (route.trajectory().finish_time())
        ImGui::Text("Node Traj finish time: %ld", route.trajectory().finish_time()->time_since_epoch().count());
      ImGui::Text("Node Traj duration: %f", rmf_traffic::time::to_seconds(route.trajectory().duration()));

      ImGui::NewLine();

      rmf_traffic::Time start_timestamp = *route.trajectory().start_time();
      rmf_traffic::Time finish_timestamp = *route.trajectory().finish_time();
      
      int parent_count = 0;
      std::string parent_waypoint_str = "Parent waypoints idx: [";
      auto parent_node = selected_node->parent;
      while (parent_node)
      {
        auto& route_p = parent_node->route_from_parent;
        auto route_start_time = route_p.trajectory().start_time();
        if (route_start_time && start_timestamp > *route_start_time)
          start_timestamp = *route_start_time;
        auto route_fin_time = route_p.trajectory().finish_time();
        if (route_fin_time && finish_timestamp < *route_fin_time)
          finish_timestamp = *route_fin_time;
        
        if (parent_node->waypoint)
          parent_waypoint_str += std::to_string(*parent_node->waypoint);
        else
          parent_waypoint_str += "?";
        parent_waypoint_str += ", ";
        parent_node = parent_node->parent;
        ++parent_count;
      }
      ImGui::Text("Parent count: %d", parent_count);
      parent_waypoint_str += "]";
      ImGui::Text("%s", parent_waypoint_str.c_str());

      // submit trajectories to draw
      static bool render_parent_trajectories = false;
      ImGui::Checkbox("Render Parent Trajectories", &render_parent_trajectories);

      auto trajectory_start_time = rmf_traffic::time::apply_offset(
        plan_start_timing, timeline_control_value);
        
      auto add_trajectory_to_render = [trajectory_start_time, &profile](
        std::vector<rmf_planner_viz::draw::Trajectory>& to_render,
        rmf_traffic::agv::Planner::Debug::ConstNodePtr node)
      {
        const rmf_traffic::Route& route = node->route_from_parent;
        const auto& traj = route.trajectory();
        auto trajectory = rmf_planner_viz::draw::Trajectory(traj, 
          profile, trajectory_start_time, std::nullopt, sf::Color::Green, { 0.0, 0.0 }, 0.5f);
        to_render.push_back(trajectory);
      };
      add_trajectory_to_render(trajectories_to_render, selected_node);
      
      if (render_parent_trajectories)
      {
        parent_node = selected_node->parent;
        while (parent_node)
        {
          add_trajectory_to_render(trajectories_to_render, parent_node);
          parent_node = parent_node->parent;
        }
      }

      auto diff = finish_timestamp - start_timestamp;
      auto max_duration = rmf_traffic::time::to_seconds(diff);
      if (timeline_control_value > max_duration)
        timeline_control_value = max_duration;
      ImGui::SliderFloat("Timeline Control", &timeline_control_value, 0.0f, max_duration);
    }
  }
  else
    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Current plan not available");

  ImGui::End();
}


} // namespace draw
} // namespace rmf_planner_viz