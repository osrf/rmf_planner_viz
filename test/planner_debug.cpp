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

void do_planner_debug(
  const rmf_traffic::Profile& profile, 
  rmf_traffic::agv::Planner& planner,
  std::vector<rmf_traffic::agv::Planner::Start>& starts,
  rmf_traffic::agv::Planner::Goal& goal,
  rmf_traffic::agv::Planner::Debug& debug,
  rmf_traffic::agv::Planner::Debug::Progress& progress,
  const std::chrono::steady_clock::time_point& start_timing,
  bool& show_node_trajectories,
  std::vector<rmf_planner_viz::draw::Trajectory>& trajectories_to_render)
{
  static rmf_utils::optional<rmf_traffic::agv::Plan> current_plan;

  ImGui::SetWindowPos(ImVec2(800, 200));
  ImGui::SetWindowSize(ImVec2(600, 800));
  
  ImGui::Begin("Planner AStar Debug", nullptr, ImGuiWindowFlags_HorizontalScrollbar);
  
  ImGui::Checkbox("Show node trajectories", &show_node_trajectories);

  ImGui::Separator();
  
  // presets
  bool preset_triggered = false;
  if (ImGui::Button("Preset #0"))
  {
    starts.clear();
    starts.emplace_back(start_timing, 11, 0.0);
    goal = rmf_traffic::agv::Planner::Goal(3);
    preset_triggered = true;
  }
  if (ImGui::Button("Preset #1"))
  {
    starts.clear();
    starts.emplace_back(start_timing, 10, 0.0);
    goal = rmf_traffic::agv::Planner::Goal(3);
    preset_triggered = true;
  }
  if (ImGui::Button("Preset #2"))
  {
    starts.clear();
    starts.emplace_back(start_timing, 9, 0.0);
    goal = rmf_traffic::agv::Planner::Goal(3);
    preset_triggered = true;
  }
  if (preset_triggered)
  {
    progress = debug.begin(starts, goal, planner.get_default_options());
    current_plan.reset();
  }
  
  ImGui::Separator();

  if (ImGui::TreeNode("Current Plan"))
  {
    ImGui::Text("Starts: ");
    for (uint i=0; i<starts.size(); ++i)
    {
      ImGui::Text("#%d", i);
      ImGui::Text("Time: %llu", starts[i].time().time_since_epoch().count());
      ImGui::Text("Waypoint: %llu", starts[i].waypoint());
      ImGui::Text("Orientation: %f", starts[i].orientation());
    }

    //goals
    ImGui::NewLine();
    ImGui::Text("Goal waypoint: %llu", goal.waypoint());
    if (goal.orientation())
      ImGui::Text("Goal orientation: %f", goal.orientation());
    else
      ImGui::Text("No chosen goal orientation");
    ImGui::TreePop();
  }
  ImGui::Separator();

  static float timeline_duration = 0.0f;
  static int steps = 0;
  ImGui::TextColored(ImVec4(0, 1, 0, 1), "AStar plan generation", steps);
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
      for (uint i=0; i<steps_jump; ++i)
        current_plan = progress.step();
      steps = steps_jump;
    }
    ImGui::TreePop();
  }
  
  ImGui::Separator();

  trajectories_to_render.clear();
  
  if (current_plan)
  {      
    auto searchqueue = progress.queue();
    auto& container = get_priority_queue_container(searchqueue);

    ImGui::TextColored(ImVec4(0, 1, 0, 1), "AStar Node Count: %d", container.size());
    ImGui::NewLine();

    static int selected_idx = -1;
    if (ImGui::ListBoxHeader("AStar Nodes"))
    {
      for (uint i=0; i<container.size(); ++i)
      {
        auto& node = container[i];
        node->route_from_parent;
        node->parent;
        char node_name[32] = { 0 };
        snprintf(node_name, sizeof(node_name), "Node %d (score: %f)",
          i, node->current_cost + node->remaining_cost_estimate);
        if (ImGui::Selectable(node_name, (int)i == selected_idx))
          selected_idx = i;
      }
      ImGui::ListBoxFooter();
    }
    
    ImGui::NewLine();
    ImGui::Separator();
    if (selected_idx != -1)
    {
      ImGui::TextColored(ImVec4(0, 1, 0, 1), "Node #%d Inspection", selected_idx);
      auto selected_node = container[selected_idx];
      ImGui::Text("Current Cost: %f", selected_node->current_cost);
      ImGui::Text("Remaining Cost Estimate: %f", selected_node->remaining_cost_estimate);
      ImGui::Text("Waypoint: %d", selected_node->waypoint);
      if (selected_node->start_set_index)
        ImGui::Text("start_set_index: %d", *selected_node->start_set_index);
      
      const rmf_traffic::Route& route = selected_node->route_from_parent;
      if (route.trajectory().start_time())
        ImGui::Text("Node Traj start time: %llu",  route.trajectory().start_time()->time_since_epoch());
      if (route.trajectory().finish_time())
        ImGui::Text("Node Traj finish time: %llu", route.trajectory().finish_time()->time_since_epoch());
      ImGui::Text("Node Traj duration: %f", rmf_traffic::time::to_seconds(route.trajectory().duration()));

      ImGui::NewLine();

      auto trajectory_time = rmf_traffic::time::apply_offset(start_timing, static_cast<double>(timeline_duration));

      auto add_trajectory_to_render = [trajectory_time, &profile](
        std::vector<rmf_planner_viz::draw::Trajectory>& to_render,
        rmf_traffic::agv::Planner::Debug::ConstNodePtr node)
      {
        const rmf_traffic::Route& route = node->route_from_parent;
        const auto& traj = route.trajectory();
        auto trajectory = rmf_planner_viz::draw::Trajectory(traj, 
          profile, trajectory_time, traj.duration(), sf::Color::Green, { 0.0, 0.0 }, 0.5f);
        to_render.push_back(trajectory);
      };
      add_trajectory_to_render(trajectories_to_render, selected_node);

      static bool render_parent_trajectories = false;
      ImGui::Checkbox("Render Parent Trajectories", &render_parent_trajectories);

      //double max_duration = selected_node->current_cost + selected_node->remaining_cost_estimate;
      double max_duration = rmf_traffic::time::to_seconds(route.trajectory().duration());
      if (render_parent_trajectories)
      {
        int x = 0;
        auto parent_node = selected_node->parent;
        while (parent_node)
        {
          const rmf_traffic::Route& route = parent_node->route_from_parent;
          max_duration += rmf_traffic::time::to_seconds(route.trajectory().duration());
          add_trajectory_to_render(trajectories_to_render, parent_node);
          parent_node = parent_node->parent;
          ++x;
        }
        ImGui::Text("count: %d", x);
      }
      ImGui::SliderFloat("Timeline Control", &timeline_duration, 0.0f, static_cast<float>(max_duration));
    }
    else
      trajectories_to_render.clear();
  }
  else
    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Current plan not available");

  ImGui::End();

}