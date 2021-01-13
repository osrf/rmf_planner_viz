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
  std::size_t graph_num_waypoints,
  rmf_traffic::agv::Planner::Debug& debug,
  rmf_traffic::agv::Planner::Debug::Progress& progress,
  const std::chrono::steady_clock::time_point& plan_start_timing,
  bool force_replan,
  bool& show_node_trajectories,
  std::vector<rmf_planner_viz::draw::Trajectory>& trajectories_to_render)
{
  static rmf_utils::optional<rmf_traffic::agv::Plan> current_plan;
  static float node_inspection_timeline_control = 0.0f;
  static float solved_plan_timeline_control = 0.0f;
  static int steps = 0;
  static int selected_node_idx = -1;

  ImGui::SetNextWindowPos(ImVec2(800, 100), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(600, 600), ImGuiCond_FirstUseEver);
  
  ImGui::Begin("Planner AStar Debug", nullptr, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_AlwaysAutoResize);
  
  ImGui::Checkbox("Show node trajectories", &show_node_trajectories);

  ImGui::Separator();
  
  bool reset_planning = false;
  /// Current plan details
  if (ImGui::TreeNode("Current Plan"))
  {
    ImGui::Text("-= Starts --");
    
    for (uint i=0; i<starts.size(); ++i)
    {
      ImGui::PushID(i);
      ImGui::Text("Start #%d", i);

      auto duration_nano = starts[i].time() - plan_start_timing;
      auto duration_milli = std::chrono::duration_cast<std::chrono::milliseconds>(duration_nano);
      
      static int time_offset = 0;
      time_offset = duration_milli.count();
      ImGui::InputInt("Time offset (ms)", &time_offset);
      if (time_offset != duration_milli.count())
      {
        std::chrono::duration<int, std::milli> duration(time_offset);
        starts[i].time(plan_start_timing + duration);
        reset_planning = true;
      }

      ImGuiStyle& style = ImGui::GetStyle();
      float w = ImGui::CalcItemWidth();
      float button_sz = ImGui::GetFrameHeight();
      float spacing = style.ItemInnerSpacing.x;
      ImGui::PushItemWidth(w - spacing * 2.0f - button_sz * 2.0f);

      auto current_wp = starts[i].waypoint();
      std::string preview_val = std::to_string(current_wp);
      if (ImGui::BeginCombo("Waypoint", preview_val.c_str()))
      {
        for (std::size_t graph_idx = 0; graph_idx < graph_num_waypoints; ++graph_idx)
        {
          bool is_selected = current_wp == graph_idx;
          std::string val = std::to_string(graph_idx);
          if (ImGui::Selectable(val.c_str(), is_selected))
          {
            if (current_wp != graph_idx)
              reset_planning = true;
            starts[i].waypoint(graph_idx);
          }
        }
        ImGui::EndCombo();
      }

      static double orientation = 0.0;
      orientation = starts[i].orientation();
      ImGui::InputDouble("Orientation", &orientation, 0.1);
      if (orientation != starts[i].orientation())
      {
        reset_planning = true;
        starts[i].orientation(orientation);  
      }

      ImGui::PopItemWidth();
      ImGui::PopID();
    }
    if (ImGui::Button("Add start.."))
    {
      reset_planning = true;
      starts.emplace_back(plan_start_timing, 0, 0.0);
    }

    //goals
    ImGui::NewLine();
    ImGui::Text("-- Goal --");
    
    ImGui::PushItemWidth(50.f);
    auto current_goal_wp = goal.waypoint();
    std::string preview_val = std::to_string(current_goal_wp);
    if (ImGui::BeginCombo("Goal Waypoint", preview_val.c_str()))
    {
      for (std::size_t graph_idx = 0; graph_idx < graph_num_waypoints; ++graph_idx)
      {
        bool is_selected = current_goal_wp == graph_idx;
        std::string val = std::to_string(graph_idx);
        if (ImGui::Selectable(val.c_str(), is_selected))
        {
          if (current_goal_wp != graph_idx)
            reset_planning = true;
          goal.waypoint(graph_idx);
        }
      }
      ImGui::EndCombo();
    }
    ImGui::PopItemWidth();

    if (goal.orientation())
    {
      static double goal_orientation = 0.0;
      goal_orientation = *goal.orientation();
      ImGui::InputDouble("Goal Orientation", &goal_orientation, 0.1);
      if (goal_orientation != *goal.orientation())
      {
        reset_planning = true;
        goal.orientation(goal_orientation);  
      }
      if (ImGui::Button("Any goal orientation"))
      {
        goal.any_orientation();
        reset_planning = true;
      }
    }
    else
    {
      ImGui::Text("No goal orientation");
      if (ImGui::Button("Specify goal orientation"))
      {
        goal.orientation(0.0);
        reset_planning = true;
      }
    }
    ImGui::TreePop();
  }
  ImGui::Separator();

  if (reset_planning || force_replan)
  {
    progress = debug.begin(starts, goal, planner.get_default_options());
    current_plan.reset();
    steps = 0;
    selected_node_idx = -1;
  }
  
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

  if (current_plan)
  {
    // Display solved plan details
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Solved Plan is available!");

    ImGui::Text("Plan cost: %f", current_plan->get_cost());
    
    std::string wp_list_txt = "Waypoints: [";
    for (const auto& wp : current_plan->get_waypoints())
    {
      wp_list_txt += std::to_string(*wp.graph_index());
      wp_list_txt += ",";
    }
    wp_list_txt += "]";
    ImGui::Text("%s", wp_list_txt.c_str());

    // compute a timeline
    rmf_traffic::Time start_timestamp = plan_start_timing;
    rmf_traffic::Time finish_timestamp;
    for (auto route : current_plan->get_itinerary())
    {
      const auto& traj = route.trajectory();
      auto finish = traj.finish_time();

      if (finish && finish_timestamp < *finish)
        finish_timestamp = *finish;
    }

    auto diff = finish_timestamp - start_timestamp;
    auto max_duration = rmf_traffic::time::to_seconds(diff);
    if (solved_plan_timeline_control > max_duration)
      solved_plan_timeline_control = max_duration;
    ImGui::Text("Max duration: %f", max_duration);
    ImGui::SliderFloat("Solved Plan Timeline Control", &solved_plan_timeline_control, 0.0f, max_duration);

    auto trajectory_start_time = rmf_traffic::time::apply_offset(
      start_timestamp, solved_plan_timeline_control);

    static bool show_solved_plan = true;
    ImGui::Checkbox("Show solved plan itinerary", &show_solved_plan);
    if (show_solved_plan)
    {
      for (auto route : current_plan->get_itinerary())
      {
        const auto& traj = route.trajectory();
        auto trajectory = rmf_planner_viz::draw::Trajectory(traj,
          profile, trajectory_start_time, std::nullopt, sf::Color::Green, { 0.0, 0.0 }, 0.5f);
        trajectories_to_render.push_back(trajectory);
      }
    }

    auto start = current_plan->get_start();
    ImGui::Text("Start waypoint: %lu", start.waypoint());
    if (start.lane())
      ImGui::Text("Start lane: %lu", *start.lane());
    ImGui::Text("Start time: %ld", start.time().time_since_epoch().count());
    ImGui::Text("Start orientation: %f", start.orientation());
  }
  else
  {
    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Current plan not available");
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

    static bool render_inspected_node_trajectories = false;
    ImGui::Checkbox("Render Inspected Node Trajectories", &render_inspected_node_trajectories);

    float duration = 0.f;
    const std::vector<rmf_traffic::Route>& routes_from_parent = selected_node->route_from_parent;
    if (!routes_from_parent.empty())
    {
      long int start_time = LONG_MAX, end_time = 0;
      
      for (const auto& route : routes_from_parent)
      {
        auto start = route.trajectory().start_time()->time_since_epoch().count();
        auto end = route.trajectory().finish_time()->time_since_epoch().count();
        if (start < start_time)
          start_time = start;
        if (end > end_time)
          end_time = end;

        duration += rmf_traffic::time::to_seconds(route.trajectory().duration());
      }
      
      ImGui::Text("Node Traj start time: %ld", start_time);
      ImGui::Text("Node Traj finish time: %ld", end_time);
      ImGui::Text("Node Traj duration: %f", duration);

      if (node_inspection_timeline_control > duration)
        node_inspection_timeline_control = duration;
      ImGui::SliderFloat("Inspected Node Timeline Control", &node_inspection_timeline_control, 0.0f, duration);
      
      auto trajectory_start_time = rmf_traffic::time::apply_offset(
        rmf_traffic::Time(std::chrono::nanoseconds(start_time)), node_inspection_timeline_control);

      if (render_inspected_node_trajectories)
      {
        for (const auto& route : routes_from_parent)
        {
          const auto& traj = route.trajectory();
          auto trajectory = rmf_planner_viz::draw::Trajectory(traj,
            profile, trajectory_start_time, std::nullopt, sf::Color::Green, { 0.0, 0.0 }, 0.5f);
          trajectories_to_render.push_back(trajectory);
        }
      }

    }
    

    ImGui::NewLine();

    int parent_count = 0;
    std::string parent_waypoint_str = "Parent waypoints idx: [";
    auto parent_node = selected_node->parent;
    while (parent_node)
    {
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
  }

  ImGui::End();
}

bool do_planner_presets(
  std::vector<rmf_traffic::agv::Planner::Start>& starts,
  rmf_traffic::agv::Planner::Goal& goal,
  const std::chrono::steady_clock::time_point& plan_start_timing)
{
  bool reset = false;
  ImGui::SetNextWindowPos(ImVec2(800, 120), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowPos(ImVec2(0, 50), ImGuiCond_FirstUseEver);
  
  ImGui::Begin("Start/Goal Presets", nullptr, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_AlwaysAutoResize);
  
  if (ImGui::Button("Start/Goal Preset #0"))
  {
    starts.clear();
    starts.emplace_back(plan_start_timing, 11, 0.0);
    goal = rmf_traffic::agv::Planner::Goal(3);
    reset = true;
  }
  if (ImGui::Button("Start/Goal Preset #1"))
  {
    starts.clear();
    starts.emplace_back(plan_start_timing, 10, 0.0);
    goal = rmf_traffic::agv::Planner::Goal(3);
    reset = true;
  }
  if (ImGui::Button("Start/Goal Preset #2"))
  {
    starts.clear();
    starts.emplace_back(plan_start_timing, 9, 0.0);
    starts.emplace_back(plan_start_timing, 11, 0.0);
    goal = rmf_traffic::agv::Planner::Goal(3);
    reset = true;
  }
  ImGui::End();

  return reset;
}

} // namespace draw
} // namespace rmf_planner_viz
