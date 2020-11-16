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

#include <SFML/Graphics.hpp>
#include <imgui.h>

#include <iostream>

#include <rmf_planner_viz/draw/Graph.hpp>
#include <rmf_planner_viz/draw/Schedule.hpp>
#include <rmf_planner_viz/draw/IMDraw.hpp>

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/Planner.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include "imgui-SFML.h"

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

int main()
{
  const rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph_0;
  graph_0.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph_0.add_waypoint(test_map_name, {0.0, -5.0});  // 1
  graph_0.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true);  // 2
  graph_0.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph_0.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  graph_0.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph_0.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph_0.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph_0.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  graph_0.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
  graph_0.add_waypoint(test_map_name, {0.0, 10.0}); // 10
  graph_0.add_waypoint(test_map_name, {5.0, 10.0}); // 11
  graph_0.add_waypoint(test_map_name, {-12.0, 10.0}); // 12

  /*            0<------------1<------------2
   *                                        ^
   *                                        |
   *  12------------->10----->11            |
   *                   |      |             |
   *                   |      v             |
   *                   8------9             |
   *                   |      |             |
   *                   |      |             |
   *     3------4------5------6------7      3
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph_0.add_lane(w0, w1);
      graph_0.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(1, 5);
  add_bidir_lane(2, 6);
  add_bidir_lane(3, 4);
  add_bidir_lane(4, 5);
  add_bidir_lane(5, 6);
  add_bidir_lane(6, 7);
  add_bidir_lane(5, 8);
  add_bidir_lane(6, 9);
  add_bidir_lane(8, 9);
  add_bidir_lane(8, 10);
  graph_0.add_lane(10, 11);
  graph_0.add_lane(11, 9);
  graph_0.add_lane(12, 10);

  rmf_planner_viz::draw::Graph graph_0_drawable(graph_0, 1.0);

  rmf_traffic::agv::Graph graph_1;
  graph_1.add_waypoint(test_map_name, {-5.0, 15.0}); // 0
  graph_1.add_waypoint(test_map_name, { 5.0, 15.0}); // 1
  graph_1.add_waypoint(test_map_name, {15.0, 15.0}); // 2
  graph_1.add_waypoint(test_map_name, {15.0,  0.0}); // 3
  graph_1.add_lane(1, 0);
  graph_1.add_lane(2, 1);
  graph_1.add_lane(3, 2);

  rmf_planner_viz::draw::Graph graph_1_drawable(graph_1, 0.5);

  std::shared_ptr<rmf_traffic::schedule::Database> database =
      std::make_shared<rmf_traffic::schedule::Database>();

  rmf_traffic::agv::Planner planner_0(
        rmf_traffic::agv::Planner::Configuration(graph_0, traits),
        rmf_traffic::agv::Planner::Options(nullptr));

  /// Setup participants
  auto p0 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_0",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto p1 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_1",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto pa = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_0",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto pb = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_0",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

//  auto pc = rmf_traffic::schedule::make_participant(
//        rmf_traffic::schedule::ParticipantDescription{
//          "participant_0",
//          "simple_test",
//          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
//          profile
//        },
//        database);

  auto p2 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_2",
          "simple_test",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);


  // set plans for the participants
  const auto now = std::chrono::steady_clock::now();

  std::vector<rmf_traffic::agv::Planner::Start> starts;
  starts.emplace_back(now, 11, 0.0);
  // starts.emplace_back(now, 12, 0.0);
  // starts.emplace_back(now, 10, 0.0);

  //p0.set(planner_0.plan(starts[0], 3)->get_itinerary());
  // p1.set(planner_0.plan(starts[1], 2)->get_itinerary());
  // p2.set(planner_0.plan(starts[2], 7,
  //   rmf_traffic::agv::Plan::Options(
  //   rmf_traffic::agv::ScheduleRouteValidator::make(
  //       database, p2.id(), p2.description().profile())))->get_itinerary());

  rmf_traffic::agv::Planner::Goal planner_goal(3);
  rmf_traffic::agv::Planner::Debug planner_dbg_0(planner_0);
  rmf_traffic::agv::Planner::Debug::Progress progress =
    planner_dbg_0.begin(starts, planner_goal, planner_0.get_default_options());

  rmf_utils::optional<rmf_traffic::agv::Plan> current_plan;
  
  //rmf_traffic::agv::Planner::Progress planner_dbg_0(planner_0);
  rmf_planner_viz::draw::Schedule schedule_drawable(
        database, 0.25, test_map_name, now);

  rmf_planner_viz::draw::Fit fit(
    {graph_0_drawable.bounds(), graph_1_drawable.bounds()}, 0.02);

  sf::RenderWindow app_window(
        sf::VideoMode(1250, 1028),
        "Simple Test",
        sf::Style::Default);

  app_window.resetGLStates();

  ImGui::SFML::Init(app_window);

  sf::Clock deltaClock;
  while (app_window.isOpen())
  {
    sf::Event event;
    while (app_window.pollEvent(event))
    {
      ImGui::SFML::ProcessEvent(event);

      if (event.type == sf::Event::Closed)
      {
        return 0;
      }

      if (event.type == sf::Event::Resized)
      {
        sf::FloatRect visibleArea(0, 0, event.size.width, event.size.height);
        app_window.setView(sf::View(visibleArea));
      }

      if (event.type == sf::Event::MouseButtonPressed)
      {
        const sf::Vector2f p =
            fit.compute_transform(app_window.getSize()).getInverse()
            * sf::Vector2f(event.mouseButton.x, event.mouseButton.y);

        const auto pick = graph_0_drawable.pick(p.x, p.y);
        if (pick)
          graph_0_drawable.select(*pick);
      }
    }

    ImGui::SFML::Update(app_window, deltaClock.restart());

    ImGui::SetWindowSize(ImVec2(600, 200));
    
    ImGui::Begin("Demo control panel", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Demo control panel");
    ImGui::End();
    
    // Planner debugging
    ImGui::SetWindowPos(ImVec2(800, 200));
    ImGui::SetWindowSize(ImVec2(600, 800));
    
    ImGui::Begin("Planner AStar Debug", nullptr, ImGuiWindowFlags_None);

    static bool show_node_trajectories = false;
    if (ImGui::Checkbox("Show node trajectories", &show_node_trajectories))
      ;
    ImGui::Separator();
    if (ImGui::Button("Preset #0"))
    {
      
    }

    
    ImGui::Separator();

    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Current plan:");
    ImGui::NewLine();
    
    ImGui::Text("Starts: ");
    for (uint i=0; i<starts.size(); ++i)
    {
      char node_name[32] = { 0 };
      snprintf(node_name, sizeof(node_name), "#%d", i);
      if (ImGui::TreeNode(node_name))
      {
        ImGui::Text("Time: %f", starts[i].time().time_since_epoch());
        ImGui::Text("Waypoint: %d", starts[i].waypoint());
        ImGui::Text("Orientation: %d", starts[i].orientation());
        ImGui::TreePop();
      }
    }

    //goals
    ImGui::NewLine();
    ImGui::Text("Goal waypoint: %d", planner_goal.waypoint());
    if (planner_goal.orientation())
      ImGui::Text("Goal orientation: %f", *planner_goal.orientation());
    else
      ImGui::Text("No chosen goal orientation");

    static int steps = 0;
    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Steps taken: %d", steps);
    if (ImGui::Button("Step Forward"))
    {
      current_plan = progress.step();
      ++steps;
    }
    if (ImGui::Button("Reset"))
    {
      progress = planner_dbg_0.begin(starts, planner_goal, planner_0.get_default_options());
      steps = 0;
      current_plan.reset();
    }

    ImGui::Separator();

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
          node->remaining_cost_estimate;
          node->current_cost;
          node->waypoint;
          node->parent;
          char node_name[32] = { 0 };
          snprintf(node_name, sizeof(node_name), "Node %d", i);
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

      }
      
      //bool changed = ImGui::ListBox("AStar Nodes", &item_selected, node_names.data(), node_names.size());
    }
    else
      ImGui::TextColored(ImVec4(0, 1, 0, 1), "Current plan not available");

    ImGui::End();

    ImGui::EndFrame();

    /*** drawing ***/
    app_window.clear();

    schedule_drawable.timespan(std::chrono::steady_clock::now());

    sf::RenderStates states;
    fit.apply_transform(states.transform, app_window.getSize());
    app_window.draw(graph_0_drawable, states);
    app_window.draw(graph_1_drawable, states);
    app_window.draw(schedule_drawable, states);

    {
      sf::Transformable vqs;
      vqs.setScale(1.f, -1.f);
      auto tx_flipped_2d = vqs.getTransform();
      rmf_planner_viz::draw::IMDraw::flush_and_render(app_window, tx_flipped_2d);
    }

    ImGui::SFML::Render(app_window);
    app_window.display();
  }
}
