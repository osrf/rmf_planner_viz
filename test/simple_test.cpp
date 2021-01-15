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
#include <rmf_planner_viz/draw/Trajectory.hpp>

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/debug_Planner.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include "imgui-SFML.h"
#include "planner_debug.hpp"

int main(int argc, char* argv[])
{
  sf::Font font;
  if (!font.loadFromFile("./build/rmf_planner_viz/fonts/OpenSans-Bold.ttf"))
  {
    std::cout << "Failed to load font. Make sure you run the executable from the colcon directory" << std::endl;
    return -1;
  }
  
  const rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  rmf_traffic::agv::Graph graph_0;

  const std::string test_map_name = "test_map";
  if (argc == 2)
  {
    std::cout << "Loading nav_graph file " << argv[1] << std::endl;
    graph_0 = rmf_fleet_adapter::agv::parse_graph(
      argv[1], 
      traits);
  }
  else // default graph
  {
    std::cout << "Using default map" << std::endl;
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
    graph_0.add_key("Interesting Waypoint", 0);

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
  }

  rmf_planner_viz::draw::Graph graph_0_drawable(graph_0, 1.0, font);
  std::vector<std::string> map_names = graph_0_drawable.get_map_names();
  std::string chosen_map;
  if (graph_0_drawable.current_map())
    chosen_map = *graph_0_drawable.current_map();
  
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
  using namespace std::chrono_literals;
  auto plan_start_timing = std::chrono::steady_clock::now();

  std::vector<rmf_traffic::agv::Planner::Start> starts;
  starts.emplace_back(plan_start_timing, 0, 0.0);
  // starts.emplace_back(now, 12, 0.0);
  // starts.emplace_back(now, 10, 0.0);

  // p0.set(planner_0.plan(starts[0], 3)->get_itinerary());
  // p1.set(planner_0.plan(starts[1], 2)->get_itinerary());
  // p2.set(planner_0.plan(starts[2], 7,
  //   rmf_traffic::agv::Plan::Options(
  //   rmf_traffic::agv::ScheduleRouteValidator::make(
  //       database, p2.id(), p2.description().profile())))->get_itinerary());

  rmf_traffic::agv::Planner::Goal goal(0);
  rmf_traffic::agv::Planner::Debug planner_debug(planner_0);
  rmf_traffic::agv::Planner::Debug::Progress progress =
    planner_debug.begin(starts, goal, planner_0.get_default_options());

  rmf_planner_viz::draw::Schedule schedule_drawable(
        database, 0.25, test_map_name, plan_start_timing);

  rmf_planner_viz::draw::Fit fit(
    {graph_0_drawable.bounds()}, 0.02);

  sf::RenderWindow app_window(
        sf::VideoMode(1250, 1028),
        "Simple Test",
        sf::Style::Default);

  app_window.resetGLStates();
  sf::View max_cam_view = app_window.getView();
  sf::Vector2f max_cam_size = max_cam_view.getSize();
  printf("max size: %f, %f\n", max_cam_view.getSize().x, max_cam_view.getSize().y);
  sf::Vector2f min_cam_size = 0.05f * max_cam_size;
  float cam_zoom = 1.f;
  sf::Vector2f min_lookat(max_cam_size * 0.125f);
  sf::Vector2f max_lookat(max_cam_size * 0.875f);
  
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

      if (event.type == sf::Event::KeyReleased)
      {
        if (event.key.code == sf::Keyboard::F1)
        {
          for (uint i=0; i<map_names.size(); ++i)
          {
            if (map_names[i] != chosen_map)
              continue;
            if (i > 0)
              chosen_map = map_names[i - 1];
            else
              chosen_map = map_names[map_names.size() - 1];
            break;
          }
        }
        if (event.key.code == sf::Keyboard::F2)
        {
          for (uint i=0; i<map_names.size(); ++i)
          {
            if (map_names[i] != chosen_map)
              continue;
            if (i < (map_names.size() - 1))
              chosen_map = map_names[i + 1];
            else
              chosen_map = map_names[0];
            break;
          }
        }
      }

      static bool cam_mouse_down = false;
      static sf::Vector2f cam_mouse_down_origin;
      if (event.type == sf::Event::MouseButtonPressed)
      {
        cam_mouse_down = true;

        //printf("mousebtn pos: %d %d\n", event.mouseButton.x, event.mouseButton.y);
        //cam_mouse_down_origin = app_window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
        cam_mouse_down_origin = sf::Vector2f(event.mouseButton.x, event.mouseButton.y);
        // cam_mouse_down_origin.x = (int)projected.x;
        // cam_mouse_down_origin.y = (int)projected.y;
        //sf::Vector2f projected(event.mouseButton.x, event.mouseButton.y);
        //printf("prjected: %f %f\n", projected.x, projected.y);
      }

      if (cam_mouse_down && event.type == sf::Event::MouseMoved)
      {
        // sf::View camview = app_window.getView();
        // auto center = camview.getCenter();

        //sf::Vector2f projected = app_window.mapPixelToCoords(sf::Vector2i(event.mouseMove.x, event.mouseMove.y));
        sf::Vector2f projected(event.mouseMove.x, event.mouseMove.y);
        printf("prjected: %f %f\n", projected.x, projected.y);
        printf("cam_mouse_down_origin: %f %f\n", cam_mouse_down_origin.x, cam_mouse_down_origin.y);
        // camview.setCenter(projected);
        // app_window.setView(camview);

        sf::View camview = app_window.getView();
        auto center = camview.getCenter();
        auto diff = projected - cam_mouse_down_origin;

        if (diff.x != 0.f || diff.y != 0.f)
        {
          printf("%f %f\n", diff.x, diff.y);
          float min_move_speed = 0.75f;
          float max_move_speed = 3.0f;
          float speed = min_move_speed + cam_zoom * (max_move_speed - min_move_speed);
          auto new_center = center + diff * speed;

          if (new_center.x < min_lookat.x)
            new_center.x = min_lookat.x;
          if (new_center.x > max_lookat.x)
            new_center.x = max_lookat.x;
          if (new_center.y < min_lookat.y)
            new_center.y = min_lookat.y;
          if (new_center.y > max_lookat.y)
            new_center.y = max_lookat.y;

          camview.setCenter(new_center);
          app_window.setView(camview);
        }

        sf::Mouse::setPosition(sf::Vector2i(cam_mouse_down_origin.x, cam_mouse_down_origin.y), app_window);
      }

      if (event.type == sf::Event::MouseButtonReleased)
        cam_mouse_down = false;


      if (event.type == sf::Event::MouseWheelScrolled)
      {
        sf::View camview = app_window.getView();
        float dt = deltaClock.getElapsedTime().asSeconds();

        if (event.mouseWheelScroll.delta > 0) //scroll up
          cam_zoom -= dt * 100.0f;
        if (event.mouseWheelScroll.delta < 0) //scroll down
          cam_zoom += dt * 100.0f;

        if (cam_zoom < 0.f)
            cam_zoom = 0.f;
        if (cam_zoom > 1.f)
          cam_zoom = 1.f;
        
        auto newsize = min_cam_size + cam_zoom * (max_cam_size - min_cam_size);
        camview.setSize(newsize);
        
        app_window.setView(camview);
      }

      if (event.type == sf::Event::KeyPressed)
      {
        float move_x = 30.0f, move_y = 30.f;
        auto move_view = [&app_window](sf::Vector2f offset)
        {
          sf::View camview = app_window.getView();
          auto center = camview.getCenter();
          camview.setCenter(sf::Vector2f(center.x + offset.x, center.y + offset.y));
          app_window.setView(camview);
        };

        if (event.key.code == sf::Keyboard::W || event.key.code == sf::Keyboard::Up)
          move_view(sf::Vector2f(0.0f, -move_y));
        if (event.key.code == sf::Keyboard::S || event.key.code == sf::Keyboard::Down)
          move_view(sf::Vector2f(0.0f, move_y));
        if (event.key.code == sf::Keyboard::A || event.key.code == sf::Keyboard::Left)
          move_view(sf::Vector2f(-move_x, 0.0f));
        if (event.key.code == sf::Keyboard::D || event.key.code == sf::Keyboard::Right)
          move_view(sf::Vector2f(move_x, 0.0f));

        if (event.key.code == sf::Keyboard::Z)
        {
          sf::View camview = app_window.getView();
          camview.setSize(max_cam_size);
          camview.setCenter(0.5f * max_cam_size);
          app_window.setView(camview);
          
          cam_zoom = 1.0f;
        }
      }
    }

    ImGui::SFML::Update(app_window, deltaClock.restart());


    bool force_replan = false;
    if (ImGui::BeginMainMenuBar())
    {
      if (ImGui::BeginMenu("UI"))
      {
        static int sz = 24;
        if (ImGui::InputInt("Text size", &sz))
        {
          if (sz > 0)
            graph_0_drawable.set_text_size((uint)sz);
        }
        ImGui::Separator();
        ImGui::NewLine();

        ImGui::Text("Maps (Use keys F1/F2 to iterate through)");

        for (uint i=0; i<map_names.size(); ++i)
        {
          bool activated = map_names[i] == chosen_map;
          if (ImGui::RadioButton(map_names[i].c_str(), activated))
            chosen_map = map_names[i];
        }

        ImGui::NewLine();

        ImGui::Text("Use WASD/mousedrag to move camera, Z to reset");

        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    graph_0_drawable.choose_map(chosen_map);
    
    static bool show_node_trajectories = true;
    static std::vector<rmf_planner_viz::draw::Trajectory> trajectories_to_render;

    bool startgoal_force_replan = rmf_planner_viz::draw::do_planner_presets(starts, goal, plan_start_timing);
    force_replan |= startgoal_force_replan;

    rmf_planner_viz::draw::do_planner_debug(
      profile, chosen_map,
      planner_0, starts, goal, graph_0.num_waypoints(), planner_debug, progress, plan_start_timing,
      force_replan, show_node_trajectories, trajectories_to_render);
    
    ImGui::EndFrame();

    /*** drawing ***/
    app_window.clear();

    schedule_drawable.timespan(std::chrono::steady_clock::now());

    sf::RenderStates states;
    fit.apply_transform(states.transform, app_window.getSize());
    app_window.draw(graph_0_drawable, states);
    app_window.draw(schedule_drawable, states);
    if (show_node_trajectories)
    {
      for (const auto& trajectory : trajectories_to_render)
        app_window.draw(trajectory, states);
    }

    rmf_planner_viz::draw::IMDraw::draw_aabb(min_lookat, max_lookat, sf::Color::Red);
    sf::Transform ident;
    rmf_planner_viz::draw::IMDraw::flush_and_render(app_window, ident);
    

    ImGui::SFML::Render(app_window);
    app_window.display();
  }
}
