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
#include <SFGUI/SFGUI.hpp>
#include <SFGUI/Widgets.hpp>

#include <iostream>

#include <rmf_planner_viz/draw/Graph.hpp>

int main()
{
  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0});  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true);  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
//  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {2.5, 2.5}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10
  graph.add_waypoint(test_map_name, {5.0, 10.0}); // 11

  /*
   *                  10------11
   *                   |      |
   *                   |      |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6------7
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
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
  graph.add_lane(10, 11);
  graph.add_lane(11, 9);

  rmf_planner_viz::draw::Graph graph_drawable(graph, 1.0);


  sf::RenderWindow app_window(
        sf::VideoMode(1250, 1028),
        "Simple Test",
        sf::Style::Default);

  app_window.resetGLStates();

  sfg::SFGUI sfgui;
  auto window = sfg::Window::Create();

  auto box = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);

  auto button = sfg::Button::Create("Click me");
  box->Pack(button);

  button->GetSignal(sfg::Widget::OnLeftClick).Connect(
        [&button]()
  {
    button->SetLabel("I was clicked");
  });

  window->Add(box);

  while (app_window.isOpen())
  {
    sf::Event event;
    while (app_window.pollEvent(event))
    {
      window->HandleEvent(event);

      if (event.type == sf::Event::Closed)
      {
        return 0;
      }

      if (event.type == sf::Event::Resized)
      {
        sf::FloatRect visibleArea(0, 0, event.size.width, event.size.height);
        app_window.setView(sf::View(visibleArea));
      }

      if (event.type == sf::Event::MouseMoved)
      {
        const auto pick = graph_drawable.pick(
              event.mouseMove.x,
              event.mouseMove.y,
              app_window.getSize());
      }

      if (event.type == sf::Event::MouseButtonPressed)
      {
        const auto pick = graph_drawable.pick(
              event.mouseButton.x,
              event.mouseButton.y,
              app_window.getSize());

        if (pick)
          graph_drawable.select(*pick);
      }
    }

    window->Update(0.f);
    app_window.clear();

    app_window.draw(graph_drawable);

    app_window.display();
  }
}
