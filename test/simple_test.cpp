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

  rmf_planner_viz::draw::Fit fit(
    {graph_0_drawable.bounds(), graph_1_drawable.bounds()}, 0.02);

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
    button->SetLabel(" --------- I was clicked --------- ");
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

    const auto& rect = window->GetAllocation();
    fit.left_border(rect.left + rect.width);

    window->Update(0.f);
    app_window.clear();

    sf::RenderStates states;
    fit.apply_transform(states.transform, app_window.getSize());
    app_window.draw(graph_0_drawable, states);
    app_window.draw(graph_1_drawable, states);

    sfgui.Display(app_window);
    app_window.display();
  }
}
