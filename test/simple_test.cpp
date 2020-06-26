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

#include <rmf_planner_viz/draw/Capsule.hpp>

int main()
{
  sf::RenderWindow app_window(
        sf::VideoMode(800, 600),
        "Simple Test",
        sf::Style::Titlebar | sf::Style::Close);

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
    }

    window->Update(0.f);
    app_window.clear();


//    std::vector<sf::Vertex> lines =
//    {
//      {sf::Vector2f(10.f, 10.f), sf::Color::Green},
//      {sf::Vector2f(150.f, 150.f), sf::Color::Yellow},
//      sf::Vector2f(150.f, 200.f),
//      sf::Vector2f(200.f, 200.f)
//    };
//    app_window.draw(lines.data(), lines.size(), sf::Lines);

    sf::VertexArray rectangle(sf::Triangles, 6);
    // define the position of the triangle's points
    rectangle[0].position = sf::Vector2f(10.f, 10.f);
    rectangle[1].position = sf::Vector2f(500.f, 10.f);
    rectangle[2].position = sf::Vector2f(500.f, 100.f);

    // define the color of the triangle's points
    rectangle[0].color = sf::Color::Yellow;
    rectangle[1].color = sf::Color::Green;
    rectangle[2].color = sf::Color::Green;

    // define the position of the triangle's points
    rectangle[3].position = sf::Vector2f(10.f, 10.f);
    rectangle[4].position = sf::Vector2f(10.f, 100.f);
    rectangle[5].position = sf::Vector2f(500.f, 100.f);

    // define the color of the triangle's points
    rectangle[3].color = sf::Color::Yellow;
    rectangle[4].color = sf::Color::Yellow;
    rectangle[5].color = sf::Color::Green;

    app_window.draw(rectangle);


    app_window.draw(
          rmf_planner_viz::draw::Capsule(
            {sf::Vector2f(50.0, 300.0), sf::Color::Yellow},
            {sf::Vector2f(500.0, 500.0), sf::Color::Green},
            20.0));

//    sfgui.Display(app_window);
    app_window.display();
  }
}
