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

    sf::Vertex line[] =
    {
      sf::Vector2f(10.f, 10.f),
      sf::Vector2f(150.f, 150.f)
    };

    window->Update(0.f);
    app_window.clear();

    app_window.draw(line, 2, sf::Lines);
    sfgui.Display(app_window);
    app_window.display();
  }
}
