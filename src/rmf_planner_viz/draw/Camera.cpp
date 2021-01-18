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

#include <rmf_planner_viz/draw/Camera.hpp>
#include <rmf_planner_viz/draw/IMDraw.hpp>
#include <SFML/Graphics/View.hpp>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
Camera::Camera(const sf::Vector2f& max_cam_size)
  :_max_cam_size(max_cam_size), _min_cam_size(0.05f * max_cam_size),
  _min_lookat(max_cam_size * 0.125f), _max_lookat(max_cam_size * 0.875f)
{
}

void Camera::on_mouse_button_pressed(int mouseX, int mouseY)
{
  _mouse_btn_down = true;
  _mouse_down_origin = sf::Vector2f(mouseX, mouseY);
}

void Camera::on_mouse_button_released()
{
  _mouse_btn_down = false;
}

void Camera::on_mouse_wheel_scrolled(const sf::Event::MouseWheelScrollEvent& evt)
{
  if (evt.delta > 0) //scroll up
    _scrolled_up = true;
  if (evt.delta < 0) //scroll down
    _scrolled_down = true;
}

void Camera::update(float dt, sf::RenderWindow& app_window)
{
  sf::View camview = app_window.getView();
  sf::Vector2f center = camview.getCenter();

  // keyboard input
  float move_x = 500.0f * dt, move_y = 500.f * dt;
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::W) || sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
    center += sf::Vector2f(0.0f, -move_y);
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::S) || sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
    center += sf::Vector2f(0.0f, move_y);
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::A) || sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
    center += sf::Vector2f(-move_x, 0.0f);
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::D) || sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
    center += sf::Vector2f(move_x, 0.0f);

  if (sf::Keyboard::isKeyPressed(sf::Keyboard::Z))
  {
    center = 0.5f * _max_cam_size;
    _zoom = 1.0f;
  }

  // mouse drag lookat
  if (_mouse_btn_down)
  {
    sf::Vector2i mouse_pos = sf::Mouse::getPosition(app_window);
    auto diff = sf::Vector2f((float)mouse_pos.x, (float)mouse_pos.y) - _mouse_down_origin;
    center = center + diff;
    
    //printf("diff: %f %f\n", diff.x, diff.y);

    sf::Mouse::setPosition(sf::Vector2i(_mouse_down_origin.x, _mouse_down_origin.y), app_window);
  }

  // clamp center/lookat
  if (center.x < _min_lookat.x)
    center.x = _min_lookat.x;
  if (center.x > _max_lookat.x)
    center.x = _max_lookat.x;
  if (center.y < _min_lookat.y)
    center.y = _min_lookat.y;
  if (center.y > _max_lookat.y)
    center.y = _max_lookat.y;
  
  // handle zoom
  float scrollspeed = 10.f;
  if (_scrolled_up)
    _zoom -= dt * scrollspeed;
  else if (_scrolled_down)
    _zoom += dt * scrollspeed;
  _scrolled_up = _scrolled_down = false;

  // clamp zoom
  if (_zoom < 0.f)
    _zoom = 0.f;
  if (_zoom > 1.f)
    _zoom = 1.f;
  
  auto newsize = _min_cam_size + _zoom * (_max_cam_size - _min_cam_size);
  camview.setSize(newsize);
  camview.setCenter(center);

  app_window.setView(camview);

  IMDraw::draw_aabb(_min_lookat, _max_lookat, sf::Color::Red);
}

} // namespace draw
} // namespace rmf_planner_viz
