/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef RMF_PLANNER_VIZ__DRAW__CAPSULE_HPP
#define RMF_PLANNER_VIZ__DRAW__CAPSULE_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <SFML/Graphics/Drawable.hpp>
#include <SFML/Graphics/Vertex.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Window/Event.hpp>

namespace rmf_planner_viz {
namespace draw {

//==============================================================================
class Camera
{
public:
  Camera(const sf::Vector2f& max_cam_size);

  void  set_camera_zoom(float z) { _zoom = z; }
  float get_camera_zoom() const { return _zoom; }

  void on_mouse_button_pressed(int mouseX, int mouseY);
  void on_mouse_button_released();
  void on_mouse_wheel_scrolled(const sf::Event::MouseWheelScrollEvent& evt);

  void update(float dt, sf::RenderWindow& app_window);
  
private:
  sf::Vector2f _max_cam_size;
  sf::Vector2f _min_cam_size;
  float _zoom = 1.f;

  sf::Vector2f _min_lookat;
  sf::Vector2f _max_lookat;

  bool _mouse_btn_down = false;
  sf::Vector2f _mouse_down_origin;

  bool _scrolled_up = false;
  bool _scrolled_down = false;
};

} // namespace draw
} // namespace capsule

#endif // RMF_PLANNER_VIZ__DRAW__CAPSULE_HPP
