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

#include <rmf_planner_viz/draw/IMDraw.hpp>
#include <rmf_traffic/Motion.hpp>

namespace rmf_planner_viz {
namespace draw {

static std::vector<sf::VertexArray> g_vertexarrays; //internal use only

void IMDraw::draw_circle(const sf::Vector2f& center, double radius, const sf::Color& color, uint slices)
{
  sf::VertexArray arr(sf::Lines);

  double rotation_per_slice = (2.0 * M_PI) / (double)slices;
  for (uint i=0; i<slices; ++i)
  {
    sf::Vertex v;
    v.color = color;

    double rot = rotation_per_slice * (double)i;
    v.position = center + sf::Vector2f(radius * cos(rot), radius * sin(rot));
    arr.append(v);

    rot = rotation_per_slice * (double)(i + 1);
    v.position = center + sf::Vector2f(radius * cos(rot), radius * sin(rot));
    arr.append(v);
  }
  g_vertexarrays.push_back(arr);
}


void IMDraw::draw_axis()
{
  // x-axis + arrowhead
  {
    sf::VertexArray arr(sf::Lines);
    sf::Vertex v;
    v.color = sf::Color(255, 0, 0, 255);

    v.position = sf::Vector2f(0, 0);
    arr.append(v);

    v.position = sf::Vector2f(1, 0);
    arr.append(v);

    float cap_x = 0.85f;
    v.position = sf::Vector2f(1, 0);
    arr.append(v);
    v.position = sf::Vector2f(cap_x, 0.15f);
    arr.append(v);

    v.position = sf::Vector2f(1, 0);
    arr.append(v);
    v.position = sf::Vector2f(cap_x, -0.15f);
    arr.append(v);

    g_vertexarrays.push_back(arr);
  }
  // y-axis + arrowhead
  {
    sf::VertexArray arr(sf::Lines);
    sf::Vertex v;
    v.color = sf::Color(0, 255, 0, 255);

    v.position = sf::Vector2f(0, 0);
    arr.append(v);

    v.position = sf::Vector2f(0, 1);
    arr.append(v);

    float cap_y = 0.85f;
    v.position = sf::Vector2f(0, 1);
    arr.append(v);
    v.position = sf::Vector2f(0.15f, cap_y);
    arr.append(v);

    v.position = sf::Vector2f(0, 1);
    arr.append(v);
    v.position = sf::Vector2f(-0.15f, cap_y);
    arr.append(v);
    g_vertexarrays.push_back(arr);
  }
}

void IMDraw::draw_trajectory(const rmf_traffic::Trajectory& trajectory, const sf::Color& color)
{
  //lifted from Trajectory::Implementation::accurate_curve_drawing

  const auto motion = rmf_traffic::Motion::compute_cubic_splines(trajectory);

  const auto step = std::chrono::milliseconds(100);
  // const auto end = duration ?
  //       start + duration.value() : motion->finish_time();
  const auto end = motion->finish_time();
  const auto begin = motion->start_time();

  /*if (motion->start_time() <= start)
  {
    const Eigen::Vector3d p = motion->compute_position(start);
    configure_arrow(p, profile.footprint()->get_characteristic_length(), color);
    configure_footprint(p, profile.footprint()->get_characteristic_length());
    configure_vicinity(p, profile.vicinity()->get_characteristic_length());
  }*/
  sf::VertexArray vtx_arr(sf::Lines);

  for (auto time=begin; time <= end; time += step)
  {
    const auto next_time = std::min(time + step, end);

    const Eigen::Vector2d p =
        motion->compute_position(time).block<2,1>(0, 0);
    const Eigen::Vector2d pn =
        motion->compute_position(next_time).block<2,1>(0, 0);

    sf::Vertex v;
    v.color = color;

    v.position = sf::Vector2f(p.x(), p.y());
    vtx_arr.append(v);
    
    v.position = sf::Vector2f(pn.x(), pn.y());
    vtx_arr.append(v);
  }

  g_vertexarrays.push_back(vtx_arr);
}

void IMDraw::flush_and_render(sf::RenderWindow& app_window, const sf::Transform& tx_flipped_2d)
{
  for (auto& single_array : g_vertexarrays)
    app_window.draw(single_array, tx_flipped_2d);
  g_vertexarrays.clear();
}

} // namespace draw
} // namespace rmf_planner_viz
