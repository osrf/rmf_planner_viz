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


#include <SFML/Graphics.hpp>

#include <rmf_planner_viz/draw/Schedule.hpp>

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/DetectConflict.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <fstream>

class SerializedWaypoint
{
public:
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  rmf_traffic::Time time;
};

SerializedWaypoint read_waypoint(const std::string& file_name)
{
  std::ifstream file;
  file.open(file_name, std::ios::in);
  SerializedWaypoint serialized_waypoint;
  file.seekg(0);
  file.read((char*)&serialized_waypoint, sizeof(SerializedWaypoint));
  file.close();
  return serialized_waypoint;
}

int main()
{
  const auto database = std::make_shared<rmf_traffic::schedule::Database>();

  const rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(0.2)
  };

  const std::string test_map_name = "test_map";

  auto p0 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_0",
          "test_trajectory",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto p1 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_1",
          "test_trajectory",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  SerializedWaypoint serialized_waypoint[4];
  serialized_waypoint[0] =
    read_waypoint(RESOURCES_DIR + std::string("Waypoint1.txt"));
  serialized_waypoint[1] =
    read_waypoint(RESOURCES_DIR + std::string("Waypoint2.txt"));
  serialized_waypoint[2] =
    read_waypoint(RESOURCES_DIR + std::string("Waypoint3.txt"));
  serialized_waypoint[3] =
    read_waypoint(RESOURCES_DIR + std::string("Waypoint4.txt"));

  rmf_traffic::Trajectory traj0;
  traj0.insert(
    serialized_waypoint[0].time,
    serialized_waypoint[0].position,
    serialized_waypoint[0].velocity);
  traj0.insert(
    serialized_waypoint[1].time,
    serialized_waypoint[1].position,
    serialized_waypoint[1].velocity);

  rmf_traffic::Trajectory traj1;
  traj1.insert(
    serialized_waypoint[2].time,
    serialized_waypoint[2].position,
    serialized_waypoint[2].velocity);
  traj1.insert(
    serialized_waypoint[3].time,
    serialized_waypoint[3].position,
    serialized_waypoint[3].velocity);

  {
    // Comment/Uncomment this block to see how changing the rotation affects the
    // collision detection
    for (auto& traj : {&traj0, &traj1})
    {
      for (auto& wp : *traj)
      {
        const Eigen::Vector3d p = wp.position();
        wp.position({p.x(), p.y(), 0.0});
        const Eigen::Vector3d v = wp.velocity();
        wp.velocity({v.x(), v.y(), 0.0});
      }
    }
  }

//  {
//    // Comment/Uncomment this block to see how changing the precision affects
//    // the collision detection
//    for (auto& traj : {&traj0, &traj1})
//    {
//      for (auto& wp : *traj)
//      {
//        const Eigen::Vector3f p = wp.position().cast<float>();
//        wp.position({p.x(), p.y(), p.z()});
//      }
//    }
//  }

  const auto collision_detected =
    rmf_traffic::DetectConflict::between(
      p0.description().profile(),
      traj0,
      p1.description().profile(),
      traj1);

  if (collision_detected)
    std::cout << "Collision detected!" << std::endl;
  else
    std::cout << "Collision NOT detected!" << std::endl;

  p0.set({{test_map_name, traj0}});
  p1.set({{test_map_name, traj1}});

  using namespace std::chrono_literals;
  const auto initial_time = std::max(*traj0.start_time(), *traj1.start_time());
  const auto final_time = std::min(*traj0.finish_time(), *traj1.finish_time());

  rmf_planner_viz::draw::Schedule schedule_drawable(
        database, 0.25, test_map_name, initial_time, final_time-initial_time);

  sf::RenderWindow app_window(
        sf::VideoMode(1250, 1028),
        "Test Trajectory",
        sf::Style::Default);

  app_window.resetGLStates();

  const auto bounds = schedule_drawable.bounds();

  rmf_planner_viz::draw::Fit fit({bounds}, 0.02);

  const auto initial_clock_time = std::chrono::steady_clock::now();
  sf::Clock deltaClock;
  while (app_window.isOpen())
  {
    sf::Event event;
    while (app_window.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
      {
        return 0;
      }

      if (event.type == sf::Event::Resized)
      {
        sf::FloatRect visibleArea(0, 0, event.size.width, event.size.height);
        app_window.setView(sf::View(visibleArea));
      }
    }

    app_window.clear();

    const auto elapsed_time =
      std::chrono::steady_clock::now() - initial_clock_time;
    const auto current_time = initial_time + elapsed_time;
    schedule_drawable.timespan(current_time, final_time - current_time);

    sf::RenderStates states;
    fit.apply_transform(states.transform, app_window.getSize());
    app_window.draw(schedule_drawable, states);


    app_window.display();
  }
}
