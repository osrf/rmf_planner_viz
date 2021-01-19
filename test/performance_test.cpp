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

#include <yaml-cpp/yaml.h>

const std::size_t NotObstacleID = std::numeric_limits<std::size_t>::max();

struct Plan
{
  std::string robot;
  std::size_t initial_time;
  double initial_orientation;
  std::string initial_waypoint, goal;
};

struct Scenario
{
  std::string map_file;
  std::size_t samples;

  std::unordered_map<std::string, rmf_traffic::agv::VehicleTraits> robots;
  std::vector<Plan> obstacles;

  Plan plan;
};

void parse_scenario(std::string scenario_file, Scenario& scenario)
{
  if (scenario_file.rfind(".yaml") == std::string::npos)
  {
    scenario_file.append(".yaml");
  }

  YAML::Node scenario_config;

  try
  {
    scenario_config = YAML::LoadFile(std::string(
        TEST_SCENARIO_DIR) + scenario_file);
  }
  catch (YAML::BadFile& e)
  {
    throw std::runtime_error(
        "Failed to load scenario file [" + std::string(
            TEST_SCENARIO_DIR) + scenario_file + "]");
  }

  if (scenario_config["map"])
  {
    scenario.map_file = scenario_config["map"].as<std::string>();
  }
  else
  {
    throw std::runtime_error("Scenario file is missing the [map] key");
  }

  if (scenario_config["samples"])
  {
    scenario.samples = scenario_config["samples"].as<std::size_t>(100);
  }
  else
  {
    std::cout <<
              "Scenario file is missing the [samples] key. Using default value [100]" <<
              std::endl;
  }

  const YAML::Node robots = scenario_config["robots"];

  for (auto iter = robots.begin(); iter != robots.end(); ++iter)
  {
    const auto& name = iter->first.as<std::string>();
    const auto& robot = iter->second;

    double linear_velocity, linear_acceleration, angular_velocity,
        angular_acceleration;

    const auto& traits = robot["traits"];
    if (traits)
    {
      const auto& linear = traits["linear"];
      const auto& angular = traits["angular"];

      if (linear)
      {
        const auto& velocity = linear["velocity"];
        const auto& acceleration = linear["acceleration"];

        if (velocity)
        {
          linear_velocity = velocity.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
                    "] is missing key [traits[linear[velocity]]]. Skipping entry" <<
                    std::endl;
          continue;
        }

        if (acceleration)
        {
          linear_acceleration = acceleration.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
                    "] is missing key [traits[linear[accleration]]]. Skipping entry" <<
                    std::endl;
          continue;
        }
      }
      else
      {
        std::cout << "Robot [" << name <<
                  "] is missing key [traits[linear]]. Skipping entry" << std::endl;
      }

      if (angular)
      {
        const auto& velocity = angular["velocity"];
        const auto& acceleration = angular["acceleration"];

        if (velocity)
        {
          angular_velocity = velocity.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
                    "] is missing key [traits[angular[velocity]]]. Skipping entry" <<
                    std::endl;
          continue;
        }

        if (acceleration)
        {
          angular_acceleration = acceleration.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
                    "] is missing key [traits[angular[accleration]]]. Skipping entry" <<
                    std::endl;
          continue;
        }
      }
      else
      {
        std::cout << "Robot [" << name <<
                  "] is missing key [traits[angular]]. Skipping entry" << std::endl;
      }
    }
    else
    {
      std::cout << "Robot [" << name <<
                "] is missing key [traits]. Skipping entry" << std::endl;
      continue;
    }

    const auto& profile = robot["profile"];
    if (profile)
    {
      const auto& footprint = profile["footprint"];
      if (footprint)
      {
        const auto& shape = footprint["shape"];
        if (shape)
        {
          if (strcasecmp("circle", shape.as<std::string>().c_str()) == 0)
          {
            const auto& radius = footprint["radius"];
            if (radius)
            {
              scenario.robots.insert({
                                         name,
                                         rmf_traffic::agv::VehicleTraits {
                                             {linear_velocity, linear_acceleration},
                                             {angular_velocity, angular_acceleration},
                                             rmf_traffic::Profile{
                                                 rmf_traffic::geometry::make_final_convex(
                                                     rmf_traffic::geometry::Circle(radius.as<double>()))
                                             }}});
            }
            else
            {
              std::cout << "Robot [" << name <<
                        "] is missing key [profile[footprint[radius]]]. Skipping entry"
                        << std::endl;
              continue;
            }
          }
          else
          {
            std::cout << "Robot [" << name <<
                      "] has unsupported shape value [" << shape.as<std::string>() <<
                      "]. Skipping entry" << std::endl;
            continue;
          }
        }
        else
        {
          std::cout << "Robot [" << name <<
                    "] is missing key [profile[footprint[shape]]]. Skipping entry" <<
                    std::endl;
          continue;
        }
      }
      else
      {
        std::cout << "Robot [" << name <<
                  "] is missing key [profile[footprint]]. Skipping entry" << std::endl;
        continue;
      }
    }
    else
    {
      std::cout << "Robot [" << name <<
                "] is missing key [profile]. Skipping entry" << std::endl;
      continue;
    }
  }

  const YAML::Node obstacles = scenario_config["obstacles"];
  for (const auto& obstacle : obstacles)
  {
    if (!obstacle["robot"])
    {
      std::cout << "Missing [robot] key. Skipping entry" << std::endl;
      continue;
    }

    const std::string& name = obstacle["robot"].as<std::string>();

    std::size_t initial_time;
    double initial_orientation;
    std::string initial_waypoint;

    const auto& start = obstacle["start"];
    if (start)
    {
      const auto& time = start["initial_time"];
      if (time)
      {
        initial_time = time.as<std::size_t>(0);
      }
      else
      {
        std::cout << "Robot [" << name <<
                  "] is missing key [start[initial_time]]. Using default value [0]." <<
                  std::endl;
      }
      const auto& waypoint = start["initial_waypoint"];
      if (waypoint)
      {
        initial_waypoint = waypoint.as<std::string>();
      }
      else
      {
        std::cout << "Robot [" << name <<
                  "] is missing key [start[initial_waypoint]]. Skipping entry." <<
                  std::endl;
        continue;
      }
      const auto& orientation = start["initial_orientation"];
      if (orientation)
      {
        initial_orientation = waypoint.as<double>(0);
      }
      else
      {
        std::cout << "Robot [" << name <<
                  "] is missing key [start[initial_orientation]]. Assuming initial_orientation 0."
                  << std::endl;
      }
    }
    else
    {
      std::cout << "Robot [" << name <<
                "] is missing key [start]. Skipping entry." << std::endl;
      continue;
    }

    const auto& goal = obstacle["goal"];
    if (goal)
    {
      scenario.obstacles.push_back({name, initial_time, initial_orientation,
                                    initial_waypoint, goal.as<std::string>()});
    }
    else
    {
      std::cout << "Robot [" << name <<
                "] is missing key [goal]. Skipping entry." << std::endl;
      continue;
    }
  }

  const YAML::Node plan = scenario_config["plan"];
  if (plan)
  {
    const auto& robot = plan["robot"];
    if (robot)
    {
      scenario.plan.robot = robot.as<std::string>();
    }
    else
    {
      throw std::runtime_error("Scenario file is missing the [plan[robot]] key");
    }
    const auto& start = plan["start"];
    if (start)
    {
      const auto& time = start["initial_time"];
      if (time)
      {
        scenario.plan.initial_time = time.as<std::size_t>(0);
      }
      else
      {
        std::cout <<
                  "Plan is missing key [start[initial_time]]. Using default value [0]."
                  <<
                  std::endl;
      }
      const auto& waypoint = start["initial_waypoint"];
      if (waypoint)
      {
        scenario.plan.initial_waypoint = waypoint.as<std::string>();
      }
      else
      {
        throw std::runtime_error(
            "Plan is missing [start[initial_waypoint]] key.");
      }
      const auto& orientation = start["initial_orientation"];
      if (orientation)
      {
        scenario.plan.initial_orientation = waypoint.as<double>(0);
      }
      else
      {
        std::cout <<
                  "Plan is missing key [start[initial_orientation]]. Assuming initial_orientation 0."
                  << std::endl;
      }
    }
    else
    {
      throw std::runtime_error("Plan is missing [start] key.");
    }

    const auto& goal = plan["goal"];
    if (goal)
    {
      scenario.plan.goal = goal.as<std::string>();
    }
    else
    {
      throw std::runtime_error("Plan is missing [goal] key.");
    }
  }
  else
  {
    throw std::runtime_error("Scenario file is missing the [plan] key");
  }
}

rmf_traffic::schedule::Participant add_obstacle(
    const rmf_traffic::agv::Planner& planner,
    const std::shared_ptr<rmf_traffic::schedule::Database>& database,
    const rmf_traffic::agv::Plan::Start& start,
    const rmf_traffic::agv::Plan::Goal& goal)
{
  const auto N = database->participant_ids().size();

  auto new_obstacle = rmf_traffic::schedule::make_participant(
      rmf_traffic::schedule::ParticipantDescription{
          "obstacle_" + std::to_string(N),
          "obstacles",
          rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
          planner.get_configuration().vehicle_traits().profile()
      }, database);

  const auto plan = planner.plan(start, goal);
  new_obstacle.set(plan->get_itinerary());
  return new_obstacle;
}

void print_result(
    const std::string& label,
    const std::size_t samples,
    const double total_time,
    const std::size_t node_count)
{
  std::cout << label
            << "\n -- Total time for " << samples << " samples: "
            << total_time
            << "\n -- Average time per run: " << total_time/samples
            << "\n -- Node count: " << node_count
            << "\n" << std::endl;
}

double test_planner_timing_no_cache(
    const std::string& label,
    const std::size_t samples,
    const rmf_traffic::agv::Planner::Configuration& config,
    const rmf_traffic::agv::Planner::Options& options,
    const rmf_traffic::agv::Plan::Start& start,
    const rmf_traffic::agv::Plan::Goal& goal)
{
  // Run a test where we produce a new planner every time so we see what the
  // timing is if the cache is blank
  const auto begin_time = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < samples; ++i)
  {
    rmf_traffic::agv::Planner planner(config, options);
    planner.plan(start, goal);
  }
  const auto finish_time = std::chrono::steady_clock::now();

  const double total_time = rmf_traffic::time::to_seconds(
      finish_time - begin_time);

  const auto nodes = rmf_traffic::agv::Planner::Debug::node_count(
      rmf_traffic::agv::Planner(config, options).plan(start, goal));

  print_result(label + " | No Cache", samples, total_time, nodes);

  return total_time;
}

double test_planner_timing_with_cache(
    const std::string& label,
    const std::size_t samples,
    const rmf_traffic::agv::Planner::Configuration& config,
    const rmf_traffic::agv::Planner::Options& options,
    const rmf_traffic::agv::Plan::Start& start,
    const rmf_traffic::agv::Plan::Goal& goal)
{
  // Run a test where we prime the planner by solving it once. Future runs will
  // not need to recompute the heuristic.
  rmf_traffic::agv::Planner planner(config, options);
  planner.plan(start, goal);

  const auto begin_time = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < samples; ++i)
  {
    planner.plan(start, goal);
  }
  const auto finish_time = std::chrono::steady_clock::now();

  const double total_time = rmf_traffic::time::to_seconds(
      finish_time - begin_time);

  const auto nodes = rmf_traffic::agv::Planner::Debug::node_count(
      rmf_traffic::agv::Planner(config, options).plan(start, goal));

  print_result(label + " | With Cache", samples, total_time, nodes);

  return total_time;
}

void test_planner_timing(
    const std::string& label,
    const std::size_t samples,
    const rmf_traffic::agv::Planner::Configuration& config,
    const rmf_traffic::agv::Planner::Options& options,
    const rmf_traffic::agv::Plan::Start& start,
    const rmf_traffic::agv::Plan::Goal& goal)
{
  std::cout << " --------- \n" << std::endl;

  // For each variation of test, we run many samples and then see what the
  // average time is.
  const double no_cache_time = test_planner_timing_no_cache(
      label, samples, config, options, start, goal);

  const double with_cache_time = test_planner_timing_with_cache(
      label, samples, config, options, start, goal);

  std::cout << "Cache speed boost: x" << no_cache_time/with_cache_time
            << "\n" << std::endl;
}

void test_planner(
    const std::string& label,
    const std::size_t samples,
    const rmf_traffic::agv::Graph& graph,
    const rmf_traffic::agv::VehicleTraits& traits,
    const std::shared_ptr<rmf_traffic::schedule::Database>& database,
    const rmf_traffic::agv::Plan::Start& start,
    const rmf_traffic::agv::Plan::Goal& goal)
{
  test_planner_timing(
      label + " | No Obstacles",
      samples,
      {graph, traits},
      {nullptr},
      start, goal
  );

  const auto obstacle_validator =
      rmf_traffic::agv::ScheduleRouteValidator::make(
          database, NotObstacleID, traits.profile());

  test_planner_timing(
      label + " | With Obstacles",
      samples,
      {graph, traits},
      {obstacle_validator},
      start, goal
  );
}


int main(int argc, char* argv[])
{
  sf::Font font;
  if (!font.loadFromFile("./build/rmf_planner_viz/fonts/OpenSans-Bold.ttf"))
  {
    std::cout << "Failed to load font. Make sure you run the executable from the colcon directory" << std::endl;
    return -1;
  }

  if (argc < 2)
  {
    std::cout << "Please provide scenario file name" << std::endl;
    return 0;
  }

  Scenario scenario;
  try
  {
    parse_scenario(argv[1], scenario);
  }
  catch (std::runtime_error e)
  {
    std::cout << e.what() << std::endl;
    return 0;
  }

  using namespace std::chrono_literals;

  const std::string map_file = std::string(TEST_MAP_DIR) + scenario.map_file;
  std::cout << "Loading [" << map_file << "]" << std::endl;

  const auto& plan_robot = scenario.robots.find(scenario.plan.robot);
  if (plan_robot == scenario.robots.end())
  {
    std::cout << "Plan robot [" << scenario.plan.robot <<
              "]'s traits and profile missing" << std::endl;
    return 0;
  }

  rmf_traffic::agv::Graph graph_0;
  try
  {
    graph_0 = rmf_fleet_adapter::agv::parse_graph(map_file, plan_robot->second);
  }
  catch (YAML::BadFile& e)
  {
    std::cout << "Failed to load map file [" << map_file << "]" << std::endl;
    return 0;
  }

  const auto get_wp = [&](const std::string& name)
  {
    return graph_0.find_waypoint(name)->index();
  };

  const auto start_time = std::chrono::steady_clock::now();

  // We'll make some "obstacles" in the environment by planning routes between
  // various waypoints.

  const auto database = std::make_shared<rmf_traffic::schedule::Database>();
  std::vector<rmf_traffic::schedule::Participant> obstacles;

  for (const auto& obstacle : scenario.obstacles)
  {
    const auto& robot = scenario.robots.find(obstacle.robot);

    rmf_traffic::agv::Planner planner = rmf_traffic::agv::Planner{
        {graph_0, plan_robot->second},
        {nullptr}
    };
    if (robot == scenario.robots.end())
    {
      std::cout << "Robot [" << obstacle.robot <<
                "] is missing traits and profile. Using traits and profile of plan_robot."
                << std::endl;
    }
    else
    {
      planner = rmf_traffic::agv::Planner{
          {graph_0, robot->second},
          {nullptr}
      };
    }

    obstacles.emplace_back(
        add_obstacle(
            planner, database,
            {
                start_time + std::chrono::seconds(obstacle.initial_time),
                get_wp(obstacle.initial_waypoint),
                obstacle.initial_orientation * M_PI / 180.0
            },
            get_wp(obstacle.goal)
        )
    );
  }

  const auto& plan = scenario.plan;

  rmf_planner_viz::draw::Graph graph_0_drawable(graph_0, 1.0, font);
  std::vector<std::string> map_names = graph_0_drawable.get_map_names();
  std::string chosen_map;
  if (graph_0_drawable.current_map())
    chosen_map = *graph_0_drawable.current_map();

  const auto obstacle_validator =
      rmf_traffic::agv::ScheduleRouteValidator::make(
          database, NotObstacleID, plan_robot->second.profile());

  rmf_traffic::agv::Planner planner_0(
        rmf_traffic::agv::Planner::Configuration(graph_0, plan_robot->second),
        rmf_traffic::agv::Planner::Options(obstacle_validator));

  /// Setup participants

  // set plans for the participants
  using namespace std::chrono_literals;

  std::vector<rmf_traffic::agv::Planner::Start> starts;
  starts.emplace_back(start_time + std::chrono::seconds(plan.initial_time), get_wp(plan.initial_waypoint), plan.initial_orientation);

  rmf_traffic::agv::Planner::Goal goal(get_wp(plan.goal));
  rmf_traffic::agv::Planner::Debug planner_debug(planner_0);
  rmf_traffic::agv::Planner::Debug::Progress progress =
    planner_debug.begin(starts, goal, planner_0.get_default_options());

  rmf_planner_viz::draw::Schedule schedule_drawable(
        database, 0.25, scenario.map_file, start_time + std::chrono::seconds(plan.initial_time));

  rmf_planner_viz::draw::Fit fit(
    {graph_0_drawable.bounds()}, 0.02);

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
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    graph_0_drawable.choose_map(chosen_map);
    
    static bool show_node_trajectories = true;
    static std::vector<rmf_planner_viz::draw::Trajectory> trajectories_to_render;

    bool startgoal_force_replan = rmf_planner_viz::draw::do_planner_presets(starts, goal, start_time);
    force_replan |= startgoal_force_replan;

    rmf_planner_viz::draw::do_planner_debug(
        plan_robot->second.profile(), chosen_map,
      planner_0, starts, goal, graph_0.num_waypoints(), planner_debug, progress, start_time,
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

    rmf_planner_viz::draw::IMDraw::flush_and_render(app_window, states.transform);
    

    ImGui::SFML::Render(app_window);
    app_window.display();
  }
}
