
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

#include "test_sidecar_utils.hpp"

#include <SFML/Graphics.hpp>
#include <imgui.h>

#include <fcl/math/motion/interp_motion.h>

#include "imgui-SFML.h"
#include "spline_offset_utils.hpp"
#include <fcl/geometry/shape/sphere.h>
#include <fcl/narrowphase/detail/primitive_shape_algorithm/sphere_sphere.h>
#include <fcl/narrowphase/detail/primitive_shape_algorithm/sphere_box.h>
#include <fcl/narrowphase/detail/primitive_shape_algorithm/box_box.h>

#define DO_LOGGING 1

namespace rmf_planner_viz {
namespace draw {

Eigen::Vector3d closestpt_on_lineseg_to_pt(
  const Eigen::Vector3d& ls_start, const Eigen::Vector3d& ls_end,
  const Eigen::Vector3d& p, double& dist)
{
  auto ls_vec = ls_end - ls_start;
  double len = ls_vec.norm();
  auto ls_vec_norm = ls_vec / len;
  auto ls_start_to_p = p - ls_start;
  
  double l = ls_vec_norm.dot(ls_start_to_p);
  if (l < 0.0)
    l = 0.0;
  if (l > len)
    l = len;
  
  auto closest_pt = ls_start + l * ls_vec_norm;
  //rmf_planner_viz::draw::IMDraw::draw_circle(sf::Vector2f(closest_pt.x(), closest_pt.y()), 0.115, sf::Color(255, 255, 0, 255));
  dist = (closest_pt - p).norm();
  return closest_pt;
}

double box_box_closest_pts(
      const fcl::Boxd& box_a,
      const fcl::Transform3d& tx_a,
      const fcl::Boxd& box_b,
      const fcl::Transform3d& tx_b,
      Eigen::Vector3d& a, Eigen::Vector3d& b)
{
  double halfsize_x_a = box_a.side.x() * 0.5;
  double halfsize_y_a = box_a.side.y() * 0.5;
  double halfsize_x_b = box_b.side.x() * 0.5;
  double halfsize_y_b = box_b.side.y() * 0.5;

  auto linear_a = tx_a.linear();
  auto col0_a = linear_a.col(0);
  auto col1_a = linear_a.col(1);

  auto linear_b = tx_b.linear();
  auto col0_b = linear_b.col(0);
  auto col1_b = linear_b.col(1);

  auto center_a = tx_a.translation();
  auto center_b = tx_b.translation();

  // form points of box a and b
  Eigen::Vector3d pts_a[4];
  pts_a[0] = center_a + col0_a * halfsize_x_a + col1_a * halfsize_y_a;
  pts_a[1] = center_a - col0_a * halfsize_x_a + col1_a * halfsize_y_a;
  pts_a[2] = center_a - col0_a * halfsize_x_a - col1_a * halfsize_y_a;
  pts_a[3] = center_a + col0_a * halfsize_x_a - col1_a * halfsize_y_a;

  Eigen::Vector3d pts_b[4];
  pts_b[0] = center_b + col0_b * halfsize_x_b + col1_b * halfsize_y_b;
  pts_b[1] = center_b - col0_b * halfsize_x_b + col1_b * halfsize_y_b;
  pts_b[2] = center_b - col0_b * halfsize_x_b - col1_b * halfsize_y_b;
  pts_b[3] = center_b + col0_b * halfsize_x_b - col1_b * halfsize_y_b;

  auto closest_box_pt_along_projection_outside_box = [](
    Eigen::Vector3d (&pts)[4], const Eigen::Vector3d& center, double halfside, 
    const Eigen::Vector3d& proj,
    Eigen::Vector3d& closest, bool& pts_on_positive_side)
      -> bool
  {
    double dotp[4] = { 0.0, 0.0, 0.0, 0.0 };
    dotp[0] = proj.dot(pts[0] - center);
    dotp[1] = proj.dot(pts[1] - center);
    dotp[2] = proj.dot(pts[2] - center);
    dotp[3] = proj.dot(pts[3] - center);

    uint negative_points = 0, positive_points = 0;
    int negative_point_gap_idx = -1, positive_point_gap_idx = -1;
    double negative_point_gap = DBL_MAX, positive_point_gap = DBL_MAX;
    for (int i=0; i<4; ++i)
    {
      //printf ("dotp %d: %f\n", i, dotp[i]);
      if (dotp[i] > halfside)
      {
        ++positive_points;
        double gap = dotp[i] - halfside;
        if (gap < positive_point_gap)
        {
          positive_point_gap_idx = i;
          positive_point_gap = gap;
        }
      }
      if (dotp[i] < -halfside)
      {
        ++negative_points;
        double gap = -halfside - dotp[i];
        if (gap < negative_point_gap)
        {
          negative_point_gap_idx = i;
          negative_point_gap = gap;
        }
      }
    }

    //printf("+count: %d -count: %d\n", positive_points, negative_points);
    if (!(positive_points == 4 || negative_points == 4))
    {
      //printf("fail\n");
      return false;
    }

    uint chosen_id = 0;
    if (positive_points == 4) // all points on +ve col_a side
    {
      pts_on_positive_side = true;
      chosen_id = positive_point_gap_idx;
    }
    else if (negative_points == 4) // all points on -ve side
    {
      pts_on_positive_side = false;
      chosen_id = negative_point_gap_idx;
    }

    // check for tiebreak
    uint next_id = (chosen_id + 1) % 4;
    uint prev_id = chosen_id == 0 ? 3 : chosen_id - 1;
    bool tiebreak = false;
    Eigen::Vector3d other_pt;
    if (dotp[next_id] == dotp[chosen_id])
    {
      tiebreak = true;
      other_pt = pts[next_id];
    }
    if (dotp[prev_id] == dotp[chosen_id])
    {
      tiebreak = true;
      other_pt = pts[prev_id];
    }

    if (tiebreak)
    {
      Eigen::Vector3d projection_perp(-proj.y(), proj.x(), 0);
      double dotp_c0 = projection_perp.dot(pts[chosen_id] - center);
      double dotp_c1 = projection_perp.dot(other_pt - center);
      if (abs(dotp_c0) < abs(dotp_c1))
        closest = pts[chosen_id];
      else
        closest = other_pt;
    }
    else
    {
      printf ("no tiebreak!\n");
      closest = pts[chosen_id];
    }

    return true;
  };

  Eigen::Vector3d closest(0,0,0);
  bool positive_side = false;
  // test with box b points using box a projections
  if (closest_box_pt_along_projection_outside_box(
      pts_b, center_a, halfsize_x_a, col0_a, closest, positive_side))
  {
    if (positive_side)
    {
      double dist = 0.0;
      auto closest_pt = closestpt_on_lineseg_to_pt(
        pts_a[0], pts_a[3], closest, dist);
      
      b = closest;
      a = closest_pt;
      return dist;
    }
    else
    {
      double dist = 0.0;
      auto closest_pt = closestpt_on_lineseg_to_pt(
        pts_a[1], pts_a[2], closest, dist);

      // rmf_planner_viz::draw::IMDraw::draw_circle(sf::Vector2f(pts_a[1].x(), pts_a[1].y()), 0.115, sf::Color(255, 255, 0, 255));
      // rmf_planner_viz::draw::IMDraw::draw_circle(sf::Vector2f(pts_a[2].x(), pts_a[2].y()), 0.115, sf::Color(255, 255, 0, 255));
      b = closest;
      a = closest_pt;
      //rmf_planner_viz::draw::IMDraw::draw_circle(sf::Vector2f(a.x(), a.y()), 0.115, sf::Color(255, 255, 0, 255));
      //rmf_planner_viz::draw::IMDraw::draw_circle(sf::Vector2f(b.x(), b.y()), 0.115, sf::Color(255, 255, 0, 255));

      return dist;
    }
  }

  if (closest_box_pt_along_projection_outside_box(
      pts_b, center_a, halfsize_y_a, col1_a, closest, positive_side))
  {
    if (positive_side)
    {
      double dist = 0.0;
      auto closest_pt = closestpt_on_lineseg_to_pt(
        pts_a[0], pts_a[1], closest, dist);
      b = closest;
      a = closest_pt;
      return dist;
    }
    else
    {
      double dist = 0.0;
      auto closest_pt = closestpt_on_lineseg_to_pt(
        pts_a[2], pts_a[3], closest, dist);
      b = closest;
      a = closest_pt;
      return dist;
    }
  }

  // test with box a points using box b projections
  if (closest_box_pt_along_projection_outside_box(
      pts_a, center_b, halfsize_x_b, col0_b, closest, positive_side))
  {
    if (positive_side)
    {
      double dist = 0.0;
      auto closest_pt = closestpt_on_lineseg_to_pt(
        pts_b[0], pts_b[3], closest, dist);
      b = closest;
      a = closest_pt;
      return dist;
    }
    else
    {
      double dist = 0.0;
      auto closest_pt = closestpt_on_lineseg_to_pt(
        pts_b[1], pts_b[2], closest, dist);
      b = closest;
      a = closest_pt;
      return dist;
    }
  }

  if (closest_box_pt_along_projection_outside_box(
      pts_a, center_b, halfsize_y_b, col1_b, closest, positive_side))
  {
    if (positive_side)
    {
      double dist = 0.0;
      auto closest_pt = closestpt_on_lineseg_to_pt(
        pts_b[0], pts_b[1], closest, dist);
      b = closest;
      a = closest_pt;
      return dist;
    }
    else
    {
      double dist = 0.0;
      auto closest_pt = closestpt_on_lineseg_to_pt(
        pts_b[2], pts_b[3], closest, dist);
      b = closest;
      a = closest_pt;
      return dist;
    }
  }
  return -1.0;
}

struct SeperationComputation
{
  int support_vertex_a = -1;
  int support_vertex_b = -1;

  int count_a = 0;
  int count_b = 0;
  Eigen::Vector3d local_points_a[4];
  Eigen::Vector3d local_points_b[4];

  Eigen::Vector3d seperation_axis;

  SeperationComputation(
    const Eigen::Vector3d& d_normalized,
    const ModelSpaceShape& a_shape, const ModelSpaceShape& b_shape)
    : seperation_axis(d_normalized)
  {
    auto update_locals = [](const ModelSpaceShape& shp,
      int& count, Eigen::Vector3d (&local_points)[4])
    {
      if (shp.shape->getNodeType() == fcl::GEOM_SPHERE)
      {
        count = 1;
        local_points[0].setZero();
      }
      else if (shp.shape->getNodeType() == fcl::GEOM_BOX)
      {
        auto box =
          std::dynamic_pointer_cast<fcl::Boxd>(shp.shape);
        count = 4;
        auto halfside = box->side * 0.5;
        local_points[0] = Eigen::Vector3d(-halfside.x(), -halfside.y(), 0.0);
        local_points[1] = Eigen::Vector3d( halfside.x(), -halfside.y(), 0.0);
        local_points[2] = Eigen::Vector3d( halfside.x(),  halfside.y(), 0.0);
        local_points[3] = Eigen::Vector3d(-halfside.x(),  halfside.y(), 0.0);

        //@attempt: eliminate furthest vertex and use 2 edges?
      }
      //@todo: type error
    };

    update_locals(a_shape, count_a, local_points_a);
    update_locals(b_shape, count_b, local_points_b);
  }

  double compute_seperation(double t,
    fcl::SplineMotion<double>& motion_a,
    fcl::SplineMotion<double>& motion_b,
    const ModelSpaceShape& a_shape,
    const ModelSpaceShape& b_shape)
  {
    motion_a.integrate(t);
    motion_b.integrate(t);

    fcl::Transform3d a_tx, b_tx;
    motion_a.getCurrentTransform(a_tx);
    motion_b.getCurrentTransform(b_tx);

    auto a_shape_tx = a_tx * a_shape._transform;
    auto b_shape_tx = b_tx * b_shape._transform;

    // get one support point on both a and b
    // if e_points
    if (count_a == 1 && count_b == 1) 
    {
      // shortcut for sphere-sphere
      support_vertex_a = 0;
      support_vertex_b = 0;

      auto tx1 = a_shape_tx * local_points_a[support_vertex_a];
      auto tx2 = b_shape_tx * local_points_b[support_vertex_b];
      double s = seperation_axis.dot(tx2 - tx1);
      return s;
    }
    else
    {
      auto get_support = [](const Eigen::Vector3d& axis,
        int count, Eigen::Vector3d (&local_points)[4])
      {
        int best_idx = 0;
        double best_dotp = axis.dot(local_points[0]);
        printf("dotp 0: %f\n", best_dotp);
        for (int i=1; i<count; ++i)
        {
          double dotp = axis.dot(local_points[i]);
          printf("dotp %d: %f\n", i, dotp);
          if (dotp > best_dotp)
          {
            best_idx = i;
            best_dotp = dotp;
          }
        }
        return best_idx;
      };
      
      support_vertex_a = get_support(a_shape_tx.linear() * seperation_axis, count_a, local_points_a);
      support_vertex_b = get_support(b_shape_tx.linear() * -seperation_axis, count_b, local_points_b);
      printf("vtx a: %d b: %d\n", support_vertex_a, support_vertex_b);
      
      auto point_a = a_shape_tx * local_points_a[support_vertex_a];
      auto point_b = b_shape_tx * local_points_b[support_vertex_b];
      rmf_planner_viz::draw::IMDraw::draw_circle(sf::Vector2f(point_b.x(), point_b.y()), 0.15f);
      double s = seperation_axis.dot(point_b - point_a);
      return s;
    }
    //@todo
    return 0.0;
  }

  double evaluate(double t,
    fcl::SplineMotion<double>& motion_a,
    fcl::SplineMotion<double>& motion_b,
    const ModelSpaceShape& a_shape,
    const ModelSpaceShape& b_shape)
  {
    motion_a.integrate(t);
    motion_b.integrate(t);

    fcl::Transform3d a_tx, b_tx;
    motion_a.getCurrentTransform(a_tx);
    motion_b.getCurrentTransform(b_tx);
    
    auto a_shape_tx = a_tx * a_shape._transform;
    auto b_shape_tx = b_tx * b_shape._transform;
    
    auto p_a = a_shape_tx * local_points_a[support_vertex_a];
    auto p_b = b_shape_tx * local_points_b[support_vertex_b];
    double s = seperation_axis.dot(p_b - p_a);
    return s;
  }
};

enum MOTION_ADVANCEMENT_RESULT
{
  RESTART = 0,
  COLLIDE,
  FAIL,
};

static MOTION_ADVANCEMENT_RESULT max_motion_advancement(double current_t,
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const ModelSpaceShape& a_shape,
  const ModelSpaceShape& b_shape,
  const Eigen::Vector3d& d_normalized,
  double target_length, double tolerance, 
  uint& dist_checks, double& t_out)
{
  assert(tolerance >= 0.0);
  
  target_length = target_length;

#ifdef DO_LOGGING
  printf("======\n");
  printf("target_length: %f\n", target_length);
#endif
  SeperationComputation computation(d_normalized, a_shape, b_shape);
  uint outerloop_iter = 0;
  double t1 = current_t, t2 = 1.0;
  for (;;)
  {
    // find min seperation of points
    float s2 = computation.compute_seperation(t2,
      motion_a, motion_b, a_shape, b_shape);
#ifdef DO_LOGGING
    printf("outerloop_iter %d\n", outerloop_iter);
    printf("  (t2:%f, s2:%f)\n", t2, s2);
#endif
    // final configuration reached
    if (s2 > target_length + tolerance)
    {
#ifdef DO_LOGGING
      //output->state = b2TOIOutput::e_separated;
      printf("e_separated\n");
#endif
      t_out = 1.0;
      return COLLIDE;
    }

    if (s2 > target_length - tolerance)
    {
      t_out = t2;
#ifdef DO_LOGGING
      printf("restart %f\n", t_out);
#endif
      return RESTART; // restart from a new configuration
    }
    
    // Compute the initial separation of the witness points.
    double s1 = computation.evaluate(t1,
      motion_a, motion_b, a_shape, b_shape);
#ifdef DO_LOGGING
    printf("  (t1:%f, s1:%f)\n", t1, s1);
#endif

    // initial overlap test
    if (s1 < target_length - tolerance)
    {
#ifdef DO_LOGGING
      //output->state = b2TOIOutput::e_failed;
      printf("e_failed, target_length:%f\n", target_length);
#endif
      t_out = t1;
      return FAIL;
    }

    // Check for touching
    if (s1 <= target_length + tolerance)
    {
      // Victory! t1 should hold the TOI (could be 0.0).
#ifdef DO_LOGGING
      //output->state = b2TOIOutput::e_touching;
      printf("e_touching, target_length: %f\n", target_length);
#endif
      t_out = t1;
      return COLLIDE;
    }

    // Compute 1D root of: f(x) - target_length = 0
    uint rootfind_iter = 0;
    double a1 = t1, a2 = t2;
    for (;;)
    {
      double t;
      if (rootfind_iter & 1) // Secant rule
        t = a1 + (target_length - s1) * (a2 - a1) / (s2 - s1);
      else
        t = 0.5 * (a1 + a2); // Bisection
      
      ++rootfind_iter;
      
      double s = computation.evaluate(t,
        motion_a, motion_b, a_shape, b_shape);
#ifdef DO_LOGGING
			printf("    (t:%f, s:%f)\n", t, s);
#endif

      if (abs(s - target_length) < tolerance)
      {
        t2 = t;
        break;
      }

      // Ensure we continue to bracket the root.
      if (s > target_length)
      {
        a1 = t;
        s1 = s;
      }
      else
      {
        a2 = t;
        s2 = s;
      }

      if (rootfind_iter >= 25)
        break;
    }

    ++outerloop_iter;
    dist_checks = dist_checks + rootfind_iter + outerloop_iter;
    //if (pushBackIter == b2_maxPolygonVertices)
    if (outerloop_iter >= 50)
      break;
  }
#ifdef DO_LOGGING
  printf("exit\n");
#endif
  return FAIL;
}

bool collide_pairwise_shapes(
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const ModelSpaceShape& a_shape,
  const ModelSpaceShape& b_shape,
  double& impact_time, uint& dist_checks, 
  uint safety_maximum_checks, double tolerance)
{
  auto calc_min_dist = [](
    const fcl::Transform3d& a_tx,
    const fcl::Transform3d& b_tx,
    const ModelSpaceShape& a_shape,
    const ModelSpaceShape& b_shape,
    Eigen::Vector3d& d, double& dist_closest_features,
    double& target_length)
  {
    auto a_shape_tx = a_tx * a_shape._transform;
    auto b_shape_tx = b_tx * b_shape._transform;

    if (a_shape.shape->getNodeType() == fcl::GEOM_SPHERE && 
        b_shape.shape->getNodeType() == fcl::GEOM_SPHERE)
    {
      auto sphere_a =
        std::dynamic_pointer_cast<fcl::Sphered>(a_shape.shape);
      auto sphere_b =
        std::dynamic_pointer_cast<fcl::Sphered>(b_shape.shape);

      auto a = a_shape_tx.translation();
      auto b = b_shape_tx.translation();
      d = b - a;
      dist_closest_features = d.norm();
      
      d /= dist_closest_features;
      target_length = sphere_a->radius + sphere_b->radius;
    }
    else if (a_shape.shape->getNodeType() == fcl::GEOM_SPHERE && 
        b_shape.shape->getNodeType() == fcl::GEOM_BOX)
    {
      auto sphere_a =
        std::dynamic_pointer_cast<fcl::Sphered>(a_shape.shape);
      auto box_b =
        std::dynamic_pointer_cast<fcl::Boxd>(b_shape.shape);

      Eigen::Vector3d a(0, 0, 0), b(0, 0, 0);
      fcl::detail::sphereBoxDistance(*sphere_a, a_shape_tx, *box_b, b_shape_tx, 
        &dist_closest_features, &a, &b);

      d = b - a;
      d /= dist_closest_features;
      target_length = sphere_a->radius;
    }
    else if (a_shape.shape->getNodeType() == fcl::GEOM_BOX && 
        b_shape.shape->getNodeType() == fcl::GEOM_SPHERE)
    {
      auto box_a =
        std::dynamic_pointer_cast<fcl::Boxd>(a_shape.shape);
      auto sphere_b =
        std::dynamic_pointer_cast<fcl::Sphered>(b_shape.shape);

      Eigen::Vector3d a(0, 0, 0), b(0, 0, 0);
      fcl::detail::sphereBoxDistance(*sphere_b, b_shape_tx, *box_a, a_shape_tx, 
        &dist_closest_features, &b, &a);
      
      d = b - a;
      d /= dist_closest_features;
      target_length = sphere_b->radius;
    }
    else if (a_shape.shape->getNodeType() == fcl::GEOM_BOX && 
        b_shape.shape->getNodeType() == fcl::GEOM_BOX)
    {
      auto box_a =
        std::dynamic_pointer_cast<fcl::Boxd>(a_shape.shape);
      auto box_b =
        std::dynamic_pointer_cast<fcl::Boxd>(b_shape.shape);

      Eigen::Vector3d a(0, 0, 0), b(0, 0, 0);
      dist_closest_features = box_box_closest_pts(*box_a, a_shape_tx, *box_b, b_shape_tx,
        a, b);
      
      d = b - a;
      d /= dist_closest_features;
      target_length = 0.0;
    }
    //@todo: exception
  };

  fcl::Transform3d a_start_tf, b_start_tf;
  fcl::Transform3d a_tf, b_tf;

  motion_a.integrate(0.0);
  motion_b.integrate(0.0);
  motion_a.getCurrentTransform(a_start_tf);
  motion_b.getCurrentTransform(b_start_tf);

  double target_length = 0.0;
  double dist_to_cover = 0.0;
  Eigen::Vector3d d(0,0,0);
  calc_min_dist(a_start_tf, b_start_tf, a_shape, b_shape,
    d, dist_to_cover, target_length);
  
  double t = 0.0;
  uint iter = 0;
  while (dist_to_cover > 0.0 && t < 1.0)
  {
#ifdef DO_LOGGING
    printf("======= iter:%d\n", iter);
    std::cout << "d_norm: \n" << d << std::endl;
    std::cout << "dist_to_cover: " << dist_to_cover << std::endl;
    rmf_planner_viz::draw::IMDraw::draw_arrow(sf::Vector2f(0, 0), sf::Vector2f(d.x(), d.y()));
#endif
    auto collide_result = max_motion_advancement(t, motion_a, motion_b, a_shape, b_shape, 
      d, target_length, tolerance, dist_checks, t);
    if (collide_result == COLLIDE)
      break;
    else if (collide_result == FAIL)
      return false;

#ifdef DO_LOGGING
    printf("max_motion_advancement returns t: %f\n", t);
#endif

    motion_a.integrate(t);
    motion_b.integrate(t);

    motion_a.getCurrentTransform(a_tf);
    motion_b.getCurrentTransform(b_tf);

    calc_min_dist(a_tf, b_tf, a_shape, b_shape,
      d, dist_to_cover, target_length);

    ++dist_checks;
    ++iter;
    
    //infinite loop prevention. you should increase safety_maximum_checks if you still want a solution
    if (dist_checks > safety_maximum_checks)
      break;
  }
  
  if (dist_checks > safety_maximum_checks)
    return false;

  if (t >= 0.0 && t < 1.0)
  {
    impact_time = t;
#ifdef DO_LOGGING
    printf("time of impact: %f\n", t);
#endif
    return true;
  }
#ifdef DO_LOGGING
  printf("no collide\n");
#endif
  return false;
}

bool collide_seperable_shapes(
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  double& impact_time, uint& iterations, uint safety_maximum_iterations, double tolerance)
{
  for (const auto& a_shape : a_shapes)
  {
    for (const auto& b_shape : b_shapes)
    {
      uint iterations_this_pair = 0;
      double toi = 0.0;
      bool collide = collide_pairwise_shapes(
        motion_a, motion_b, a_shape, b_shape, toi, 
        iterations_this_pair, safety_maximum_iterations,
        tolerance);

      iterations += iterations_this_pair;

      if (collide)
      {
        //we can have some leeway on accurancy of the TOI
        impact_time = toi; 
        return true;
      }
    }
  }
  return false;
}

fcl::SplineMotion<double> to_fcl(const std::array<Eigen::Vector3d, 4>& knots)
{
  std::array<Eigen::Vector3d, 4> Td;
  std::array<Eigen::Vector3d, 4> Rd;

  for (std::size_t i = 0; i < 4; ++i) {
    const Eigen::Vector3d p = knots[i];
    Td[i] = Eigen::Vector3d(p[0], p[1], 0.0);
    Rd[i] = Eigen::Vector3d(0.0, 0.0, p[2]);
  }

  return fcl::SplineMotion<double>(Td[0], Td[1], Td[2], Td[3], Rd[0], Rd[1], Rd[2],
                            Rd[3]);
}

std::vector<Preset> setup_presets()
{
  std::vector<Preset> presets;
  fcl::Transform3<double> identity;
  identity.setIdentity();

  {
    Preset p;
    p._description = "Straight Line vs Stationary";
    p._type = PRESET_SPLINEMOTION;

    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(1.5, 1.5, 0));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(1.0, 1.0, 0));
    //p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    //p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));

    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -3.0, 0));
    //p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Boxd>(1.2, 1.2, 0));
    
    p.b_start = Eigen::Vector3d(-3, 0, 0);
    p.b_end = Eigen::Vector3d(0, 0, 0);
    presets.push_back(p);
  }

  {
    Preset p;
    p._description = "On the spot rotation vs Stationary";
    p._type = PRESET_SPLINEMOTION;

    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    //p.b_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(1.0, 1.0, 0));

    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    //p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Boxd>(1.0, 1.0, 0));
    
    p.b_start = Eigen::Vector3d(-2, 0, 0);
    p.b_end = Eigen::Vector3d(-2, 0, EIGEN_PI / 2.0);

    presets.push_back(p);
  }
  {
    Preset p;
    p._description = "2 sidecars rotating and hitting";
    p._type = PRESET_SPLINEMOTION;

    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));

    fcl::Transform3<double> shape_a2_offset;
    shape_a2_offset.setIdentity();
    shape_a2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.a_shapes.emplace_back(shape_a2_offset, std::make_shared<fcl::Sphered>(0.6));
    
    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));
    
    p.a_end = Eigen::Vector3d(0, 0, -EIGEN_PI);

    //p.b_start = Eigen::Vector3d(-3.8, 0, 0);
    p.b_start = Eigen::Vector3d(-2.0, 0, 0);
    p.b_end = Eigen::Vector3d(-2.0, 0, EIGEN_PI);

    presets.push_back(p);
  }

  {
    Preset p;
    p._description = "2 sidecars crossing each other #1";
    p._type = PRESET_SPLINEMOTION;

    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));

    fcl::Transform3<double> shape_a2_offset;
    shape_a2_offset.setIdentity();
    shape_a2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.a_shapes.emplace_back(shape_a2_offset, std::make_shared<fcl::Sphered>(0.6));
    
    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));
    
    p.a_start = Eigen::Vector3d(0, -3, 0);
    p.a_end = Eigen::Vector3d(0, 3, 0);

    p.b_start = Eigen::Vector3d(-3, 0, 0);
    p.b_end = Eigen::Vector3d(3, 0, 0);
    
    presets.push_back(p);
  }

  {
    Preset p;
    p._description = "2 sidecars crossing each other #2";
    p._type = PRESET_SPLINEMOTION;

    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));

    fcl::Transform3<double> shape_a2_offset;
    shape_a2_offset.setIdentity();
    shape_a2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.a_shapes.emplace_back(shape_a2_offset, std::make_shared<fcl::Sphered>(0.6));
    
    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));

    p.a_start = Eigen::Vector3d(-1, -3, 0);
    p.a_end = Eigen::Vector3d(-1, 3, 0);

    p.b_start = Eigen::Vector3d(-3, 0, 0);
    p.b_end = Eigen::Vector3d(3, 0, 0);
    
    presets.push_back(p);
  }

  {
    Preset p;
    p._description = "2 sidecars crossing each other #3";
    p._type = PRESET_SPLINEMOTION;

    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));

    fcl::Transform3<double> shape_a2_offset;
    shape_a2_offset.setIdentity();
    shape_a2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.a_shapes.emplace_back(shape_a2_offset, std::make_shared<fcl::Sphered>(0.6));
    
    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));

    p.a_start = Eigen::Vector3d(2, 3, 0);
    p.a_end = Eigen::Vector3d(2, -3, 0);

    p.b_start = Eigen::Vector3d(-3, 0, 0);
    p.b_end = Eigen::Vector3d(3, 0, 0);
    
    presets.push_back(p);
  }

  /***  splines with arcs ***/
  {
    Preset p;
    p._description = "Arc without rotation vs Stationary";
    p._type = PRESET_SPLINEMOTION;

    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    
    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(1, 0, 0));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));

    p.tolerance = 0.1;
    
    p.a_start = Eigen::Vector3d(0,0,0);
    p.a_end = Eigen::Vector3d(0,0,0);

    p.b_start = Eigen::Vector3d(-5, 0, 0);
    p.b_end = Eigen::Vector3d(-2, 0, 0);

    p.b_vel = Eigen::Vector3d(16, 0, 0);

    presets.push_back(p);
  }

  {
    Preset p;
    p._description = "Arc with rotation vs Stationary";
    p._type = PRESET_SPLINEMOTION;

    //p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    //p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(0.75, 0.5, 0));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(0.5, 0.5, 0));
    
    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));

    p.tolerance = 0.1;
    
    p.a_start = Eigen::Vector3d(0,0,0);
    p.a_end = Eigen::Vector3d(0,0,0);

    p.b_start = Eigen::Vector3d(-5, 0, 0);
    p.b_end = Eigen::Vector3d(-1.5, 0, EIGEN_PI / 2.0);

    p.b_vel = Eigen::Vector3d(0, 16, 0);
    presets.push_back(p);
  }

  /*** Single body ***/
  {
    Preset p;
    p._description = "Single body straight lines";
    p._type = PRESET_SPLINEMOTION;

    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));

    p.a_start = Eigen::Vector3d(-3,0,0);
    p.a_end = Eigen::Vector3d(3,0,0);

    p.b_start = Eigen::Vector3d(0, -4, 0);
    p.b_end = Eigen::Vector3d(0, 5, 0);
    presets.push_back(p);
  }
  return presets;
}

} // namespace draw
} // namespace rmf_planner_viz
