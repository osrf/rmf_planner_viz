
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

//#define DO_LOGGING 1

namespace rmf_planner_viz {
namespace draw {

inline double distsq_lineseg_to_pt(
  const Eigen::Vector3d (&pts)[4], int ls_idx_start, int ls_idx_end,
  const Eigen::Vector3d& pt, int* pt_on_lineseg_idx = nullptr)
{
  if (pt_on_lineseg_idx)
    *pt_on_lineseg_idx = -1; // not on either endpoint

  auto ls_vec = pts[ls_idx_end] - pts[ls_idx_start];
  double len = ls_vec.norm();
  auto ls_vec_norm = ls_vec / len;
  
  double l = ls_vec_norm.dot(pt - pts[ls_idx_start]);
  if (l <= 0.0)
  {
    l = 0.0;
    if (pt_on_lineseg_idx)
      *pt_on_lineseg_idx = ls_idx_start;
  }
  if (l >= len)
  {
    l = len;
    if (pt_on_lineseg_idx)
      *pt_on_lineseg_idx = ls_idx_end;
  }

  auto closest_pt_on_ls = pts[ls_idx_start] + l * ls_vec_norm;

  double distsq = (closest_pt_on_ls - pt).squaredNorm();
  return distsq;
}

inline double min_distsq_lineseg_to_lineseg(
  const Eigen::Vector3d& p1, const Eigen::Vector3d& q1,
  const Eigen::Vector3d& p2, const Eigen::Vector3d& q2,
  double& s1, double& s2)
{
  // From Real Time Collision Detection, chapter 5.1.9
  double s = 0.0, t = 0.0;

  auto d1 = q1 - p1;
  auto d2 = q2 - p2;
  auto r = p1 - p2;
  double a = d1.squaredNorm();
  double e = d2.squaredNorm();
  double f = d2.dot(r);

  double epsilon = 1e-06;
  if (a <= epsilon && e <= epsilon)
    return (p2 - p1).squaredNorm();
  if (a <= epsilon)
  {
    s = 0.0;
    t = f / e;
    fcl::clipToRange(t, 0.0, 1.0);
  }
  else
  {
    double c = d1.dot(r);
    if (e <= epsilon)
    {
      t = 0.0;
      s = -c / a;
      fcl::clipToRange(s, 0.0, 1.0);
    }
    else
    {
      double b = d1.dot(d2);
      double denom = a * e - b * b;
      if (denom != 0.0)
      {
        s = (b * f - c * e) / denom;
        fcl::clipToRange(s, 0.0, 1.0);
      }
      else
        s = 0.0;
      
      t = (b * s + f) / e;
      if (t < 0.0)
      {
        t = 0.0;
        s = -c /a;
        fcl::clipToRange(s, 0.0, 1.0);
      }
      else if (t > 1.0)
      {
        t = 1.0;
        s = (b - c) / a;
        fcl::clipToRange(s, 0.0, 1.0);
      }
    }
  }

  auto c1 = p1 + d1 * s;
  auto c2 = p2 + d2 * t;
  s1 = s;
  s2 = t;
  return (c2 - c1).squaredNorm();
}

struct SeperationInfo
{
  // closest features
  uint count_a = 0;
  uint count_b = 0;
  int pointindices_a[2] = { -1, -1 };
  int pointindices_b[2] = { -1, -1 };

  void set_indices_from_lineseg(double parameter, int ls_start_idx, int ls_end_idx, 
    bool use_a = true)
  {
    if (use_a)
    {
      if (parameter == 0.0)
      {
        count_a = 1;
        pointindices_a[0] = ls_start_idx;
      }
      else if (parameter == 1.0)
      {
        count_a = 1;
        pointindices_a[0] = ls_end_idx;
      }
      else
      {
        count_a = 2;
        pointindices_a[0] = ls_start_idx;
        pointindices_a[1] = ls_end_idx;
      }
    }
    else
    {
      if (parameter == 0.0)
      {
        count_b = 1;
        pointindices_b[0] = ls_start_idx;
      }
      else if (parameter == 1.0)
      {
        count_b = 1;
        pointindices_b[0] = ls_end_idx;
      }
      else
      {
        count_b = 2;
        pointindices_b[0] = ls_start_idx;
        pointindices_b[1] = ls_end_idx;
      }
    }
  }

  void swap()
  {
    for (int i=0; i<2; ++i)
      std::swap(pointindices_a[i], pointindices_b[i]);
    std::swap(count_a, count_b);
  }
};

static const int INSIDE = 0b0000;
static const int LEFT   = 0b0001;
static const int RIGHT  = 0b0010;
static const int BOTTOM = 0b0100;
static const int TOP    = 0b1000;

inline double sphere_box_closest_features(
  const fcl::Sphered& sphere,
  const fcl::Transform3d& tx_a,
  const fcl::Boxd& box,
  const fcl::Transform3d& tx_b,
  SeperationInfo& seperation_info)
{
  auto sphere_center = tx_a.translation();
  auto box_center = tx_b.translation();

  auto linear_b = tx_b.linear();
  auto col0_b = linear_b.col(0);
  auto col1_b = linear_b.col(1);

  double halfsize_x = box.side.x() * 0.5;
  double halfsize_y = box.side.y() * 0.5;

  auto box_to_sphere = sphere_center - box_center;
  double dotp1 = box_to_sphere.dot(col0_b);
  double dotp2 = box_to_sphere.dot(col1_b);

  int outcode = 0;
  if (dotp1 < -halfsize_x)
    outcode |= LEFT;
  if (dotp1 > halfsize_x)
    outcode |= RIGHT;
  if (dotp2 < -halfsize_y)
    outcode |= BOTTOM;
  if (dotp2 > halfsize_y)
    outcode |= TOP;
    
  if (outcode == INSIDE)
    return -1.0; //point inside box; collision

  // start bottom left move anticlockwise
  Eigen::Vector3d pts[4];
  pts[0] = box_center - col0_b * halfsize_x - col1_b * halfsize_y;
  pts[1] = box_center + col0_b * halfsize_x - col1_b * halfsize_y;
  pts[2] = box_center + col0_b * halfsize_x + col1_b * halfsize_y;
  pts[3] = box_center - col0_b * halfsize_x + col1_b * halfsize_y;
  double radius_sq = sphere.radius * sphere.radius;

  if (outcode == (LEFT | BOTTOM))
  {
    auto boxpt_to_sphere = sphere_center - pts[0];
    double distsq = boxpt_to_sphere.squaredNorm();
    if (distsq <= radius_sq)
      return -1.0; //circle intersecting box; collision

    // pick 2 points to be the closest features
    seperation_info.count_b = 1;
    seperation_info.pointindices_b[0] = 0;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }
  if (outcode == (RIGHT | BOTTOM))
  {
    auto boxpt_to_sphere = sphere_center - pts[1];
    double distsq = boxpt_to_sphere.squaredNorm();
    if (distsq <= radius_sq)
      return -1.0; //circle intersecting box; collision

    // pick 2 points to be the closest features
    seperation_info.count_b = 1;
    seperation_info.pointindices_b[0] = 1;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }
  if (outcode == (RIGHT | TOP))
  {
    auto boxpt_to_sphere = sphere_center - pts[2];
    double distsq = boxpt_to_sphere.squaredNorm();
    if (distsq <= radius_sq)
      return -1.0; //circle intersecting box; collision
    
    // pick points to be the closest features
    seperation_info.count_b = 1;
    seperation_info.pointindices_b[0] = 2;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }
  if (outcode == (LEFT | TOP))
  {
    auto boxpt_to_sphere = sphere_center - pts[3];
    double distsq = boxpt_to_sphere.squaredNorm();
    if (distsq <= radius_sq)
      return -1.0; //circle intersecting box; collision

    seperation_info.count_b = 1;
    seperation_info.pointindices_b[0] = 3;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;
    return std::sqrt(distsq);
  }

  // singular sides, compare with line segments
  if (outcode == LEFT)
  {
    double distsq = distsq_lineseg_to_pt(pts, 0, 3, sphere_center);
    if (distsq <= radius_sq)
      return -1.0; //intersection

    seperation_info.count_b = 2;
    seperation_info.pointindices_b[0] = 0;
    seperation_info.pointindices_b[1] = 3;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }

  if (outcode == RIGHT)
  {
    double distsq = distsq_lineseg_to_pt(pts, 2, 1, sphere_center);
    if (distsq <= radius_sq)
      return -1.0; //intersection

    seperation_info.count_b = 2;
    seperation_info.pointindices_b[0] = 2;
    seperation_info.pointindices_b[1] = 1;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }

  if (outcode == BOTTOM)
  {
    double distsq = distsq_lineseg_to_pt(pts, 0, 1, sphere_center);
    if (distsq <= radius_sq)
      return -1.0; //intersection

    seperation_info.count_b = 2;
    seperation_info.pointindices_b[0] = 0;
    seperation_info.pointindices_b[1] = 1;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }

  if (outcode == TOP)
  {
    double distsq = distsq_lineseg_to_pt(pts, 3, 2, sphere_center);
    if (distsq <= radius_sq)
      return -1.0; //intersection

    // pick the line segment's 2 points to be closest features
    seperation_info.count_b = 2;
    seperation_info.pointindices_b[0] = 3;
    seperation_info.pointindices_b[1] = 2;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }
  
  return -1.0;
}

inline double box_box_closest_features(
      const fcl::Boxd& box_a,
      const fcl::Transform3d& tx_a,
      const fcl::Boxd& box_b,
      const fcl::Transform3d& tx_b,
      SeperationInfo& seperation_info)
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
  pts_a[0] = center_a - col0_a * halfsize_x_a - col1_a * halfsize_y_a;
  pts_a[1] = center_a + col0_a * halfsize_x_a - col1_a * halfsize_y_a;
  pts_a[2] = center_a + col0_a * halfsize_x_a + col1_a * halfsize_y_a;
  pts_a[3] = center_a - col0_a * halfsize_x_a + col1_a * halfsize_y_a;

  Eigen::Vector3d pts_b[4];
  pts_b[0] = center_b - col0_b * halfsize_x_b - col1_b * halfsize_y_b;
  pts_b[1] = center_b + col0_b * halfsize_x_b - col1_b * halfsize_y_b;
  pts_b[2] = center_b + col0_b * halfsize_x_b + col1_b * halfsize_y_b;
  pts_b[3] = center_b - col0_b * halfsize_x_b + col1_b * halfsize_y_b;

  auto closest_box_features_outside_box_face = [](
    Eigen::Vector3d (&pts)[4], const Eigen::Vector3d& center,
    double halfside_x, double halfside_y,
    const Eigen::Vector3d& projection_x, const Eigen::Vector3d& projection_y,
    int& face_idx_1, int& face_idx_2,
    int& closest_idx_1, int& closest_idx_2)
      -> bool
  { 
    face_idx_1 = -1;
    face_idx_2 = -1;
    closest_idx_1 = -1;
    closest_idx_2 = -1;

    uint pts_left = 0, pts_right = 0, pts_top = 0, pts_bottom = 0;
    Eigen::Vector2d dotp[4];
    for (int i=0; i<4; ++i)
    {
      auto center_to_boxpt = pts[i] - center;
      dotp[i].x() = projection_x.dot(center_to_boxpt);
      dotp[i].y() = projection_y.dot(center_to_boxpt);

      if (dotp[i].x() > halfside_x)
        ++pts_right;
      else if (dotp[i].x() < -halfside_x)
        ++pts_left;

      if (dotp[i].y() > halfside_y)
        ++pts_top;
      else if (dotp[i].y() < -halfside_y)
        ++pts_bottom;
    }

    if (pts_left == 4 || pts_right == 4 || pts_top == 4 || pts_bottom == 4)
    {
      // get the minimum 2 point indices
      double distsq[4];
      for (int i=0; i<4; ++i)
        distsq[i] = dotp[i].squaredNorm();

      closest_idx_1 = 0;
      closest_idx_2 = 1;
      if (distsq[0] > distsq[1])
        std::swap(closest_idx_1, closest_idx_2);
      
      for (int i=2; i<4; ++i)
      { 
        if (distsq[i] < distsq[closest_idx_2])
        {
          if (distsq[i] < distsq[closest_idx_1])
          {
            closest_idx_2 = closest_idx_1;
            closest_idx_1 = i;
          }
          else
            closest_idx_2 = i;
        }
      }
      
      if (pts_left == 4)
      {
        face_idx_1 = 0;
        face_idx_2 = 3;
        return LEFT;
      }
      if (pts_right == 4)
      {
        face_idx_1 = 2;
        face_idx_2 = 1;
        return RIGHT;
      }
      if (pts_bottom == 4)
      {
        face_idx_1 = 0;
        face_idx_2 = 1;
        return BOTTOM;
      }
      if (pts_top == 4)
      {
        face_idx_1 = 2;
        face_idx_2 = 3;
        return TOP;
      }
    }
    
    return 0;
  };

  int face_a_1 = -1;
  int face_a_2 = -1;
  int closest_b_1 = -1;
  int closest_b_2 = -1;
  if (closest_box_features_outside_box_face(pts_b, center_a, 
    halfsize_x_a, halfsize_y_a, col0_a, col1_a, 
    face_a_1, face_a_2, closest_b_1, closest_b_2) != INSIDE)
  {
    double param_a = 0.0;
    double param_b = 0.0;
    double distsq = min_distsq_lineseg_to_lineseg(pts_a[face_a_1], pts_a[face_a_2], 
      pts_b[closest_b_1], pts_b[closest_b_2], param_a, param_b);

    seperation_info.set_indices_from_lineseg(param_a, face_a_1, face_a_2);
    seperation_info.set_indices_from_lineseg(param_b, closest_b_1, closest_b_2, false);
    
#ifdef DO_LOGGING
    rmf_planner_viz::draw::IMDraw::draw_circle(
      sf::Vector2f(pts_a[face_a_1].x(), pts_a[face_a_1].y()), 0.0625f, sf::Color(128,128,128));
    rmf_planner_viz::draw::IMDraw::draw_circle(
      sf::Vector2f(pts_a[face_a_2].x(), pts_a[face_a_2].y()), 0.0625f, sf::Color(128,128,128));

    rmf_planner_viz::draw::IMDraw::draw_circle(
      sf::Vector2f(pts_b[closest_b_1].x(), pts_b[closest_b_1].y()), 0.0625f, sf::Color(0,0,255));
    rmf_planner_viz::draw::IMDraw::draw_circle(
      sf::Vector2f(pts_b[closest_b_2].x(), pts_b[closest_b_2].y()), 0.0625f, sf::Color(0,0,255));
#endif
    return std::sqrt(distsq);
  }

  int face_b_1 = -1;
  int face_b_2 = -1;
  int closest_a_1 = -1;
  int closest_a_2 = -1;
  if (closest_box_features_outside_box_face(pts_a, center_b, 
    halfsize_x_b, halfsize_y_b, col0_b, col1_b, 
    face_b_1, face_b_2, closest_a_1, closest_a_2) != INSIDE)
  {
    double param_a = 0.0;
    double param_b = 0.0;
    double distsq = min_distsq_lineseg_to_lineseg(pts_b[face_b_1], pts_b[face_b_2],
      pts_a[closest_a_1], pts_a[closest_a_2], param_b, param_a);

    seperation_info.set_indices_from_lineseg(param_a, closest_a_1, closest_a_2);
    seperation_info.set_indices_from_lineseg(param_b, face_b_1, face_b_2, false);
    
    return std::sqrt(distsq);
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
  Eigen::Vector3d local_axis_point;

  const SeperationInfo initial_seperation_props;

  SeperationComputation(
    fcl::SplineMotion<double>& motion_a,
    fcl::SplineMotion<double>& motion_b,
    const ModelSpaceShape& shape_a,
    const ModelSpaceShape& shape_b,
    const SeperationInfo& seperation_info)
    :initial_seperation_props(seperation_info)
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
      }
      //@todo: type error
    };

    update_locals(shape_a, count_a, local_points_a);
    update_locals(shape_b, count_b, local_points_b);

    fcl::Transform3d tx_a, tx_b;
    motion_a.getCurrentTransform(tx_a);
    motion_b.getCurrentTransform(tx_b);

    tx_a = tx_a * shape_a._transform;
    tx_b = tx_b * shape_b._transform;
    
    if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 1)
    {
      const auto& localpoint_a = local_points_a[initial_seperation_props.pointindices_a[0]];
      const auto& localpoint_b = local_points_b[initial_seperation_props.pointindices_b[0]];
      auto point_a = tx_a * localpoint_a;
      auto point_b = tx_b * localpoint_b;

      seperation_axis = point_b - point_a;
      seperation_axis.normalize();
#ifdef DO_LOGGING
      printf("seperation_axis: %f, %f\n", seperation_axis.x(), seperation_axis.y());
#endif
    }
    else if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 2)
    {
      // get seperation_axis in local space
      auto& localpoint_b1 = local_points_b[initial_seperation_props.pointindices_b[0]];
      auto& localpoint_b2 = local_points_b[initial_seperation_props.pointindices_b[1]];
      
      auto vec = localpoint_b2 - localpoint_b1;
      seperation_axis = Eigen::Vector3d(vec.y(), -vec.x(), 0.0);
      seperation_axis.normalize();

      auto transformed_axis = tx_b.linear() * seperation_axis;
      local_axis_point = (localpoint_b1 + localpoint_b2) * 0.5;

      auto& localpoint_a = local_points_a[initial_seperation_props.pointindices_a[0]];
      auto point_a = tx_a * localpoint_a;
      auto point_b = tx_b * local_axis_point;

      double s = transformed_axis.dot(point_a - point_b);
      if (s < 0.0)
        seperation_axis = -seperation_axis;

#ifdef DO_LOGGING
      printf("a1 vs b2 %d, %d\n", initial_seperation_props.pointindices_b[0], initial_seperation_props.pointindices_b[1]);
      printf("localpoint_b1: %f, %f\n", localpoint_b1.x(), localpoint_b1.y());
      printf("localpoint_b2: %f, %f\n", localpoint_b2.x(), localpoint_b2.y());

      printf("seperation_axis: %f, %f\n", seperation_axis.x(), seperation_axis.y());
      printf("point_b: %f, %f\n", point_b.x(), point_b.y());
#endif
    }
    else //if (initial_seperation_props.count_a == 2)
    {
      //2 points on A
      // get seperation_axis in local space
      auto& localpoint_a1 = local_points_a[initial_seperation_props.pointindices_a[0]];
      auto& localpoint_a2 = local_points_a[initial_seperation_props.pointindices_a[1]];
      
      auto vec = localpoint_a2 - localpoint_a1;
      seperation_axis = Eigen::Vector3d(vec.y(), -vec.x(), 0.0);
      seperation_axis.normalize();

      auto transformed_axis = tx_a.linear() * seperation_axis;
      local_axis_point = (localpoint_a1 + localpoint_a2) * 0.5;

      auto point_a = tx_a * local_axis_point;

      auto& localpoint_b = local_points_b[initial_seperation_props.pointindices_b[0]];
      auto point_b = tx_b * localpoint_b;
      double s = transformed_axis.dot(point_b - point_a);
      if (s < 0.0)
        seperation_axis = -seperation_axis;

#ifdef DO_LOGGING
      printf("seperation_axis: %f, %f\n", seperation_axis.x(), seperation_axis.y());
#endif
    }
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

    a_tx = a_tx * a_shape._transform;
    b_tx = b_tx * b_shape._transform;

    auto get_support = [](const Eigen::Vector3d& axis,
      int count, Eigen::Vector3d (&local_points)[4])
    {
      int best_idx = 0;
      double best_dotp = axis.dot(local_points[0]);
      for (int i=1; i<count; ++i)
      {
        double dotp = axis.dot(local_points[i]);
        if (dotp > best_dotp)
        {
          best_idx = i;
          best_dotp = dotp;
        }
      }
      return best_idx;
    };

    // get support points on both a and b
    if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 1) 
    {
      // shortcut for sphere-sphere
      auto transformed_axis_a = a_tx.linear().inverse() * seperation_axis;
      auto transformed_axis_b = b_tx.linear().inverse() * -seperation_axis;
      
      support_vertex_a = get_support(transformed_axis_a, count_a, local_points_a);
      support_vertex_b = get_support(transformed_axis_b, count_b, local_points_b);

#ifdef DO_LOGGING
      printf("point-point\n");
      printf("transformed_axis_b: %f %f\n", transformed_axis_b.x(), transformed_axis_b.y());
      printf("support_vertex_a: %d support_vertex_b: %d\n", support_vertex_a, support_vertex_b);
#endif
      auto point_a = a_tx * local_points_a[support_vertex_a];
      auto point_b = b_tx * local_points_b[support_vertex_b];
      double s = seperation_axis.dot(point_b - point_a);
      return s;
    }
    else if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 2)
    {

      auto transformed_axis = b_tx.linear() * seperation_axis;
      auto axis_a = a_tx.linear().inverse() * -transformed_axis;

      support_vertex_a = get_support(axis_a, count_a, local_points_a);
      support_vertex_b = -1;

      auto point_a = a_tx * local_points_a[support_vertex_a];
      auto point_b = b_tx * local_axis_point;
#ifdef DO_LOGGING
      printf("  initial_seperation_props.count_b == 2, t:%f\n", t);
      printf("  transformed_axis: %f, %f\n", transformed_axis.x(), transformed_axis.y());
      printf("  axis_a: %f, %f\n", axis_a.x(), axis_a.y());
      printf("  support_vertex_a: %d\n", support_vertex_a);
      printf("  point_a: %f %f\n", point_a.x(), point_a.y());
      printf("  point_b: %f %f\n", point_b.x(), point_b.y());
#endif
      double s = transformed_axis.dot(point_a - point_b);
      return s;
    }
    else //if (initial_seperation_props.count_a == 2)
    {
      auto transformed_axis = a_tx.linear() * seperation_axis;
      auto axis_b = b_tx.linear().inverse() * -transformed_axis;
      
      support_vertex_a = -1;
      support_vertex_b = get_support(axis_b, count_b, local_points_b);

      auto point_a = a_tx * local_axis_point;
      auto point_b = b_tx * local_points_b[support_vertex_b];
#ifdef DO_LOGGING
      printf("  initial_seperation_props.count_a == 2, t:%f\n", t);
      printf("  transformed_axis: %f, %f\n", transformed_axis.x(), transformed_axis.y());
      printf("  axis_b: %f, %f\n", axis_b.x(), axis_b.y());
      printf("  point_a: %f %f\n", point_a.x(), point_a.y());
      printf("  point_b: %f %f\n", point_b.x(), point_b.y());
#endif
      double s = transformed_axis.dot(point_b - point_a);
      return s;
      
    }
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
    
    a_tx = a_tx * a_shape._transform;
    b_tx = b_tx * b_shape._transform;
    
    if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 1) 
    {
      auto point_a = a_tx * local_points_a[support_vertex_a];
      auto point_b = b_tx * local_points_b[support_vertex_b];
      double s = seperation_axis.dot(point_b - point_a);
      return s;
    }
    else if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 2)
    {
      auto normal = b_tx.linear() * seperation_axis;
      auto pt_a = a_tx * local_points_a[support_vertex_a];
      auto pt_b = b_tx * local_axis_point;
      double s = normal.dot(pt_a - pt_b);
      return s;
    }
    else //if (initial_seperation_props.count_a == 2)
    {
      auto normal = a_tx.linear() * seperation_axis;
      auto pt_a = a_tx * local_axis_point;
      auto pt_b = b_tx * local_points_b[support_vertex_b];
      double s = normal.dot(pt_b - pt_a);
      return s;
    }
  }
};

enum MOTION_ADVANCEMENT_RESULT
{
  RESTART = 0,
  COLLIDE,
  SEPERATED,
};

static inline MOTION_ADVANCEMENT_RESULT max_motion_advancement(double current_t,
  double t_max,
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const ModelSpaceShape& a_shape,
  const ModelSpaceShape& b_shape,
  SeperationInfo& seperation_info,
  double target_length, double tolerance, 
  uint& iterations, double& t_out)
{
  assert(tolerance >= 0.0);
  
  target_length = target_length;
  t_out = t_max;

#ifdef DO_LOGGING
  printf("======\n");
  printf("target_length: %f\n", target_length);
#endif
  SeperationComputation computation(motion_a, motion_b, a_shape, b_shape, seperation_info);
  uint outerloop_iter = 0;
  double t1 = current_t, t2 = t_max;
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
      t_out = t_max;
      return SEPERATED;
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
      return SEPERATED;
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
      ++rootfind_iter;
    }

    iterations = iterations + rootfind_iter + outerloop_iter;
    //if (pushBackIter == b2_maxPolygonVertices)
    if (outerloop_iter >= 6)
      break;
    ++outerloop_iter;
  }
#ifdef DO_LOGGING
  printf("restart\n");
#endif
  return RESTART;
}

inline bool collide_pairwise_shapes(
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const ModelSpaceShape& a_shape,
  const ModelSpaceShape& b_shape,
  double& impact_time, uint& dist_checks, 
  uint safety_maximum_checks, double tolerance,
  double t_min = 0.0,
  double t_max = 1.0)
{
  auto calc_min_dist = [](
    const fcl::Transform3d& a_tx,
    const fcl::Transform3d& b_tx,
    const ModelSpaceShape& a_shape,
    const ModelSpaceShape& b_shape,
    SeperationInfo& seperation_info,
    double& dist,
    double& target_length)
  {
    seperation_info.count_a = 0;
    seperation_info.count_b = 0;

    if (a_shape.shape->getNodeType() == fcl::GEOM_SPHERE && 
        b_shape.shape->getNodeType() == fcl::GEOM_SPHERE)
    {
      auto sphere_a =
        std::dynamic_pointer_cast<fcl::Sphered>(a_shape.shape);
      auto sphere_b =
        std::dynamic_pointer_cast<fcl::Sphered>(b_shape.shape);

      auto a = a_tx.translation();
      auto b = b_tx.translation();

      target_length = sphere_a->radius + sphere_b->radius;
      
      double distsq = (b - a).squaredNorm();
      //distsq = distsq - target_length * target_length;
      dist = std::sqrt(distsq) - target_length;
      
      seperation_info.count_a = 1;
      seperation_info.pointindices_a[0] = 0;
      seperation_info.count_b = 1;
      seperation_info.pointindices_b[0] = 0;
    }
    else if (a_shape.shape->getNodeType() == fcl::GEOM_SPHERE && 
        b_shape.shape->getNodeType() == fcl::GEOM_BOX)
    {
      auto sphere_a =
        std::dynamic_pointer_cast<fcl::Sphered>(a_shape.shape);
      auto box_b =
        std::dynamic_pointer_cast<fcl::Boxd>(b_shape.shape);

      dist = 
        sphere_box_closest_features(*sphere_a, a_tx, *box_b, b_tx,
          seperation_info);
      
      target_length = sphere_a->radius;
    }
    else if (a_shape.shape->getNodeType() == fcl::GEOM_BOX && 
        b_shape.shape->getNodeType() == fcl::GEOM_SPHERE)
    {
      auto box_a =
        std::dynamic_pointer_cast<fcl::Boxd>(a_shape.shape);
      auto sphere_b =
        std::dynamic_pointer_cast<fcl::Sphered>(b_shape.shape);

      dist = 
        sphere_box_closest_features(*sphere_b, b_tx, *box_a, a_tx,
          seperation_info);
      
      //swap
      seperation_info.swap();
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
      dist = 
        box_box_closest_features(*box_a, a_tx, *box_b, b_tx,
          seperation_info);
      
      target_length = 0.0;
    }
    //@todo: exception
  };

  fcl::Transform3d a_start_tf, b_start_tf;
  SeperationInfo seperation_info;

  motion_a.integrate(t_min);
  motion_b.integrate(t_min);
  motion_a.getCurrentTransform(a_start_tf);
  motion_b.getCurrentTransform(b_start_tf);

  a_start_tf = a_start_tf * a_shape._transform;
  b_start_tf = b_start_tf * b_shape._transform;

  double target_length = 0.0;
  double dist_to_cover = 0.0;
  calc_min_dist(a_start_tf, b_start_tf, a_shape, b_shape,
    seperation_info,
    dist_to_cover, target_length);
  
  double t = t_min;
  uint iter = 0;
  while (dist_to_cover > 0.0 && t < t_max)
  {
#ifdef DO_LOGGING
    printf("======= iter:%d\n", iter);
    auto d = seperation_info.a_to_b;
    std::cout << "a_to_b: \n" << d << std::endl;
    std::cout << "dist_to_cover: " << dist_to_cover << std::endl;
    rmf_planner_viz::draw::IMDraw::draw_arrow(sf::Vector2f(0, 0), sf::Vector2f(d.x(), d.y()));
#endif
    auto collide_result = max_motion_advancement(t, t_max, motion_a, motion_b, a_shape, b_shape, 
      seperation_info,
      target_length, tolerance, dist_checks, t);
    if (collide_result == COLLIDE)
      break;
    else if (collide_result == SEPERATED)
      return false;

#ifdef DO_LOGGING
    printf("max_motion_advancement returns t: %f\n", t);
#endif

    fcl::Transform3d a_tf, b_tf;
    motion_a.getCurrentTransform(a_tf);
    motion_b.getCurrentTransform(b_tf);

    a_tf = a_tf * a_shape._transform;
    b_tf = b_tf * b_shape._transform;

    calc_min_dist(a_tf, b_tf, a_shape, b_shape, seperation_info,
      dist_to_cover, target_length);

    ++dist_checks;
    ++iter;
    
    //infinite loop prevention. you should increase safety_maximum_checks if you still want a solution
    if (dist_checks > safety_maximum_checks)
      break;
  }
  
  if (dist_checks > safety_maximum_checks)
    return false;

  if (t >= t_min && t < t_max)
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
  uint sweeps,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  double& impact_time, uint& iterations, uint safety_maximum_iterations,
  double tolerance)
{
  for (const auto& a_shape : a_shapes)
  {
    for (const auto& b_shape : b_shapes)
    {
      uint iterations_this_pair = 0;
      double t_per_sweep = 1.0 / (double)sweeps;
      for (uint i=0; i<sweeps; ++i)
      {
        double t_min = t_per_sweep * (double)i;
        double t_max = t_per_sweep * (double)(i + 1);

        double toi = 0.0;
        bool collide = collide_pairwise_shapes(
          motion_a, motion_b, a_shape, b_shape, toi, 
          iterations_this_pair, safety_maximum_iterations,
          tolerance, t_min, t_max);

        iterations += iterations_this_pair;
        if (collide)
        {
          //we can have some leeway on accurancy of the TOI
          impact_time = toi; 
          return true;
        }

      }      
    }
  }
  return false;
}

uint get_sweep_divisions(const Eigen::Vector3d& a_x0, const Eigen::Vector3d& a_x1, 
  const Eigen::Vector3d& b_x0, const Eigen::Vector3d& b_x1)
{
  //it's been noted in the bilateral advancement that rotations can miss collisions
  //the only way to handle it is to introduces sweep ranges

  //@todo: it would be good to support spline velocities, ie. spline curvatures
  double rot_diff_a = std::abs(a_x1.z() - a_x0.z());
  double rot_diff_b = std::abs(b_x1.z() - b_x0.z());

  double half_pi = EIGEN_PI * 0.5;
  uint a_intervals = (uint)std::ceil(rot_diff_a / half_pi);
  uint b_intervals = (uint)std::ceil(rot_diff_b / half_pi);
  if (a_intervals == 0 && b_intervals == 0)
    return 1;
  return a_intervals > b_intervals ? a_intervals : b_intervals;
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

    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(1.0, 1.0, 0));
    //p.b_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(1.0, 1.0, 0));
    //p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    //p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));

    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    //p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Boxd>(1.2, 1.2, 0));
    
    p.b_start = Eigen::Vector3d(-4, 3.25, 0);
    p.b_end = Eigen::Vector3d(4, 3.25, 0);
    presets.push_back(p);
  }

  {
    Preset p;
    p._description = "On the spot rotation vs Stationary";
    p._type = PRESET_SPLINEMOTION;

    //p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(1.0, 1.0, 0));
    //p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    //p.b_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(1.0, 1.0, 0));

    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    //p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Boxd>(1.0, 1.0, 0));
    
    p.b_start = Eigen::Vector3d(-1.75, -0.25, 0);
    //p.b_end = Eigen::Vector3d(-1.75, -0.25, EIGEN_PI / 2.0 * 1.25f);
    p.b_end = Eigen::Vector3d(-1.75, -0.25, EIGEN_PI / 2.0);

    presets.push_back(p);
  }
  {
    Preset p;
    p._description = "2 sidecars rotating and hitting";
    p._type = PRESET_SPLINEMOTION;

    //p.a_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(1.0, 1.0, 0));
    //p.b_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(1.0, 1.0, 0));
    // p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    // p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));

    fcl::Transform3<double> shape_a2_offset;
    shape_a2_offset.setIdentity();
    shape_a2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    //p.a_shapes.emplace_back(shape_a2_offset, std::make_shared<fcl::Sphered>(0.6));
    p.a_shapes.emplace_back(shape_a2_offset, std::make_shared<fcl::Boxd>(1.5, 1.0, 0));
    
    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    //p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));
    p.b_shapes.emplace_back(shape_a2_offset, std::make_shared<fcl::Boxd>(0.75, 1.0, 0));
    
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

    p.b_vel = Eigen::Vector3d(0, 16, 0);

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
