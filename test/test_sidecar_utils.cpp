
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

static double seperation_sphere_box(
      const fcl::Sphered& sphere,
      const fcl::Transform3d& tx_sphere,
      const fcl::Boxd& box,
      const fcl::Transform3d& tx_box,
      const Eigen::Vector3d& vec_normalized,
      const Eigen::Vector3d& reference_pt)
{
  double projected_sphere = vec_normalized.dot(tx_sphere.translation() - reference_pt);
  double projected_box = vec_normalized.dot(tx_box.translation() - reference_pt);
  
  // project the entire box onto vec_normalized    
  auto linear = tx_box.linear();
  auto col0 = linear.col(0);
  auto col1 = linear.col(1);
  auto halfside = box.side * 0.5;

  auto v0 = col0 * halfside.x() + col1 * halfside.y();
  auto v1 = col0 * halfside.x() - col1 * halfside.y();
  
  double l0 = abs(vec_normalized.dot(v0));
  double l1 = abs(vec_normalized.dot(v1));
  double l = std::max(l0, l1);
  //printf("abs(%g - %g)=%g\n", projected_sphere, projected_box, abs(projected_sphere - projected_box));
  return abs(projected_sphere - projected_box) - (sphere.radius + l);
}

static double compute_seperation_between_shapes_along_vector(
  std::shared_ptr<fcl::ShapeBase<double>> shape_a, 
  const fcl::Transform3d& tx_a,
  std::shared_ptr<fcl::ShapeBase<double>> shape_b, 
  const fcl::Transform3d& tx_b,
  const Eigen::Vector3d& vec_normalized,
  const Eigen::Vector3d& reference_pt)
{
  if (shape_a->getNodeType() == fcl::GEOM_SPHERE && 
    shape_b->getNodeType() == fcl::GEOM_SPHERE)
  {
    std::shared_ptr<fcl::Sphered> sphere_a =
      std::dynamic_pointer_cast<fcl::Sphered>(shape_a);
    std::shared_ptr<fcl::Sphered> sphere_b =
      std::dynamic_pointer_cast<fcl::Sphered>(shape_b);

    //and project, slower than 
#if 0
    Eigen::Vector3d b_to_a = tx_a.translation() - tx_b.translation();
    double b_to_a_dist = b_to_a.norm();
    if (b_to_a_dist > 1e-04)
    {
      auto b_to_a_norm = b_to_a / b_to_a_dist;
      double dist_along_b_to_a = b_to_a_dist - (sphere_a->radius + sphere_b->radius);
      auto v = dist_along_b_to_a * b_to_a_norm;
      double dist_between_shapes_along_d = v.dot(vec_normalized);
      return dist_between_shapes_along_d;
    }
#endif
    //project onto vec_normalized, use the difference. 
    //faster by 50%, but requires a higher tolerance value
    double projected_a = vec_normalized.dot(tx_a.translation() - reference_pt);
    double projected_b = vec_normalized.dot(tx_b.translation() - reference_pt);
    return abs(projected_a - projected_b) - (sphere_a->radius + sphere_b->radius);
  }
  else if (shape_a->getNodeType() == fcl::GEOM_SPHERE && 
    shape_b->getNodeType() == fcl::GEOM_BOX)
  {
    auto sphere_a =
      std::dynamic_pointer_cast<fcl::Sphered>(shape_a);
    auto box_b =
      std::dynamic_pointer_cast<fcl::Boxd>(shape_b);

    return seperation_sphere_box(*sphere_a, tx_a, *box_b, tx_b,
      vec_normalized, reference_pt);
  }
  else if (shape_a->getNodeType() == fcl::GEOM_BOX && 
    shape_b->getNodeType() == fcl::GEOM_SPHERE)
  { 
    auto box_a =
      std::dynamic_pointer_cast<fcl::Boxd>(shape_a);
    auto sphere_b =
      std::dynamic_pointer_cast<fcl::Sphered>(shape_b);

    return seperation_sphere_box(*sphere_b, tx_b, *box_a, tx_a,
      vec_normalized, reference_pt);
  }

  return 0.0;
}

static double compute_dist_between_shapes(
  std::shared_ptr<fcl::ShapeBase<double>> shape_a, 
  const fcl::Transform3d& tx_a,
  std::shared_ptr<fcl::ShapeBase<double>> shape_b, 
  const fcl::Transform3d& tx_b,
  Eigen::Vector3d& a, Eigen::Vector3d& b
)
{
  if (shape_a->getNodeType() == fcl::GEOM_SPHERE && 
    shape_b->getNodeType() == fcl::GEOM_SPHERE)
  {
    std::shared_ptr<fcl::Sphered> sphere_a =
      std::dynamic_pointer_cast<fcl::Sphered>(shape_a);
    std::shared_ptr<fcl::Sphered> sphere_b =
      std::dynamic_pointer_cast<fcl::Sphered>(shape_b);

    double dist = 0.0;
    fcl::detail::sphereSphereDistance(
      *sphere_a, tx_a, *sphere_b, tx_b,
      &dist, &a, &b);
    /*Eigen::Vector3d b_to_a = a_shape_tx.translation() - b_shape_tx.translation();
    dist = b_to_a.norm() - (a_shape._radius + b_shape._radius);*/
    return dist;
  }
  else if (shape_a->getNodeType() == fcl::GEOM_SPHERE && 
    shape_b->getNodeType() == fcl::GEOM_BOX)
  {
    std::shared_ptr<fcl::Sphered> sphere_a =
      std::dynamic_pointer_cast<fcl::Sphered>(shape_a);
    std::shared_ptr<fcl::Boxd> box_b =
      std::dynamic_pointer_cast<fcl::Boxd>(shape_b);

    double dist = 0.0;
    fcl::detail::sphereBoxDistance(*sphere_a, tx_a, *box_b, tx_b, &dist, &a, &b);
    return dist;
  }
  else if (shape_a->getNodeType() == fcl::GEOM_BOX && 
    shape_b->getNodeType() == fcl::GEOM_SPHERE)
  {
    std::shared_ptr<fcl::Boxd> box_a =
      std::dynamic_pointer_cast<fcl::Boxd>(shape_a);
    std::shared_ptr<fcl::Sphered> sphere_b =
      std::dynamic_pointer_cast<fcl::Sphered>(shape_b);

    double dist = 0.0;
    fcl::detail::sphereBoxDistance(*sphere_b, tx_b, *box_a, tx_a, &dist, &b, &a);
    return dist;
  }

  return 0.0;
}

static double max_splinemotion_advancement(double current_t,
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  const Eigen::Vector3d& d_normalized, double max_dist,
  const Eigen::Vector3d& reference_point,
  uint& dist_checks, double tolerance)
{
  assert(tolerance >= 0.0);
  
  double lower_t_limit = current_t;
  double upper_t_limit = 1.0;
  uint bilateral_adv_iter = 0;

  auto compute_seperation = [&](double t)
  {
    motion_a.integrate(t);
    motion_b.integrate(t);

    fcl::Transform3d a_tx, b_tx;
    motion_a.getCurrentTransform(a_tx);
    motion_b.getCurrentTransform(b_tx);

    double s = DBL_MAX;
    // compute seperation between all shapes in direction d and take the minimum
    for (const auto& a_shape : a_shapes)
    {
      auto a_shape_tx = a_tx * a_shape._transform;
      for (const auto& b_shape : b_shapes)
      {
        auto b_shape_tx = b_tx * b_shape._transform;
        
        double seperation = compute_seperation_between_shapes_along_vector(
          a_shape.shape, a_shape_tx, 
          b_shape.shape, b_shape_tx,
          d_normalized, reference_point);

        // get the minimum
        if (seperation < s)
          s = seperation;
      }
    }
    return s;
  };
  
  double s1 = max_dist; 
  double s2 = compute_seperation(1.0);
  //printf("s2: %f\n", s2);
  double sample_t = 0.0;
  for (;;)
  {
#ifdef DO_LOGGING
    //if (bilateral_adv_iter < 3)
      printf("#1: (%f,%f) #2: (%f,%f)\n", lower_t_limit, s1, upper_t_limit, s2);
#endif
    
    // alternate between bisection and false position methods
    if (bilateral_adv_iter & 1/* && ((s1 < 0.0 && s2 > 0.0) || (s1 > 0.0 && s2 < 0.0))*/)
    {
      // use false position method
      // solve for t where (t, tolerance) for a line with 
      // endpoints (lower_t_limit, s1) and (upper_t_limit, s2)
      // where s2 is negative and s1 is positive
      double inv_m = (upper_t_limit - lower_t_limit) / (s2 - s1);
      sample_t = lower_t_limit + (tolerance - s1) * inv_m;
    }
    else // bisection method
      sample_t = lower_t_limit + 0.5 * (upper_t_limit - lower_t_limit);
#ifdef DO_LOGGING
    printf("iteration: %d picked t: %f\n", bilateral_adv_iter, sample_t);
#endif

    double s = compute_seperation(sample_t);

#ifdef DO_LOGGING
    printf("dist_output: %f\n", s);
#endif
    if (abs(s) < tolerance)
    {
#ifdef DO_LOGGING
      printf("minimal dist %f within tolerance range %f\n", s, tolerance);
#endif
      break;
    }

    // our window is very small and we're hopping around, so we stop.
    // Also, this is what box2d does.
    if (bilateral_adv_iter >= 25) 
    {
#ifdef DO_LOGGING
      printf("range too small\n");
#endif
      break;
    }
    
    if (s < 0.0)
    {
      upper_t_limit = sample_t;
      s2 = s;
    }
    else if (s > 0.0)
    {
      lower_t_limit = sample_t;
      s1 = s;
    }
    ++bilateral_adv_iter;
  }
  dist_checks += bilateral_adv_iter;

  return sample_t;
}

bool collide_seperable_circles(
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  double& impact_time, uint& dist_checks, uint safety_maximum_checks, double tolerance)
{
  if (a_shapes.empty() || b_shapes.empty())
    return false;

  auto calc_min_dist = [](
    const fcl::Transform3d& a_tx,
    const fcl::Transform3d& b_tx,
    const std::vector<ModelSpaceShape>& a_shapes,
    const std::vector<ModelSpaceShape>& b_shapes,
    Eigen::Vector3d& d, Eigen::Vector3d& reference_pt, double& min_dist)
  {
    min_dist = DBL_MAX;
    for (const auto& a_shape : a_shapes)
    {
      auto a_shape_tx = a_tx * a_shape._transform;

      for (const auto& b_shape : b_shapes)
      {
        auto b_shape_tx = b_tx * b_shape._transform;
        Eigen::Vector3d a(0,0,0), b(0,0,0);
        double dist = compute_dist_between_shapes(a_shape.shape, a_shape_tx,
          b_shape.shape, b_shape_tx, a, b);

        if (dist < min_dist)
        {
          min_dist = dist;
          d = a - b;
          reference_pt = a;
        }
      }
    }
  };

  fcl::Transform3d a_start_tf, b_start_tf;
  fcl::Transform3d a_tf, b_tf;

  motion_a.integrate(0.0);
  motion_b.integrate(0.0);
  motion_a.getCurrentTransform(a_start_tf);
  motion_b.getCurrentTransform(b_start_tf);

  double dist_along_d_to_cover = 0.0;
  Eigen::Vector3d d(0,0,0), reference_pt(0,0,0);;
  calc_min_dist(a_start_tf, b_start_tf, a_shapes, b_shapes,
    d, reference_pt, dist_along_d_to_cover);
  
  double t = 0.0;
  uint iter = 0;
  while (dist_along_d_to_cover > tolerance && t < 1.0)
  {
    Eigen::Vector3d d_normalized = d.normalized();
#ifdef DO_LOGGING
    printf("======= iter:%d\n", iter);
    std::cout << "d_norm: \n" << d_normalized << std::endl;
    std::cout << "dist_along_d_to_cover: " << dist_along_d_to_cover << std::endl;
#endif

    t = max_splinemotion_advancement(t, motion_a, motion_b, a_shapes, b_shapes, 
      d_normalized, dist_along_d_to_cover, reference_pt, dist_checks, tolerance);
#ifdef DO_LOGGING
    printf("max_splinemotion_advancement returns t: %f\n", t);
#endif

    motion_a.integrate(t);
    motion_b.integrate(t);

    motion_a.getCurrentTransform(a_tf);
    motion_b.getCurrentTransform(b_tf);

    double next_dist_to_cover = 0.0;
    calc_min_dist(a_tf, b_tf, a_shapes, b_shapes,
      d, reference_pt, next_dist_to_cover);

    /*if (next_dist_to_cover <= tolerance)
      break;
    if (next_dist_to_cover > dist_along_d_to_cover)
      return false;*/
    dist_along_d_to_cover = next_dist_to_cover;
    //printf("dist_along_d_to_cover: %g\n", dist_along_d_to_cover);
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
    //p.b_shapes.emplace_back(identity, std::make_shared<fcl::Boxd>(0.5, 0.5, 0));
    //p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));

    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));
    
    p.b_start = Eigen::Vector3d(-3, 2, 0);
    p.b_end = Eigen::Vector3d(0, 2, 0);
    presets.push_back(p);
  }

  {
    Preset p;
    p._description = "On the spot rotation vs Stationary";
    p._type = PRESET_SPLINEMOTION;

    p.a_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));
    p.b_shapes.emplace_back(identity, std::make_shared<fcl::Sphered>(0.5));

    fcl::Transform3<double> shape_b2_offset;
    shape_b2_offset.setIdentity();
    shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));
    p.b_shapes.emplace_back(shape_b2_offset, std::make_shared<fcl::Sphered>(0.6));
    
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

    p.b_start = Eigen::Vector3d(-3.8, 0, 0);
    p.b_end = Eigen::Vector3d(-2.5, 0, EIGEN_PI);

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
