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

#include "spline_offset_utils.hpp"
#include <SFML/Graphics.hpp>

#include <rmf_planner_viz/draw/IMDraw.hpp>

namespace rmf_planner_viz {
namespace draw {


Eigen::Matrix4d make_M_inv() 
{
  Eigen::Matrix4d M;
  M.block<1, 4>(0, 0) << 1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0, 0.0;
  M.block<1, 4>(1, 0) << -1.0 / 2.0, 0.0, 1.0 / 2.0, 0.0;
  M.block<1, 4>(2, 0) << 1.0 / 2.0, -1.0, 1.0 / 2.0, 0.0;
  M.block<1, 4>(3, 0) << -1.0 / 6.0, 1.0 / 2.0, -1.0 / 2.0, 1.0 / 6.0;

  return M.inverse();
}

std::array<Eigen::Vector4d, 3> compute_coefficients(const Eigen::Vector3d& x0,
                                                    const Eigen::Vector3d& x1,
                                                    const Eigen::Vector3d& v0,
                                                    const Eigen::Vector3d& v1) 
{
  std::array<Eigen::Vector4d, 3> coeffs;
  for (int i = 0; i < 3; ++i) {
    // *INDENT-OFF*
    std::size_t si = static_cast<std::size_t>(i);
    coeffs[si][0] = x0[i];                                       // = d
    coeffs[si][1] = v0[i];                                       // = c
    coeffs[si][2] = -v1[i] - 2 * v0[i] + 3 * x1[i] - 3 * x0[i];  // = b
    coeffs[si][3] = v1[i] + v0[i] - 2 * x1[i] + 2 * x0[i];       // = a
    // *INDENT-ON*
  }

  return coeffs;
}

std::array<Eigen::Vector3d, 4> compute_knots(
  Eigen::Vector3d x0,
  Eigen::Vector3d x1,
  Eigen::Vector3d v0,
  Eigen::Vector3d v1)
{
//   printf("x0: %f %f %f x1: %f %f %f\n", x0[0], x0[1], x0[2], x1[0], x1[1], x1[2]);
//   printf("v0: %f %f %f v1: %f %f %f\n", v0[0], v0[1], v0[2], v1[0], v1[1], v1[2]);
  const std::array<Eigen::Vector4d, 3> subspline_coeffs =
      compute_coefficients(x0, x1, v0, v1);

  Eigen::Matrix4d M_inv = make_M_inv();
  std::array<Eigen::Vector3d, 4> result;
  for (std::size_t i = 0; i < 3; ++i) {
    const Eigen::Vector4d p = M_inv * subspline_coeffs[i];
    //printf("p: %f %f %f %f\n", p[0], p[1], p[2], p[3]);
    for (int j = 0; j < 4; ++j)
      result[j][i] = p[j];
  }

  return result;
}

fcl::SplineMotion<double> convert_catmullrom_to_bspline(
  Eigen::Vector3d p0,
  Eigen::Vector3d p1,
  Eigen::Vector3d p2,
  Eigen::Vector3d p3,
  bool show_control_poly)
{
  // @reference:
  // https://computergraphics.stackexchange.com/questions/8267/conversion-from-cubic-catmull-rom-spline-to-cubic-b-spline
  // the topright and botleft values of the final matrix are wrong, should be negative

  Eigen::Matrix4d mtx_catmullrom;
  mtx_catmullrom << 
    -1.0,  3.0, -3.0,  1.0,
      2.0, -5.0,  4.0, -1.0,
    -1.0,  0.0,  1.0,  0.0,
      0.0,  2.0,  0.0,  0.0;
  mtx_catmullrom *= 0.5;

  Eigen::Matrix4d mtx_bspline;
  mtx_bspline << 
    -1.0,  3.0, -3.0,  1.0,
      3.0, -6.0,  3.0,  0.0,
    -3.0,  0.0,  3.0,  0.0,
      1.0,  4.0,  1.0,  0.0;
  mtx_bspline /= 6.0;
  Eigen::Matrix4d mtx_bspline_inv = mtx_bspline.inverse();

  Eigen::Matrix<double, 4, 3> input;
  input << 
    p0[0], p0[1], p0[2],
    p1[0], p1[1], p1[2],
    p2[0], p2[1], p2[2],
    p3[0], p3[1], p3[2];

  auto result = mtx_bspline_inv * mtx_catmullrom * input; // 4x3 matrix
  Eigen::Vector3d bspline_p0(result(0, 0), result(0, 1), result(0, 2));
  Eigen::Vector3d bspline_p1(result(1, 0), result(1, 1), result(1, 2));
  Eigen::Vector3d bspline_p2(result(2, 0), result(2, 1), result(2, 2));
  Eigen::Vector3d bspline_p3(result(3, 0), result(3, 1), result(3, 2));

  if (show_control_poly)
  {
    IMDraw::draw_circle(sf::Vector2f(bspline_p0.x(), bspline_p0.y()), 0.0625f, sf::Color(128,128,128));
    IMDraw::draw_circle(sf::Vector2f(bspline_p1.x(), bspline_p1.y()), 0.0625f, sf::Color(128,128,128));
    IMDraw::draw_circle(sf::Vector2f(bspline_p2.x(), bspline_p2.y()), 0.0625f, sf::Color(128,128,128));
    IMDraw::draw_circle(sf::Vector2f(bspline_p3.x(), bspline_p3.y()), 0.0625f, sf::Color(128,128,128));

    IMDraw::draw_line(sf::Vector2f(bspline_p0.x(), bspline_p0.y()), sf::Vector2f(bspline_p1.x(), bspline_p1.y()), sf::Color(128,128,128));
    IMDraw::draw_line(sf::Vector2f(bspline_p1.x(), bspline_p1.y()), sf::Vector2f(bspline_p2.x(), bspline_p2.y()), sf::Color(128,128,128));
    IMDraw::draw_line(sf::Vector2f(bspline_p2.x(), bspline_p2.y()), sf::Vector2f(bspline_p3.x(), bspline_p3.y()), sf::Color(128,128,128));
  }

  const Eigen::Vector3d zero = Eigen::Vector3d(0,0,0);
  return fcl::SplineMotion<double>(
    bspline_p0, bspline_p1, bspline_p2, bspline_p3,
    zero, zero, zero, zero);

}

} // namespace draw
} // namespace rmf_planner_viz
