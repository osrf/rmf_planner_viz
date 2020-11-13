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

} // namespace draw
} // namespace rmf_planner_viz