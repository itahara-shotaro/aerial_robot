// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

/* Removal of the xy position integral along given (body-fixed) axes, e.g. the stiff axis of a rod
   clamped between drones that integrate their own, mutually inconsistent, absolute position errors.
   Only one unit should integrate along a stiff connection; the others keep P/D there. ROS-free so
   that it can be unit tested. */

namespace aerial_robot_control
{
  namespace integral_projection
  {
    /* Map unit body axes into the world frame and keep their normalised horizontal parts. An axis
       whose horizontal part is shorter than min_horizontal_norm (mostly vertical) is skipped and
       counted in skipped. */
    inline std::vector<Eigen::Vector2d> horizontalAxes(const Eigen::Matrix3d& rot,
                                                       const std::vector<Eigen::Vector3d>& body_axes,
                                                       const double min_horizontal_norm = 0.5,
                                                       int* skipped = nullptr)
    {
      std::vector<Eigen::Vector2d> axes;
      if(skipped) *skipped = 0;
      for(const auto& b: body_axes)
        {
          Eigen::Vector2d a = (rot * b.normalized()).head<2>();
          double norm = a.norm();
          if(norm < min_horizontal_norm)
            {
              if(skipped) (*skipped)++;
              continue;
            }
          axes.push_back(a / norm);
        }
      return axes;
    }

    /* Orthonormal basis of span{axes} (Gram-Schmidt): parallel axes collapse to one vector, two
       independent ones give the whole plane. */
    inline std::vector<Eigen::Vector2d> orthonormalBasis(const std::vector<Eigen::Vector2d>& axes,
                                                         const double tolerance = 1e-3)
    {
      std::vector<Eigen::Vector2d> basis;
      for(const auto& a: axes)
        {
          if(basis.size() == 2) break;
          Eigen::Vector2d v = a;
          for(const auto& q: basis) v -= q.dot(a) * q;
          double norm = v.norm();
          if(norm > tolerance) basis.push_back(v / norm);
        }
      return basis;
    }

    /* w and w_prev are the integral states after and before this cycle's update, in i-term space
       (i_gain * err_i, so that unequal x/y gains are handled). Returns the corrected state: this
       cycle's increment along the axes is dropped, and the component already stored along them
       decays by the factor decay (= du / tau, clamped to [0, 1]). */
    inline Eigen::Vector2d projectOutIntegralAxes(const Eigen::Vector2d& w, const Eigen::Vector2d& w_prev,
                                                  const std::vector<Eigen::Vector2d>& horizontal_axes,
                                                  const double decay)
    {
      Eigen::Matrix2d projection = Eigen::Matrix2d::Zero();
      for(const auto& q: orthonormalBasis(horizontal_axes)) projection += q * q.transpose();

      double d = std::min(std::max(decay, 0.0), 1.0);
      return w - projection * (w - w_prev) - d * projection * w_prev;
    }

    /* The same on the PIDs' err_i states (after and before this cycle's update) with i_gains k:
       maps them into i-term space and back. An axis whose gain is ~0 keeps its err_i. */
    inline Eigen::Vector2d projectOutErrI(const Eigen::Vector2d& err_i, const Eigen::Vector2d& err_i_prev,
                                          const Eigen::Vector2d& k,
                                          const std::vector<Eigen::Vector2d>& horizontal_axes,
                                          const double decay)
    {
      Eigen::Vector2d w = projectOutIntegralAxes(k.cwiseProduct(err_i), k.cwiseProduct(err_i_prev),
                                                 horizontal_axes, decay);
      Eigen::Vector2d err_i_new = err_i;
      for(int i = 0; i < 2; i++)
        if(std::abs(k(i)) > 1e-9) err_i_new(i) = w(i) / k(i);
      return err_i_new;
    }
  };
};
