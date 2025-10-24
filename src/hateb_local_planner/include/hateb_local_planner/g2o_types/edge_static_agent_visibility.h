/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2023-2025 LAAS-CNRS
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Phani Teja Singamaneni
 *********************************************************************************/

#ifndef EDGE_STATIC_AGENT_VISIBILITY_H_
#define EDGE_STATIC_AGENT_VISIBILITY_H_

#include <hateb_local_planner/g2o_types/base_teb_edges.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/hateb_config.h>

#include "g2o/core/base_unary_edge.h"

namespace hateb_local_planner {

/**
 * @class EdgeStaticAgentVisibility
 * @brief Edge defining a visibility constraint between a static agent and the robot in the optimization graph.
 *
 * This edge penalizes close configurations when the robot is not in the visible region of human.
 * The cost is computed based on the visibility between the static agent and robot, and is used to enforce perception constraints.
 *
 * Inherits from BaseTebUnaryEdge<1, double, VertexPose>.
 */
class EdgeStaticAgentVisibility : public BaseTebUnaryEdge<1, double, VertexPose> {
 public:
  /**
   * @brief Construct edge and set measurement to zero (unused).
   */
  EdgeStaticAgentVisibility() { this->setMeasurement(0.0); }

  /**
   * @brief Compute the error / cost for the edge.
   *
   * Calculates the visibility between the static agent and robot and applies a penalty if the visibility constraint is violated.
   * The error is stored in _error[0].
   */
  void computeError() override {
    HATEB_ASSERT_MSG(cfg_, "You must call setParameters() on EdgeStaticAgentVisibility()");
    const auto *robot_bandpt = static_cast<const VertexPose *>(_vertices[0]);

    Eigen::Vector2d d_rtoh = static_agent_.position() - robot_bandpt->position();
    Eigen::Vector2d d_htor = robot_bandpt->position() - static_agent_.position();
    Eigen::Vector2d agent_look_at = {cos(static_agent_.theta()), sin(static_agent_.theta())};
    Eigen::Vector2d robot_look_at = {cos(robot_bandpt->theta()), sin(robot_bandpt->theta())};

    // calculate the angle between human heading and direction vector from human to robot
    double delta_psi = fabs(acos(agent_look_at.dot(d_htor) / (agent_look_at.norm() * d_htor.norm())));
    double c_visibility = 0.0;
    double ang = agent_look_at.dot(robot_look_at);

    // Apply the cost if the robot is not within the FOV of human (behind the human)
    if (delta_psi >= cfg_->agent.fov * M_PI / 180) {
      if (ang >= 0) {
        c_visibility = 5 * ((std::pow(2, -(std::pow(d_rtoh.x(), 2)))) * (std::pow(2, -(std::pow(d_rtoh.y(), 2)))));
      } else {
        c_visibility = 0.;
      }
    }

    _error[0] = penaltyBoundFromAbove(c_visibility, cfg_->hateb.visibility_cost_threshold, cfg_->optim.penalty_epsilon);

    HATEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeStaticAgentVisibility::computeError() _error[0]=%f\n", _error[0]);
  }

  /**
   * @brief Set parameters for the edge, including configuration and static agent pose.
   * @param cfg HATebConfig configuration
   * @param static_agent Pose of the static agent
   */
  void setParameters(const HATebConfig &cfg, const PoseSE2 &static_agent) {
    cfg_ = &cfg;
    static_agent_ = static_agent;
  }

 protected:
  /**
   * @brief Pose of the static agent used for visibility calculation.
   */
  PoseSE2 static_agent_;

 public:
  /**
   * @brief Ensure proper alignment for fixed-size Eigen types.
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};  // namespace hateb_local_planner

#endif
