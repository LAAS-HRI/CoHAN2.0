/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2021-2025 LAAS-CNRS
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

#ifndef EDGE_AGENT_ROBOT_REL_VELOCIY_H_
#define EDGE_AGENT_ROBOT_REL_VELOCIY_H_

#include <hateb_local_planner/g2o_types/base_teb_edges.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/g2o_types/vertex_timediff.h>

#include <hateb_local_planner/hateb_config.hpp>

namespace hateb_local_planner {

/**
 * @class EdgeAgentRobotRelVelocity
 * @brief Edge defining a relative velocity constraint between robot and agent trajectories in the optimization graph.
 *
 * This edge penalizes configurations where the relative velocity between the robot and an agent exceeds a specified threshold.
 * The cost is computed based on the difference in velocities projected along the direction between the robot and agent, and is penalized if it exceeds cfg_->hateb.rel_vel_cost_threshold.
 *
 * Inherits from BaseTebMultiEdge<1, double>.
 */
class EdgeAgentRobotRelVelocity : public BaseTebMultiEdge<1, double> {
 public:
  /**
   * @brief Construct edge and resize to 6 vertices (robot/agent poses and time diffs).
   */
  EdgeAgentRobotRelVelocity() { this->resize(6); }

  /**
   * @brief Compute the error / cost for the edge.
   *
   * Calculates the relative velocity between the robot and agent over a time interval, and applies a penalty if the cost exceeds the allowed threshold.
   * The error is stored in _error[0].
   */
  void computeError() override {
    HATEB_ASSERT_MSG(cfg_, "You must call setHATebConfig() on EdgeAgentRobotRelVelocity()");
    const auto* robot_bandpt = static_cast<const VertexPose*>(_vertices[0]);
    const auto* robot_bandpt_nxt = static_cast<const VertexPose*>(_vertices[1]);
    const auto* dt_robot = static_cast<const VertexTimeDiff*>(_vertices[2]);
    const auto* agent_bandpt = static_cast<const VertexPose*>(_vertices[3]);
    const auto* agent_bandpt_nxt = static_cast<const VertexPose*>(_vertices[4]);
    const auto* dt_agent = static_cast<const VertexTimeDiff*>(_vertices[5]);

    // Get the velocities of the robot and agent
    Eigen::Vector2d diff_robot = robot_bandpt_nxt->position() - robot_bandpt->position();
    Eigen::Vector2d robot_vel = diff_robot / dt_robot->dt();
    Eigen::Vector2d diff_agent = agent_bandpt_nxt->position() - agent_bandpt->position();
    Eigen::Vector2d agent_vel = diff_agent / dt_agent->dt();

    // Get the direction vectors
    Eigen::Vector2d d_rtoh = agent_bandpt->position() - robot_bandpt->position();
    Eigen::Vector2d d_htor = robot_bandpt->position() - agent_bandpt->position();

    // Cost definition
    double rel_vel_cost = (std::max((robot_vel - agent_vel).dot(d_rtoh), 0.0) + robot_vel.norm() + 1) / d_rtoh.norm();

    _error[0] = penaltyBoundFromAbove(rel_vel_cost, cfg_->hateb.rel_vel_cost_threshold, cfg_->optim.penalty_epsilon);

    HATEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAgentRobot::computeError() _error[0]=%f\n", _error[0]);
  }

 public:
  /**
   * @brief Ensure proper alignment for fixed-size Eigen types.
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};  // namespace hateb_local_planner

#endif  // EDGE_AGENT_ROBOT_REL_VELOCIY_H_
