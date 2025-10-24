/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2020-2025 LAAS-CNRS
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

#ifndef EDGE_AGENT_ROBOT_SAFETY_H_
#define EDGE_AGENT_ROBOT_SAFETY_H_

#include <hateb_local_planner/footprint_model.h>
#include <hateb_local_planner/g2o_types/base_teb_edges.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/hateb_config.h>
#include <hateb_local_planner/obstacles.h>

namespace hateb_local_planner {

/**
 * @class EdgeAgentRobotSafety
 * @brief Edge defining a safety constraint between a robot and an agent (e.g., human) in the optimization graph.
 *
 * This edge penalizes configurations where the robot and an agent come closer than a specified minimum distance.
 * The cost is computed as a quadratic penalty if the distance between the robot (using its footprint model) and the agent (using its radius) is less than
 * cfg_->hateb.min_agent_robot_dist. The edge is used to enforce social distancing or collision avoidance between the robot and agents.
 *
 * Inherits from BaseTebBinaryEdge<1, double, VertexPose, VertexPose>.
 */
class EdgeAgentRobotSafety : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose> {
 public:
  /**
   * @brief Construct edge and set measurement to zero (unused).
   */
  EdgeAgentRobotSafety() { this->setMeasurement(0.); }

  /**
   * @brief Compute the error / cost for the edge.
   *
   * Calculates the distance between the robot and agent, subtracts the agent radius, and applies a quadratic penalty if the result is below the minimum allowed agent-robot distance.
   * The error is stored in _error[0].
   */
  void computeError() override {
    HATEB_ASSERT_MSG(cfg_, "You must call setHATebConfig() on EdgeAgentRobotSafety()");
    const auto *robot_bandpt = static_cast<const VertexPose *>(_vertices[0]);
    const auto *agent_bandpt = static_cast<const VertexPose *>(_vertices[1]);
    static_cast<PointObstacle *>(obs_)->setCentroid(agent_bandpt->x(), agent_bandpt->y());

    // The distance is taken according to the footprint model of the robot. Could have some errors
    double agent_radius = cfg_->human_model->getCircumscribedRadius();
    double dist = cfg_->robot_model->calculateDistance(robot_bandpt->pose(), obs_) - agent_radius;

    // Apply a quadratic penalty
    _error[0] = penaltyBoundFromBelowQuad(dist, cfg_->hateb.min_agent_robot_dist, cfg_->optim.penalty_epsilon);

    HATEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAgentRobotSafety::computeError() _error[0]=%f\n", _error[0]);
  }

 protected:
  /**
   * @brief Obstacle pointer used to represent the agent as a point obstacle for distance calculation.
   */
  Obstacle *obs_ = new PointObstacle();

 public:
  /**
   * @brief Ensure proper alignment for fixed-size Eigen types.
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};  // namespace hateb_local_planner

#endif
