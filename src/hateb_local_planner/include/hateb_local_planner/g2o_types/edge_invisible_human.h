/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2022-2025 LAAS-CNRS
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

#ifndef EDGE_INVISIBLEHUMAN_H
#define EDGE_INVISIBLEHUMAN_H

#include <hateb_local_planner/footprint_model.h>
#include <hateb_local_planner/g2o_types/base_teb_edges.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/g2o_types/vertex_timediff.h>
#include <hateb_local_planner/obstacles.h>

#include <hateb_local_planner/hateb_config.hpp>

namespace hateb_local_planner {

/**
 * @class EdgeInvisibleHuman
 * @brief Edge defining a constraint for invisible human agents in the optimization graph.
 *
 * This edge penalizes robot configurations that are close to the predicted position of an invisible human agent.
 * The cost is computed based on the agent's predicted position and the robot's relative to this prediction, enforcing safe and plausible robot behavior even when the human is not directly
 * observed.
 *
 * Inherits from BaseTebMultiEdge<1, const Obstacle*>.
 */
class EdgeInvisibleHuman : public BaseTebMultiEdge<1, const Obstacle*> {
 public:
  /**
   * @brief Construct edge and resize to 3 vertices (agent poses and time diff).
   */
  EdgeInvisibleHuman() : t_(0) { this->resize(3); }

  /**
   * @brief Construct edge and specify the time for its associated pose (necessary for computeError).
   * @param t Estimated time until current pose is reached
   */
  explicit EdgeInvisibleHuman(double t) : t_(t) { this->resize(3); }

  /**
   * @brief Compute the error / cost for the edge.
   *
   * Calculates the cost associated with the invisible human agent's predicted state, applying penalties if constraints are violated.
   * The error is stored in _error[0].
   */
  void computeError() override {
    HATEB_ASSERT_MSG(cfg_ && _measurement, "You must call setHATebConfig() on EdgeInvisibleHuman()");
    const auto* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    const auto* bandpt_nxt = static_cast<const VertexPose*>(_vertices[1]);
    const auto* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    // Get the distance and the velocity
    double dist = cfg_->robot_model->calculateDistance(bandpt->pose(), _measurement);
    auto robot_vel = (bandpt_nxt->position() - bandpt->position()) / dt->dt();

    double cost = dist / (V_i_ + robot_vel.norm() - 0.1);
    if (t_ > 0.5) {  // Accounting for human reaction time
      // Cost calculation
      cost = dist / (std::max(V_i_ - (a_norm_ * t_) + robot_vel.norm() - 0.1, 0.01));
    }
    _error[0] = penaltyBoundFromBelow(cost, cfg_->hateb.invisible_human_threshold, cfg_->optim.penalty_epsilon);

    HATEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeInvisibleHuman::computeError() _error[0]=%f\n", _error[0]);
  }

  /**
   * @brief Set all parameters at once
   * @param cfg HATebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle Obstacle msg constaining the invisible human agent's position and velocity
   */
  void setParameters(const HATebConfig& cfg, const Obstacle* obstacle) {
    cfg_ = &cfg;
    _measurement = obstacle;
  }

  double t_;              // Estimated time until current pose is reached
  double V_i_ = 1.5;      // Nominal Velocity of the invisible human agent
  double a_min_ = 0.1;    // Minimum acceleration of the invisible human agent
  double a_norm_ = 0.68;  // Nominal acceleration of the invisible human agent
  double a_max_ = 2.94;   // 0.3g - Maximum possible acceleration of the invisible human agent

 public:
  /**
   * @brief Ensure proper alignment for fixed-size Eigen types.
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace hateb_local_planner

#endif
