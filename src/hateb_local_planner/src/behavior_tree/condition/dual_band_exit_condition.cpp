/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2024-2025 LAAS-CNRS
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
 * Author: Phani Teja Singamaneni
 *********************************************************************************/

#include <hateb_local_planner/behavior_tree/condition/dual_band_exit_condition.h>

namespace hateb_local_planner {

DualBandExitCondition::DualBandExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf), clock_(RCL_ROS_TIME) {
  name_ = condition_name;
  // Initialize the variables
  dist_threshold_ = 999;
  goal_dist_ = 999;
  stopped_time_ = clock_.now();
  BT_INFO(name_, "Starting the DualBandExitCondition BT Node");
}

DualBandExitCondition::~DualBandExitCondition() {
  // BT_INFO in destructor
  BT_INFO(name_, "Shutting down the DualBandExitCondition BT Node");
}

BT::NodeStatus DualBandExitCondition::tick() {
  // Get the data from blackboard
  getInput("agents_info", agents_info_);
  getInput("dist_threshold", dist_threshold_);  // set in the xml of bt tree

  // Exit the mode if the robot is stuck in any of the two cases
  if (isRobotStuck()) {
    if (!agents_info_.humans.empty()) {
      auto human = agents_info_.humans[0];
      // Only exit the band if the human is under the dist_threshold
      if (human.dist <= dist_threshold_ && (human.state == hateb_local_planner::AgentState::STOPPED)) {
        BT_INFO(name_, "The robot is stuck and human is still, Exiting Dual Band!")
        return BT::NodeStatus::SUCCESS;
      }
      if (human.dist <= dist_threshold_) {
        BT_INFO(name_, "The robot is stuck and human is moving, Exiting Dual Band!")
        return BT::NodeStatus::SUCCESS;
      }
    }
    // Exit even when no human is detected
    else {
      BT_INFO(name_, "The robot is stuck, Exiting Dual Band!")
      return BT::NodeStatus::SUCCESS;
    }
  }

  BT_INFO(name_, "in Dual Band")
  return BT::NodeStatus::FAILURE;
}

bool DualBandExitCondition::isRobotStuck() {
  // Get the inputs from blackboard
  getInput("agents_info", agents_info_);
  getInput("nav_goal", goal_);

  double dx = goal_.pose.position.x - agents_info_.robot_pose.x;
  double dy = goal_.pose.position.y - agents_info_.robot_pose.y;

  // Check if distance to goal is constantly decreasing or not
  if (fabs(goal_dist_ - std::hypot(dx, dy)) > DIST_EPS) {
    stopped_time_ = rclcpp::Clock().now();
    goal_dist_ = std::hypot(dx, dy);
  }

  // If the goal_dist is not decreasing for over 2.0 sec, exit the mode
  if ((rclcpp::Clock().now() - stopped_time_).seconds() >= 2.0) {  // TODO(sphanit): Remove the magic number 2.0s here
    if (goal_dist_ - std::hypot(dx, dy) < 2 * DIST_EPS) {
      return true;
    }
  }

  // Otherwise the robot is moving well
  BT_INFO(name_, "Robot is moving!")
  return false;
}

};  // namespace hateb_local_planner