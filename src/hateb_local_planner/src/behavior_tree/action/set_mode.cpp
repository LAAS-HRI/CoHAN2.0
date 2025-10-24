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

#include <hateb_local_planner/behavior_tree/action/set_mode.h>

namespace hateb_local_planner {

SetMode::SetMode(::SharedPtr node, const std::string& name, const BT::NodeConfig& config) : StatefulActionNodeROS(node, name, config) {
  // set the name of the node
  name_ = name;
  // planning_mode_pub_ = node_->create_publisher<hateb_local_planner::msg::PlanningMode>("planning_mode", 10);
}

SetMode::~SetMode() {
  // Set the RCLCPP_INFO while shutting down the node
  RCLCPP_INFO(node_->get_logger(), "Shutting down the setMode BT Node");
}

BT::NodeStatus SetMode::onStart() {
  getInput("plan_type", plan_type_);
  getInput("predict_type", predict_type_);

  // Update the planning mode on the blackboard based on the type
  if (plan_type_ == "single") {
    p_msg_.plan = PLAN::SINGLE_BAND;
  }

  else if (plan_type_ == "dual") {
    p_msg_.plan = PLAN::DUAL_BAND;
  }

  else if (plan_type_ == "velobs") {
    p_msg_.plan = PLAN::VELOBS;
  }

  else if (plan_type_ == "backoff") {
    p_msg_.plan = PLAN::BACKOFF;
  }

  else if (plan_type_ == "passthrough") {
    p_msg_.plan = PLAN::PASSTHROUGH;
  }

  if (predict_type_ == "const_vel") {
    p_msg_.predict = PREDICTION::CONST_VEL;
  }

  else if (predict_type_ == "behind") {
    p_msg_.predict = PREDICTION::BEHIND;
  }

  else if (predict_type_ == "predict") {
    p_msg_.predict = PREDICTION::PREDICT;
  }

  else if (predict_type_ == "external") {
    p_msg_.predict = PREDICTION::EXTERNAL;
  }

  // Port Remapping has to be called before using setOutput (notice name change)
  setOutput("mode", p_msg_);

  BT_INFO(name_, "onStart()")
  // Set the status to RUNNING
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetMode::onRunning() {
  // Keep Running until halt
  BT_INFO(name_, "onRunning()")
  return BT::NodeStatus::RUNNING;
}

void SetMode::onHalted() {
  // Just update the info
  BT_INFO(name_, "Node is interrupted by calling onHalted()");
}

};  // namespace hateb_local_planner
