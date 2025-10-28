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

#include <hateb_local_planner/behavior_tree/condition/single_band_exit_condition.h>

namespace hateb_local_planner {

SingleBandExitCondition::SingleBandExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf) {
  // Initialize the node
  name_ = condition_name;
  dist_max_ = 999;
  BT_INFO(name_, "Starting the SingleBandExitCondition BT Node");
}

SingleBandExitCondition::~SingleBandExitCondition() {
  // BT_INFO in destructor
  BT_INFO(name_, "Shutting down the SingleBandExitCondition BT Node");
}

BT::NodeStatus SingleBandExitCondition::tick() {
  // Read the values from blackboard
  getInput("agents_info", agents_info_);
  getInput("dist_max", dist_max_);  // Set in bt tree xml

  // If no humans, single band mode always
  if (!agents_info_.humans.empty()) {
    // Get the distance of the nearest human from the robot
    auto dist = agents_info_.humans[0].dist;
    // If any human inside planning radius is moving, switch to dual band mode
    if (dist < dist_max_) {
      for (auto& human : agents_info_.humans) {
        if (human.state != agents::AgentState::STATIC && human.state != agents::AgentState::STOPPED) {  //<! the STOPPED condition to reset things properly
          BT_INFO(name_, "Exiting Single Band!")
          return BT::NodeStatus::SUCCESS;
        }
      }
    }
  }

  BT_INFO(name_, "in Single Band")
  return BT::NodeStatus::FAILURE;
}

};  // namespace hateb_local_planner