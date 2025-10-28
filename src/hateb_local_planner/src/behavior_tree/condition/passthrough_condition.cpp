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

#include <hateb_local_planner/behavior_tree/condition/passthrough_condition.h>

namespace hateb_local_planner {

PassThroughCondition::PassThroughCondition(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf) {
  // Initialize the node
  name_ = condition_name;
  BT_INFO(name_, "Starting the PassThroughCondition BT Node");
}

PassThroughCondition::~PassThroughCondition() {
  // BT_INFO in destructor
  BT_INFO(name_, "Shutting down the PassThroughCondition BT Node");
}

BT::NodeStatus PassThroughCondition::tick() {
  getInput("passage_type", psg_type_);

  // PassThrough is activated when a DOOR or PILLAR is detected
  if (psg_type_ == cohan_msgs::msg::PassageType::DOOR || psg_type_ == cohan_msgs::msg::PassageType::PILLAR) {
    BT_INFO(name_, "Passage detected!")
    return BT::NodeStatus::SUCCESS;
  }

  BT_INFO(name_, "No Passage Detected!")
  return BT::NodeStatus::FAILURE;
}
};  // namespace hateb_local_planner