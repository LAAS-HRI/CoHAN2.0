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

#include <hateb_local_planner/behavior_tree/condition/backoff_exit_condition.h>

namespace hateb_local_planner {

BackoffExitCondition::BackoffExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf) {
  // set the node name
  name_ = condition_name;
  // Initialize the flags
  started_ = false;
  new_goal_ = false;
  backed_off_ = false;
  BT_INFO(name_, "Starting the BackoffExitCondition BT Node");
}

BackoffExitCondition::~BackoffExitCondition() {
  // BT_INFO in destructor
  BT_INFO(name_, "Shutting down the BackoffExitCondition BT Node");
}

BT::NodeStatus BackoffExitCondition::tick() {
  if (isRecoveryComplete()) {
    // Reset the recovery flag when done
    setOutput("recovery", false);

    // Abort recovery if new goal is received and return SUCCESS
    if (new_goal_) {
      // Reset everything
      new_goal_ = false;
      started_ = false;
      backed_off_ = false;
      agents_ptr_->resetAgents();
      BT_INFO(name_, "New Goal recieved during the recovery. Aborting backoff!")
      return BT::NodeStatus::SUCCESS;
    }

    // Otherwise, the recovery is complete, reset states and set back the goal
    new_goal_ = false;
    started_ = false;
    backed_off_ = false;

    agents_ptr_->resetAgents();
    backoff_ptr_->setbackGoal(current_goal_);
    setOutput("nav_goal", current_goal_);
    BT_INFO(name_, "Recovery is completed, setting back the goal!")

    return BT::NodeStatus::SUCCESS;
  }

  // Wait for the recovery behavior to complete
  BT_INFO(name_, "in Backoff recovery")
  return BT::NodeStatus::FAILURE;
}

bool BackoffExitCondition::isRecoveryComplete() {
  new_goal_ = false;
  getInput("agents_info", agents_info_);

  // Get the data from blackboard and start the recovery
  if (!started_) {
    getInput("backoff_ptr", backoff_ptr_);
    getInput("agents_ptr", agents_ptr_);
    setOutput("recovery", true);
    getInput("nav_goal", current_goal_);
    BT_INFO(name_, "Received navigation goal for recovery")
    started_ = backoff_ptr_->startRecovery();
    BT_INFO(name_, "Starting recovery!")
    return false;
  }

  // If a new goal is given, stop recovery
  if (started_ && backoff_ptr_->checkNewGoal()) {
    new_goal_ = true;
    getInput("nav_goal", current_goal_);
    BT_INFO(name_, "New goal!")
    return true;
  }

  // If timeout has been completed, reset goal
  if (started_ && backoff_ptr_->timeOut()) {
    BT_INFO(name_, "Time out!")
    return true;
  }

  // After reaching backoff goal, the human is not there anymore -> human moved away
  if (!backed_off_ && backoff_ptr_->isBackoffGoalReached()) {
    backed_off_ = true;
    if (!agents_ptr_->isAgentStuck()) {
      BT_INFO(name_, "Agent moved away while backoff is being performed!")
    }
    return !agents_ptr_->isAgentStuck();
  }

  // Stop waiting if the agent has cleared the way and return
  if (backed_off_ && !agents_ptr_->isAgentStuck()) {
    BT_INFO(name_, "Agent cleared the way!")
    return true;
  }

  return false;
}

};  // namespace hateb_local_planner