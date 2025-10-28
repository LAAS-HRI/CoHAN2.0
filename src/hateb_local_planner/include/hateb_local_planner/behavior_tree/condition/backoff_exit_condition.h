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

#include <tf2/utils.h>

#include <rclcpp/rclcpp.hpp>

// New
#include <hateb_local_planner/backoff.h>
#include <hateb_local_planner/behavior_tree/bt_core.h>

#include <agent_path_prediction/agents_class.hpp>
#include <agent_path_prediction/msg/agents_info.hpp>

namespace hateb_local_planner {

/**
 * @brief Class implementing a condition node for managing backoff recovery behavior
 *
 * This class provides functionality to control when to exit the backoff recovery mode.
 * It monitors agent states and recovery progress to determine when to resume normal navigation.
 * It inherits from BT::ConditionNode to integrate with the behavior tree framework.
 */
class BackoffExitCondition : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor with condition name and configuration
   * @param condition_name Name of the condition node
   * @param conf Configuration for the behavior tree node
   */
  BackoffExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  BackoffExitCondition() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~BackoffExitCondition() override;

  /**
   * @brief Method called to evaluate the condition
   * @return SUCCESS if backoff recovery should be exited, FAILURE otherwise
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines input and output ports for recovery condition
   * @return Ports list containing agents_info, backoff_ptr, nav_goal, agents_ptr as inputs,
   *         and recovery status as output
   */
  static BT::PortsList providedPorts() {
    return {BT::InputPort<agent_path_prediction::msg::AgentsInfo>("agents_info"), BT::InputPort<std::shared_ptr<Backoff>>("backoff_ptr"),
            BT::BidirectionalPort<geometry_msgs::msg::PoseStamped>("nav_goal"), BT::InputPort<std::shared_ptr<agents::Agents>>("agents_ptr"), BT::OutputPort<bool>("recovery")};
  }

 private:
  /**
   * @brief Checks if the recovery behavior is complete
   * @return True if recovery is complete, false otherwise
   */
  bool isRecoveryComplete();

  // Blackboard entries
  agent_path_prediction::msg::AgentsInfo agents_info_;    //!< Information about agents in the environment
  geometry_msgs::msg::PoseStamped current_goal_;          //!< Current navigation goal
  std::shared_ptr<Backoff> backoff_ptr_ = nullptr;        //!< Pointer to backoff behavior handler
  std::shared_ptr<agents::Agents> agents_ptr_ = nullptr;  //!< Pointer to agents manager
  int stuck_agent_;                                       //!< ID of the agent that is stuck
  double dist_max_;                                       //!< Maximum distance threshold

  std::string name_;  //!< Name of the node

  bool started_;     //!< Flag indicating if recovery has started
  bool new_goal_;    //!< Flag indicating if a new goal was received
  bool backed_off_;  //!< Flag indicating if robot has backed off
};
};  // namespace hateb_local_planner