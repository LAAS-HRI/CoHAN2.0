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
#include <hateb_local_planner/behavior_tree/bt_core.h>

#include <agent_path_prediction/msg/agents_info.hpp>
#include <hateb_local_planner/agents_class.hpp>

namespace hateb_local_planner {

/**
 * @brief Class implementing a condition node for dual band exit criteria
 *
 * This class checks conditions to determine when to exit the dual band planning mode.
 * It monitors robot's progress and agent states to make this decision.
 * It inherits from BT::ConditionNode to integrate with the behavior tree framework.
 */
class DualBandExitCondition : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor with condition name and configuration
   * @param condition_name Name of the condition node
   * @param conf Configuration for the behavior tree node
   */
  DualBandExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  DualBandExitCondition() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~DualBandExitCondition() override;

  /**
   * @brief Method called to evaluate the condition
   * @return SUCCESS if dual band mode should be exited, FAILURE otherwise
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines input ports for condition evaluation
   * @return Ports list containing agents_info, distance threshold and navigation goal as inputs
   */
  static BT::PortsList providedPorts() {
    // This action has a single input port called "agents_info"
    return {BT::InputPort<agent_path_prediction::msg::AgentsInfo>("agents_info"), BT::InputPort<double>("dist_threshold"), BT::InputPort<geometry_msgs::msg::PoseStamped>("nav_goal")};
  }

 private:
  /**
   * @brief Checks if the robot has been stuck in place
   * @return True if robot hasn't made progress, false otherwise
   */
  bool isRobotStuck();

  // Blackboard entries
  agent_path_prediction::msg::AgentsInfo agents_info_;  //!< Information about agents in the environment
  geometry_msgs::msg::PoseStamped goal_;                //!< Current navigation goal
  double dist_threshold_;                               //!< Distance threshold for dual band mode

  // Class Variables
  double goal_dist_;           //!< Distance to goal for progress tracking
  rclcpp::Clock clock_;        // clock instance for time tracking
  rclcpp::Time stopped_time_;  //!< Time when robot was last detected as stopped

  std::string name_;  //!< Name of the node
};
};  // namespace hateb_local_planner
