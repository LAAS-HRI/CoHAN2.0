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
#include <agent_path_prediction/AgentsInfo.h>
#include <agent_path_prediction/agents_class.h>
#include <hateb_local_planner/behavior_tree/bt_core.h>

#include <mutex>

namespace hateb_local_planner {

/**
 * @brief Condition node that checks if an obstacle (human) has stopped in the environment
 *
 * This class implements a behavior tree condition that monitors the velocity state of
 * nearby human agents and determines if any have come to a stop. It is used to detect
 * situations where the robot's path may be blocked by a stationary human obstacle.
 */
class VelObsExitCondition : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor for the VelObsExitCondition node
   * @param condition_name Name of the condition node
   * @param conf Node configuration containing the input/output ports
   */
  VelObsExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  VelObsExitCondition() = delete;

  /**
   * @brief Destructor
   */
  ~VelObsExitCondition() override;

  /**
   * @brief Main execution tick of the condition node
   * @return NodeStatus SUCCESS if a human has stopped, FAILURE otherwise
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines the input and output ports for the condition node
   * @return PortsList containing the node's ports configuration
   */
  static BT::PortsList providedPorts() {
    // This action has a single input port called "agents_info"
    return {BT::InputPort<agent_path_prediction::msg::AgentsInfo>("agents_info"), BT::InputPort<std::shared_ptr<agents::Agents>>("agents_ptr"), BT::OutputPort<int>("stuck_agent")};
  }

 private:
  /**
   * @brief Checks if any human agent in the environment has stopped moving
   * @return true if a human has stopped, false otherwise
   */
  bool hasHumanStopped();

  // bool isHumanPlaying(); // Add this later

  // Blackboard entries
  agent_path_prediction::msg::AgentsInfo agents_info_;  //!< Current information about all agents in the environment
  std::shared_ptr<agents::Agents> agents_ptr_;          //!< Pointer to the agents management class

  std::string name_;         //!< Name of this behavior tree node
  int nearest_human_id_;     //!< ID of the nearest human agent
  int t_stuck_;              //!< Time counter for how long an agent has been stuck
  std::mutex agents_mutex_;  //!< Mutex to protect concurrent access to agent data
};
};  // namespace hateb_local_planner