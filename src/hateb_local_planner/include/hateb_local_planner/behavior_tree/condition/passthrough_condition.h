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
#include <cohan_msgs/msg/passage_type.hpp>
#include <hateb_local_planner/agents_class.hpp>

namespace hateb_local_planner {

/**
 * @brief Class implementing a condition node for checking passthrough scenarios
 *
 * This class evaluates whether the robot is at a passage situation that requires special handling.
 * It inherits from BT::ConditionNode to integrate with the behavior tree framework.
 */
class PassThroughCondition : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor with condition name and configuration
   * @param condition_name Name of the condition node
   * @param conf Configuration for the behavior tree node
   */
  PassThroughCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  PassThroughCondition() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~PassThroughCondition() override;

  /**
   * @brief Method called to evaluate passage conditions
   * @return SUCCESS if in door/pillar passage, FAILURE otherwise
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines input port for passage type
   * @return Ports list containing passage type as input
   */
  static BT::PortsList providedPorts() {
    return {BT::InputPort<int8_t>("passage_type")};  //!< Type of passage (door, pillar, open, wall)
  }

 private:
  int8_t psg_type_;  //!< Current passage type being evaluated

  std::string name_;  //!< Name of the node
};
};  // namespace hateb_local_planner
