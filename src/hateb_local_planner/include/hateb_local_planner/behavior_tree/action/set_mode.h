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

#include <hateb_local_planner/behavior_tree/bt_core.h>

#include <hateb_local_planner/msg/planning_mode.hpp>
#include <memory>
#include <string>

namespace hateb_local_planner {

/**
 * @brief Class implementing a behavior tree action node to set planning mode
 *
 * This class provides functionality to set planning modes in CoHAN.
 * It inherits from StatefulActionNodeROS to integrate with ROS and behavior tree framework.
 */
class SetMode : public BT::StatefulActionNode {
 public:
  /**
   * @brief Constructor with ROS2 node and behavior tree configuration
   * @param node The ROS2 node shared pointer for communication
   * @param name Name of the behavior tree node
   * @param config Configuration for the behavior tree node
   */
  SetMode(const std::string& name, const BT::NodeConfiguration& config);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  SetMode() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~SetMode() override;

  /**
   * @brief Defines input and output ports for the behavior tree node
   * @return Ports list containing plan_type and predict_type as inputs and mode as output
   */
  static BT::PortsList providedPorts() {
    // define the input and output ports
    return {BT::InputPort<std::string>("plan_type"), BT::InputPort<std::string>("predict_type"), BT::OutputPort<ModeInfo>("mode")};
  }

  /**
   * @brief Method called when the node is first ticked
   * @return Status of the node after starting
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Method called while the node is running
   * @return Current status of the node
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Method called when the node is halted
   */
  void onHalted() override;

 private:
  std::string name_;          //!< Name of the behavior tree node
  std::string plan_type_;     //!< Type of planning mode to use
  std::string predict_type_;  //!< Type of prediction to use
  ModeInfo p_msg_;            //!< Message containing mode information
};
};  // namespace hateb_local_planner