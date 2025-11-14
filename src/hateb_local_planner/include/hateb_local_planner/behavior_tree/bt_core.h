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

#ifndef BT_CORE_HH_
#define BT_CORE_HH_

/**
 * @brief Core header file for behavior tree implementation in HATEB local planner
 *
 * This file contains the core definitions and components for implementing behavior trees
 * in the HATEB (Human-Aware Timed Elastic Band) local planner. It provides classes and
 * utilities for creating and managing ROS-integrated behavior tree nodes.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "behaviortree_cpp/bt_factory.h"

/**
 * @brief Constants for numerical precision and distance thresholds
 */
#define EPS 0.01       // Small epsilon value for floating-point comparisons
#define DIST_EPS 0.06  // Distance threshold for proximity checks

/**
 * @brief Debug printing configuration and macros
 * When BTPRINT is enabled (via CMake option ENABLE_BT_DEBUG), provides colored console output for different message types:
 * - BT_INFO: Regular informational messages
 * - BT_WARN: Warning messages (yellow)
 * - BT_ERROR: Error messages (red)
 *
 * Note: BTPRINT is defined by CMake during compilation. To enable, build with:
 * colcon build --cmake-args -DENABLE_BT_DEBUG=ON
 */
#ifndef BTPRINT
#define BTPRINT 0  // Default to disabled if not set by CMake
#endif

#if BTPRINT
#define BT_INFO(x, y) std::cout << "BT_INFO: " << x << " -> " << y << std::endl;

#define BT_WARN(x, y)                                           \
  std::cout << "\033[33m";                                      \
  std::cout << "BT_WARNING: " << x << " -> " << y << std::endl; \
  std::cout << "\033[0m";

#define BT_ERROR(x, y)                                        \
  std::cout << "\033[31m";                                    \
  std::cout << "BT_ERROR: " << x << " -> " << y << std::endl; \
  std::cout << "\033[0m";

#else
#define BT_INFO(x, y)
#define BT_WARN(x, y)
#define BT_ERROR(x, y)
#endif

/**
 * @brief Normalizes an angle to the range [-π, π]
 * @param angle_radians The input angle to normalize
 * @return The normalized angle in radians
 */
inline double normalize_angle(double angle_radians) { return angle_radians - (2.0 * M_PI * std::floor((angle_radians + (M_PI)) / (2.0 * M_PI))); }

/*This part of the code is inspired from here: https://github.com/BehaviorTree/BehaviorTree.ROS (bt_action_node.hh)*/
namespace hateb_local_planner {

/**
 * @brief Base class for stateful action nodes in the behavior tree
 *
 * This class provides the foundation for creating ROS-integrated behavior tree nodes
 * that maintain state between executions. It inherits from BT::StatefulActionNode
 * and provides integration with ROS node handles.
 */
class StatefulActionNodeROS : public BT::StatefulActionNode {
 protected:
  /**
   * @brief Constructor initializes the node with ROS and BT configurations
   * @param node ROS2 node shared pointer for communication
   * @param name Name of the behavior tree node
   * @param conf Configuration for the behavior tree node
   */
  StatefulActionNodeROS(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string& name, const BT::NodeConfiguration& conf) : BT::StatefulActionNode(name, conf), node_(node) {}

 public:
  // using BaseClass = StatefulActionNodeROS<ActionT>;
  // using ActionType = ActionT;

  StatefulActionNodeROS() = delete;

  ~StatefulActionNodeROS() override = default;

  /**
   * @brief Defines the input/output ports for the node
   * @return List of ports including the action_name input port
   */
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("action_name")}; }

  /**
   * @brief Called when the node starts execution
   * @return Status of the node after starting
   */
  BT::NodeStatus onStart() override = 0;

  /**
   * @brief Called during node execution
   * @return Current status of the node
   */
  BT::NodeStatus onRunning() override = 0;

  /**
   * @brief Called when the node is halted
   */
  void onHalted() override = 0;

 protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;  // ROS2 node shared pointer for communication
};

/**
 * @brief Registers a stateful action node with the behavior tree factory
 *
 * This template function creates and registers a builder for derived node types,
 * allowing them to be instantiated by the behavior tree factory.
 *
 * @param factory The behavior tree factory to register with
 * @param registration_ID Unique identifier for the node type
 * @param node ROS2 node shared pointer for communication
 */
template <class DerivedT>
static void RegisterStatefulActionNodeROS(BT::BehaviorTreeFactory& factory, const std::string& registration_ID, rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
  BT::NodeBuilder builder = [node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<DerivedT>(node, name, config); };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = StatefulActionNodeROS::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
};

/**
 * @brief Planning mode enumeration
 * Defines different modes of operation for the local planner
 */
enum PLAN : std::uint8_t {
  SINGLE_BAND,  //!< Single elastic band optimization
  DUAL_BAND,    //!< Dual elastic band optimization for robot and human
  VELOBS,       //!< Velocity obstacles-based planning
  BACKOFF,      //!< Backoff behavior when stuck
  PASSTHROUGH   //!< PassThrough Mode at passages
};

/**
 * @brief Prediction mode enumeration
 * Defines different strategies for predicting human motion
 */
enum PREDICTION : std::uint8_t {
  CONST_VEL,  //!< Constant velocity prediction
  BEHIND,     //!< Predicting the goal as behind the robot
  PREDICT,    //!< Use internal goal prediction scheme
  EXTERNAL    //!< External prediction provided the human goals
};

/**
 * @brief Structure combining planning and prediction modes
 * Used to configure the behavior of the local planner
 */
// 'Using' leads to linkage errors
typedef struct {
  PLAN plan;           //!< Selected planning mode
  PREDICTION predict;  //!< Selected prediction strategy
} ModeInfo;

}  // namespace hateb_local_planner

#endif  // BT_CORE_HH_
