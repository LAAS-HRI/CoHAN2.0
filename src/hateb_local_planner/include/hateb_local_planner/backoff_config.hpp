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

#ifndef BACKOFF_CONFIG_H_
#define BACKOFF_CONFIG_H_

#include <rclcpp/rclcpp.hpp>
#include <ros2_helpers/parameters.hpp>
#include <string>

// Default parameter values
#define MAP_FRAME "map"
#define FOOTPRINT_FRAME "base_footprint"
#define PUBLISH_GOAL_TOPIC "/move_base_simple/goal"
#define GET_PLAN_SRV_NAME "/move_base/GlobalPlanner/make_plan"
#define CURRENT_GOAL_TOPIC_NAME "/move_base/current_goal"

namespace hateb_local_planner {

/**
 * @brief Configuration class for Backoff recovery behavior
 */
class BackoffConfig {
 public:
  BackoffConfig() = default;
  ~BackoffConfig() = default;

  /**
   * @brief Initializes the parameter helper for this config
   */
  void initialize(::SharedPtr node) {
    param_helper_.initialize(node);
    RCLCPP_INFO(node->get_logger(), "Initializing BackoffConfig parameters...");
  }

  /**
   * @brief Sets up parameter declarations and callback for parameter updates
   */
  void setupParameterCallback() {
    // Bind all parameters with automatic updates
    bindParameters();

    // Set up parameter change callback - parameters are auto-updated by bindings
    param_helper_.setupParameterCallback([this](const std::vector<rclcpp::Parameter>& params) -> bool {
      // Parameters are automatically updated by ParameterHelper bindings
      return true;
    });

    // Load initial parameter values
    param_helper_.loadBoundParameters();
  }

  // Frame IDs
  std::string map_frame;        //!< Name of the map frame
  std::string footprint_frame;  //!< Name of the robot's footprint frame

  // ROS topic and service names
  std::string ns;                  //!< Namespace for the node
  std::string publish_goal_topic;  //!< Topic name for publishing backoff goals
  std::string current_goal_topic;  //!< Topic name for current goal
  std::string get_plan_srv_name;   //!< Service name for path planning

  // Backoff behavior parameters
  double backoff_timeout;  //!< Maximum allowed time for backoff maneuver
  bool visualize_backoff;  //!< Enable visualization of backoff grids

 private:
  /**
   * @brief Binds all configuration variables to parameters for auto-update
   */
  void bindParameters() {
    // Set default values for parameters BEFORE binding
    ns = "";
    map_frame = MAP_FRAME;
    footprint_frame = FOOTPRINT_FRAME;
    publish_goal_topic = PUBLISH_GOAL_TOPIC;
    current_goal_topic = CURRENT_GOAL_TOPIC_NAME;
    get_plan_srv_name = GET_PLAN_SRV_NAME;
    backoff_timeout = 30.0;
    visualize_backoff = false;

    // Bind parameters
    param_helper_.bindStringParam("ns", ns, "Namespace for multiple robots");
    param_helper_.bindStringParam("map_frame", map_frame, "Frame ID for the map");
    param_helper_.bindStringParam("footprint_frame", footprint_frame, "Frame ID for the robot footprint");
    param_helper_.bindStringParam("publish_goal_topic", publish_goal_topic, "Topic name for publishing backoff goals");
    param_helper_.bindStringParam("current_goal_topic", current_goal_topic, "Topic name for current goal subscription");
    param_helper_.bindStringParam("get_plan_srv_name", get_plan_srv_name, "Service name for path planning");
    param_helper_.bindFloatParam("backoff_timeout", backoff_timeout, 0.0, 120.0, "Maximum allowed time for backoff maneuver (seconds)");
    param_helper_.bindBoolParam("visualize_backoff", visualize_backoff, "Enable visualization of backoff search grids");
  }

  parameters::ParameterHelper param_helper_;  //!< Parameter helper for managing ROS2 parameters
};

}  // namespace hateb_local_planner

#endif  // BACKOFF_CONFIG_H_
