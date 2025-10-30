/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2020-2025 LAAS-CNRS
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

#ifndef AGENTS_CONFIG_HPP_
#define AGENTS_CONFIG_HPP_
#include <cmath>
#include <cohan_msgs/msg/tracked_segment_type.hpp>
#include <ros2_helpers/parameters.hpp>

#define WINDOW_MOVING_AVG 5
#define HUM_RADIUS 0.3
#define ROBOT_RADIUS 0.35
#define PLANNING_RADIUS 10.0
#define BASE_LINK_FRAME "base_link"
#define MAP_FRAME "map"
#define ODOM_FRAME "odom"

namespace hateb_local_planner {
class AgentsConfig {
 public:
  AgentsConfig() = default;

  ~AgentsConfig() = default;

  /**
   * @brief Initializes the parameter helper for this config
   */
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
    param_helper_.initialize(node);
    RCLCPP_INFO(node->get_logger(), "Initializing AgentsConfig parameters...");
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

  double inflation_radius;      //!< Inflation radius for local costmap
  bool use_simulated_fov;       //!< Flag to use simulated field of view
  int window_moving_avg;        //!< Window size for moving average filter
  double human_radius;          //!< Radius of human agents
  double robot_radius;          //!< Radius of the robot
  std::string base_link_frame;  //!< Frame ID for robot base link
  std::string odom_frame;       //!< Frame ID for odometry
  double planning_radius;       //!< Radius used for planning

 private:
  /**
   * @brief Binds all configuration variables to parameters for auto-update
   */
  void bindParameters() {
    // Set default values for parameters BEFORE binding
    use_simulated_fov = false;
    base_link_frame = BASE_LINK_FRAME;
    odom_frame = ODOM_FRAME;
    planning_radius = PLANNING_RADIUS;
    inflation_radius = 0.1;
    window_moving_avg = WINDOW_MOVING_AVG;
    human_radius = HUM_RADIUS;
    robot_radius = ROBOT_RADIUS;

    // ROS topic names and service names
    param_helper_.bindBoolParam("use_simulated_fov", use_simulated_fov, "Flag for using simulated field of view");
    param_helper_.bindStringParam("base_link_frame", base_link_frame, "Frame ID for robot base link");
    param_helper_.bindStringParam("odom_frame", odom_frame, "Frame ID for odometry");

    param_helper_.bindFloatParam("planning_radius", planning_radius, 0.1, 10.0, "Radius used for planning");
    param_helper_.bindFloatParam("local_costmap/inflater/inflation_radius", inflation_radius, 0.0, 10.0, "Inflation radius for local costmap");
    param_helper_.bindIntParam("window_moving_avg", window_moving_avg, 1, 20, "Window size for moving average filter");
    param_helper_.bindFloatParam("HATebLocalPlannerROS/agent_radius", human_radius, 0.1, 2.0, "Radius of human agents");
    param_helper_.bindFloatParam("HATebLocalPlannerROS/robot_radius", robot_radius, 0.1, 2.0, "Radius of the robot");
  }

  parameters::ParameterHelper param_helper_;  //!< Parameter helper for managing ROS2 parameters
};
}  // namespace hateb_local_planner
#endif  // AGENTS_CONFIG_HPP_