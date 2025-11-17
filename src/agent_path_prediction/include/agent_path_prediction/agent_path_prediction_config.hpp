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

#ifndef AGENT_PATH_PREDICTION_CONFIG_H_
#define AGENT_PATH_PREDICTION_CONFIG_H_
#include <cmath>
#include <cohan_msgs/msg/tracked_segment_type.hpp>
#include <ros2_helpers/parameters.hpp>

// Reconfigurable Parameters
#define EXTERNAL_PATHS_SUB_TOPIC "~/external_agent_paths"
#define PREDICTED_GOAL_SUB_TOPIC "~/predicted_goal"
#define AGENTS_SUB_TOPIC "/tracked_agents"
#define GET_PLAN_SRV_NAME "/agent_planner/compute_path_to_pose"
#define ROBOT_FRAME_ID "base_footprint"
#define MAP_FRAME_ID "map"
#define AGENT_DIST_BEHIND_ROBOT 0.5
#define AGENT_ANGLE_BEHIND_ROBOT M_PI
#define DEFAULT_AGENT_PART cohan_msgs::msg::TrackedSegmentType::TORSO

namespace agents {
class AgentPathPredictConfig {
 public:
  AgentPathPredictConfig() = default;

  ~AgentPathPredictConfig() = default;

  /**
   * @brief Initializes the parameter helper for this config
   */
  void initialize(rclcpp::Node::SharedPtr node) {
    param_helper_.initialize(node);
    RCLCPP_INFO(node->get_logger(), "Initializing AgentPathPredictConfig parameters...");
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

  // ROS topic names
  std::string tracked_agents_sub_topic;  //!< Topic for tracked agents subscription
  std::string external_paths_sub_topic;  //!< Topic for external agent paths subscription
  std::string get_plan_srv_name;         //!< Service name for getting navigation plan
  std::string predicted_goal_topic;      //!< Topic for predicted goal subscription
  std::string goals_file;                //!< File path for goals YAML file

  // Frame IDs
  std::string robot_frame_id;  //!< Frame ID for robot base coordinate frame
  std::string map_frame_id;    //!< Frame ID for map coordinate frame

  // Velocity obstacle parameters
  double velobs_mul;           //!< Velocity obstacle multiplier
  double velobs_min_rad;       //!< Minimum radius for velocity obstacles
  double velobs_max_rad;       //!< Maximum radius for velocity obstacles
  double velobs_max_rad_time;  //!< Maximum time for velocity obstacle radius calculation
  bool velobs_use_ang;         //!< Flag to use angular velocity in velocity obstacles

  // Agent detection parameters
  double agent_dist_behind_robot;   //!< Distance behind robot for agent detection
  double agent_angle_behind_robot;  //!< Angle behind robot for agent detection (radians)
  int default_agent_part;           //!< Default body part to track for agents

  // Visualization and configuration
  bool publish_markers;  //!< Flag to enable marker visualization
  std::string ns;        //!< Namespace for the node

 private:
  /**
   * @brief Binds all configuration variables to parameters for auto-update
   */
  void bindParameters() {
    // Set default values for parameters BEFORE binding
    ns = "";
    tracked_agents_sub_topic = AGENTS_SUB_TOPIC;
    external_paths_sub_topic = EXTERNAL_PATHS_SUB_TOPIC;
    predicted_goal_topic = PREDICTED_GOAL_SUB_TOPIC;
    get_plan_srv_name = GET_PLAN_SRV_NAME;
    goals_file = "";
    robot_frame_id = ROBOT_FRAME_ID;
    map_frame_id = MAP_FRAME_ID;
    velobs_mul = 1.0;
    velobs_min_rad = 0.25;
    velobs_max_rad = 0.75;
    velobs_max_rad_time = 4.0;
    velobs_use_ang = true;
    agent_dist_behind_robot = AGENT_DIST_BEHIND_ROBOT;
    agent_angle_behind_robot = AGENT_ANGLE_BEHIND_ROBOT;
    default_agent_part = DEFAULT_AGENT_PART;
    publish_markers = true;

    // ROS topic names and service names
    param_helper_.bindStringParam("ns", ns, "Namespace for multiple agents");
    param_helper_.bindStringParam("tracked_agents_sub_topic", tracked_agents_sub_topic, "Topic name for subscribing to tracked agents");
    param_helper_.bindStringParam("external_paths_sub_topic", external_paths_sub_topic, "Topic name for subscribing to external agent paths");
    param_helper_.bindStringParam("predicted_goal_topic", predicted_goal_topic, "Topic name for subscribing to predicted goals");
    param_helper_.bindStringParam("get_plan_srv_name", get_plan_srv_name, "Service name for path planning");
    param_helper_.bindStringParam("goals_file", goals_file, "Path to the goals YAML file");

    // Frame IDs
    param_helper_.bindStringParam("robot_frame_id", robot_frame_id, "Frame ID for the robot base");
    param_helper_.bindStringParam("map_frame_id", map_frame_id, "Frame ID for the map");

    // Velocity obstacle parameters
    param_helper_.bindFloatParam("velobs_mul", velobs_mul, 0.001, 10.0, "Multiplier for agent velocities for velocity-obstacle calculation");
    param_helper_.bindFloatParam("velobs_min_rad", velobs_min_rad, 0.0, 10.0, "Minimum radius for velocity-obstacle calculation");
    param_helper_.bindFloatParam("velobs_max_rad", velobs_max_rad, 0.0, 10.0, "Maximum radius for velocity-obstacle calculation");
    param_helper_.bindFloatParam("velobs_max_rad_time", velobs_max_rad_time, 0.0, 60.0, "Time for maximum radius for velocity-obstacle calculation");
    param_helper_.bindBoolParam("velobs_use_ang", velobs_use_ang, "Whether to use angular velocity for velocity-obstacle calculation");

    // Agent detection parameters
    param_helper_.bindFloatParam("agent_dist_behind_robot", agent_dist_behind_robot, 0.0, 10.0, "Distance behind the robot where agents should be positioned");
    param_helper_.bindFloatParam("agent_angle_behind_robot", agent_angle_behind_robot, -M_PI, M_PI, "Angle behind the robot where agents should be positioned (radians)");
    param_helper_.bindIntParam("default_agent_part", default_agent_part, 0, 10, "Default agent body part to track");

    // Visualization
    param_helper_.bindBoolParam("publish_markers", publish_markers, "Whether to publish visualization markers for predicted agent poses");
  }

  parameters::ParameterHelper param_helper_;  //!< Parameter helper for managing ROS2 parameters
};
}  // namespace agents

#endif  // AGENT_PATH_PREDICTION_CONFIG_H_