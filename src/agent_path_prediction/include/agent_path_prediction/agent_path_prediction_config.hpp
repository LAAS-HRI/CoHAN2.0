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
#define GET_PLAN_SRV_NAME "/move_base/GlobalPlanner/make_plan"
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
    // Declare parameters using the generic helper
    param_helper_.declareStringParam("ns", "", "Namespace for multiple agents");
    param_helper_.declareFloatParam("velobs_mul", 1.0, 0.001, 10.0, "Multiplier for agent velocities for velocity-obstacle calculation");
    param_helper_.declareFloatParam("velobs_min_rad", 0.25, 0.0, 10.0, "Minimum radius for velocity-obstacle calculation");
    param_helper_.declareFloatParam("velobs_max_rad", 0.75, 0.0, 10.0, "Maximum radius for velocity-obstacle calculation");
    param_helper_.declareFloatParam("velobs_max_rad_time", 4.0, 0.0, 60.0, "Time for maximum radius for velocity-obstacle calculation");
    param_helper_.declareBoolParam("velobs_use_ang", true, "Whether to use angular velocity for velocity-obstacle calculation");
    param_helper_.declareBoolParam("publish_markers", true, "Whether to publish visualization markers for predicted agent poses");
    param_helper_.declareStringParam("robot_frame_id", ROBOT_FRAME_ID, "Frame ID for the robot base");
    param_helper_.declareStringParam("map_frame_id", MAP_FRAME_ID, "Frame ID for the map");
    param_helper_.declareFloatParam("agent_dist_behind_robot", AGENT_DIST_BEHIND_ROBOT, 0.0, 10.0, "Distance behind the robot where agents should be positioned");
    param_helper_.declareFloatParam("agent_angle_behind_robot", AGENT_ANGLE_BEHIND_ROBOT, -M_PI, M_PI, "Angle behind the robot where agents should be positioned (radians)");
    param_helper_.declareStringParam("tracked_agents_sub_topic", AGENTS_SUB_TOPIC, "Topic name for subscribing to tracked agents");
    param_helper_.declareStringParam("external_paths_sub_topic", EXTERNAL_PATHS_SUB_TOPIC, "Topic name for subscribing to external agent paths");
    param_helper_.declareStringParam("predicted_goal_topic", PREDICTED_GOAL_SUB_TOPIC, "Topic name for subscribing to predicted goals");
    param_helper_.declareStringParam("get_plan_srv_name", GET_PLAN_SRV_NAME, "Service name for path planning");
    param_helper_.declareIntParam("default_agent_part", DEFAULT_AGENT_PART, 0, 10, "Default agent body part to track");
    param_helper_.declareStringParam("goals_file", "", "Path to the goals YAML file");

    // Set up parameter change callback with custom validation
    param_helper_.setupParameterCallback([this](const std::vector<rclcpp::Parameter>& params) -> bool {
      // Custom parameter validation logic for this specific node
      for (const auto& param : params) {
        const std::string& name = param.get_name();

        // Update internal variables when parameters change
        if (name == "ns")
          ns_ = param.as_string();
        else if (name == "velobs_mul")
          velobs_mul_ = param.as_double();
        else if (name == "velobs_min_rad")
          velobs_min_rad_ = param.as_double();
        else if (name == "velobs_max_rad")
          velobs_max_rad_ = param.as_double();
        else if (name == "velobs_max_rad_time")
          velobs_max_rad_time_ = param.as_double();
        else if (name == "velobs_use_ang")
          velobs_use_ang_ = param.as_bool();
        else if (name == "publish_markers")
          publish_markers_ = param.as_bool();
        else if (name == "robot_frame_id")
          robot_frame_id_ = param.as_string();
        else if (name == "map_frame_id")
          map_frame_id_ = param.as_string();
        else if (name == "agent_dist_behind_robot")
          agent_dist_behind_robot_ = param.as_double();
        else if (name == "agent_angle_behind_robot")
          agent_angle_behind_robot_ = param.as_double();
        else if (name == "tracked_agents_sub_topic")
          tracked_agents_sub_topic_ = param.as_string();
        else if (name == "external_paths_sub_topic")
          external_paths_sub_topic_ = param.as_string();
        else if (name == "predicted_goal_topic")
          predicted_goal_topic_ = param.as_string();
        else if (name == "get_plan_srv_name")
          get_plan_srv_name_ = param.as_string();
        else if (name == "default_agent_part")
          default_agent_part_ = param.as_int();
        else if (name == "goals_file") {
          goals_file_ = param.as_string();
        }
      }
      return true;
    });

    // Load initial parameter values
    loadParameters();
  }

 private:
  /**
   * @brief Loads and initializes all parameters from ROS2 parameter server
   */
  void loadParameters() {
    // Get parameter values and store them in member variables
    ns_ = param_helper_.getParam<std::string>("ns", "");
    publish_markers_ = param_helper_.getParam<bool>("publish_markers", true);
    robot_frame_id_ = param_helper_.getParam<std::string>("robot_frame_id", ROBOT_FRAME_ID);
    map_frame_id_ = param_helper_.getParam<std::string>("map_frame_id", MAP_FRAME_ID);
    agent_dist_behind_robot_ = param_helper_.getParam<double>("agent_dist_behind_robot", AGENT_DIST_BEHIND_ROBOT);
    agent_angle_behind_robot_ = param_helper_.getParam<double>("agent_angle_behind_robot", AGENT_ANGLE_BEHIND_ROBOT);
    tracked_agents_sub_topic_ = param_helper_.getParam<std::string>("tracked_agents_sub_topic", AGENTS_SUB_TOPIC);
    external_paths_sub_topic_ = param_helper_.getParam<std::string>("external_paths_sub_topic", EXTERNAL_PATHS_SUB_TOPIC);
    predicted_goal_topic_ = param_helper_.getParam<std::string>("predicted_goal_topic", PREDICTED_GOAL_SUB_TOPIC);
    get_plan_srv_name_ = param_helper_.getParam<std::string>("get_plan_srv_name", GET_PLAN_SRV_NAME);
    default_agent_part_ = param_helper_.getParam<int>("default_agent_part", DEFAULT_AGENT_PART);
    velobs_mul_ = param_helper_.getParam<double>("velobs_mul", 1.0);
    velobs_min_rad_ = param_helper_.getParam<double>("velobs_min_rad", 0.25);
    velobs_max_rad_ = param_helper_.getParam<double>("velobs_max_rad", 0.75);
    velobs_max_rad_time_ = param_helper_.getParam<double>("velobs_max_rad_time", 4.0);
    velobs_use_ang_ = param_helper_.getParam<bool>("velobs_use_ang", true);
    goals_file_ = param_helper_.getParam<std::string>("goals_file", "");
  }

  parameters::ParameterHelper param_helper_;  //!< Parameter helper for managing ROS2 parameters

 public:
  // ROS topic names
  std::string tracked_agents_sub_topic_;            //!< Topic for tracked agents subscription
  std::string external_paths_sub_topic_;            //!< Topic for external agent paths subscription
  std::string predict_service_name_;                //!< Service name for prediction
  std::string predicted_agents_markers_pub_topic_;  //!< Topic for predicted agent markers publication
  std::string get_plan_srv_name_;                   //!< Service name for getting navigation plan
  std::string predicted_goal_topic_;                //!< Topic for predicted goal subscription
  std::string goals_file_;                          //!< File path for goals YAML file

  // Frame IDs
  std::string robot_frame_id_;  //!< Frame ID for robot base coordinate frame
  std::string map_frame_id_;    //!< Frame ID for map coordinate frame

  // Velocity obstacle parameters
  double velobs_mul_;           //!< Velocity obstacle multiplier
  double velobs_min_rad_;       //!< Minimum radius for velocity obstacles
  double velobs_max_rad_;       //!< Maximum radius for velocity obstacles
  double velobs_max_rad_time_;  //!< Maximum time for velocity obstacle radius calculation
  bool velobs_use_ang_;         //!< Flag to use angular velocity in velocity obstacles

  // Agent detection parameters
  double agent_dist_behind_robot_;   //!< Distance behind robot for agent detection
  double agent_angle_behind_robot_;  //!< Angle behind robot for agent detection (radians)
  int default_agent_part_;           //!< Default body part to track for agents

  // Visualization and configuration
  bool publish_markers_;  //!< Flag to enable marker visualization
  std::string ns_;        //!< Namespace for the node
};
}  // namespace agents

#endif  // AGENT_PATH_PREDICTION_CONFIG_H_