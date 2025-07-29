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

#ifndef AGENT_PATH_PREDICTION_HPP_
#define AGENT_PATH_PREDICTION_HPP_

#include <agent_path_prediction/agents_class.hpp>
#include <agent_path_prediction/parameter_utils.hpp>
#include <agent_path_prediction/predict_goal.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ros
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

// msgs
#include <agent_path_prediction/msg/agent_pose.hpp>
#include <agent_path_prediction/msg/predicted_goal.hpp>
#include <agent_path_prediction/msg/predicted_goals.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// srvs
#include <agent_path_prediction/srv/agent_goal.hpp>
#include <agent_path_prediction/srv/agent_pose_predict.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// Some fixed parameters
#define ANG_VEL_EPS 0.001
#define MAX_AGENT_MARKERS 1000
#define MIN_MARKER_LIFETIME 1.0
#define MINIMUM_COVARIANCE_MARKERS 0.1
#define RECALC_DIST 0.5
#define DEFAULT_AGENT_PART cohan_msgs::TrackedSegmentType::TORSO
#define NODE_NAME "agent_path_prediction"

// Reconfigurable Parameters
#define EXTERNAL_PATHS_SUB_TOPIC "external_agent_paths"
#define PREDICTED_GOAL_SUB_TOPIC "predicted_goal"
#define AGENTS_SUB_TOPIC "/tracked_agents"
#define GET_PLAN_SRV_NAME "/move_base/GlobalPlanner/make_plan"
#define ROBOT_FRAME_ID "base_footprint"
#define MAP_FRAME_ID "map"
#define AGENT_DIST_BEHIND_ROBOT 0.5
#define AGENT_ANGLE_BEHIND_ROBOT 3.14

namespace agents {
class AgentPathPrediction {
 public:
  AgentPathPrediction(std::shared_ptr<rclcpp::Node> node) : node_(node), param_helper_(node) {
    RCLCPP_INFO(node_->get_logger(), "AgentPathPrediction initialized");
    setupParameterCallback();
  };

  ~AgentPathPrediction() = default;

  void initialize();

 private:
  // Structs
  struct AgentPathVel {
    uint64_t id;
    nav_msgs::msg::Path path;
    geometry_msgs::msg::TwistWithCovariance start_vel;
  };

  struct AgentStartPoseVel {
    uint64_t id;
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::TwistWithCovariance vel;
  };

  // Internal Methods
  static nav_msgs::msg::Path setFixedPath(const geometry_msgs::msg::PoseStamped &start_pose);
  static size_t prunePath(size_t begin_index, const geometry_msgs::msg::Pose &pose, const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> &path);
  bool transformPoseTwist(const cohan_msgs::msg::TrackedAgents &tracked_agents, const uint64_t &agent_id, const std::string &to_frame, geometry_msgs::msg::PoseStamped &pose,
                          geometry_msgs::msg::TwistStamped &twist) const;
  geometry_msgs::msg::Twist getRelativeTwist(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &time_now, const rclcpp::Duration &duration) const;
  static double checkdist(geometry_msgs::msg::Pose agent, geometry_msgs::msg::Pose robot) { return std::hypot(agent.position.x - robot.position.x, agent.position.y - robot.position.y); }

  // subscriber callbacks
  void trackedAgentsCB(const cohan_msgs::msg::TrackedAgents &tracked_agents);
  void externalPathsCB(const cohan_msgs::msg::AgentPathArray::ConstPtr &external_paths);
  void predictedGoalCB(const agent_path_prediction::msg::PredictedGoals::ConstPtr &predicted_goal);

  // Service callbacks
  void predictAgents(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);
  void predictAgentsVelObs(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) const;
  void predictAgentsExternal(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);
  void predictAgentsBehind(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);
  void predictAgentsGoal(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);
  void setGoal(const std::shared_ptr<agent_path_prediction::srv::AgentGoal::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentGoal::Response> res);
  void resetPredictionSrvs(const std::shared_ptr<std_srvs::Empty::Request> req, std::shared_ptr<std_srvs::Empty::Response> res);
  void predictAgentsFromPaths(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);

  void setupParameterCallback() {
    // Declare parameters using the generic helper
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

    // Set up parameter change callback with custom validation
    param_helper_.setupParameterCallback([this](const std::vector<rclcpp::Parameter> &params) -> bool {
      // Custom parameter validation logic for this specific node
      for (const auto &param : params) {
        const std::string &name = param.get_name();

        // Update internal variables when parameters change
        if (name == "velobs_mul")
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
      }
      return true;
    });

    // Load initial parameter values
    loadParameters();
  }

  // parameters
  void loadParameters() {
    // Get parameter values and store them in member variables
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
  }

  // Properties
  parameter_utils::ParameterHelper param_helper_;
  cohan_msgs::msg::TrackedAgents tracked_agents_;
  cohan_msgs::msg::AgentPathArray::ConstPtr external_paths_;
  agent_path_prediction::msg::PredictedGoals predicted_goals_;
  std::vector<agent_path_prediction::msg::AgentPose> external_goals_;
  std::vector<agent_path_prediction::msg::PredictedPoses> last_predicted_poses_;
  visualization_msgs::msg::MarkerArray predicted_agents_markers_;
  geometry_msgs::msg::Transform behind_pose_;
  std::vector<AgentPathVel> path_vels_;
  std::vector<int> path_vels_pos_;
  std::map<uint64_t, size_t> last_prune_indices_;
  std::map<uint64_t, int> last_markers_size_map_;
  std::string tracked_agents_sub_topic_, external_paths_sub_topic_, predict_service_name_, predicted_agents_markers_pub_topic_, get_plan_srv_name_, predicted_goal_topic_;
  std::string robot_frame_id_, map_frame_id_;
  double velobs_mul_, velobs_min_rad_, velobs_max_rad_, velobs_max_rad_time_;
  double agent_dist_behind_robot_, agent_angle_behind_robot_;
  bool velobs_use_ang_, check_path_;
  bool publish_markers_, showing_markers_, got_new_agent_paths_, got_external_goal_;
  int default_agent_part_;

  // ROS 2
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predicted_agents_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr front_pose_pub_;
  rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr tracked_agents_sub_;
  rclcpp::Subscription<cohan_msgs::msg::AgentPathArray>::SharedPtr external_paths_sub_;
  rclcpp::Subscription<agent_path_prediction::msg::PredictedGoals>::SharedPtr predicted_goal_sub_;
  rclcpp::Service<agent_path_prediction::srv::AgentPosePredict>::SharedPtr predict_agents_server_;
  rclcpp::Service<agent_path_prediction::srv::AgentGoal>::SharedPtr set_goal_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_prediction_services_server_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr get_plan_client_;
  auto tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  std::shared_ptr<rclcpp::Node> node_;
};
}  // namespace agents

#endif  // AGENT_PATH_PREDICTION_HPP_