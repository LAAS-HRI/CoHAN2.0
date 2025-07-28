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
#include <agent_path_prediction/predict_goal.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ros
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

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
  AgentPathPrediction(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    RCLCPP_INFO(node_->get_logger(), "MyNodeWrapper initialized");
    setupParameterCallback();
  };

  ~AgentPathPrediction() = default;

  void initialize();

  void setupParameterCallback() {
    declareFloatParam("velobs_mul", 1.0, 0.001, 10.0, "Multiplier for agent velocities for velocity-obstacle calculation");
    declareFloatParam("velobs_min_rad", 0.25, 0.0, 10.0, "Minimum radius for velocity-obstacle calculation");
    declareFloatParam("velobs_max_rad", 0.75, 0.0, 10.0, "Maximum radius for velocity-obstacle calculation");
    declareFloatParam("velobs_max_rad_time", 4.0, 0.0, 60.0, "Time for maximum radius for velocity-obstacle calculation");
    declareBoolParam("velobs_use_ang", true, "Whether to use angular velocity for velocity-obstacle calculation");
    this->set_on_parameters_set_callback([this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
      for (const auto &param : params) {
        RCLCPP_DEBUG(this->get_logger(), "Parameter '%s' changed", param.get_name().c_str());
        // You can also validate or reject changes here
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });
  }

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
  geometry_msgs::msg::Twist getRelativeTwist(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &time_now, const rclcpp::Duration &duration);
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

  // parameters
  void AgentPathPrediction::loadParameters();

  void declareFloatParam(const std::string &name, double default_value, double min, double max, const std::string &description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;

    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = min;
    range.to_value = max;
    range.step = 0.001;
    descriptor.floating_point_range.push_back(range);

    this->declare_parameter(name, default_value, descriptor);
  }

  void declareBoolParam(const std::string &name, bool default_value, const std::string &description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    this->declare_parameter(name, default_value, descriptor);
  }

  // Properties
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