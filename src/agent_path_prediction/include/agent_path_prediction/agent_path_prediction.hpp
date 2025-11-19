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

#ifndef AGENT_PATH_PREDICTION_H_
#define AGENT_PATH_PREDICTION_H_

// ROS2 core
#include <rclcpp/rclcpp.hpp>
#include <ros2_helpers/parameters.hpp>
#include <ros2_helpers/utils.hpp>

//  TF2
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Local and generic headers
#include <agent_path_prediction/agent_path_prediction_config.hpp>
#include <agent_path_prediction/predict_goal.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Messages
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <agent_path_prediction/msg/predicted_goals.hpp>
#include <cohan_msgs/msg/agent_path_array.hpp>
#include <cohan_msgs/msg/tracked_agents.hpp>
#include <cohan_msgs/msg/tracked_segment_type.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Services
#include <agent_path_prediction/srv/agent_goal.hpp>
#include <agent_path_prediction/srv/agent_pose_predict.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// Actions
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Internal Parameters
#define ANG_VEL_EPS 0.001
#define MAX_AGENT_MARKERS 1000
#define MIN_MARKER_LIFETIME 1.0
#define MINIMUM_COVARIANCE_MARKERS 0.1
#define RECALC_DIST 0.5
#define NODE_NAME "agent_path_prediction"

namespace agents {
// Pattern 1 of ROS2 Nodes --> Inherit from rclcpp::Node (Good for Standalone nodes)
/**
 * @brief Class for agent path prediction using different methods
 */
class AgentPathPrediction : public rclcpp::Node {
 public:
  /**
   * @brief  Constructor for AgentPathPrediction class */
  AgentPathPrediction() : Node(NODE_NAME) {}

  /**
   * @brief Default destructor for AgentPathPrediction class */
  ~AgentPathPrediction() = default;

  /**
   * @brief Initializes the AgentPathPrediction node and sets up ROS communication */
  void initialize();

  // ROS2 node and helpers
  std::shared_ptr<AgentPathPredictConfig> cfg_;  //!< Configuration parameters for agent path prediction

 private:
  // Structs
  /**
   * @brief Structure to store agent path and velocity information */
  struct AgentPathVel {
    uint64_t id;                                        // Agent ID
    nav_msgs::msg::Path path;                           // Predicted path for the agent
    geometry_msgs::msg::TwistWithCovariance start_vel;  // Initial velocity of the agent
  };

  /**
   * @brief Structure to store agent's initial pose and velocity
   */
  struct AgentStartPoseVel {
    uint64_t id;                                  // Agent ID
    geometry_msgs::msg::PoseStamped pose;         // Initial pose of the agent
    geometry_msgs::msg::TwistWithCovariance vel;  // Initial velocity of the agent
  };

  // subscriber callbacks
  /**
   * @brief Callback for tracked agents updates
   * @param tracked_agents The tracked agents message containing current agent states
   */
  void trackedAgentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr tracked_agents);

  /**
   * @brief Callback for external path updates
   * @param external_paths Array of external paths for agents
   */
  void externalPathsCB(const cohan_msgs::msg::AgentPathArray::SharedPtr external_paths);

  /**
   * @brief Callback for predicted goal updates
   * @param predicted_goal The predicted goals message
   */
  void predictedGoalCB(const agent_path_prediction::msg::PredictedGoals::SharedPtr predicted_goal);

  // Service callbacks
  /**
   * @brief Service to predict agent paths using default prediction method
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   */
  void predictAgents(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);

  /**
   * @brief Service to predict agent paths using velocity obstacle method
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   */
  void predictAgentsVelObs(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) const;

  /**
   * @brief Service to predict agent paths using external path information
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   */
  void predictAgentsExternal(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);

  /**
   * @brief Service to predict paths for agents assuming their goal is behind the robot
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   */
  void predictAgentsBehind(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);

  /**
   * @brief Service to predict agent paths based on predicted goals
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   */
  void predictAgentsGoal(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);

  /**
   * @brief Service to predict agent paths from existing path data. This method is called internally at the end of different prediction mechanisms.
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   * @param path_vels Vector of agent paths and velocities
   */
  void predictAgentsFromPaths(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);
  /**
   * @brief Service to reset all prediction services
   * @param req Empty service request
   * @param res Empty service response
   */
  void resetPredictionSrvs(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);

  /**
   * @brief Service to set goals for agents
   * @param req Service request containing goal data
   * @param res Service response
   */
  void setGoal(const std::shared_ptr<agent_path_prediction::srv::AgentGoal::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentGoal::Response> res);

  /**
   * @brief Creates a fixed path from a start pose. It adds constant velocity based prediction for 0.5m infront of the human.
   * @param start_pose The starting pose for the path (human's current position)
   * @return The generated fixed path
   */
  static nav_msgs::msg::Path setFixedPath(const geometry_msgs::msg::PoseStamped& start_pose);

  /**
   * @brief Prunes a path starting from a given index based on pose
   * @param begin_index Starting index for pruning
   * @param pose Reference pose for pruning
   * @param path Vector of poses to prune
   * @return Index of the given pose in the vector
   */
  static size_t prunePath(size_t begin_index, const geometry_msgs::msg::Pose& pose, const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& path);

  /**
   * @brief Transforms pose and twist of agent to a target frame
   * @param tracked_agents Tracked agents data
   * @param agent_id ID of the agent to transform
   * @param to_frame Target frame for transformation
   * @param pose Output transformed pose
   * @param twist Output transformed twist
   * @return True if transformation successful, false otherwise
   */
  bool transformPoseTwist(const cohan_msgs::msg::TrackedAgents& tracked_agents, const uint64_t& agent_id, const std::string& to_frame, geometry_msgs::msg::PoseStamped& pose,
                          geometry_msgs::msg::TwistStamped& twist) const;

  /**
   * @brief Calculates Euclidean distance between agent and robot poses
   * @param agent Agent pose
   * @param robot Robot pose
   * @return Distance between agent and robot
   */
  static double checkdist(geometry_msgs::msg::Pose agent, geometry_msgs::msg::Pose robot) { return std::hypot(agent.position.x - robot.position.x, agent.position.y - robot.position.y); }

  // ROS2 message handlers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predicted_agents_pub_;         //!< Publisher for predicted agent paths
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr front_pose_pub_;                    //!< Publisher for front pose information
  rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr tracked_agents_sub_;              //!< Subscriber for tracked agents information
  rclcpp::Subscription<cohan_msgs::msg::AgentPathArray>::SharedPtr external_paths_sub_;             //!< Subscriber for external path information
  rclcpp::Subscription<agent_path_prediction::msg::PredictedGoals>::SharedPtr predicted_goal_sub_;  //!< Subscriber for predicted goals

  // ROS2 Services
  rclcpp::Service<agent_path_prediction::srv::AgentPosePredict>::SharedPtr predict_agents_server_;  //!< Server for agent prediction service
  rclcpp::Service<agent_path_prediction::srv::AgentGoal>::SharedPtr set_goal_srv_;                  //!< Server for setting agent goals
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_prediction_services_server_;               //!< Server for resetting predictions

  // ROS2 Action Clients
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr get_plan_client_;  //!< Action client for computing navigation paths

  // Transform listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;      //!< TF2 buffer for coordinate transformations
  std::shared_ptr<tf2_ros::TransformListener> tf_;  //!< TF2 transform listener for coordinate transformations

  // Internal variables
  cohan_msgs::msg::TrackedAgents tracked_agents_;                                 //!< Current state of tracked agents in the environment
  cohan_msgs::msg::AgentPathArray external_paths_;                                //!< External paths provided for agents
  agent_path_prediction::msg::PredictedGoals predicted_goals_;                    //!< Collection of predicted goals for agents
  std::vector<agent_path_prediction::msg::AgentPose> external_goals_;             //!< Vector storing external goals for agents
  std::vector<AgentPathVel> path_vels_;                                           //!< Vector storing path and velocity information for agents
  std::vector<int> path_vels_pos_;                                                //!< Vector storing positions in the path velocity array
  std::vector<agent_path_prediction::msg::PredictedPoses> last_predicted_poses_;  //!< Vector storing the last predicted poses for agents
  std::map<uint64_t, size_t> last_prune_indices_;                                 //!< Map storing last pruning indices for each agent
  std::map<uint64_t, int> last_markers_size_map_;                                 //!< Map storing last marker sizes for each agent
  visualization_msgs::msg::MarkerArray predicted_agents_markers_;                 //!< Visualization markers for predicted agent paths
  bool check_path_;                                                               //!< Flag path checking
  bool showing_markers_, got_new_agent_paths_, got_external_goal_;                //!< Flags for marker visualization and path updates
  geometry_msgs::msg::Transform behind_pose_;                                     //!< Transform for behind pose calculation
  std::string tracked_agents_sub_topic_;                                          //!< Topic name for tracked agents subscription
  std::string get_plan_srv_name_;                                                 //!< Service name for get plan service
  std::string ns_;                                                                //!< Namespace for the node
};
}  // namespace agents

#endif  // AGENT_PATH_PREDICTION_H_
