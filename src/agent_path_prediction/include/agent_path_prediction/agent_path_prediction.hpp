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

// Local and generic headers
#include <agent_path_prediction/agents_class.hpp>
#include <agent_path_prediction/predict_goal.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <ros2_helpers/parameters.hpp>
#include <ros2_helpers/utils.hpp>
#include <string>

// ROS2 core
#include <rclcpp/rclcpp.hpp>

//  TF2
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Messages
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <agent_path_prediction/msg/agent_pose.hpp>
#include <agent_path_prediction/msg/predicted_goal.hpp>
#include <agent_path_prediction/msg/predicted_goals.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Services
#include <agent_path_prediction/srv/agent_goal.hpp>
#include <agent_path_prediction/srv/agent_pose_predict.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// Internal Parameters
#define ANG_VEL_EPS 0.001
#define MAX_AGENT_MARKERS 1000
#define MIN_MARKER_LIFETIME 1.0
#define MINIMUM_COVARIANCE_MARKERS 0.1
#define RECALC_DIST 0.5
#define DEFAULT_AGENT_PART cohan_msgs::msg::TrackedSegmentType::TORSO
#define NODE_NAME "agent_path_prediction"

// Reconfigurable Parameters
#define EXTERNAL_PATHS_SUB_TOPIC "~/external_agent_paths"
#define PREDICTED_GOAL_SUB_TOPIC "~/predicted_goal"
#define AGENTS_SUB_TOPIC "/tracked_agents"
#define GET_PLAN_SRV_NAME "/move_base/GlobalPlanner/make_plan"
#define ROBOT_FRAME_ID "base_footprint"
#define MAP_FRAME_ID "map"
#define AGENT_DIST_BEHIND_ROBOT 0.5
#define AGENT_ANGLE_BEHIND_ROBOT M_PI

namespace agents {
class AgentPathPrediction {
 public:
  /**
   * @brief  Constructor for AgentPathPrediction class */
  AgentPathPrediction(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    param_helper_.initialize(node);
    RCLCPP_INFO(node_->get_logger(), "AgentPathPrediction initialized");
    // Initialize TF2 buffer and transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    setupParameterCallback();
  };

  /**
   * @brief Default destructor for AgentPathPrediction class */
  ~AgentPathPrediction() = default;

  /**
   * @brief Initializes the AgentPathPrediction node and sets up ROS communication */
  void initialize();

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
      }
      return true;
    });

    // Load initial parameter values
    loadParameters();
  }

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
  }

  // ROS2 node and helpers
  rclcpp::Node::SharedPtr node_;              //!< Pointer to the ROS2 node
  parameters::ParameterHelper param_helper_;  //!< Helper for managing parameters
  std::string ns_;                            //!< Namespace for the node

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
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr get_plan_client_;                               //!< Client for getting navigation plans

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
  std::string tracked_agents_sub_topic_, external_paths_sub_topic_, predict_service_name_, predicted_agents_markers_pub_topic_, get_plan_srv_name_,
      predicted_goal_topic_;                                                   //!< ROS topic names for subscribers and publishers
  std::string robot_frame_id_, map_frame_id_;                                  //!< Frame IDs for coordinate transformations
  double velobs_mul_, velobs_min_rad_, velobs_max_rad_, velobs_max_rad_time_;  //!< Velocity obstacle parameters
  double agent_dist_behind_robot_, agent_angle_behind_robot_;                  //!< Parameters for agents behind robot detection
  bool velobs_use_ang_, check_path_;                                           //!< Flags for velocity obstacles and path checking
  bool publish_markers_, showing_markers_,                                     //!< Flags for marker visualization
      got_new_agent_paths_, got_external_goal_;                                //!< Flags for path updates
  int default_agent_part_;                                                     //!< Default body part to track for agents
  geometry_msgs::msg::Transform behind_pose_;                                  //!< Transform for behind pose calculation
};
}  // namespace agents

#endif  // AGENT_PATH_PREDICTION_H_
