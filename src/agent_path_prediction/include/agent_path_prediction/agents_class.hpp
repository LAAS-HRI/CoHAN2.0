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

#ifndef AGENTS_HH_
#define AGENTS_HH_

// ROS2 core
#include <rclcpp/rclcpp.hpp>
#include <ros2_helpers/parameters.hpp>

// TF2
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// COSTMAP
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

// MSGS
#include <agent_path_prediction/msg/agents_info.hpp>
#include <cohan_msgs/msg/agent_path_array.hpp>
#include <cohan_msgs/msg/agent_trajectory.hpp>
#include <cohan_msgs/msg/agent_trajectory_array.hpp>
#include <cohan_msgs/msg/state_array.hpp>
#include <cohan_msgs/msg/tracked_agents.hpp>
#include <cohan_msgs/msg/tracked_segment_type.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

// OTHERS
#include <Eigen/Core>
#include <map>
#include <memory>
#include <string>
#include <vector>

// Constants
#define CALC_EPS 0.0001
#define COST_MIN 250
#define COST_OBS 255
#define MAX_PTS 1000
#define MIN_PTS 100
#define AGENT_NUM_TH 5
#define DEFAULT_AGENT_SEGMENT cohan_msgs::msg::TrackedSegmentType::TORSO

namespace agents {
/**
 * @brief Enum representing the states of an agent
 */
enum AgentState { NO_STATE, STATIC, MOVING, STOPPED, BLOCKED };

// Pattern 2 of ROS2 Nodes --> Use a private node instance that can be obtained from another node
/**
 * @brief Class representing agents and their states
 */
class Agents {
 public:
  /**
   * @brief Constructor with parameters for node, tf buffer and costmap
   * @param node Shared pointer to ROS2 node
   * @param tf Pointer to tf buffer
   * @param costmap_ros Pointer to costmap
   */
  Agents(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Destructor of the class
   */
  ~Agents() = default;

  /**
   * @brief Set the state of an agent
   * @param state The state to set
   * @param id The ID of the agent
   */
  void setState(AgentState state, int id) {
    agents_states_[id] = state;
    if (state == AgentState::BLOCKED) {
      stuck_ = true;
      stuck_agent_id_ = id;
    }
  }

  /**
   * @brief Reset all agent states
   */
  void resetAgents();

  /**
   * @brief Check if any agent is stuck
   * @return True if an agent is stuck, false otherwise
   */
  bool isAgentStuck() const { return stuck_; }

  /**
   * @brief Reset the stuck agent ID
   */
  void resetStuckAgent() { stuck_agent_id_ = -1; }

  /**
   * @brief Get the poses of all agents
   * @return Map of agent IDs to their poses
   */
  std::map<int, geometry_msgs::msg::Pose> getAgents() { return agents_; }

  /**
   * @brief Get the state of the agent
   * @param id The index of agent whose state is required
   * @return State of the given agent
   */
  AgentState agentState(int id) { return agents_states_[id]; }

  /**
   * @brief Get nominal velocities of all agents based on a moving average filter
   * @return Map of agent IDs to their nominal velocities
   */
  std::map<int, double> getNominalVels() { return agent_nominal_vels_; }

 private:
  // Callbacks
  /**
   * @brief Callback for tracked agents updates
   * @param tracked_agents The tracked agents message containing agent states
   */
  void trackedAgentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr tracked_agents);

  // Methods
  /**
   * @brief Filters agents that are in the FOV of the robot (for e.g., in simulation or mocap)
   * @param tr_agents Transformed agent poses
   * @param sorted_ids Vector of sorted agent IDs
   * @param agents_radii Agent radii
   * @param robot_pose Current robot pose
   * @return Vector of visible agent IDs in the FOV of the robot
   */
  std::vector<int> filterVisibleAgents(std::map<int, geometry_msgs::msg::Pose> tr_agents, std::vector<int> sorted_ids, std::map<int, double> agents_radii, geometry_msgs::msg::Pose2D robot_pose);

  /**
   * @brief Loads and initializes all parameters from ROS2 parameter server
   */
  void setupParameterCallback() {
    // Declare parameters using the generic helper
    param_helper_.declareStringParam("ns", "", "Namespace for multiple agents");
    param_helper_.declareFloatParam("local_costmap/inflater/inflation_radius", 0.0, 0.0, 10.0, "Inflation radius for costmap");
    param_helper_.declareIntParam("planning_mode", 0, 0, 10, "Mode of planning (different strategies)");
    param_helper_.declareBoolParam("use_simulated_fov", false, "Flag for using simulated field of view");
    param_helper_.declareIntParam("window_moving_avg", 5, 1, 20, "Window size for moving average calculation");
    param_helper_.declareFloatParam("HATebLocalPlannerROS/agent_radius", 0.3, 0.1, 2.0, "Radius considered for human agents");
    param_helper_.declareFloatParam("HATebLocalPlannerROS/robot_radius", 0.3, 0.1, 2.0, "Robot's physical radius");
    param_helper_.declareStringParam("base_link_frame", "base_footprint", "Robot's base link frame ID");
    param_helper_.declareStringParam("map_frame", "map", "Map frame ID");
    param_helper_.declareStringParam("odom_frame", "odom", "Odometry frame ID");
    param_helper_.declareFloatParam("planning_radius", 1.0, 0.1, 10.0, "Radius used for planning");

    // Set up parameter change callback with custom validation
    param_helper_.setupParameterCallback([this](const std::vector<rclcpp::Parameter>& params) -> bool {
      // Custom parameter validation logic for this specific node
      for (const auto& param : params) {
        const std::string& name = param.get_name();

        // Update internal variables when parameters change
        if (name == "ns")
          ns_ = param.as_string();
        else if (name == "local_costmap/inflater/inflation_radius")
          inflation_radius_ = param.as_double();
        else if (name == "planning_mode")
          planning_mode_ = param.as_int();
        else if (name == "use_simulated_fov")
          use_simulated_fov_ = param.as_bool();
        else if (name == "window_moving_avg")
          window_moving_avg_ = param.as_int();
        else if (name == "HATebLocalPlannerROS/agent_radius")
          human_radius_ = param.as_double();
        else if (name == "HATebLocalPlannerROS/robot_radius")
          robot_radius_ = param.as_double();
        else if (name == "base_link_frame")
          base_link_frame_ = param.as_string();
        else if (name == "map_frame")
          map_frame_ = param.as_string();
        else if (name == "odom_frame")
          odom_frame_ = param.as_string();
        else if (name == "planning_radius")
          planning_radius_ = param.as_double();
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
    ns_ = node_->get_parameter("ns").as_string();
    inflation_radius_ = node_->get_parameter("local_costmap/inflater/inflation_radius").as_double();
    planning_mode_ = node_->get_parameter("planning_mode").as_int();
    use_simulated_fov_ = node_->get_parameter("use_simulated_fov").as_bool();
    window_moving_avg_ = node_->get_parameter("window_moving_avg").as_int();
    human_radius_ = node_->get_parameter("HATebLocalPlannerROS/agent_radius").as_double();
    robot_radius_ = node_->get_parameter("HATebLocalPlannerROS/robot_radius").as_double();
    tracked_agents_sub_topic_ = node_->get_parameter("tracked_agents_sub_topic").as_string();
    base_link_frame_ = node_->get_parameter("base_link_frame").as_string();
    map_frame_ = node_->get_parameter("map_frame").as_string();
    odom_frame_ = node_->get_parameter("odom_frame").as_string();
    planning_radius_ = node_->get_parameter("planning_radius").as_double();
  }

  // Agent State Variables
  cohan_msgs::msg::TrackedAgents::SharedPtr tracked_agents_;      //!< Latest tracked agents message
  std::map<int, geometry_msgs::msg::Pose> agents_, prev_agents_;  //!< Current and previous agent poses
  std::map<int, bool> agent_still_;                               //!< Map tracking if agents are stationary
  std::map<int, std::vector<double>> agent_vels_;                 //!< List of agent velocities over time
  std::map<int, double> agent_nominal_vels_;                      //!< Nominal velocities based on moving average
  std::map<int, AgentState> agents_states_, prev_agents_states_;  //!< Current and previous agent states
  std::vector<int> visible_agent_ids_;                            //!< List of visible agent IDs

  // Configuration and Status
  std::string ns_;                        //!< Namespace for multiple agents
  std::string tracked_agents_sub_topic_;  //!< Topic for tracked agents subscription
  bool initialized_;                      //!< Initialization status flag
  bool stuck_;                            //!< Flag indicating if agent is stuck
  int stuck_agent_id_;                    //!< ID of agent blocking robot's path

  // Parameters
  int window_moving_avg_;        //!< Window size for moving average calculation
  int planning_mode_;            //!< Mode of planning (different strategies)
  double human_radius_;          //!< Radius considered for human agents
  double robot_radius_;          //!< Robot's physical radius
  double planning_radius_;       //!< Radius used for planning
  std::string base_link_frame_;  //!< Robot's base link frame ID
  std::string map_frame_;        //!< Map frame ID
  std::string odom_frame_;       //!< Odometry frame ID
  bool use_simulated_fov_;       //!< Flag for using simulated field of view

  // ROS2
  std::shared_ptr<rclcpp::Node> node_;                                                    //!< ROS2 node handle
  parameters::ParameterHelper param_helper_;                                              //!< Helper for managing parameters
  rclcpp::Publisher<agent_path_prediction::msg::AgentsInfo>::SharedPtr agents_info_pub_;  //!< Publisher for agent information
  rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr tracked_agents_sub_;    //!< Subscriber for tracked agents
  std::shared_ptr<tf2_ros::Buffer> tf_;                                                   //!< Pointer to tf buffer
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;                            //!< Pointer to the costmap ros wrapper
  nav2_costmap_2d::Costmap2D* costmap_;                                                   //!< Pointer to the 2d costmap
  double inflation_radius_;                                                               //!< Inflation radius for costmap
};
}  // namespace agents
#endif
