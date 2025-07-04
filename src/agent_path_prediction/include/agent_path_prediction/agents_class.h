/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024 LAAS/CNRS
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Phani Teja Singamaneni
 *********************************************************************/

#ifndef AGENTS_HH_
#define AGENTS_HH_

// ROS
#include <ros/ros.h>

// TF2
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// COSTMAP
#include <costmap_2d/costmap_2d_ros.h>

// MSGS
#include <agent_path_prediction/AgentsInfo.h>
#include <cohan_msgs/AgentPathArray.h>
#include <cohan_msgs/AgentTrajectory.h>
#include <cohan_msgs/AgentTrajectoryArray.h>
#include <cohan_msgs/StateArray.h>
#include <cohan_msgs/TrackedAgents.h>
#include <cohan_msgs/TrackedSegmentType.h>

// OTHERS
#include <Eigen/Core>
#include <string>

// Constants
#define CALC_EPS 0.0001
#define COST_MIN 200
#define COST_OBS 255
#define MAX_PTS 1000
#define MIN_PTS 100
#define AGENT_NUM_TH 5
#define DEFAULT_AGENT_SEGMENT cohan_msgs::TrackedSegmentType::TORSO

namespace agents {
/**
 * @brief Enum representing the states of an agent
 */
enum AgentState { NO_STATE, STATIC, MOVING, STOPPED, BLOCKED };

/**
 * @brief Class representing agents and their states
 */
class Agents {
 public:
  /**
   * @brief Default constructor for Agents class
   */
  Agents();

  /**
   * @brief Constructor with parameters for tf buffer and costmap
   * @param tf Pointer to tf buffer
   * @param costmap_ros Pointer to costmap
   */
  Agents(tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

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
  std::map<int, geometry_msgs::Pose> getAgents() { return agents_; }

  /** 
   * @brief Get the state of the agent
   * @param id The index of agent whose state is required
   * @return State of the given agent
   */
  AgentState agentState(int id){return agents_states_[id];}

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
  void trackedAgentsCB(const cohan_msgs::TrackedAgents &tracked_agents);

  // Methods
  /**
   * @brief Filters agents that are in the FOV of the robot (for e.g., in simulation or mocap)
   * @param tr_agents Transformed agent poses
   * @param sorted_ids Vector of sorted agent IDs
   * @param agents_radii Agent radii
   * @param robot_pose Current robot pose
   * @return Vector of visible agent IDs in the FOV of the robot
   */
  std::vector<int> filterVisibleAgents(std::map<int, geometry_msgs::Pose> tr_agents, std::vector<int> sorted_ids, std::map<int, double> agents_radii, geometry_msgs::Pose2D robot_pose);

  /**
   * @brief Loads parameters from ROS parameter server
   * @param private_nh Private node handle containing parameters
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle &private_nh);

  // Agent State Variables
  cohan_msgs::TrackedAgents tracked_agents_;                      //!< Latest tracked agents message
  std::map<int, geometry_msgs::Pose> agents_, prev_agents_;       //!< Current and previous agent poses
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

  // ROS
  ros::Publisher agents_info_pub_;         //!< Publisher for agent information
  ros::Subscriber tracked_agents_sub_;     //!< Subscriber for tracked agents
  tf2_ros::Buffer *tf_;                    //!< Pointer to tf buffer
  costmap_2d::Costmap2DROS *costmap_ros_;  //!< Pointer to the costmap ros wrapper
  costmap_2d::Costmap2D *costmap_;         //!< Pointer to the 2d costmap
  double inflation_radius_;                //!< Inflation radius for costmap
};
}  // namespace agents
#endif
