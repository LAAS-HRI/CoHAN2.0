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

#include <yaml-cpp/yaml.h>

#include <agent_path_prediction/msg/predicted_goal.hpp>
#include <agent_path_prediction/msg/predicted_goals.hpp>
#include <agent_path_prediction/predict_goal.hpp>
#include <cohan_msgs/msg/tracked_agent.hpp>
#include <cohan_msgs/msg/tracked_agents.hpp>
#include <cohan_msgs/msg/tracked_segment_type.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace agents {
/**
 * @brief ROS wrapper for the Bayesian goal prediction system
 */
class PredictGoalROS {
 public:
  /**
   *@brief Default constructor, initializes ROS communication
   */
  PredictGoalROS();

  /**
   * @brief Default destructor
   */
  ~PredictGoalROS() = default;

 private:
  /**
   * @brief Callback for tracked agents updates. This method processes incoming tracked agents data and updates the internal state.
   * @param msg Message containing tracked agents data
   */
  void trackedAgentsCB(const cohan_msgs::TrackedAgents::ConstPtr& msg);

  /**
   * @brief Load goal positions from a YAML file. This method reads goal positions from the specified YAML file and populates the goals map.
   * @param file Path to the YAML file containing goals
   * @return True if goals were loaded successfully, false otherwise
   */
  bool loadGoals(const std::string& file);

  // Internal Methods
  /**
   * @brief Loads ROS parameters from the node handle
   * @param private_nh Private node handle containing parameters
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& private_nh);

  // ROS
  ros::Subscriber agents_sub_;  //!< Subscriber for tracked agents updates
  ros::Publisher goal_pub_;     //!< Publisher for predicted goals

  // Core components
  agents::BayesianGoalPrediction predictor_;  //!< Instance of the Bayesian goal prediction algorithm

  // Data storage
  std::map<std::string, Eigen::Vector2d> goals_;    //!< Map of goal names to their positions
  std::map<int, std::string> agent_goal_predicts_;  //!< Map of agent IDs to their predicted goals

  // Configuration
  int window_size_;  //!< Size of the sliding window used for predictions

  std::string tracked_agents_sub_topic_, predicted_goal_topic_;  //!< ROS topic names for subscribers and publishers
  std::string ns_;                                               //!< Namespace for the node
  std::string goals_file_;                                       //!< File name of the env goals
};
}  // namespace agents