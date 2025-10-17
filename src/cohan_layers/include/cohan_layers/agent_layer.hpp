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

#ifndef AGENT_LAYER_H
#define AGENT_LAYER_H
#include <tf2/utils.h>

#include <agent_path_prediction/msg/agents_info.hpp>
#include <cohan_msgs/msg/state_array.hpp>
#include <cohan_msgs/msg/tracked_agents.hpp>
#include <cohan_msgs/msg/tracked_segment_type.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace cohan_layers {
class AgentLayer : public nav2_costmap_2d::Layer {
 public:
  AgentLayer() { layered_costmap_ = nullptr; }

  /**
   * @brief Initializes the agent layer
   */
  void onInitialize() override;

  /**
   * @brief Updates the bounds of the costmap based on agent positions
   * @param origin_x Origin x-coordinate of the costmap
   * @param origin_y Origin y-coordinate of the costmap
   * @param origin_yaw Rotation angle of the costmap origin
   * @param min_x Pointer to the minimum x-coordinate of the bounds
   * @param min_y Pointer to the minimum y-coordinate of the bounds
   * @param max_x Pointer to the maximum x-coordinate of the bounds
   * @param max_y Pointer to the maximum y-coordinate of the bounds
   */
  void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;

  /**
   * @brief Updates the cost values in the costmap based on agent positions.
   * @param master_grid Reference to the master costmap grid.
   * @param min_i Minimum index in the i-direction defining the region to update.
   * @param min_j Minimum index in the j-direction defining the region to update.
   * @param max_i Maximum index in the i-direction defining the region to update.
   * @param max_j Maximum index in the j-direction defining the region to update.
   */
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override = 0;

  /**
   * @brief Updates the bounds from agent positions
   * @param min_x Pointer to the minimum x-coordinate of the bounds
   * @param min_y Pointer to the minimum y-coordinate of the bounds
   * @param max_x Pointer to the maximum x-coordinate of the bounds
   * @param max_y Pointer to the maximum y-coordinate of the bounds
   */
  virtual void updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y) = 0;

  /**
   * @brief Indicates whether the layer is discretized
   * @return Always returns false as this layer is not discretized
   */
  static bool isDiscretized() { return false; }

 protected:
  struct AgentPoseVel {
    int track_id;  //!< Unique identifier for the agent
    int type;      //!< Type of the agent (e.g., human, robot)
    int state;     //!< State of the agent
    std_msgs::msg::Header header;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist velocity;
  };

  /**
   * @brief Callback for TrackedAgents
   * @param agents - Tracked agents data received from the topic
   * Updates the agents_ member variable with the received data.
   */
  void agentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr agents);

  /**
   * @brief Callback for AgentsInfo to get the agent states
   * @param agents_info - Information about agents including their states
   * Updates the states_ member variable with the received data.
   */
  void statesCB(const agent_path_prediction::msg::AgentsInfo::SharedPtr agents_info);

  /**
   * @brief Callback for shutdown service. Handles the shutdown of the agent layer.
   * @param req Service request containing the shutdown flag
   * @param res Service response indicating success or failure
   */
  void shutdownCB(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  /**
   * @brief Computes a 1D Gaussian value
   * @param x Input value
   * @param x0 Mean of the Gaussian
   * @param A Amplitude of the Gaussian
   * @param varx Variance of the Gaussian
   * @return 1D Gaussian value for the given input
   */
  static double Guassian1D(double x, double x0, double A, double varx) {
    double dx = x - x0;
    return A * exp(-pow(dx, 2.0) / (2.0 * varx));
  }

  /**
   * @brief Computes a 2D Gaussian value
   * @param x Input x-coordinate
   * @param y Input y-coordinate
   * @param x0 Mean x-coordinate of the Gaussian
   * @param y0 Mean y-coordinate of the Gaussian
   * @param A Amplitude of the Gaussian
   * @param varx Variance in the x-direction
   * @param vary Variance in the y-direction
   * @return 2D Gaussian value for the given inputs
   */
  static double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary) {
    double dx = x - x0;
    double dy = y - y0;
    double d = sqrt((dx * dx) + (dy * dy));
    double theta = atan2(dy, dx);
    double xx = d * cos(theta);
    double yy = d * sin(theta);
    return A / std::max(d, 1.0) * Guassian1D(xx, 0.0, 1.0, varx) * Guassian1D(yy, 0.0, 1.0, vary);
  }

  /**
   * @brief Computes a skewed 2D Gaussian value
   * @param x Input x-coordinate
   * @param y Input y-coordinate
   * @param x0 Mean x-coordinate of the Gaussian
   * @param y0 Mean y-coordinate of the Gaussian
   * @param A Amplitude of the Gaussian
   * @param varx Variance in the x-direction
   * @param vary Variance in the y-direction
   * @param skew_ang Skew angle for the Gaussian
   * @return Skewed 2D Gaussian value for the given inputs
   */
  static double Gaussian2D_skewed(double x, double y, double x0, double y0, double A, double varx, double vary, double skew_ang) {
    double dx = x - x0;
    double dy = y - y0;
    double d = sqrt((dx * dx) + (dy * dy));
    double theta = atan2(dy, dx);
    double xx = d * cos(theta - skew_ang);
    double yy = d * sin(theta - skew_ang);
    return A / std::max(d, 1.0) * Guassian1D(xx, 0.0, 1.0, varx) * Guassian1D(yy, 0.0, 1.0, vary);
  }

  rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr agents_sub_;  //!< ros subscribers
  rclcpp::Subscription<agent_path_prediction::msg::AgentsInfo>::SharedPtr agents_states_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stopmap_srv_;  //!< ros services
  cohan_msgs::msg::TrackedAgents agents_;                           //!< agents
  std::map<int, int> states_;                                       //!< agent_states
  std::vector<AgentPoseVel> transformed_agents_;                    //!< transformed agents
  std::recursive_mutex lock_;
  bool first_time_, reset_, shutdown_;                                   //!< flags
  rclcpp::Time last_time_;                                               //!< time checks
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;             //!< min and max x and y values for the costmap
  double radius_, amplitude_, covar_, cutoff_;                           //!< parameters for the gaussian
  double robot_radius_, agent_radius_;                                   //!< radii for the agents
  std::string ns_, tracked_agents_sub_topic_, agents_states_sub_topic_;  //!< ROS namespace and topic names
};
}  // namespace cohan_layers

#endif  // AGENT_LAYERS_H
