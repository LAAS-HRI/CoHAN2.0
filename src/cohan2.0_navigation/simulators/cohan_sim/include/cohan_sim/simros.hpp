/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2025 LAAS-CNRS
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

#ifndef SIMROS_HPP
#define SIMROS_HPP

#if ROS == 1
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#elif ROS == 2
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#endif

#include <cohan_sim/sim.hpp>
#include <map>

namespace cohan_sim {

/**
 * @brief ROS interface wrapper for the 2D simulator
 *
 * This class provides ROS integration for the Simulator2D class, handling
 * topics for odometry, laser scans, velocity commands, and transforms.
 * It manages the lifecycle of simulation entities and provides real-time
 * updates of their states through ROS messages and transforms.
 */
class SimROS {
 public:
  /**
   * @brief Constructs a new SimROS object
   * @param filename Path to the simulation configuration file containing world and entity definitions
   * @param gui Flag to enable/disable GUI visualization of the simulation
   */
  SimROS(const char* filename, bool gui);

  /**
   * @brief Destructor - Cleans up simulation resources
   */
  ~SimROS();

  /**
   * @brief Publishes simulation state to ROS topics
   *
   * Updates and publishes:
   * - Odometry for each entity
   * - Laser scan data
   * - Ground truth poses
   * - TF transforms
   */
  void publishROS();

  /**
   * @brief Initializes ROS messages with default values
   *
   * Sets up message templates for:
   * - Laser scan configurations
   * - Odometry frame IDs
   */
  void initMessages();

  /**
   * @brief Updates the simulation world state
   *
   * Steps the simulation forward in time, updating:
   * - Entity positions and velocities
   * - Collision detection
   * - Sensor data
   * - Publishes ROS topics, tf and clock
   */
  void updateWorld();

  /**
   * @brief Checks if simulation should quit
   * @return true if simulation should terminate, false otherwise
   */
  bool quitSim() const { return quit_sim_; }

#if ROS == 2
  /**
   * @brief Get the ROS 2 node pointer
   * @return Shared pointer to the ROS 2 node
   */
  rclcpp::Node::SharedPtr getNode() const { return node_; }
#endif

 private:
  /**
   * @brief Converts Euler angles to quaternion for 3D orientation representation
   * @param roll Roll angle in radians around X-axis
   * @param pitch Pitch angle in radians around Y-axis
   * @param yaw Yaw angle in radians around Z-axis
   * @return geometry_msgs::Quaternion containing the converted orientation
   */
#if ROS == 1
  geometry_msgs::Quaternion quaternionFromEuler(double roll, double pitch, double yaw);
#elif ROS == 2
  geometry_msgs::msg::Quaternion quaternionFromEuler(double roll, double pitch, double yaw);
#endif

  /**
   * @brief Callback for velocity command messages
   * @param msg Twist message containing linear and angular velocity commands
   * @param robot_idx Index of the robot to control in the simulation
   */
#if ROS == 1
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg, int robot_idx);
#elif ROS == 2
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg, int robot_idx);
#endif

  /**
   * @brief Callback for head rotation messages
   * @param msg Vector3 message containing the roll, pitch and yaw
   * @param robot_idx Index of the robot to control in the simulation
   */
#if ROS == 1
  void headRotationCallback(const geometry_msgs::Vector3ConstPtr& msg, int robot_idx);
#elif ROS == 2
  void headRotationCallback(const geometry_msgs::msg::Vector3::SharedPtr msg, int robot_idx);
#endif

  std::unique_ptr<cohan_sim::Simulator2D> sim_;  //!< Core 2D simulator instance managing the world state

#if ROS == 1
  ros::Time sim_time_;  //!< Current simulation time for synchronization

  std::map<int, ros::Publisher> odom_pubs_;            //!< Publishers for odometry messages, keyed by robot index
  std::map<int, ros::Publisher> scan_pubs_;            //!< Publishers for laser scan messages, keyed by robot index
  std::map<int, ros::Publisher> ground_truth_pubs_;    //!< Publishers for ground truth poses, keyed by robot index
  std::map<int, ros::Subscriber> head_rotation_subs_;  //!< Subscribers for head rotation, keyed by robot index
  std::map<int, ros::Subscriber> cmd_vel_subs_;        //!< Subscribers for velocity commands, keyed by robot index
  tf2_ros::TransformBroadcaster tf_broadcaster_;       //!< Broadcaster for publishing coordinate transforms
  ros::Publisher clock_pub_;                           //!< Publisher for simulation clock messages

  std::map<int, sensor_msgs::LaserScan> scan_msgs_;  //!< Cached laser scan messages for each robot
  std::map<int, nav_msgs::Odometry> odom_msgs_;      //!< Cached odometry messages for each robot
#elif ROS == 2
  rclcpp::Time sim_time_;  //!< Current simulation time for synchronization

  std::map<int, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs_;                  //!< Publishers for odometry messages, keyed by robot index
  std::map<int, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> scan_pubs_;              //!< Publishers for laser scan messages, keyed by robot index
  std::map<int, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> ground_truth_pubs_;          //!< Publishers for ground truth poses, keyed by robot index
  std::map<int, rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr> head_rotation_subs_;  //!< Subscribers for head rotation, keyed by robot index
  std::map<int, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_subs_;          //!< Subscribers for velocity commands, keyed by robot index
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;                                   //!< Broadcaster for publishing coordinate transforms
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;                               //!< Publisher for simulation clock messages

  std::map<int, sensor_msgs::msg::LaserScan> scan_msgs_;  //!< Cached laser scan messages for each robot
  std::map<int, nav_msgs::msg::Odometry> odom_msgs_;      //!< Cached odometry messages for each robot
  rclcpp::Node::SharedPtr node_;                          //!< ROS 2 node handle
#endif

  bool quit_sim_;  //!< Flag indicating if simulation should terminate
};

}  // namespace cohan_sim

#endif  // SIMROS_HPP