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
#include <cohan_sim/simros.hpp>
#if ROS == 1
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#endif

namespace cohan_sim {
SimROS::SimROS(const char* filename, bool gui) {
  sim_ = std::make_unique<cohan_sim::Simulator2D>(filename, gui);
#if ROS == 1
  ros::NodeHandle nh;

  clock_pub_ = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);

  for (auto& robot : sim_->robots()) {
    if (robot.idx > 0) {
      std::string ns = robot.name.empty() ? "default" : robot.name;
      odom_pubs_[robot.idx] = nh.advertise<nav_msgs::Odometry>(ns + "/odom", 10);
      scan_pubs_[robot.idx] = nh.advertise<sensor_msgs::LaserScan>(ns + "/scan", 10);
      ground_truth_pubs_[robot.idx] = nh.advertise<nav_msgs::Odometry>(ns + "/base_pose_ground_truth", 10);
      cmd_vel_subs_[robot.idx] = nh.subscribe<geometry_msgs::Twist>(ns + "/cmd_vel", 1, boost::bind(&SimROS::cmdVelCallback, this, _1, robot.idx));
      head_rotation_subs_[robot.idx] = nh.subscribe<geometry_msgs::Vector3>(ns + "/head_rotation", 1, boost::bind(&SimROS::headRotationCallback, this, _1, robot.idx));
    } else {
      std::string ns = robot.name.empty() ? "default" : robot.name;
      odom_pubs_[robot.idx] = nh.advertise<nav_msgs::Odometry>("/odom", 10);
      scan_pubs_[robot.idx] = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);
      ground_truth_pubs_[robot.idx] = nh.advertise<nav_msgs::Odometry>("/base_pose_ground_truth", 10);
      cmd_vel_subs_[robot.idx] = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&SimROS::cmdVelCallback, this, _1, robot.idx));
      head_rotation_subs_[robot.idx] = nh.subscribe<geometry_msgs::Vector3>("/head_rotation", 1, boost::bind(&SimROS::headRotationCallback, this, _1, robot.idx));
    }
  }
#elif ROS == 2
  node_ = rclcpp::Node::make_shared("simros_node");

  clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  for (auto& robot : sim_->robots()) {
    if (robot.idx > 0) {
      std::string ns = robot.name.empty() ? "default" : robot.name;
      odom_pubs_[robot.idx] = node_->create_publisher<nav_msgs::msg::Odometry>(ns + "/odom", 10);
      scan_pubs_[robot.idx] = node_->create_publisher<sensor_msgs::msg::LaserScan>(ns + "/scan", 10);
      ground_truth_pubs_[robot.idx] = node_->create_publisher<nav_msgs::msg::Odometry>(ns + "/base_pose_ground_truth", 10);
      cmd_vel_subs_[robot.idx] =
          node_->create_subscription<geometry_msgs::msg::Twist>(ns + "/cmd_vel", 1, [this, robot](const geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(msg, robot.idx); });
      head_rotation_subs_[robot.idx] =
          node_->create_subscription<geometry_msgs::msg::Vector3>(ns + "/head_rotation", 1, [this, robot](const geometry_msgs::msg::Vector3::SharedPtr msg) { headRotationCallback(msg, robot.idx); });
    } else {
      std::string ns = robot.name.empty() ? "default" : robot.name;
      odom_pubs_[robot.idx] = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
      scan_pubs_[robot.idx] = node_->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
      ground_truth_pubs_[robot.idx] = node_->create_publisher<nav_msgs::msg::Odometry>("/base_pose_ground_truth", 10);
      cmd_vel_subs_[robot.idx] =
          node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, [this, robot](const geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(msg, robot.idx); });
      head_rotation_subs_[robot.idx] =
          node_->create_subscription<geometry_msgs::msg::Vector3>("/head_rotation", 1, [this, robot](const geometry_msgs::msg::Vector3::SharedPtr msg) { headRotationCallback(msg, robot.idx); });
    }
  }
#endif

  // Initialize messages
  initMessages();

  // Initialize time
#if ROS == 1
  sim_time_.fromSec(0.0);
#elif ROS == 2
  sim_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
#endif

  // Initialize state variables
  quit_sim_ = false;

  // Initialize tf_broadcaster_
#if ROS == 1
  tf_broadcaster_ = tf2_ros::TransformBroadcaster();
#elif ROS == 2
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
#endif
}

SimROS::~SimROS() {}

void SimROS::initMessages() {
  for (auto& robot : sim_->robots()) {
    // Scan Msgs
#if ROS == 1
    sensor_msgs::LaserScan scan_msg;
#elif ROS == 2
    sensor_msgs::msg::LaserScan scan_msg;
#endif
    std::string frame_id = (robot.idx == 0) ? "base_laser_link" : robot.name + "/base_laser_link";
    scan_msg.header.frame_id = frame_id;
    scan_msg.angle_min = -robot.laser_angle / 2;
    scan_msg.angle_max = robot.laser_angle / 2;
    scan_msg.angle_increment = robot.laser_angle / robot.laser_resolution;
    scan_msg.range_min = 0.01;
    scan_msg.range_max = robot.laser_range;

    // Store them in the state variable
    scan_msgs_[robot.idx] = scan_msg;

    // Odom Msgs
#if ROS == 1
    nav_msgs::Odometry odom_msg;
#elif ROS == 2
    nav_msgs::msg::Odometry odom_msg;
#endif
    frame_id = (robot.idx == 0) ? "odom" : robot.name + "/odom";
    odom_msg.header.frame_id = frame_id;

    // Store in state variable
    odom_msgs_[robot.idx] = odom_msg;
  }
}

void SimROS::publishROS() {
#if ROS == 1
  auto now = ros::Time::now();
#elif ROS == 2
  auto now = node_->now();
#endif
  for (auto& robot : sim_->robots()) {
    std::string prefix = (robot.idx == 0) ? "" : robot.name + "/";
    scan_msgs_[robot.idx].header.stamp = now;
    scan_msgs_[robot.idx].ranges = robot.laser_data;

    odom_msgs_[robot.idx].header.stamp = now;
    odom_msgs_[robot.idx].pose.pose.position.x = robot.entity.x();
    odom_msgs_[robot.idx].pose.pose.position.y = robot.entity.y();
    odom_msgs_[robot.idx].pose.pose.position.z = 0;
    auto q = quaternionFromEuler(0, 0, robot.entity.theta());
    odom_msgs_[robot.idx].pose.pose.orientation = q;
    odom_msgs_[robot.idx].twist.twist.linear.x = robot.entity.vx();
    odom_msgs_[robot.idx].twist.twist.linear.y = robot.entity.vy();
    odom_msgs_[robot.idx].twist.twist.angular.z = robot.entity.omega();

#if ROS == 1
    scan_pubs_[robot.idx].publish(scan_msgs_[robot.idx]);
    odom_pubs_[robot.idx].publish(odom_msgs_[robot.idx]);
#elif ROS == 2
    scan_pubs_[robot.idx]->publish(scan_msgs_[robot.idx]);
    odom_pubs_[robot.idx]->publish(odom_msgs_[robot.idx]);
#endif
    odom_msgs_[robot.idx].twist.twist.linear.x = robot.entity.g_vx();
    odom_msgs_[robot.idx].twist.twist.linear.y = robot.entity.g_vy();
    odom_msgs_[robot.idx].twist.twist.angular.z = robot.entity.g_omega();
#if ROS == 1
    ground_truth_pubs_[robot.idx].publish(odom_msgs_[robot.idx]);

    std::vector<geometry_msgs::TransformStamped> transforms;

    // odom -> base_footprint
    geometry_msgs::TransformStamped t;
#elif ROS == 2
    ground_truth_pubs_[robot.idx]->publish(odom_msgs_[robot.idx]);

    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // odom -> base_footprint
    geometry_msgs::msg::TransformStamped t;
#endif
    t.header.stamp = now;
    t.header.frame_id = prefix + "odom";
    t.child_frame_id = prefix + "base_footprint";
    t.transform.translation.x = robot.entity.x();
    t.transform.translation.y = robot.entity.y();
    t.transform.translation.z = 0.0;
    q = quaternionFromEuler(0, 0, robot.entity.theta());
    t.transform.rotation = q;
    transforms.push_back(t);

    // base_footprint -> base_link
    t.header.stamp = now;
    t.header.frame_id = prefix + "base_footprint";
    t.child_frame_id = prefix + "base_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    q = quaternionFromEuler(0, 0, 0);
    t.transform.rotation = q;
    transforms.push_back(t);

    // base_link -> base_laser_link
    t.header.stamp = now;
    t.header.frame_id = prefix + "base_link";
    t.child_frame_id = prefix + "base_laser_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.1;
    q = quaternionFromEuler(0, 0, 0);
    t.transform.rotation = q;
    transforms.push_back(t);

    // base_link -> head_rotation_frame
    t.header.stamp = now;
    t.header.frame_id = prefix + "base_link";
    t.child_frame_id = prefix + "head_rotation_frame";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 1.0;
    q = quaternionFromEuler(0, 0, robot.head_rotation);
    t.transform.rotation = q;
    transforms.push_back(t);

    for (auto& transform : transforms) {
#if ROS == 1
      tf_broadcaster_.sendTransform(transform);
#elif ROS == 2
      tf_broadcaster_->sendTransform(transform);
#endif
    }
  }
}

#if ROS == 1
geometry_msgs::Quaternion SimROS::quaternionFromEuler(double roll, double pitch, double yaw) {
  geometry_msgs::Quaternion q;
#elif ROS == 2
geometry_msgs::msg::Quaternion SimROS::quaternionFromEuler(double roll, double pitch, double yaw) {
  geometry_msgs::msg::Quaternion q;
#endif

  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

void SimROS::updateWorld() {
  if (sim_->stepSimulation()) {
#if ROS == 1
    sim_time_.fromSec(sim_->getSimTime());
#elif ROS == 2
    sim_time_ = rclcpp::Time(static_cast<int64_t>(sim_->getSimTime() * 1e9), RCL_ROS_TIME);
#endif
    publishROS();
#if ROS == 1
    rosgraph_msgs::Clock clock_msg;
#elif ROS == 2
    rosgraph_msgs::msg::Clock clock_msg;
#endif
    clock_msg.clock = sim_time_;
#if ROS == 1
    clock_pub_.publish(clock_msg);
#elif ROS == 2
    clock_pub_->publish(clock_msg);
#endif
  } else {
    quit_sim_ = true;
  }
}

#if ROS == 1
void SimROS::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg, int robot_idx) {
#elif ROS == 2
void SimROS::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg, int robot_idx) {
#endif
  // set velocity to a particular agent
  sim_->setRobotVelocity(robot_idx, msg->linear.x, msg->linear.y, msg->angular.z);
}

#if ROS == 1
void SimROS::headRotationCallback(const geometry_msgs::Vector3ConstPtr& msg, int robot_idx) {
#elif ROS == 2
void SimROS::headRotationCallback(const geometry_msgs::msg::Vector3::SharedPtr msg, int robot_idx) {
#endif
  // set head rotation to a particular agent
  sim_->setHeadRotation(robot_idx, msg->z);
}

}  // namespace cohan_sim

int main(int argc, char** argv) {
#if ROS == 1
  ros::init(argc, argv, "simros");  // Correct initialization
#elif ROS == 2
  rclcpp::init(argc, argv);
#endif

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " arg1 (optional) arg2 (optional)  <world_file.yaml>" << std::endl;
    return 1;
  }

  bool gui = false;
  double rate = 60;

  for (int i = 0; i < (argc - 1); i++) {
    if (!strcmp(argv[i], "-g")) {
      gui = true;
    }
    if (!strcmp(argv[i], "-f")) {
      rate = 200;
    }
  }

  if (gui && rate == 200) {
#if ROS == 1
    ROS_ERROR("Fast mode cannot be run with GUI on");
#elif ROS == 2
    RCLCPP_ERROR(rclcpp::get_logger("simros"), "Fast mode cannot be run with GUI on");
#endif
    rate = 60;
  }

  cohan_sim::SimROS sn(argv[argc - 1], gui);

#if ROS == 1
  ros::WallRate r(rate);
  while (ros::ok() && !sn.quitSim()) {
    ros::spinOnce();  // Process ROS callbacks
    sn.updateWorld();
    r.sleep();
  }
#elif ROS == 2
  rclcpp::WallRate r(rate);
  while (rclcpp::ok() && !sn.quitSim()) {
    rclcpp::spin_some(sn.getNode());  // Process ROS callbacks
    sn.updateWorld();
    r.sleep();
  }
  rclcpp::shutdown();
#endif

  return 0;
}
