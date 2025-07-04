/**********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2025 LAAS/CNRS
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
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 *********************************************************************/

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <cohan_sim/simros.hpp>

namespace cohan_sim {
SimROS::SimROS(const char* filename, bool gui) {
  sim_ = std::make_unique<cohan_sim::Simulator2D>(filename, gui);

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

  // Initialize messages
  initMessages();

  // Initialize time
  sim_time_.fromSec(0.0);

  // Initialize state variables
  quit_sim_ = false;

  // Initialize tf_broadcaster_
  tf_broadcaster_ = tf2_ros::TransformBroadcaster();
}

SimROS::~SimROS() {}

void SimROS::initMessages() {
  for (auto& robot : sim_->robots()) {
    // Scan Msgs
    sensor_msgs::LaserScan scan_msg;
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
    nav_msgs::Odometry odom_msg;
    frame_id = (robot.idx == 0) ? "odom" : robot.name + "/odom";
    odom_msg.header.frame_id = frame_id;

    // Store in state variable
    odom_msgs_[robot.idx] = odom_msg;
  }
}

void SimROS::publishROS() {
  auto now = ros::Time::now();
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

    scan_pubs_[robot.idx].publish(scan_msgs_[robot.idx]);
    odom_pubs_[robot.idx].publish(odom_msgs_[robot.idx]);
    ground_truth_pubs_[robot.idx].publish(odom_msgs_[robot.idx]);

    std::vector<geometry_msgs::TransformStamped> transforms;

    // odom -> base_footprint
    geometry_msgs::TransformStamped t;
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
      tf_broadcaster_.sendTransform(transform);
    }
  }
}

geometry_msgs::Quaternion SimROS::quaternionFromEuler(double roll, double pitch, double yaw) {
  geometry_msgs::Quaternion q;

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
    sim_time_.fromSec(sim_->getSimTime());
    publishROS();
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = sim_time_;
    clock_pub_.publish(clock_msg);
  } else {
    quit_sim_ = true;
  }
}

void SimROS::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg, int robot_idx) {
  // set velocity to a particular agent
  sim_->setRobotVelocity(robot_idx, msg->linear.x, msg->linear.y, msg->angular.z);
}

void SimROS::headRotationCallback(const geometry_msgs::Vector3ConstPtr& msg, int robot_idx) {
  // set head rotation to a particular agent
  sim_->setHeadRotation(robot_idx, msg->z);
}

}  // namespace cohan_sim

int main(int argc, char** argv) {
  ros::init(argc, argv, "simros");  // Correct initialization

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
    ROS_ERROR("Fast mode cannot be run with GUI on");
    rate = 60;
  }

  cohan_sim::SimROS sn(argv[argc - 1], gui);

  ros::WallRate r(rate);
  while (ros::ok() && !sn.quitSim()) {
    ros::spinOnce();  // Process ROS callbacks
    sn.updateWorld();
    r.sleep();
  }

  return 0;
}
