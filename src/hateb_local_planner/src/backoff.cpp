/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2024-2025 LAAS-CNRS
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

#include <hateb_local_planner/backoff.h>

#define NODE_NAME "BackoffRecovery"

namespace hateb_local_planner {
// empty constructor and destructor
Backoff::Backoff(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, std::shared_ptr<HATebConfig> cfg) {
  // Initialize the backoff recovery behavior
  cfg_ = cfg;
  initialize(node, costmap_ros);
}

Backoff::~Backoff() = default;

void Backoff::initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  // get node from costmap_ros
  node_ = node;

  // Initialize tf2 listener with buffer
  tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  // If a namespace is associated update some topics accordingly
  auto publish_goal_topic = cfg_->backoff.publish_goal_topic;
  auto get_plan_srv_name = cfg_->backoff.get_plan_srv_name;
  if (!cfg_->ns.empty()) {
    publish_goal_topic = "/" + cfg_->ns + cfg_->backoff.publish_goal_topic;
    get_plan_srv_name = "/" + cfg_->ns + cfg_->backoff.get_plan_srv_name;
  }

  // Get costmap
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  // costmap_model_ = std::make_shared<nav2_costmap_2d::CostmapModel>(*costmap_);
  collision_checker_ = std::make_shared<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(costmap_);

  // Initialize ros topics, services and subscribers
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(publish_goal_topic, 1);

  // Initialize Action clients
  get_plan_client_ = rclcpp_action::create_client<ComputePathToPose>(node_, get_plan_srv_name);

  poly_pub_l_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>("left_polygon", 100);
  poly_pub_r_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>("right_polygon", 100);

  // Initialize the variables
  self_published_ = false;
  last_time_ = node_->now();

  RCLCPP_DEBUG(node_->get_logger(), "node %s initialized", NODE_NAME);
}

bool Backoff::startRecovery() {
  bool transform_found = false;
  auto r = robot_circumscribed_radius_;

  // Get the transform from robot frame to map frame
  geometry_msgs::msg::TransformStamped robot_to_map_tr;
  try {
    auto footprint_frame = cfg_->footprint_frame;
    if (!cfg_->ns.empty()) {
      footprint_frame = "/" + cfg_->ns + "/" + cfg_->footprint_frame;
    }
    robot_to_map_tr = tf_->lookupTransform(cfg_->global_frame, footprint_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
    transform_found = true;
    tf2::fromMsg(robot_to_map_tr.transform, robot_to_map_tf_);

  } catch (tf2::LookupException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "No Transform available Error: %s\n", ex.what());
  } catch (tf2::ConnectivityException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Connectivity Error: %s\n", ex.what());
  } catch (tf2::ExtrapolationException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Extrapolation Error: %s\n", ex.what());
  }

  geometry_msgs::msg::PoseStamped backoff_goal;
  backoff_goal.header.frame_id = "map";
  backoff_goal.header.stamp = node_->now();
  backoff_goal.pose.orientation.w = 1;

  if (transform_found && get_plan_client_) {
    std::vector<geometry_msgs::msg::Point32> right_grid_vis;
    std::vector<geometry_msgs::msg::Point32> left_grid_vis;

    // Set the polygons for searching (we assume square)
    const double right_grid_offsets_vis[4][2] = {{r, -r}, {r, -3 * r}, {-r, -3 * r}, {-r, -r}};
    const double left_grid_offsets_vis[4][2] = {{r, 3 * r}, {r, r}, {-r, r}, {-r, 3 * r}};

    // Initialize the necessary
    double back_dist = 0;
    double robot_theta = 0;
    double search_angle = 0.0;
    double angle_increment = 0.174;
    bool found = false;
    bool flipped = false;
    int count = 0;

    std::vector<geometry_msgs::msg::PoseStamped> goals;
    ComputePathToPose::Goal goal_msg;
    goal_msg.start.header.frame_id = cfg_->global_frame;
    goal_msg.start.pose.position.x = robot_to_map_tr.transform.translation.x;
    goal_msg.start.pose.position.y = robot_to_map_tr.transform.translation.y;
    goal_msg.start.pose.position.z = robot_to_map_tr.transform.translation.z;
    goal_msg.start.pose.orientation = robot_to_map_tr.transform.rotation;
    goal_msg.goal.header.frame_id = cfg_->global_frame;

    // Start the search
    while (true) {
      start_pose_tr_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2::Quaternion q;
      q.setRPY(0, 0, search_angle);
      start_pose_tr_.setRotation(q);
      start_pose_tr_ = robot_to_map_tf_ * start_pose_tr_;
      start_pose_ = tf2::toMsg(start_pose_tr_);
      robot_theta = tf2::getYaw(start_pose_.rotation);

      // Clearing visualizes only the lastest search angle.
      // left_grid_vis.clear();
      // right_grid_vis.clear();

      // Search upto 50 configurations for each angle
      // TODO: Needs to configure or adjust this number
      for (int i = 0; i < 50; i++) {
        if (cfg_->backoff.visualize) {
          tf2::Vector3 point_tf;
          point_tf.setZero();

          // Get the configuration on right
          for (const auto* offset : right_grid_offsets_vis) {
            point_tf.setValue(offset[0] - back_dist, offset[1], 0.0);
            point_tf = start_pose_tr_ * point_tf;

            geometry_msgs::msg::Point32 p32;
            p32.x = point_tf.x();
            p32.y = point_tf.y();
            p32.z = 0;
            right_grid_vis.push_back(p32);
          }

          // Get the configuration on left
          for (const auto* offset : left_grid_offsets_vis) {
            point_tf.setValue(offset[0] - back_dist, offset[1], 0.0);
            point_tf = start_pose_tr_ * point_tf;

            geometry_msgs::msg::Point32 p32;
            p32.x = point_tf.x();
            p32.y = point_tf.y();
            p32.z = 0;
            left_grid_vis.push_back(p32);
          }

          // Publish the Left and Right polygons
          geometry_msgs::msg::PolygonStamped r_polygon;
          r_polygon.header.frame_id = "map";
          r_polygon.header.stamp = node_->now();
          r_polygon.polygon.points = right_grid_vis;
          poly_pub_r_->publish(r_polygon);

          geometry_msgs::msg::PolygonStamped l_polygon;
          l_polygon.header.frame_id = "map";
          l_polygon.header.stamp = node_->now();
          l_polygon.polygon.points = left_grid_vis;
          poly_pub_l_->publish(l_polygon);
        }

        // Find the polygon center on right
        auto r_center = tf2::Vector3(-back_dist, -2 * r, 0.0);
        r_center = start_pose_tr_ * r_center;

        // Find the polygon center on left
        auto l_center = tf2::Vector3(-back_dist, 2 * r, 0.0);
        l_center = start_pose_tr_ * l_center;

        // Check overlap with costmap (right)
        if (collision_checker_->footprintCostAtPose(r_center.x(), r_center.y(), robot_theta, right_grid_) == 0) {
          // ROS_INFO("Found safe spot on right");
          backoff_goal.pose.position.x = r_center.x();
          backoff_goal.pose.position.y = r_center.y();
          backoff_goal.pose.position.z = 0;
          backoff_goal.pose.orientation = start_pose_.rotation;
          goals.push_back(backoff_goal);
          found = true;
          count++;
          break;
        }

        // Check overlap with costmap (left)
        if (collision_checker_->footprintCostAtPose(l_center.x(), l_center.y(), robot_theta, left_grid_) == 0) {
          // ROS_INFO("Found safe spot on left");
          backoff_goal.pose.position.x = l_center.x();
          backoff_goal.pose.position.y = l_center.y();
          backoff_goal.pose.position.z = 0;
          backoff_goal.pose.orientation = start_pose_.rotation;
          goals.push_back(backoff_goal);
          found = true;
          count++;
          break;
        }

        // Increment the distance and repeat
        back_dist += 0.5;
      }

      // Reset the distance for new angle
      back_dist = 0.0;

      // increase the angle to continue search
      if (!flipped) {
        search_angle += angle_increment;
        if (search_angle > M_PI / 2) {
          // Flip when positive limit is reached
          search_angle = -0.175;
          flipped = true;
        }
      }

      else {
        search_angle -= angle_increment;
        // Break when negative limit is reached
        if (search_angle < -M_PI / 2) {
          break;
        }
      }
    }
    if (!found) {
      RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "No safespot found with the current search range!");
      return false;
    }

    std::map<int, int> length_idx_map;

    // Interate through the positions and get the length of global plans.
    // The global plan lengths are then used to determine the backoff goal.
    for (int i = 0; i < count; i++) {
      goal_msg.goal.header.stamp = node_->now();
      goal_msg.goal.header.stamp = node_->now();
      goal_msg.goal.pose.position.x = goals[i].pose.position.x;
      goal_msg.goal.pose.position.y = goals[i].pose.position.y;
      goal_msg.goal.pose.position.z = 0;
      goal_msg.goal.pose.orientation = goals[i].pose.orientation;

      auto goal_handle_future = get_plan_client_->async_send_goal(goal_msg);
      if (goal_handle_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        if (auto goal_handle = goal_handle_future.get()) {
          auto result_future = get_plan_client_->async_get_result(goal_handle);
          if (result_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto result = result_future.get();
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED && !result.result->path.poses.empty()) {
              length_idx_map[result.result->path.poses.size()] = i;
            }
          }
        }
      }
    }
    // Map arranges the distances sorted according to length.
    // Select the one with shortest length for backoff.
    backoff_goal_ = goals[length_idx_map.begin()->second];
  }

  // Finally publish the backoff goal
  self_published_ = true;
  start_pose_tr_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  start_pose_tr_.setRotation(q);
  start_pose_tr_ = robot_to_map_tf_ * start_pose_tr_;
  start_pose_ = tf2::toMsg(start_pose_tr_);

  goal_pub_->publish(backoff_goal_);
  last_time_ = node_->now();

  return true;
}

bool Backoff::setbackGoal(geometry_msgs::msg::PoseStamped goal) {
  // Set back the goal that is existing before recovery
  RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Setting back the goal !!");
  goal.header.stamp = node_->now();
  goal.header.frame_id = "map";
  goal_pub_->publish(goal);
  RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Goal set back success");
  return true;
}

bool Backoff::timeOut() {
  // Return TRUE after a given timeout
  return (node_->now() - last_time_).seconds() > cfg_->backoff.timeout;
}

bool Backoff::isBackoffGoalReached() {
  // Get the transform from robot frame to map frame
  geometry_msgs::msg::TransformStamped robot_to_map_tr;
  try {
    auto footprint_frame = cfg_->footprint_frame;
    if (!cfg_->ns.empty()) {
      footprint_frame = "/" + cfg_->ns + "/" + cfg_->footprint_frame;
    }
    robot_to_map_tr = tf_->lookupTransform(cfg_->global_frame, footprint_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));

  } catch (tf2::LookupException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "No Transform available Error: %s\n", ex.what());
  } catch (tf2::ConnectivityException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Connectivity Error: %s\n", ex.what());
  } catch (tf2::ExtrapolationException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Extrapolation Error: %s\n", ex.what());
  }

  // True if Backoff goal is reached
  double delta_orient = normalize_angle(tf2::getYaw(backoff_goal_.pose.orientation) - tf2::getYaw(robot_to_map_tr.transform.rotation));
  double dg = std::hypot(backoff_goal_.pose.position.x - robot_to_map_tr.transform.translation.x, backoff_goal_.pose.position.y - robot_to_map_tr.transform.translation.y);

  return fabs(dg) < 0.1 && fabs(delta_orient) < 0.1;
}

}  // namespace hateb_local_planner
