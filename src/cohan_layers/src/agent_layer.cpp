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

#include <angles/angles.h>

#include <cohan_layers/agent_layer.hpp>
#include <pluginlib/class_list_macros.hpp>

#define DEFAULT_AGENT_PART cohan_msgs::msg::TrackedSegmentType::TORSO
#define TRACKED_AGENTS_SUB "/tracked_agents"
#define AGENTS_STATES_SUB "/move_base/agentsInfo"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cohan_layers {
void AgentLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  // Common parameters for all agent layers
  cfg_ = std::make_shared<AgentLayerConfig>();
  cfg_->initialize(node, name_);
  cfg_->setupParameterCallback();

  // Get the robot radius costmap parameters
  robot_radius_ = node->get_parameter("robot_radius").as_double();

  // Map the subscriptions properly
  tracked_agents_sub_topic_ = std::string(TRACKED_AGENTS_SUB);
  agents_states_sub_topic_ = std::string(AGENTS_STATES_SUB);
  if (!cfg_->ns.empty()) {
    tracked_agents_sub_topic_ = "/" + cfg_->ns + tracked_agents_sub_topic_;
    agents_states_sub_topic_ = "/" + cfg_->ns + agents_states_sub_topic_;
  }

  agents_sub_ = node->create_subscription<cohan_msgs::msg::TrackedAgents>(tracked_agents_sub_topic_, 1, std::bind(&AgentLayer::agentsCB, this, std::placeholders::_1));

  agents_states_sub_ = node->create_subscription<agent_path_prediction::msg::AgentsInfo>(agents_states_sub_topic_, 1, std::bind(&AgentLayer::statesCB, this, std::placeholders::_1));

  stopmap_srv_ = node->create_service<std_srvs::srv::SetBool>(name_ + "/shutdown_layer", std::bind(&AgentLayer::shutdownCB, this, std::placeholders::_1, std::placeholders::_2));

  current_ = true;
  first_time_ = true;
  shutdown_ = false;
  reset_ = false;
  last_time_ = node->now();
}

void AgentLayer::agentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr agents) {
  std::lock_guard<std::recursive_mutex> lock(lock_);
  agents_ = *agents;
}

void AgentLayer::statesCB(const agent_path_prediction::msg::AgentsInfo::SharedPtr agents_info) {
  states_.clear();
  for (const auto& human : agents_info->humans) {
    states_[human.id] = static_cast<int>(human.state);
  }
  reset_ = false;
  auto node = node_.lock();
  if (node) {
    last_time_ = node->now();
  }
}

void AgentLayer::shutdownCB(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  shutdown_ = request->data;
  if (shutdown_) {
    response->success = true;
    response->message = "Shutting down the agent layer costmaps..";
  } else {
    response->success = true;
    response->message = "Agent layer is switched on !";
  }
}

void AgentLayer::updateBounds(double /*origin_x*/, double /*origin_y*/, double /*origin_z*/, double* min_x, double* min_y, double* max_x, double* max_y) {
  std::lock_guard<std::recursive_mutex> lock(lock_);

  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::string global_frame = layered_costmap_->getGlobalFrameID();
  transformed_agents_.clear();

  // if ((node->now() - last_time_).seconds() > 1.0) {
  //   reset_ = true;
  //   states_.clear();
  // }

  for (auto& agent : agents_.agents) {
    if ((node->now() - rclcpp::Time(agents_.header.stamp, node->get_clock()->get_clock_type())).seconds() > 0.1) {
      continue;
    }
    for (auto& segment : agent.segments) {
      if ((segment.type == DEFAULT_AGENT_PART) && !reset_) {
        // if (!states_.empty() && !shutdown_) {
        if (!shutdown_) {
          // if (states_[agent.track_id] != 0) {
          AgentPoseVel agent_pose_vel;
          agent_pose_vel.track_id = agent.track_id;
          agent_pose_vel.type = static_cast<int>(agent.type);
          agent_pose_vel.state = states_[agent.track_id];
          agent_pose_vel.header.frame_id = agents_.header.frame_id;
          agent_pose_vel.header.stamp = agents_.header.stamp;
          geometry_msgs::msg::PoseStamped before_pose;
          geometry_msgs::msg::PoseStamped after_pose;

          try {
            before_pose.pose = segment.pose.pose;
            before_pose.header.frame_id = agents_.header.frame_id;
            before_pose.header.stamp = agents_.header.stamp;
            tf_->transform(before_pose, after_pose, global_frame, tf2::durationFromSec(0.5));
            agent_pose_vel.pose = after_pose.pose;

            before_pose.pose.position.x += segment.twist.twist.linear.x;
            before_pose.pose.position.y += segment.twist.twist.linear.y;
            auto hb_yaw = tf2::getYaw(before_pose.pose.orientation);
            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, segment.twist.twist.angular.z + hb_yaw);
            before_pose.pose.orientation = tf2::toMsg(quat);
            tf_->transform(before_pose, after_pose, global_frame, tf2::durationFromSec(0.5));
            agent_pose_vel.velocity.linear.x = after_pose.pose.position.x - agent_pose_vel.pose.position.x;
            agent_pose_vel.velocity.linear.y = after_pose.pose.position.y - agent_pose_vel.pose.position.y;
            agent_pose_vel.velocity.angular.z = angles::shortest_angular_distance(tf2::getYaw(after_pose.pose.orientation), tf2::getYaw(agent_pose_vel.pose.orientation));

            transformed_agents_.push_back(agent_pose_vel);
          } catch (tf2::LookupException& ex) {
            RCLCPP_ERROR(node->get_logger(), "No Transform available Error: %s", ex.what());
            continue;
          } catch (tf2::ConnectivityException& ex) {
            RCLCPP_ERROR(node->get_logger(), "Connectivity Error: %s", ex.what());
            continue;
          } catch (tf2::ExtrapolationException& ex) {
            RCLCPP_ERROR(node->get_logger(), "Extrapolation Error: %s", ex.what());
            continue;
          }
          // }
        }
      } else if (reset_ && !shutdown_) {
        AgentPoseVel agent_pose_vel;
        agent_pose_vel.header.frame_id = agents_.header.frame_id;
        agent_pose_vel.header.stamp = agents_.header.stamp;
        geometry_msgs::msg::PoseStamped before_pose;
        geometry_msgs::msg::PoseStamped after_pose;

        try {
          before_pose.pose = segment.pose.pose;
          before_pose.header.frame_id = agents_.header.frame_id;
          before_pose.header.stamp = agents_.header.stamp;
          tf_->transform(before_pose, after_pose, global_frame, tf2::durationFromSec(0.5));
          agent_pose_vel.pose = after_pose.pose;

          before_pose.pose.position.x += segment.twist.twist.linear.x;
          before_pose.pose.position.y += segment.twist.twist.linear.y;
          auto hb_yaw = tf2::getYaw(before_pose.pose.orientation);
          tf2::Quaternion quat;
          quat.setRPY(0.0, 0.0, segment.twist.twist.angular.z + hb_yaw);
          before_pose.pose.orientation = tf2::toMsg(quat);
          tf_->transform(before_pose, after_pose, global_frame, tf2::durationFromSec(0.5));
          agent_pose_vel.velocity.linear.x = after_pose.pose.position.x - agent_pose_vel.pose.position.x;
          agent_pose_vel.velocity.linear.y = after_pose.pose.position.y - agent_pose_vel.pose.position.y;
          agent_pose_vel.velocity.angular.z = angles::shortest_angular_distance(tf2::getYaw(after_pose.pose.orientation), tf2::getYaw(agent_pose_vel.pose.orientation));

          transformed_agents_.push_back(agent_pose_vel);
        } catch (tf2::LookupException& ex) {
          RCLCPP_ERROR(node->get_logger(), "No Transform available Error: %s", ex.what());
          continue;
        } catch (tf2::ConnectivityException& ex) {
          RCLCPP_ERROR(node->get_logger(), "Connectivity Error: %s", ex.what());
          continue;
        } catch (tf2::ExtrapolationException& ex) {
          RCLCPP_ERROR(node->get_logger(), "Extrapolation Error: %s", ex.what());
          continue;
        }
      }
    }
  }

  updateBoundsFromAgents(min_x, min_y, max_x, max_y);
  if (first_time_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    first_time_ = false;
  } else {
    double a = *min_x;
    double b = *min_y;
    double c = *max_x;
    double d = *max_y;
    *min_x = std::min(last_min_x_, *min_x);
    *min_y = std::min(last_min_y_, *min_y);
    *max_x = std::max(last_max_x_, *max_x);
    *max_y = std::max(last_max_y_, *max_y);
    last_min_x_ = a;
    last_min_y_ = b;
    last_max_x_ = c;
    last_max_y_ = d;
  }
}

};  // namespace cohan_layers
