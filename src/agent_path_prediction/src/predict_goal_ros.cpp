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

#include <agent_path_prediction/predict_goal_ros.hpp>

namespace agents {

void PredictGoalROS::initialize() {
  if (cfg_->goals_file.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Please provide a valid file path for goals files!");
  }

  goal_pub_ = node_->create_publisher<agent_path_prediction::msg::PredictedGoals>("~/predicted_goal", 2);

  // Need to remap tracked agents subscriber properly
  tracked_agents_sub_topic_ = cfg_->tracked_agents_sub_topic;
  if (!cfg_->ns.empty()) {
    tracked_agents_sub_topic_ = "/" + cfg_->ns + cfg_->tracked_agents_sub_topic;
  }

  // Initialize Subscribers
  agents_sub_ = node_->create_subscription<cohan_msgs::msg::TrackedAgents>(tracked_agents_sub_topic_, 1, std::bind(&PredictGoalROS::trackedAgentsCB, this, std::placeholders::_1));

  // Load goals file
  loadGoals(cfg_->goals_file);
  predictor_.initialize(goals_, window_size_);
  RCLCPP_INFO(node_->get_logger(), "Goal prediction intialized!");
}

void PredictGoalROS::trackedAgentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr msg) {
  bool changed = false;
  for (const auto& agent : msg->agents) {
    for (const auto& segment : agent.segments) {
      // Make sure you are getting the correct segment data
      if (segment.type == cohan_msgs::msg::TrackedSegmentType::TORSO) {
        auto xy = Eigen::Vector2d(segment.pose.pose.position.x, segment.pose.pose.position.y);
        if (agent_goal_predicts_.find(agent.track_id) == agent_goal_predicts_.end()) {
          agent_goal_predicts_[agent.track_id] = "None";
        }
        auto goal = predictor_.predictGoal(agent.track_id, xy);

        if (agent_goal_predicts_[agent.track_id] != goal) {
          changed = true;
        }
        // Update the predicted goals
        agent_goal_predicts_[agent.track_id] = goal;
      }
    }
  }
  if (changed) {
    // Publish the new goals
    agent_path_prediction::msg::PredictedGoals predicted_goals_msg;
    predicted_goals_msg.header.frame_id = "map";
    predicted_goals_msg.header.stamp = node_->now();
    for (auto& agent_goal : agent_goal_predicts_) {
      agent_path_prediction::msg::PredictedGoal p_goal;
      p_goal.id = agent_goal.first;
      p_goal.goal.position.x = goals_[agent_goal.second].x();
      p_goal.goal.position.y = goals_[agent_goal.second].y();
      p_goal.goal.position.z = 0.0;
      p_goal.goal.orientation.w = 1;
      predicted_goals_msg.goals.push_back(p_goal);
    }
    goal_pub_->publish(predicted_goals_msg);
  }
}

bool PredictGoalROS::loadGoals(const std::string& file) {
  goals_.clear();
  try {
    // Load goals from the YAML file
    YAML::Node config = YAML::LoadFile(file);

    window_size_ = config["window_size"].as<int>();
    const YAML::Node& goals = config["goals"];

    // Iterate through each goal
    for (const auto& goal : goals) {
      std::string name = goal["name"].as<std::string>();
      const auto& coordinates = goal["goal"];

      goals_[name] = Eigen::Vector2d(coordinates[0].as<double>(), coordinates[1].as<double>());
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return false;
  }
  return false;
}

};  // namespace agents