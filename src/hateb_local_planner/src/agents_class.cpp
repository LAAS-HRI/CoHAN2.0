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

#include <algorithm>
#include <hateb_local_planner/agents_class.hpp>
#include <numeric>

// Configuarable parameters
#define AGENTS_SUB_TOPIC "/tracked_agents"

namespace hateb_local_planner {

Agents::Agents(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, std::shared_ptr<HATebConfig> cfg)
    : node_(node), initialized_(false), cfg_(cfg) {
  // call the initialize class to start things
  if (!initialized_) {
    // node->get_parameter_or("ns", ns_, std::string(""));
    // node->get_parameter_or("map_frame", map_frame_, std::string("map"));
    // node->get_parameter_or("planning_mode", planning_mode_, 0);

    // Initialize the publisher
    agents_info_pub_ = node_->create_publisher<agent_path_prediction::msg::AgentsInfo>("agents_info", 10);

    tracked_agents_sub_topic_ = AGENTS_SUB_TOPIC;
    // Need to remap subscriber properly
    if (!cfg_->ns.empty()) {
      tracked_agents_sub_topic_ = "/" + cfg_->ns + tracked_agents_sub_topic_;
    }

    // Subscribers
    tracked_agents_sub_ = node_->create_subscription<cohan_msgs::msg::TrackedAgents>(tracked_agents_sub_topic_, 1, std::bind(&Agents::trackedAgentsCB, this, std::placeholders::_1));
    // Initialize variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    stuck_ = false;
    stuck_agent_id_ = -1;
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "Agents initialized");

    // spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    // rclcpp::spin_some(node_->get_node_base_interface());
  }
}

void Agents::trackedAgentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr tracked_agents) {
  tracked_agents_ = tracked_agents;

  // Msg for publishing agents info
  agent_path_prediction::msg::AgentsInfo agents_info;

  geometry_msgs::msg::TransformStamped transform_stamped;
  auto base_link = cfg_->base_frame;
  if (!cfg_->ns.empty()) {
    base_link = cfg_->ns + "/" + base_link;
  }

  // Get the robot pose
  try {
    transform_stamped = tf_->lookupTransform(cfg_->global_frame, base_link, tf2::TimePointZero, tf2::durationFromSec(0.5));
    auto xpos = transform_stamped.transform.translation.x;
    auto ypos = transform_stamped.transform.translation.y;
    auto ryaw = tf2::getYaw(transform_stamped.transform.rotation);
    Eigen::Vector2d robot_vec(std::cos(ryaw), std::sin(ryaw));
    geometry_msgs::msg::Pose2D robot_pose;
    robot_pose.x = xpos;
    robot_pose.y = ypos;
    robot_pose.theta = ryaw;

    // Add robot pose to the info message
    agents_info.robot_pose = robot_pose;

    agent_vels_.clear();
    visible_agent_ids_.clear();
    agent_still_.clear();

    std::map<double, int> agent_dist_id_map;
    std::map<int, double> agents_radii;
    std::map<int, agent_path_prediction::msg::HumanInfo> humans_info;

    for (auto& agent : tracked_agents_->agents) {
      agent_path_prediction::msg::HumanInfo human_info;
      auto h_id = agent.track_id;
      human_info.id = h_id;
      human_info.name = agent.name;
      if (agent.type == 1) {
        agents_radii[h_id] = cfg_->agent.radius;
      } else {
        agents_radii[h_id] = cfg_->robot.radius;
      }

      if (agents_states_.size() < tracked_agents_->agents.size()) {
        // Add the agent id and state
        human_info.state = hateb_local_planner::AgentState::NO_STATE;

        // Update the class variable
        if (agents_states_.find(h_id) == agents_states_.end()) {
          agents_states_[h_id] = hateb_local_planner::AgentState::NO_STATE;
        }

        std::vector<double> h_vels;
        agent_vels_[h_id] = h_vels;
        agent_nominal_vels_[h_id] = 0.0;
        geometry_msgs::msg::Pose h_pose;
        agents_[h_id] = h_pose;
      }
      // double h_xpos, h_ypos;
      for (auto& segment : agent.segments) {
        if (segment.type == DEFAULT_AGENT_SEGMENT) {
          agents_[h_id] = segment.pose.pose;
          double h_xpos = segment.pose.pose.position.x;
          double h_ypos = segment.pose.pose.position.y;

          Eigen::Vector2d rh_vec(h_xpos - xpos, h_ypos - ypos);
          auto hr_dist = rh_vec.norm();
          // Add the agent distance to robot
          human_info.dist = hr_dist;
          if (hr_dist < cfg_->planning_radius && rh_vec.dot(robot_vec) >= 0) {
            // Update the dist map
            agent_dist_id_map[hr_dist] = h_id;
          }

          agent_vels_[agent.track_id].push_back(std::hypot(segment.twist.twist.linear.x, segment.twist.twist.linear.y));

          // Human state update --> MOVING
          if ((abs(segment.twist.twist.linear.x) + abs(segment.twist.twist.linear.y) + abs(segment.twist.twist.angular.z)) > CALC_EPS) {
            if (agents_states_.find(h_id) != agents_states_.end() && agents_states_[h_id] != hateb_local_planner::AgentState::BLOCKED) {
              agents_states_[h_id] = hateb_local_planner::AgentState::MOVING;
              // Update the state info message
              human_info.state = hateb_local_planner::AgentState::MOVING;
            }
          }

          auto n = agent_vels_[h_id].size();
          float average_vel = 0.0F;
          if (n != 0) {
            average_vel = accumulate(agent_vels_[h_id].begin(), agent_vels_[h_id].end(), 0.0) / n;
          }
          agent_nominal_vels_[h_id] = average_vel;

          if (n == cfg_->agent.num_moving_avg) {
            agent_vels_[h_id].erase(agent_vels_[h_id].begin());
          }

          // Check if the human is still or halted
          if (prev_agents_.find(h_id) != prev_agents_.end()) {
            double human_move_dist = std::hypot(h_xpos - prev_agents_[h_id].position.x, h_ypos - prev_agents_[h_id].position.y);
            if (human_move_dist < CALC_EPS && agents_states_.find(h_id) != agents_states_.end()) {
              agent_still_[h_id] = true;

              // Human state update --> STOPPED
              if (agents_states_[h_id] == hateb_local_planner::AgentState::STOPPED || agents_states_[h_id] == hateb_local_planner::AgentState::MOVING) {
                // Update the state info message
                agents_states_[h_id] = hateb_local_planner::AgentState::STOPPED;
                human_info.state = hateb_local_planner::AgentState::STOPPED;
              }
            } else {
              agent_still_[h_id] = false;
              agents_states_[h_id] = hateb_local_planner::AgentState::MOVING;
              human_info.state = hateb_local_planner::AgentState::MOVING;
            }
          }

          // Check if the human is stuck and update the angle for backoff recovery
          if (h_id == stuck_agent_id_) {
            stuck_ = hr_dist <= cfg_->planning_radius && rh_vec.dot(robot_vec) >= 0;
          }

          // agents_info.humans.push_back(human_info);
          humans_info[h_id] = human_info;
        }
      }
    }
    prev_agents_ = agents_;

    // Get the distance sorted list of visible ids
    visible_agent_ids_.clear();

    for (auto& dist_id_map : agent_dist_id_map) {
      visible_agent_ids_.push_back(dist_id_map.second);
    }

    std::vector<int> sorted_ids;
    if (cfg_->robot.use_simulated_fov) {
      /**************** for a centralised perception ***************/
      sorted_ids = filterVisibleAgents(agents_, visible_agent_ids_, agents_radii, robot_pose);
    } else {
      sorted_ids = visible_agent_ids_;
    }

    // Causes the backoff to clear as soon as the agent goes out of the FOV(need to have a timeout?)
    // if (!std::binary_search(sorted_ids.begin(), sorted_ids.end(), stuck_agent_id_)) {
    //   // id does not exist
    //   stuck_ = false;
    // }

    agents_info.visible = sorted_ids;

    for (auto& f_id : sorted_ids) {
      if (agents_states_[f_id] == hateb_local_planner::AgentState::NO_STATE || agents_states_[f_id] == hateb_local_planner::AgentState::STATIC) {
        agents_states_[f_id] = hateb_local_planner::AgentState::STATIC;
        humans_info[f_id].state = hateb_local_planner::AgentState::STATIC;
      }

      if (agent_still_.find(f_id) != agent_still_.end()) {
        if (agent_still_[f_id]) {
          agents_info.still.push_back(f_id);
        } else {
          agents_info.moving.push_back(f_id);
        }
      }

      agents_info.humans.push_back(humans_info[f_id]);
    }

    // Safety step for agents if agent_layers is not added in local costmap
    // Adds a temporary costmap around the agents to let planner plan safe
    // trajectories

    if (cfg_->planning_mode > 0) {
      for (int i = 0; i < sorted_ids.size() && i < agents_.size(); i++) {
        geometry_msgs::msg::Point v1;
        geometry_msgs::msg::Point v2;
        geometry_msgs::msg::Point v3;
        geometry_msgs::msg::Point v4;
        auto idx = sorted_ids[i];
        auto agent_radius = agents_radii[idx];
        v1.x = agents_[idx].position.x - agent_radius, v1.y = agents_[idx].position.y - agent_radius, v1.z = 0.0;
        v2.x = agents_[idx].position.x - agent_radius, v2.y = agents_[idx].position.y + agent_radius, v2.z = 0.0;
        v3.x = agents_[idx].position.x + agent_radius, v3.y = agents_[idx].position.y + agent_radius, v3.z = 0.0;
        v4.x = agents_[idx].position.x + agent_radius, v4.y = agents_[idx].position.y - agent_radius, v4.z = 0.0;

        std::vector<geometry_msgs::msg::Point> agent_pos_costmap;

        transform_stamped = tf_->lookupTransform(cfg_->map_frame, cfg_->global_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
        tf2::doTransform(v1, v1, transform_stamped);
        tf2::doTransform(v2, v2, transform_stamped);
        tf2::doTransform(v3, v3, transform_stamped);
        tf2::doTransform(v4, v4, transform_stamped);

        agent_pos_costmap.push_back(v1);
        agent_pos_costmap.push_back(v2);
        agent_pos_costmap.push_back(v3);
        agent_pos_costmap.push_back(v4);

        bool set_success = false;
        set_success = costmap_->setConvexPolygonCost(agent_pos_costmap, COST_OBS);
      }
    }

    agents_info_pub_->publish(agents_info);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", cfg_->global_frame.c_str(), base_link.c_str(), ex.what());
  }
}

std::vector<int> Agents::filterVisibleAgents(std::map<int, geometry_msgs::msg::Pose> tr_agents, std::vector<int> sorted_ids, std::map<int, double> agents_radii,
                                             geometry_msgs::msg::Pose2D robot_pose) {
  std::vector<int> filtered_ids;
  auto xpos = robot_pose.x;
  auto ypos = robot_pose.y;

  if (!stuck_) {
    int n = MAX_PTS;
    if (sorted_ids.size() >= AGENT_NUM_TH) {
      n = MIN_PTS;
    }
    for (auto& it : sorted_ids) {
      // Ray Tracing
      double tm_x = tr_agents[it].position.x;
      double tm_y = tr_agents[it].position.y;
      // Get the difference between poses along x and y
      auto dx = (tm_x - xpos);
      auto dy = (tm_y - ypos);
      // Define step size in each direction
      dx = dx / n;
      dy = dy / n;

      // Checking using raytracing
      bool cell_collision = false;
      double x = xpos;
      double y = ypos;

      for (int j = 0; j < n; j++) {
        unsigned int mx;
        unsigned int my;

        double check_rad = agents_radii[it] + 0.1;  // adding a small buffer

        if (sqrt(((x - tm_x) * (x - tm_x)) + ((y - tm_y) * (y - tm_y))) <= check_rad) {
          break;
        }
        if (costmap_->worldToMap(x, y, mx, my)) {
          auto cellcost = costmap_->getCost(mx, my);
          if ((int)(cellcost) > COST_MIN && (int)(cellcost) < COST_OBS) {
            cell_collision = true;
            break;
          }
          x += dx;
          y += dy;
        }
      }

      if (!cell_collision) {
        filtered_ids.push_back(it);
      }
    }
    return filtered_ids;

  } else {
    for (int it = 0; it < 2 && it < sorted_ids.size(); it++) {
      if (sorted_ids[it] == stuck_agent_id_) {
        filtered_ids.push_back(sorted_ids[it]);
        break;
      }
    }
  }
  return filtered_ids;
}

void Agents::resetAgents() {
  // Reset the variables
  agents_states_.clear();
  agent_nominal_vels_.clear();
  stuck_agent_id_ = -1;
  stuck_ = false;
}

}  // namespace hateb_local_planner
