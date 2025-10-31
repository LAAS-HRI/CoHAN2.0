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
#include <agent_path_prediction/agent_path_prediction.hpp>
#include <agent_path_prediction/predict_goal_ros.hpp>
#include <chrono>
#include <csignal>
#include <future>
#include <utility>

namespace agents {

void AgentPathPrediction::initialize() {
  // Initialize TF2 buffer and transform listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize Parameter Server
  cfg_ = std::make_shared<AgentPathPredictConfig>();
  cfg_->initialize(shared_from_this());
  cfg_->setupParameterCallback();

  // Initialize Publishers
  predicted_agents_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/predicted_agent_poses", 1);
  front_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/front_pose", 1);

  // Initialize Service servers (with node namespace)
  set_goal_srv_ = this->create_service<agent_path_prediction::srv::AgentGoal>("~/set_agent_goal", std::bind(&AgentPathPrediction::setGoal, this, std::placeholders::_1, std::placeholders::_2));
  predict_agents_server_ =
      this->create_service<agent_path_prediction::srv::AgentPosePredict>("~/predict_agent_poses", std::bind(&AgentPathPrediction::predictAgents, this, std::placeholders::_1, std::placeholders::_2));
  reset_prediction_services_server_ =
      this->create_service<std_srvs::srv::Empty>("~/reset_prediction_services", std::bind(&AgentPathPrediction::resetPredictionSrvs, this, std::placeholders::_1, std::placeholders::_2));

  // Need to remap subscriber properly
  tracked_agents_sub_topic_ = cfg_->tracked_agents_sub_topic;
  get_plan_srv_name_ = cfg_->get_plan_srv_name;
  if (!cfg_->ns.empty()) {
    tracked_agents_sub_topic_ = "/" + cfg_->ns + tracked_agents_sub_topic_;
    get_plan_srv_name_ = "/" + cfg_->ns + get_plan_srv_name_;
  }

  // Initialize Subscribers
  tracked_agents_sub_ = this->create_subscription<cohan_msgs::msg::TrackedAgents>(tracked_agents_sub_topic_, 1, std::bind(&AgentPathPrediction::trackedAgentsCB, this, std::placeholders::_1));
  external_paths_sub_ = this->create_subscription<cohan_msgs::msg::AgentPathArray>(cfg_->external_paths_sub_topic, 1, std::bind(&AgentPathPrediction::externalPathsCB, this, std::placeholders::_1));
  predicted_goal_sub_ =
      this->create_subscription<agent_path_prediction::msg::PredictedGoals>(cfg_->predicted_goal_topic, 1, std::bind(&AgentPathPrediction::predictedGoalCB, this, std::placeholders::_1));

  // Initialize Service clients
  get_plan_client_ = this->create_client<nav_msgs::srv::GetPlan>(get_plan_srv_name_);

  // Initialize properties
  showing_markers_ = false;
  got_new_agent_paths_ = false;
  got_external_goal_ = false;

  RCLCPP_INFO(this->get_logger(), "node %s initialized", NODE_NAME);
}

void AgentPathPrediction::trackedAgentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr tracked_agents) { tracked_agents_ = *tracked_agents; }

void AgentPathPrediction::externalPathsCB(const cohan_msgs::msg::AgentPathArray::SharedPtr external_paths) {
  external_paths_ = *external_paths;
  got_new_agent_paths_ = true;
}

void AgentPathPrediction::predictedGoalCB(const agent_path_prediction::msg::PredictedGoals::SharedPtr predicted_goals) { predicted_goals_ = *predicted_goals; }

void AgentPathPrediction::predictAgents(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) {
  switch (req->type) {
    case agent_path_prediction::srv::AgentPosePredict::Request::VELOCITY_OBSTACLE:
      predictAgentsVelObs(req, res);
      break;
    case agent_path_prediction::srv::AgentPosePredict::Request::EXTERNAL:
      predictAgentsExternal(req, res);
      break;
    case agent_path_prediction::srv::AgentPosePredict::Request::BEHIND_ROBOT:
      predictAgentsBehind(req, res);
      break;
    case agent_path_prediction::srv::AgentPosePredict::Request::PREDICTED_GOAL:
      predictAgentsGoal(req, res);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "%s: unknown prediction type %d", NODE_NAME, req->type);
      return;
  }
  if (cfg_->publish_markers) {
    // create new markers
    predicted_agents_markers_.markers.clear();

    for (auto predicted_agent : res->predicted_agents_poses) {
      if (!predicted_agent.poses.empty()) {
        auto first_pose_time = predicted_agent.poses[0].header.stamp;
        int marker_id = 0;

        for (auto predicted_agent_pose : predicted_agent.poses) {
          visualization_msgs::msg::Marker predicted_agent_marker;
          predicted_agent_marker.header.frame_id = predicted_agent_pose.header.frame_id;
          predicted_agent_marker.header.stamp = first_pose_time;
          predicted_agent_marker.id = (predicted_agent.id * MAX_AGENT_MARKERS) + marker_id++;
          predicted_agent_marker.type = visualization_msgs::msg::Marker::CYLINDER;
          predicted_agent_marker.action = visualization_msgs::msg::Marker::MODIFY;
          // assuming diagonal covariance matrix (with row-major order)
          predicted_agent_marker.scale.x = std::max(predicted_agent_pose.pose.covariance[0], MINIMUM_COVARIANCE_MARKERS);
          predicted_agent_marker.scale.y = std::max(predicted_agent_pose.pose.covariance[7], MINIMUM_COVARIANCE_MARKERS);
          predicted_agent_marker.scale.z = 0.01;
          predicted_agent_marker.color.a = 1.0;
          predicted_agent_marker.color.r = 0.0;
          predicted_agent_marker.color.g = 0.0;
          predicted_agent_marker.color.b = 1.0;
          rclcpp::Time predicted_time(predicted_agent_pose.header.stamp, this->get_clock()->get_clock_type());
          rclcpp::Time first_time(first_pose_time, this->get_clock()->get_clock_type());
          predicted_agent_marker.lifetime = rclcpp::Duration::from_seconds(MIN_MARKER_LIFETIME) + (predicted_time - first_time);
          predicted_agent_marker.pose.position.x = predicted_agent_pose.pose.pose.position.x;
          predicted_agent_marker.pose.position.y = predicted_agent_pose.pose.pose.position.y;
          // time on z axis
          predicted_agent_marker.pose.position.z = (predicted_time - first_time).seconds();
          predicted_agents_markers_.markers.push_back(predicted_agent_marker);
        }

        auto it = last_markers_size_map_.find(predicted_agent.id);
        if (it != last_markers_size_map_.end()) {
          while (it->second >= marker_id) {
            visualization_msgs::msg::Marker delete_agent_marker;
            delete_agent_marker.id = (predicted_agent.id * MAX_AGENT_MARKERS) + marker_id++;
            delete_agent_marker.action = visualization_msgs::msg::Marker::DELETE;
            predicted_agents_markers_.markers.push_back(delete_agent_marker);
          }
        }
        last_markers_size_map_[predicted_agent.id] = --marker_id;
      } else {
        RCLCPP_WARN(this->get_logger(), "no predicted poses for agent %d", predicted_agent.id);
      }
    }

    predicted_agents_pub_->publish(predicted_agents_markers_);
    showing_markers_ = true;

    RCLCPP_DEBUG(this->get_logger(), "published predicted agents");
  } else {
    if (showing_markers_) {
      predicted_agents_markers_.markers.clear();
      visualization_msgs::msg::Marker delete_agent_markers;
      delete_agent_markers.action = 3;  // visualization_msgs::Marker::DELETEALL;
      predicted_agents_markers_.markers.push_back(delete_agent_markers);
      predicted_agents_pub_->publish(predicted_agents_markers_);
      showing_markers_ = false;
    }
  }
}

void AgentPathPrediction::predictAgentsVelObs(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req,
                                              std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) const {
  RCLCPP_INFO(this->get_logger(), "Predicting poses for agent using Velocity Obstacle method");

  // validate prediction time
  if (req->predict_times.empty()) {
    RCLCPP_ERROR(this->get_logger(), "prediction times cannot be empty");
    return;
  }
  if (*std::min_element(req->predict_times.begin(), req->predict_times.end()) < 0.0) {
    RCLCPP_ERROR(this->get_logger(), "prediction time cannot be negative");
    return;
  }

  // get local refrence of agents
  auto agents = tracked_agents_.agents;
  auto track_frame = tracked_agents_.header.frame_id;
  auto track_time = tracked_agents_.header.stamp;

  if ((this->now() - track_time).seconds() > *std::max_element(req->predict_times.begin(), req->predict_times.end())) {
    RCLCPP_DEBUG(this->get_logger(), "agent data is older than maximum given prediction time, predicting nothing");
    return;
  }

  for (const auto& agent : agents) {
    RCLCPP_INFO(this->get_logger(), "Predicting poses for agent %lu", agent.track_id);
    if (std::find(req->ids.begin(), req->ids.end(), agent.track_id) == req->ids.end()) {
      continue;
    }
    for (auto segment : agent.segments) {
      if (segment.type == cfg_->default_agent_part) {
        // calculate future agent poses based on current velocity
        agent_path_prediction::msg::PredictedPoses predicted_poses;
        predicted_poses.id = agent.track_id;

        // get linear velocity of the agent
        tf2::Vector3 linear_vel(segment.twist.twist.linear.x, segment.twist.twist.linear.y, segment.twist.twist.linear.z);

        for (auto predict_time : req->predict_times) {
          // validate prediction time
          if (predict_time < 0) {
            RCLCPP_ERROR(this->get_logger(), "%s: prediction time cannot be negative (give %f)", NODE_NAME, predict_time);
            return;
          }

          geometry_msgs::msg::PoseWithCovarianceStamped predicted_pose;
          predicted_pose.header.frame_id = track_frame;
          rclcpp::Time track_time_rclcpp(track_time, this->get_clock()->get_clock_type());
          rclcpp::Time predicted_time = track_time_rclcpp + rclcpp::Duration::from_seconds(predict_time);
          predicted_pose.header.stamp = predicted_time;

          if (cfg_->velobs_use_ang && std::abs(segment.twist.twist.angular.z) > ANG_VEL_EPS) {
            // velocity multiplier is only applied to linear velocities
            double r = (std::hypot(linear_vel[0], linear_vel[1]) * cfg_->velobs_mul) / segment.twist.twist.angular.z;
            double theta = segment.twist.twist.angular.z * predict_time;
            double crd = r * 2 * std::sin(theta / 2);
            double alpha = std::atan2(linear_vel[1], linear_vel[0]) + (theta / 2);
            predicted_pose.pose.pose.position.x = segment.pose.pose.position.x + crd * std::cos(alpha);
            predicted_pose.pose.pose.position.y = segment.pose.pose.position.y + crd * std::sin(alpha);
            tf2::Quaternion q;
            q.setRPY(0, 0, tf2::getYaw(segment.pose.pose.orientation) + theta);
            predicted_pose.pose.pose.orientation = tf2::toMsg(q);
          } else {
            predicted_pose.pose.pose.position.x = segment.pose.pose.position.x + linear_vel[0] * predict_time * cfg_->velobs_mul;
            predicted_pose.pose.pose.position.y = segment.pose.pose.position.y + linear_vel[1] * predict_time * cfg_->velobs_mul;
            predicted_pose.pose.pose.orientation = segment.pose.pose.orientation;
          }

          // not using velocity multiplier for covariance matrix
          double xy_vel = hypot(linear_vel[0] * predict_time, linear_vel[1] * predict_time);
          // storing only x, y covariance in diagonal matrix
          predicted_pose.pose.covariance[0] = cfg_->velobs_min_rad + (cfg_->velobs_max_rad - cfg_->velobs_min_rad) * (predict_time / cfg_->velobs_max_rad_time) * xy_vel;
          predicted_pose.pose.covariance[7] = predicted_pose.pose.covariance[0];
          predicted_poses.poses.push_back(predicted_pose);

          RCLCPP_DEBUG(this->get_logger(), "%s: predicted agent (%lu) segment (%d) pose: x=%f, y=%f, theta=%f, predict-time=%f", NODE_NAME, agent.track_id, segment.type,
                       predicted_pose.pose.pose.position.x, predicted_pose.pose.pose.position.y, tf2::getYaw(predicted_pose.pose.pose.orientation), predict_time);
        }

        geometry_msgs::msg::TwistStamped current_twist;
        current_twist.header.frame_id = track_frame;
        current_twist.header.stamp = track_time;
        current_twist.twist = segment.twist.twist;
        predicted_poses.start_velocity = current_twist;
        RCLCPP_INFO(this->get_logger(), "%s: predicted %lu poses for agent %lu", NODE_NAME, predicted_poses.poses.size(), agent.track_id);
        res->predicted_agents_poses.push_back(predicted_poses);
      }
    }
  }
}

void AgentPathPrediction::predictAgentsExternal(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req,
                                                std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) {
  // Using external paths
  if (!external_paths_.paths.empty()) {
    auto external_paths = external_paths_;
    auto tracked_agents = tracked_agents_;

    std::vector<AgentPathVel> agent_path_vel_array;
    for (const auto& path : external_paths.paths) {
      AgentPathVel agent_path_vel{.id = path.id, .path = path.path};

      // set starting velocity of the agent if we find them
      // we do not add current pose at first pose in this case
      for (auto& agent : tracked_agents.agents) {
        if (agent.track_id == path.id) {
          for (auto& segment : agent.segments) {
            if (segment.type == cfg_->default_agent_part) {
              agent_path_vel.start_vel = segment.twist;
              break;
            }
          }
          break;
        }
      }
      agent_path_vel_array.push_back(agent_path_vel);
    }
    path_vels_ = agent_path_vel_array;
    predictAgentsFromPaths(req, res);
    return;
  }

  // Using an external goal
  if (got_external_goal_) {
    auto now = this->now();
    auto tracked_agents = tracked_agents_;
    std::map<uint64_t, geometry_msgs::msg::PoseStamped> ext_goal;

    // get robot pose
    geometry_msgs::msg::TransformStamped robot_to_map_tf;
    geometry_msgs::msg::TransformStamped agent_to_map_tf;
    bool transforms_found = false;
    try {
      robot_to_map_tf = tf_buffer_->lookupTransform(cfg_->map_frame_id, cfg_->robot_frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));

      std::string agents_frame = "map";
      if (!tracked_agents.header.frame_id.empty()) {
        agents_frame = tracked_agents.header.frame_id;
      }
      agent_to_map_tf = tf_buffer_->lookupTransform(cfg_->map_frame_id, agents_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));

      transforms_found = true;
    } catch (tf2::LookupException& ex) {
      RCLCPP_ERROR(this->get_logger(), "No Transform available Error: %s\n", ex.what());
    } catch (tf2::ConnectivityException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Connectivity Error: %s\n", ex.what());
    } catch (tf2::ExtrapolationException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Extrapolation Error: %s\n", ex.what());
    }

    // first check if path calculation is needed, and for whom
    std::vector<AgentStartPoseVel> agent_start_pose_vels;
    std::vector<bool> start_poses_far;
    int idx_order = 0;
    for (auto& agent : tracked_agents.agents) {
      path_vels_pos_.push_back(-1);
      if (std::find(req->ids.begin(), req->ids.end(), agent.track_id) == req->ids.end()) {
        continue;
      }
      bool path_exist = false;
      for (auto& ex_gl : external_goals_) {
        if (ex_gl.id == agent.track_id) {
          ext_goal[ex_gl.id] = ex_gl.pose;
          break;
        }
      }
      for (const auto& path_vel : path_vels_) {
        if (path_vel.id == agent.track_id) {
          path_exist = true;
          break;
        }
      }

      // get agent pose
      for (auto& segment : agent.segments) {
        if (segment.type == cfg_->default_agent_part) {
          geometry_msgs::msg::PoseStamped agent_start;
          agent_start.header.frame_id = tracked_agents.header.frame_id;
          agent_start.header.stamp = now;
          agent_start.pose = segment.pose.pose;

          // TODO: Remove unncessary conversions
          tf2::Transform start_pose_tf;
          start_pose_tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
          geometry_msgs::msg::Pose start_pose;
          start_pose.orientation.w = 1.0;
          tf2::fromMsg(agent_start.pose, start_pose_tf);
          tf2::Transform agent_to_map_transform;
          tf2::fromMsg(agent_to_map_tf.transform, agent_to_map_transform);
          start_pose_tf = agent_to_map_transform * start_pose_tf;
          tf2::toMsg(start_pose_tf, start_pose);

          if (!path_exist) {
            AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
            agent_start_pose_vels.push_back(agent_start_pose_vel);
            path_vels_pos_[agent.track_id - 1] = idx_order;
          } else {
            if (std::find(req->ids.begin(), req->ids.end(), agent.track_id) != req->ids.end()) {
              double dist_far = std::hypot(agent_start.pose.position.x - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.x,
                                           agent_start.pose.position.y - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.y);
              if (dist_far > RECALC_DIST) {  // To ensure that the path is recalculated only if the agent is deviating from the path
                start_poses_far.push_back(true);
                AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
                agent_start_pose_vels.push_back(agent_start_pose_vel);
                path_vels_pos_[agent.track_id - 1] = idx_order;
                path_vels_.clear();
              }
            }
          }
          break;
        }
      }
      idx_order++;
    }

    if (!agent_start_pose_vels.empty()) {
      if (transforms_found) {
        for (auto& agent_start_pose_vel : agent_start_pose_vels) {
          auto get_plan_srv = std::make_shared<nav_msgs::srv::GetPlan::Request>();
          if (ext_goal.find(agent_start_pose_vel.id) == ext_goal.end()) {
            continue;
          }
          // get agent pose in map frame
          // (tf2)
          tf2::Transform start_pose_tf;
          start_pose_tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
          tf2::fromMsg(agent_start_pose_vel.pose.pose, start_pose_tf);
          tf2::Transform agent_to_map_transform;
          tf2::fromMsg(agent_to_map_tf.transform, agent_to_map_transform);
          start_pose_tf = agent_to_map_transform * start_pose_tf;
          auto start_pose_stamped = agent_start_pose_vel.pose;
          tf2::toMsg(start_pose_tf, start_pose_stamped.pose);
          auto start_path = setFixedPath(start_pose_stamped);
          //(tf2)

          get_plan_srv->start.header.frame_id = cfg_->map_frame_id;
          get_plan_srv->start.header.stamp = now;
          get_plan_srv->start.pose = start_path.poses.back().pose;
          front_pose_pub_->publish(start_path.poses.back());

          get_plan_srv->goal.header.frame_id = cfg_->map_frame_id;
          get_plan_srv->goal.header.stamp = now;
          get_plan_srv->goal.pose.position.x = ext_goal[agent_start_pose_vel.id].pose.position.x;
          get_plan_srv->goal.pose.position.y = ext_goal[agent_start_pose_vel.id].pose.position.y;
          get_plan_srv->goal.pose.position.z = ext_goal[agent_start_pose_vel.id].pose.position.z;
          get_plan_srv->goal.pose.orientation = ext_goal[agent_start_pose_vel.id].pose.orientation;

          RCLCPP_DEBUG(this->get_logger(), "agent start: x=%.2f, y=%.2f, theta=%.2f, goal: x=%.2f, y=%.2f, theta=%.2f", get_plan_srv->start.pose.position.x, get_plan_srv->start.pose.position.y,
                       tf2::getYaw(get_plan_srv->start.pose.orientation), get_plan_srv->goal.pose.position.x, get_plan_srv->goal.pose.position.y, tf2::getYaw(get_plan_srv->goal.pose.orientation));

          // make plan for agent
          if (get_plan_client_) {
            auto future = get_plan_client_->async_send_request(get_plan_srv);
            if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
              auto response = future.get();
              if (!response->plan.poses.empty()) {
                AgentPathVel agent_path_vel;
                agent_path_vel.id = agent_start_pose_vel.id;
                agent_path_vel.path = response->plan;
                agent_path_vel.start_vel = agent_start_pose_vel.vel;
                path_vels_.push_back(agent_path_vel);
                got_new_agent_paths_ = true;
              } else {
                RCLCPP_WARN(this->get_logger(), "Got empty path for agent, start or goal position is probably invalid");
              }
            } else {
              RCLCPP_WARN(this->get_logger(), "Failed to call %s service", cfg_->get_plan_srv_name.c_str());
            }
          } else {
            RCLCPP_WARN(this->get_logger(), "%s service does not exist, re-trying to subscribe", cfg_->get_plan_srv_name.c_str());
            get_plan_client_ = this->create_client<nav_msgs::srv::GetPlan>(cfg_->get_plan_srv_name);
          }
        }
      }
    }
    predictAgentsFromPaths(req, res);
    return;
  }

  std::vector<AgentPathVel> empty_path_vels;
  path_vels_ = empty_path_vels;
  predictAgentsFromPaths(req, res);
}

void AgentPathPrediction::predictAgentsBehind(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req,
                                              std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) {
  auto now = this->now();
  auto tracked_agents = tracked_agents_;

  // get robot pose
  geometry_msgs::msg::TransformStamped robot_to_map_tf;
  geometry_msgs::msg::TransformStamped agent_to_map_tf;
  bool transforms_found = false;
  try {
    robot_to_map_tf = tf_buffer_->lookupTransform(cfg_->map_frame_id, cfg_->robot_frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));
    std::string agents_frame = "map";
    if (!tracked_agents.header.frame_id.empty()) {
      agents_frame = tracked_agents.header.frame_id;
    }
    agent_to_map_tf = tf_buffer_->lookupTransform(cfg_->map_frame_id, agents_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));

    transforms_found = true;
  } catch (tf2::LookupException& ex) {
    RCLCPP_ERROR(this->get_logger(), "No Transform available Error: %s\n", ex.what());
  } catch (tf2::ConnectivityException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Connectivity Error: %s\n", ex.what());
  } catch (tf2::ExtrapolationException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Extrapolation Error: %s\n", ex.what());
  }

  // first check if path calculation is needed, and for whom
  std::vector<AgentStartPoseVel> agent_start_pose_vels;
  std::vector<bool> start_poses_far;
  int idx_order = 0;
  for (auto& agent : tracked_agents.agents) {
    path_vels_pos_.push_back(-1);
    if (std::find(req->ids.begin(), req->ids.end(), agent.track_id) == req->ids.end()) {
      continue;
    }
    bool path_exist = false;
    for (const auto& path_vel : path_vels_) {
      if (path_vel.id == agent.track_id) {
        path_exist = true;
        break;
      }
    }

    // get agent pose
    for (auto& segment : agent.segments) {
      if (segment.type == cfg_->default_agent_part) {
        geometry_msgs::msg::PoseStamped agent_start;
        agent_start.header.frame_id = tracked_agents.header.frame_id;
        agent_start.header.stamp = now;
        agent_start.pose = segment.pose.pose;

        tf2::Transform start_pose_tf;
        start_pose_tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
        geometry_msgs::msg::Pose start_pose;
        start_pose.orientation.w = 1.0;
        tf2::fromMsg(agent_start.pose, start_pose_tf);
        tf2::Transform agent_to_map_transform;
        tf2::fromMsg(agent_to_map_tf.transform, agent_to_map_transform);
        start_pose_tf = agent_to_map_transform * start_pose_tf;
        tf2::toMsg(start_pose_tf, start_pose);

        if (!path_exist) {
          AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
          agent_start_pose_vels.push_back(agent_start_pose_vel);
          path_vels_pos_[agent.track_id - 1] = idx_order;
        } else {
          if (std::find(req->ids.begin(), req->ids.end(), agent.track_id) != req->ids.end()) {
            double dist_far = std::hypot(agent_start.pose.position.x - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.x,
                                         agent_start.pose.position.y - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.y);

            if (dist_far > RECALC_DIST) {  // To ensure that the path is recalculated only if the agent is deviating from the path
              start_poses_far.push_back(true);
              AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
              agent_start_pose_vels.push_back(agent_start_pose_vel);
              path_vels_pos_[agent.track_id - 1] = idx_order;
              path_vels_.clear();
            }
          }
        }
        break;
      }
    }
    idx_order++;
  }

  if (!agent_start_pose_vels.empty()) {
    if (transforms_found) {
      for (auto& agent_start_pose_vel : agent_start_pose_vels) {
        auto get_plan_srv = std::make_shared<nav_msgs::srv::GetPlan::Request>();
        auto hum_id = agent_start_pose_vel.id;

        // get agent pose in map frame
        // (tf2)
        tf2::Transform start_pose_tf;
        start_pose_tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
        tf2::fromMsg(agent_start_pose_vel.pose.pose, start_pose_tf);
        tf2::Transform agent_to_map_transform;
        tf2::fromMsg(agent_to_map_tf.transform, agent_to_map_transform);
        start_pose_tf = agent_to_map_transform * start_pose_tf;
        auto start_pose_stamped = agent_start_pose_vel.pose;
        tf2::toMsg(start_pose_tf, start_pose_stamped.pose);
        auto start_path = setFixedPath(start_pose_stamped);
        //(tf2)

        get_plan_srv->start.header.frame_id = cfg_->map_frame_id;
        get_plan_srv->start.header.stamp = now;
        get_plan_srv->start.pose = start_path.poses.back().pose;
        front_pose_pub_->publish(start_path.poses.back());

        // calculate agent pose behind robot
        if (!check_path_) {
          check_path_ = true;
          tf2::Transform behind_tr;
          behind_tr.setOrigin(tf2::Vector3(-cfg_->agent_dist_behind_robot, 0.0, 0.0));
          behind_tr.setRotation(tf2::Quaternion(0, 0, sin(cfg_->agent_angle_behind_robot / 2.0), cos(cfg_->agent_angle_behind_robot / 2.0)));
          tf2::Transform robot_to_map_transform;
          tf2::fromMsg(robot_to_map_tf.transform, robot_to_map_transform);
          behind_tr = robot_to_map_transform * behind_tr;
          behind_pose_ = tf2::toMsg(behind_tr);
        }
        get_plan_srv->goal.header.frame_id = cfg_->map_frame_id;
        get_plan_srv->goal.header.stamp = now;
        get_plan_srv->goal.pose.position.x = behind_pose_.translation.x;
        get_plan_srv->goal.pose.position.y = behind_pose_.translation.y;
        get_plan_srv->goal.pose.position.z = behind_pose_.translation.z;
        get_plan_srv->goal.pose.orientation = behind_pose_.rotation;

        RCLCPP_DEBUG(this->get_logger(), "agent start: x=%.2f, y=%.2f, theta=%.2f, goal: x=%.2f, y=%.2f, theta=%.2f", get_plan_srv->start.pose.position.x, get_plan_srv->start.pose.position.y,
                     tf2::getYaw(get_plan_srv->start.pose.orientation), get_plan_srv->goal.pose.position.x, get_plan_srv->goal.pose.position.y, tf2::getYaw(get_plan_srv->goal.pose.orientation));

        // make plan for agent
        if (get_plan_client_) {
          auto future = get_plan_client_->async_send_request(get_plan_srv);

          if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto response = future.get();
            if (!response->plan.poses.empty()) {
              AgentPathVel agent_path_vel;
              agent_path_vel.id = agent_start_pose_vel.id;
              agent_path_vel.path = response->plan;
              agent_path_vel.start_vel = agent_start_pose_vel.vel;
              path_vels_.push_back(agent_path_vel);
              got_new_agent_paths_ = true;
            } else {
              RCLCPP_WARN(this->get_logger(), "Got empty path for agent, start or goal position is probably invalid");
            }
          } else {
            RCLCPP_WARN(this->get_logger(), "Failed to call %s service", cfg_->get_plan_srv_name.c_str());
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "%s service does not exist, re-trying to subscribe", cfg_->get_plan_srv_name.c_str());
          get_plan_client_ = this->create_client<nav_msgs::srv::GetPlan>(cfg_->get_plan_srv_name);
        }
      }
    }
  }

  return predictAgentsFromPaths(req, res);
}

void AgentPathPrediction::predictAgentsGoal(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req,
                                            std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) {
  auto now = this->now();
  auto tracked_agents = tracked_agents_;
  std::map<int, geometry_msgs::msg::Pose> predicted_goals;

  for (auto& goal : predicted_goals_.goals) {
    predicted_goals[goal.id] = goal.goal;
  }

  // get robot pose
  geometry_msgs::msg::TransformStamped robot_to_map_tf;
  geometry_msgs::msg::TransformStamped agent_to_map_tf;
  bool transforms_found = false;
  try {
    robot_to_map_tf = tf_buffer_->lookupTransform(cfg_->map_frame_id, cfg_->robot_frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));
    std::string agents_frame = "map";
    if (!tracked_agents.header.frame_id.empty()) {
      agents_frame = tracked_agents.header.frame_id;
    }
    agent_to_map_tf = tf_buffer_->lookupTransform(cfg_->map_frame_id, agents_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));

    transforms_found = true;
  } catch (tf2::LookupException& ex) {
    RCLCPP_ERROR(this->get_logger(), "No Transform available Error: %s\n", ex.what());
  } catch (tf2::ConnectivityException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Connectivity Error: %s\n", ex.what());
  } catch (tf2::ExtrapolationException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Extrapolation Error: %s\n", ex.what());
  }

  // first check if path calculation is needed, and for whom
  std::vector<AgentStartPoseVel> agent_start_pose_vels;
  std::vector<bool> start_poses_far;
  int idx_order = 0;

  for (auto& agent : tracked_agents.agents) {
    path_vels_pos_.push_back(-1);
    if (std::find(req->ids.begin(), req->ids.end(), agent.track_id) == req->ids.end()) {
      continue;
    }
    bool path_exist = false;
    for (const auto& path_vel : path_vels_) {
      if (path_vel.id == agent.track_id) {
        path_exist = true;
        break;
      }
    }

    // get agent pose
    for (auto& segment : agent.segments) {
      if (segment.type == cfg_->default_agent_part) {
        geometry_msgs::msg::PoseStamped agent_start;
        agent_start.header.frame_id = tracked_agents.header.frame_id;
        agent_start.header.stamp = now;
        agent_start.pose = segment.pose.pose;

        //(tf2)
        tf2::Transform start_pose_tf;
        start_pose_tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
        geometry_msgs::msg::Pose start_pose;
        start_pose.orientation.w = 1.0;
        tf2::fromMsg(agent_start.pose, start_pose_tf);
        tf2::Transform agent_to_map_transform;
        tf2::fromMsg(agent_to_map_tf.transform, agent_to_map_transform);
        start_pose_tf = agent_to_map_transform * start_pose_tf;
        tf2::toMsg(start_pose_tf, start_pose);
        //(tf2)

        rclcpp::Time predicted_goals_time(predicted_goals_.header.stamp, this->get_clock()->get_clock_type());
        if (!path_exist || predicted_goals_time.seconds() < 1) {
          AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
          agent_start_pose_vels.push_back(agent_start_pose_vel);
          path_vels_pos_[agent.track_id - 1] = idx_order;
        } else {
          if (std::find(req->ids.begin(), req->ids.end(), agent.track_id) != req->ids.end()) {
            double dist_far = std::hypot(agent_start.pose.position.x - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.x,
                                         agent_start.pose.position.y - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.y);

            if (dist_far > RECALC_DIST) {
              start_poses_far.push_back(true);
              AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
              agent_start_pose_vels.push_back(agent_start_pose_vel);
              path_vels_pos_[agent.track_id - 1] = idx_order;
              path_vels_.clear();
            }
          }
        }
        break;
      }
    }
    idx_order++;
  }

  if (!agent_start_pose_vels.empty()) {
    if (transforms_found) {
      for (auto& agent_start_pose_vel : agent_start_pose_vels) {
        auto get_plan_srv = std::make_shared<nav_msgs::srv::GetPlan::Request>();

        // get agent pose in map frame
        // (tf2)
        tf2::Transform start_pose_tf;
        start_pose_tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
        tf2::fromMsg(agent_start_pose_vel.pose.pose, start_pose_tf);
        tf2::Transform agent_to_map_transform;
        tf2::fromMsg(agent_to_map_tf.transform, agent_to_map_transform);
        start_pose_tf = agent_to_map_transform * start_pose_tf;
        auto start_pose_stamped = agent_start_pose_vel.pose;
        tf2::toMsg(start_pose_tf, start_pose_stamped.pose);
        auto start_path = setFixedPath(start_pose_stamped);
        //(tf2)

        get_plan_srv->start.header.frame_id = cfg_->map_frame_id;
        get_plan_srv->start.header.stamp = now;
        get_plan_srv->start.pose = start_path.poses.back().pose;
        front_pose_pub_->publish(start_path.poses.back());

        get_plan_srv->goal.header.frame_id = cfg_->map_frame_id;
        get_plan_srv->goal.header.stamp = now;
        get_plan_srv->goal.pose = predicted_goals[agent_start_pose_vel.id];

        RCLCPP_DEBUG(this->get_logger(), "agent start: x=%.2f, y=%.2f, theta=%.2f, goal: x=%.2f, y=%.2f, theta=%.2f", get_plan_srv->start.pose.position.x, get_plan_srv->start.pose.position.y,
                     tf2::getYaw(get_plan_srv->start.pose.orientation), get_plan_srv->goal.pose.position.x, get_plan_srv->goal.pose.position.y, tf2::getYaw(get_plan_srv->goal.pose.orientation));

        // make plan for agent
        if (get_plan_client_) {
          auto future = get_plan_client_->async_send_request(get_plan_srv);
          if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto response = future.get();
            if (!response->plan.poses.empty()) {
              AgentPathVel agent_path_vel;
              agent_path_vel.id = agent_start_pose_vel.id;
              agent_path_vel.path = response->plan;
              agent_path_vel.start_vel = agent_start_pose_vel.vel;
              path_vels_.push_back(agent_path_vel);
              got_new_agent_paths_ = true;
            } else {
              RCLCPP_WARN(this->get_logger(), "Got empty path for agent, start or goal position is probably invalid");
            }
          } else {
            RCLCPP_WARN(this->get_logger(), "Failed to call %s service", cfg_->get_plan_srv_name.c_str());
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "%s service does not exist, re-trying to subscribe", cfg_->get_plan_srv_name.c_str());
          get_plan_client_ = this->create_client<nav_msgs::srv::GetPlan>(cfg_->get_plan_srv_name);
        }
      }
    }
  }

  return predictAgentsFromPaths(req, res);
}

void AgentPathPrediction::predictAgentsFromPaths(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req,
                                                 std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) {
  auto tracked_agents = tracked_agents_;

  if (got_new_agent_paths_) {
    for (auto agent_path_vel : path_vels_) {
      auto& poses = agent_path_vel.path.poses;
      if (!poses.empty()) {
        agent_path_prediction::msg::PredictedPoses predicted_poses;
        predicted_poses.id = agent_path_vel.id;

        auto lin_vel = std::hypot(agent_path_vel.start_vel.twist.linear.x, agent_path_vel.start_vel.twist.linear.y);
        auto now = this->now();

        predicted_poses.poses.resize(poses.size());
        for (size_t i = 0; i < poses.size(); ++i) {
          auto& pose = poses[i];
          geometry_msgs::msg::PoseWithCovarianceStamped predicted_pose;
          if (i == 0 || lin_vel == 0.0) {
            predicted_pose.header.stamp = now;
          } else {
            auto& last_pose = poses[i - 1];
            auto dist = std::hypot(pose.pose.position.x - last_pose.pose.position.x, pose.pose.position.y - last_pose.pose.position.y);
            rclcpp::Time last_time(predicted_poses.poses[i - 1].header.stamp, this->get_clock()->get_clock_type());
            rclcpp::Time new_time = last_time + rclcpp::Duration::from_seconds(dist / lin_vel);
            predicted_pose.header.stamp = new_time;
          }
          predicted_pose.header.frame_id = pose.header.frame_id;
          predicted_pose.pose.pose = pose.pose;
          predicted_poses.poses[i] = predicted_pose;
        }

        for (auto it = last_predicted_poses_.begin(); it != last_predicted_poses_.end(); ++it) {
          if (it->id == predicted_poses.id) {
            last_predicted_poses_.erase(it);
            break;
          }
        }
        last_predicted_poses_.push_back(predicted_poses);
        last_prune_indices_.erase(predicted_poses.id);

        RCLCPP_DEBUG(this->get_logger(), "Processed new path for agent %ld with %ld poses in frame %s", agent_path_vel.id, predicted_poses.poses.size(),
                     predicted_poses.poses.front().header.frame_id.c_str());
      }
    }
  }
  got_new_agent_paths_ = false;

  for (auto& poses : last_predicted_poses_) {
    if (!poses.poses.empty()) {
      geometry_msgs::msg::PoseStamped start_pose;
      geometry_msgs::msg::TwistStamped start_twist;

      if (transformPoseTwist(tracked_agents, poses.id, poses.poses.front().header.frame_id, start_pose, start_twist)) {
        auto last_prune_index_it = last_prune_indices_.find(poses.id);
        auto begin_index = (last_prune_index_it != last_prune_indices_.end()) ? last_prune_index_it->second : 0;
        auto prune_index = prunePath(begin_index, start_pose.pose, poses.poses);
        last_prune_indices_[poses.id] = prune_index;
        if (prune_index >= poses.poses.size()) {
          RCLCPP_ERROR(this->get_logger(), "Logical error, cannot prune path");
          continue;
        }
        geometry_msgs::msg::PoseWithCovarianceStamped start_pose_co;
        start_pose_co.header.stamp = start_pose.header.stamp;
        start_pose_co.header.frame_id = start_pose.header.frame_id;
        start_pose_co.pose.pose = start_pose.pose;
        std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> pruned_path;
        pruned_path.push_back(start_pose_co);
        pruned_path.insert(pruned_path.end(), poses.poses.begin() + prune_index, poses.poses.end());

        if (!pruned_path.empty()) {
          // update time stamps for the predicted path
          auto lin_vel = std::hypot(start_twist.twist.linear.x, start_twist.twist.linear.y);
          auto now = this->now();
          for (size_t i = 0; i < pruned_path.size(); i++) {
            if (i == 0 || lin_vel == 0) {
              pruned_path[i].header.stamp = now;
            } else {
              auto& pose = pruned_path[i].pose.pose;
              auto& last_pose = pruned_path[i - 1].pose.pose;
              auto dist = std::hypot(pose.position.x - last_pose.position.x, pose.position.y - last_pose.position.y);
              rclcpp::Time last_time(pruned_path[i - 1].header.stamp, this->get_clock()->get_clock_type());
              rclcpp::Time new_time = last_time + rclcpp::Duration::from_seconds(dist / lin_vel);
              pruned_path[i].header.stamp = new_time;
            }
          }

          agent_path_prediction::msg::PredictedPoses predicted_poses;
          predicted_poses.id = poses.id;
          predicted_poses.start_velocity = start_twist;
          predicted_poses.poses = pruned_path;

          res->predicted_agents_poses.push_back(predicted_poses);
          // RCLCPP_INFO(this->get_logger(), "Pushed the poses");
          RCLCPP_DEBUG(this->get_logger(), "Giving path of %ld points from %ld points for agent %d", predicted_poses.poses.size(), poses.poses.size(), poses.id);
        }
      }
    }
  }
}

// TODO: Remove this and make it a subscriber
void AgentPathPrediction::setGoal(const std::shared_ptr<agent_path_prediction::srv::AgentGoal::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentGoal::Response> res) {
  RCLCPP_DEBUG(this->get_logger(), "Received new agent goal");
  got_external_goal_ = true;
  external_goals_.clear();
  path_vels_.clear();
  for (auto& goal : req->goals) {
    external_goals_.push_back(goal);
  }

  res->success = true;
  res->message = "Goal has been set.";
}

void AgentPathPrediction::resetPredictionSrvs(const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/, std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/) {
  got_new_agent_paths_ = false;
  got_external_goal_ = false;
  last_predicted_poses_.clear();
  path_vels_.clear();
  check_path_ = false;
  behind_pose_ = geometry_msgs::msg::Transform();
  RCLCPP_INFO(this->get_logger(), "Reset agent path prediction");
}

nav_msgs::msg::Path AgentPathPrediction::setFixedPath(const geometry_msgs::msg::PoseStamped& start_pose) {
  nav_msgs::msg::Path path;
  path.header.frame_id = start_pose.header.frame_id;
  path.header.stamp = start_pose.header.stamp;
  path.poses.push_back(start_pose);

  // Extract yaw from quaternion
  double roll;
  double pitch;
  double yaw;
  tf2::Quaternion q_start;
  tf2::fromMsg(start_pose.pose.orientation, q_start);
  tf2::Matrix3x3(q_start).getRPY(roll, pitch, yaw);
  double step_distance = 0.1;   // meters
  double total_distance = 0.5;  // meters

  for (double dist = step_distance; dist <= total_distance; dist += step_distance) {
    geometry_msgs::msg::PoseStamped new_pose = start_pose;
    new_pose.pose.position.x += dist * cos(yaw);
    new_pose.pose.position.y += dist * sin(yaw);
    path.poses.push_back(new_pose);
  }
  return path;
}

size_t AgentPathPrediction::prunePath(size_t begin_index, const geometry_msgs::msg::Pose& pose, const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& path) {
  size_t prune_index = begin_index;
  double x_diff;
  double y_diff;
  double sq_diff;
  double smallest_sq_diff = std::numeric_limits<double>::max();
  while (begin_index < path.size()) {
    x_diff = path[begin_index].pose.pose.position.x - pose.position.x;
    y_diff = path[begin_index].pose.pose.position.y - pose.position.y;
    sq_diff = x_diff * x_diff + y_diff * y_diff;
    if (sq_diff < smallest_sq_diff) {
      prune_index = begin_index;
      smallest_sq_diff = sq_diff;
    }
    ++begin_index;
  }
  return prune_index;
}

bool AgentPathPrediction::transformPoseTwist(const cohan_msgs::msg::TrackedAgents& tracked_agents, const uint64_t& agent_id, const std::string& to_frame, geometry_msgs::msg::PoseStamped& pose,
                                             geometry_msgs::msg::TwistStamped& twist) const {
  for (const auto& agent : tracked_agents.agents) {
    if (agent.track_id == agent_id) {
      for (const auto& segment : agent.segments) {
        if (segment.type == cfg_->default_agent_part) {
          geometry_msgs::msg::PoseStamped pose_ut;
          pose_ut.header.stamp = tracked_agents.header.stamp;
          pose_ut.header.frame_id = tracked_agents.header.frame_id;
          pose_ut.pose = segment.pose.pose;
          twist.header.stamp = tracked_agents.header.stamp;
          twist.header.frame_id = tracked_agents.header.frame_id;
          twist.twist = segment.twist.twist;
          try {
            tf2::Stamped<tf2::Transform> pose_tf;
            // Manual conversion from PoseStamped to tf2::Stamped<tf2::Transform>
            tf2::Transform transform;
            tf2::fromMsg(pose_ut.pose, transform);
            pose_tf.setData(transform);
            pose_tf.frame_id_ = pose_ut.header.frame_id;
            pose_tf.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(rclcpp::Time(pose_ut.header.stamp, this->get_clock()->get_clock_type()).nanoseconds()));
            geometry_msgs::msg::TransformStamped start_pose_to_plan_transform;

            if (to_frame.empty() || pose_ut.header.frame_id.empty() || twist.header.frame_id.empty()) {
              continue;
            }
            start_pose_to_plan_transform = tf_buffer_->lookupTransform(to_frame, pose_ut.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));
            tf2::Transform start_transform;
            tf2::fromMsg(start_pose_to_plan_transform.transform, start_transform);
            pose_tf.setData(start_transform * pose_tf);
            pose_tf.frame_id_ = to_frame;

            // Convert tf2::Stamped<tf2::Transform> to geometry_msgs::msg::PoseStamped manually
            pose.header.frame_id = pose_tf.frame_id_;
            pose.header.stamp = rclcpp::Time(std::chrono::nanoseconds(pose_tf.stamp_.time_since_epoch().count()).count(), this->get_clock()->get_clock_type());
            pose.pose.position.x = pose_tf.getOrigin().x();
            pose.pose.position.y = pose_tf.getOrigin().y();
            pose.pose.position.z = pose_tf.getOrigin().z();
            pose.pose.orientation = tf2::toMsg(pose_tf.getRotation());

            geometry_msgs::msg::Twist start_twist_to_plan_transform;
            lookupTwist(to_frame, twist.header.frame_id, this->now(), tf2::durationFromSec(0.1), start_twist_to_plan_transform, tf_buffer_);

            twist.twist.linear.x = start_twist_to_plan_transform.linear.x;
            twist.twist.linear.y = start_twist_to_plan_transform.linear.y;
            twist.twist.angular.z = start_twist_to_plan_transform.angular.z;
            twist.header.frame_id = to_frame;
            return true;
          } catch (tf2::LookupException& ex) {
            RCLCPP_ERROR(this->get_logger(), "No Transform available Error: %s", ex.what());

          } catch (tf2::ConnectivityException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Connectivity Error: %s", ex.what());

          } catch (tf2::ExtrapolationException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Extrapolation Error: %s", ex.what());
          }
          break;
        }
      }
      break;
    }
  }
  return false;
}

}  // namespace agents

// handler for something to do before killing the node
void sigintHandler(int sig) {
  RCLCPP_DEBUG(rclcpp::get_logger(NODE_NAME), "node %s will now shutdown", NODE_NAME);

  // the default sigint handler, it calls shutdown() on node
  rclcpp::shutdown();
}
#if !defined(DOXYGEN_SHOULD_SKIP_THIS)
// the main method starts a rosnode and initializes the agent_path_prediction class
int main(int argc, char** argv) {
  // starting the agent_path_prediction node
  rclcpp::init(argc, argv);

  auto node = std::make_shared<agents::AgentPathPrediction>();

  // Initialize after shared_ptr is created
  node->initialize();

  agents::PredictGoalROS predict_srv(node, node->cfg_);
  predict_srv.initialize();

  // look for sigint and start spinning the node
  signal(SIGINT, sigintHandler);
  rclcpp::spin(node);

  return 0;
}
#endif
