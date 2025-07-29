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
#include <csignal>
#include <utility>

namespace agents {
void AgentPathPrediction::initialize() {
  // Initialize Publishers
  predicted_agents_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("predicted_agent_poses", 1);
  front_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("front_pose", 1);

  // Initialize Service servers
  set_goal_srv_ = node_->create_service<agent_path_prediction::srv::AgentGoal>("set_agent_goal", std::bind(&AgentPathPrediction::setGoal, this, _1, _2));
  predict_agents_server_ = node_->create_service<agent_path_prediction::srv::AgentPosePredict>("predict_agent_poses", std::bind(&AgentPathPrediction::predictAgents, this, _1, _2));
  reset_prediction_services_server_ = node_->create_service<std_srvs::srv::Empty>("reset_prediction_services", std::bind(&AgentPathPrediction::resetPredictionSrvs, this, _1, _2));

  // Initialize Subscribers
  tracked_agents_sub_ = node_->create_subscription<cohan_msgs::msg::TrackedAgents>(tracked_agents_sub_topic_, 1, std::bind(&AgentPathPrediction::trackedAgentsCB, this, _1));
  external_paths_sub_ = node_->create_subscription<cohan_msgs::msg::AgentPathArray>(external_paths_sub_topic_, 1, std::bind(&AgentPathPrediction::externalPathsCB, this, _1));
  predicted_goal_sub_ = node_->create_subscription<agent_path_prediction::msg::PredictedGoals>(predicted_goal_topic_, 1, std::bind(&AgentPathPrediction::predictedGoalCB, this, _1));

  // // Initialize Service clients
  get_plan_client_ = node_->create_client<nav_msgs::srv::GetPlan>("get_plan");

  // Initialize properties
  showing_markers_ = false;
  got_new_agent_paths_ = false;
  got_external_goal_ = false;

  RCLCPP_DEBUG(node_->get_logger(), "node %s initialized", NODE_NAME);
}

void AgentPathPrediction::predictAgents(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) {
  std::function<bool(agent_path_prediction::srv::AgentPosePredict::Request & req, agent_path_prediction::srv::AgentPosePredict::Response & res)> prediction_function;

  switch (req.type) {
    case agent_path_prediction::srv::AgentPosePredictRequest::VELOCITY_OBSTACLE:
      prediction_function = std::bind(&AgentPathPrediction::predictAgentsVelObs, this, _1, _2);
      break;
    case agent_path_prediction::srv::AgentPosePredictRequest::EXTERNAL:
      prediction_function = std::bind(&AgentPathPrediction::predictAgentsExternal, this, _1, _2);
      break;
    case agent_path_prediction::srv::AgentPosePredictRequest::BEHIND_ROBOT:
      prediction_function = std::bind(&AgentPathPrediction::predictAgentsBehind, this, _1, _2);
      break;
    case agent_path_prediction::srv::AgentPosePredictRequest::PREDICTED_GOAL:
      prediction_function = std::bind(&AgentPathPrediction::predictAgentsGoal, this, _1, _2);
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "%s: unkonwn prediction type %d", NODE_NAME, req.type);
  }

  if (!prediction_function.empty() && prediction_function(req, res)) {
    if (publish_markers_) {
      // create new markers
      predicted_agents_markers_.markers.clear();

      for (auto predicted_agent : res.predicted_agents_poses) {
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
            predicted_agent_marker.lifetime = rclcpp::Duration::from_seconds(MIN_MARKER_LIFETIME) + (predicted_agent_pose.header.stamp - first_pose_time);
            predicted_agent_marker.pose.position.x = predicted_agent_pose.pose.pose.position.x;
            predicted_agent_marker.pose.position.y = predicted_agent_pose.pose.pose.position.y;
            // time on z axis
            predicted_agent_marker.pose.position.z = (predicted_agent_pose.header.stamp - first_pose_time).seconds();
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
          RCLCPP_WARN(node_->get_logger(), "no predicted poses for agent %d", predicted_agent.id);
        }
      }

      predicted_agents_pub_.publish(predicted_agents_markers_);
      showing_markers_ = true;

      RCLCPP_DEBUG(node_->get_logger(), "published predicted agents");

    } else {
      if (showing_markers_) {
        predicted_agents_markers_.markers.clear();
        visualization_msgs::msg::Marker delete_agent_markers;
        delete_agent_markers.action = 3;  // visualization_msgs::msg::Marker::DELETEALL;
        predicted_agents_markers_.markers.push_back(delete_agent_markers);
        predicted_agents_pub_.publish(predicted_agents_markers_);
        showing_markers_ = false;
      }
    }
  }
}

void predictAgentsVelObs(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) const {
  // validate prediction time
  if (req.predict_times.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "prediction times cannot be empty");
    return;
  }
  if (*std::min_element(req.predict_times.begin(), req.predict_times.end()) < 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "prediction time cannot be negative");
    return;
  }

  // get local refrence of agents
  auto agents = tracked_agents_.agents;
  auto track_frame = tracked_agents_.header.frame_id;
  auto track_time = tracked_agents_.header.stamp;

  if ((node_->get_clock()->now() - track_time).seconds() > *std::max_element(req.predict_times.begin(), req.predict_times.end())) {
    RCLCPP_DEBUG(node_->get_logger(), "agent data is older than maximum given prediction time, predicting nothing");
    return;
  }

  for (const auto &agent : agents) {
    if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) == req.ids.end()) {
      continue;
    }
    for (auto segment : agent.segments) {
      if (segment.type == default_agent_part_) {
        // calculate future agent poses based on current velocity
        agent_path_prediction::msg::PredictedPoses predicted_poses;
        predicted_poses.id = agent.track_id;

        // get linear velocity of the agent
        geometry_msgs::msg::Vector3 linear_vel;
        linear_vel.x = segment.twist.twist.linear.x;
        linear_vel.y = segment.twist.twist.linear.y;
        linear_vel.z = segment.twist.twist.linear.z;

        for (auto predict_time : req.predict_times) {
          // validate prediction time
          if (predict_time < 0) {
            RCLCPP_ERROR(node_->get_logger(), "%s: prediction time cannot be negative (give %f)", NODE_NAME, predict_time);
            return;
          }

          geometry_msgs::msg::PoseWithCovarianceStamped predicted_pose;
          predicted_pose.header.frame_id = track_frame;
          predicted_pose.header.stamp = track_time + rclcpp::Duration::from_seconds(predict_time);

          if (velobs_use_ang_ && std::abs(segment.twist.twist.angular.z) > ANG_VEL_EPS) {
            // velocity multiplier is only applied to linear velocities
            double r = (std::hypot(linear_vel[0], linear_vel[1]) * velobs_mul_) / segment.twist.twist.angular.z;
            double theta = segment.twist.twist.angular.z * predict_time;
            double crd = r * 2 * std::sin(theta / 2);
            double alpha = std::atan2(linear_vel[1], linear_vel[0]) + (theta / 2);
            predicted_pose.pose.pose.position.x = segment.pose.pose.position.x + crd * std::cos(alpha);
            predicted_pose.pose.pose.position.y = segment.pose.pose.position.y + crd * std::sin(alpha);

            tf2::Quaternion q;
            q.setRPY(0, 0, tf2::getYaw(segment.pose.pose.orientation) + theta);
            predicted_pose.pose.pose.orientation = tf2::toMsg(q);

          } else {
            predicted_pose.pose.pose.position.x = segment.pose.pose.position.x + linear_vel[0] * predict_time * velobs_mul_;
            predicted_pose.pose.pose.position.y = segment.pose.pose.position.y + linear_vel[1] * predict_time * velobs_mul_;
            predicted_pose.pose.pose.orientation = segment.pose.pose.orientation;
          }

          // not using velocity multiplier for covariance matrix
          double xy_vel = hypot(linear_vel[0] * predict_time, linear_vel[1] * predict_time);
          // storing only x, y covariance in diagonal matrix
          predicted_pose.pose.covariance[0] = velobs_min_rad_ + (velobs_max_rad_ - velobs_min_rad_) * (predict_time / velobs_max_rad_time_) * xy_vel;
          predicted_pose.pose.covariance[7] = predicted_pose.pose.covariance[0];
          predicted_poses.poses.push_back(predicted_pose);

          RCLCPP_DEBUG(node_->get_logger(), "%s: predicted agent (%lu) segment (%d) pose: x=%f, y=%f, theta=%f, predict-time=%f", NODE_NAME, agent.track_id, segment.type,
                       predicted_pose.pose.pose.position.x, predicted_pose.pose.pose.position.y, tf2::getYaw(predicted_pose.pose.pose.orientation), predict_time);
        }

        geometry_msgs::msg::TwistStamped current_twist;
        current_twist.header.frame_id = track_frame;
        current_twist.header.stamp = track_time;
        current_twist.twist = segment.twist.twist;
        predicted_poses.start_velocity = current_twist;

        res.predicted_agents_poses.push_back(predicted_poses);
      }
    }
  }
}

// bool AgentPathPrediction::predictAgentsExternal(agent_path_prediction::srv::AgentPosePredict::Request &req, agent_path_prediction::srv::AgentPosePredict::Response &res) {
void predictAgentsExternal(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) {
  // Using external paths
  if (external_paths_) {
    auto external_paths = external_paths_;
    auto tracked_agents = tracked_agents_;

    std::vector<AgentPathVel> agent_path_vel_array;
    for (const auto &path : external_paths->paths) {
      AgentPathVel agent_path_vel{.id = path.id, .path = path.path};

      // set starting velocity of the agent if we find them
      // we do not add current pose at first pose in this case
      for (auto &agent : tracked_agents.agents) {
        if (agent.track_id == path.id) {
          for (auto &segment : agent.segments) {
            if (segment.type == default_agent_part_) {
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
    return predictAgentsFromPaths(req, res);
  }

  // Using an external goal
  if (got_external_goal_) {
    auto now = node_->now();
    auto tracked_agents = tracked_agents_;
    std::map<uint64_t, geometry_msgs::msg::PoseStamped> ext_goal;

    // get robot pose (ROS 1)
    // tf::StampedTransform robot_to_map_tf;
    // tf::StampedTransform agent_to_map_tf;
    // bool transforms_found = false;
    // try {
    //   tf_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0), robot_to_map_tf);

    //   std::string agents_frame = "map";
    //   if (!tracked_agents.header.frame_id.empty()) {
    //     agents_frame = tracked_agents.header.frame_id;
    //   }
    //   tf_.lookupTransform(map_frame_id_, agents_frame, ros::Time(0), agent_to_map_tf);

    //   transforms_found = true;
    // } catch (tf::LookupException &ex) {
    //   ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n", ex.what());
    // } catch (tf::ConnectivityException &ex) {
    //   ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
    // } catch (tf::ExtrapolationException &ex) {
    //   ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
    // }

    // geometry_msgs::msg::TransformStamped robot_to_map_tf;
    geometry_msgs::msg::TransformStamped agent_to_map_tf_msg;
    tf2::Transform agent_to_map_tf;
    bool transforms_found = false;
    try {
      // robot_to_map_tf = tf_buffer_->lookupTransform(map_frame_id_, robot_frame_id_, tf2::TimePointZero);
      std::string agents_frame = "map";
      if (!tracked_agents.header.frame_id.empty()) {
        agents_frame = tracked_agents.header.frame_id;
      }
      agent_to_map_tf_msg = tf_buffer_->lookupTransform(map_frame_id_, agents_frame, tf2::TimePointZero);
      tf2::Transform agent_to_map_tf;
      tf2::fromMsg(agent_to_map_tf_msg.transform, agent_to_map_tf);
      transforms_found = true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(node->get_logger(), "Transform error: %s", ex.what());
    }

    // first check if path calculation is needed, and for whom
    std::vector<AgentStartPoseVel> agent_start_pose_vels;
    std::vector<bool> start_poses_far;
    int idx_order = 0;
    for (auto &agent : tracked_agents.agents) {
      path_vels_pos_.push_back(-1);
      if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) == req.ids.end()) {
        continue;
      }
      bool path_exist = false;
      for (auto &ex_gl : external_goals_) {
        if (ex_gl.id == agent.track_id) {
          ext_goal[ex_gl.id] = ex_gl.pose;
          break;
        }
      }
      for (const auto &path_vel : path_vels_) {
        if (path_vel.id == agent.track_id) {
          path_exist = true;
          break;
        }
      }

      // get agent pose
      for (auto &segment : agent.segments) {
        if (segment.type == default_agent_part_) {
          geometry_msgs::msg::PoseStamped agent_start;
          agent_start.header.frame_id = tracked_agents.header.frame_id;
          agent_start.header.stamp = now;
          agent_start.pose = segment.pose.pose;

          // ROS 1
          // tf::Pose start_pose_tf;
          // start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
          // geometry_msgs::msg::Pose start_pose;
          // start_pose.orientation.w = 1.0;
          // tf::poseMsgToTF(agent_start.pose, start_pose_tf);
          // start_pose_tf = agent_to_map_tf * start_pose_tf;
          // tf::poseTFToMsg(start_pose_tf, start_pose);

          geometry_msgs::msg::Pose start_pose;
          start_pose.orientation.w = 1.0;
          tf2::Transform start_pose_tf;
          tf2::fromMsg(agent_start.pose, start_pose_tf);
          start_pose_tf = agent_to_map_tf * start_pose_tf;
          start_pose = tf2::toMsg(start_pose_tf);

          if (!path_exist) {
            AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
            agent_start_pose_vels.push_back(agent_start_pose_vel);
            path_vels_pos_[agent.track_id - 1] = idx_order;
          } else {
            if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) != req.ids.end()) {
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
        for (auto &agent_start_pose_vel : agent_start_pose_vels) {
          // nav_msgs::srv::GetPlan get_plan_srv;
          auto get_plan_srv = std::make_shared<nav_msgs::srv::GetPlan::Request>();
          if (ext_goal.find(agent_start_pose_vel.id) == ext_goal.end()) {
            continue;
          }
          // get agent pose in map frame (ROS 1)
          // tf::Pose start_pose_tf;
          // start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0)); // not needed in ros2
          // tf::poseMsgToTF(agent_start_pose_vel.pose.pose, start_pose_tf);
          // start_pose_tf = agent_to_map_tf * start_pose_tf;
          // auto start_pose_stamped = agent_start_pose_vel.pose;
          // tf::poseTFToMsg(start_pose_tf, start_pose_stamped.pose);

          tf2::Transform start_pose_tf;
          tf2::fromMsg(agent_start_pose_vel.pose.pose, start_pose_tf);
          start_pose_tf = agent_to_map_tf * start_pose_tf;
          auto start_pose_stamped = agent_start_pose_vel.pose;
          start_pose_stamped.pose = tf2::toMsg(start_pose_tf);
          auto start_path = setFixedPath(start_pose_stamped);

          get_plan_srv->start.header.frame_id = map_frame_id_;
          get_plan_srv->start.header.stamp = now;
          get_plan_srv->start.pose = start_path.poses.back().pose;
          front_pose_pub_.publish(start_path.poses.back());

          get_plan_srv->goal.header.frame_id = map_frame_id_;
          get_plan_srv->goal.header.stamp = now;
          get_plan_srv->goal.pose.position.x = ext_goal[agent_start_pose_vel.id].pose.position.x;
          get_plan_srv->goal.pose.position.y = ext_goal[agent_start_pose_vel.id].pose.position.y;
          get_plan_srv->goal.pose.position.z = ext_goal[agent_start_pose_vel.id].pose.position.z;
          get_plan_srv->goal.pose.orientation = ext_goal[agent_start_pose_vel.id].pose.orientation;

          // ROS_DEBUG_NAMED(NODE_NAME, "agent start: x=%.2f, y=%.2f, theta=%.2f, goal: x=%.2f, y=%.2f, theta=%.2f", get_plan_srv->start.pose.position.x,
          //                 get_plan_srv->start.pose.position.y, tf2::getYaw(get_plan_srv->start.pose.orientation), get_plan_srv->goal.pose.position.x,
          //                 get_plan_srv->goal.pose.position.y, tf2::getYaw(get_plan_srv->goal.pose.orientation));
          RCLCPP_DEBUG(node->get_logger(), "agent start: x=%.2f, y=%.2f, theta=%.2f, goal: x=%.2f, y=%.2f, theta=%.2f", get_plan_srv->start.pose.position.x, get_plan_srv->start.pose.position.y,
                       tf2::getYaw(get_plan_srv->start.pose.orientation), get_plan_srv->goal.pose.position.x, get_plan_srv->goal.pose.position.y, tf2::getYaw(get_plan_srv->goal.pose.orientation));

          // make plan for agent
          if (get_plan_client_) {
            // if (get_plan_client_.call(get_plan_srv)) {  // fix these
            // if (!get_plan_srv.response.plan.poses.empty()) {
            auto future = get_plan_client_->async_send_request(get_plan_srv);
            if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
              auto response = future.get();
              if (!response->plan.poses.empty()) {
                AgentPathVel agent_path_vel;
                agent_path_vel.id = agent_start_pose_vel.id;
                agent_path_vel.path = response.plan;
                agent_path_vel.start_vel = agent_start_pose_vel.vel;
                path_vels_.push_back(agent_path_vel);
                got_new_agent_paths_ = true;
              } else {
                RCLCPP_WARN(node->get_logger(), "Got empty path for agent, start or goal position is probably invalid");
              }
            } else {
              RCLCPP_WARN(node->get_logger(), "Failed to call %s service", get_plan_srv_name_.c_str());
            }
          } else {
            RCLCPP_WARN(node->get_logger(), "%s service does not exist, re-trying to subscribe", get_plan_srv_name_.c_str());
            get_plan_client_ = node_->create_client<nav_msgs::srv::GetPlan>("get_plan");
          }
        }
      }
    }
    return predictAgentsFromPaths(req, res);
  }

  std::vector<AgentPathVel> empty_path_vels;
  path_vels_ = empty_path_vels;
  return predictAgentsFromPaths(req, res);
}

// bool AgentPathPrediction::predictAgentsBehind(agent_path_prediction::srv::AgentPosePredict::Request &req, agent_path_prediction::srv::AgentPosePredict::Response &res) {
void predictAgentsBehind(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) {
  // auto now = ros::Time::now();
  auto now = node_->now();
  auto tracked_agents = tracked_agents_;

  // get robot pose
  // tf::StampedTransform robot_to_map_tf;
  // tf::StampedTransform agent_to_map_tf;
  // bool transforms_found = false;
  // try {
  //   tf_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0), robot_to_map_tf);
  //   std::string agents_frame = "map";
  //   if (!tracked_agents.header.frame_id.empty()) {
  //     agents_frame = tracked_agents.header.frame_id;
  //   }
  //   tf_.lookupTransform(map_frame_id_, agents_frame, ros::Time(0), agent_to_map_tf);

  //   transforms_found = true;
  // } catch (tf::LookupException &ex) {
  //   ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n", ex.what());
  // } catch (tf::ConnectivityException &ex) {
  //   ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
  // } catch (tf::ExtrapolationException &ex) {
  //   ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
  // }

  geometry_msgs::msg::TransformStamped robot_to_map_tf_msg;
  tf2::Transform robot_to_map_tf;
  geometry_msgs::msg::TransformStamped agent_to_map_tf_msg;
  tf2::Transform agent_to_map_tf;
  bool transforms_found = false;
  try {
    robot_to_map_tf_msg = tf_buffer_->lookupTransform(map_frame_id_, robot_frame_id_, tf2::TimePointZero);
    tf2::fromMsg(agent_to_map_tf_msg.transform, robot_to_map_tf);

    std::string agents_frame = "map";
    if (!tracked_agents.header.frame_id.empty()) {
      agents_frame = tracked_agents.header.frame_id;
    }
    agent_to_map_tf_msg = tf_buffer_->lookupTransform(map_frame_id_, agents_frame, tf2::TimePointZero);
    tf2::fromMsg(agent_to_map_tf_msg.transform, agent_to_map_tf);

    transforms_found = true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(node->get_logger(), "Transform error: %s", ex.what());
  }

  // first check if path calculation is needed, and for whom
  std::vector<AgentStartPoseVel> agent_start_pose_vels;
  std::vector<bool> start_poses_far;
  int idx_order = 0;
  for (auto &agent : tracked_agents.agents) {
    path_vels_pos_.push_back(-1);
    if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) == req.ids.end()) {
      continue;
    }
    bool path_exist = false;
    for (const auto &path_vel : path_vels_) {
      if (path_vel.id == agent.track_id) {
        path_exist = true;
        break;
      }
    }

    // get agent pose
    for (auto &segment : agent.segments) {
      if (segment.type == default_agent_part_) {
        geometry_msgs::msg::PoseStamped agent_start;
        agent_start.header.frame_id = tracked_agents.header.frame_id;
        agent_start.header.stamp = now;
        agent_start.pose = segment.pose.pose;

        // ROS 1
        // tf::Pose start_pose_tf;
        // start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        // geometry_msgs::msg::Pose start_pose;
        // start_pose.orientation.w = 1.0;
        // tf::poseMsgToTF(agent_start.pose, start_pose_tf);
        // start_pose_tf = agent_to_map_tf * start_pose_tf;
        // tf::poseTFToMsg(start_pose_tf, start_pose);

        geometry_msgs::msg::Pose start_pose;
        start_pose.orientation.w = 1.0;
        tf2::Transform start_pose_tf;
        tf2::fromMsg(agent_start.pose, start_pose_tf);
        start_pose_tf = agent_to_map_tf * start_pose_tf;
        start_pose = tf2::toMsg(start_pose_tf);

        if (!path_exist) {
          AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
          agent_start_pose_vels.push_back(agent_start_pose_vel);
          path_vels_pos_[agent.track_id - 1] = idx_order;
        } else {
          if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) != req.ids.end()) {
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
      for (auto &agent_start_pose_vel : agent_start_pose_vels) {
        // nav_msgs::srv::GetPlan get_plan_srv;
        auto get_plan_srv = std::make_shared<nav_msgs::srv::GetPlan::Request>();

        auto hum_id = agent_start_pose_vel.id;
        // get agent pose in map frame
        // ROS 1
        // tf::Pose start_pose_tf;
        // start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        // tf::poseMsgToTF(agent_start_pose_vel.pose.pose, start_pose_tf);
        // start_pose_tf = agent_to_map_tf * start_pose_tf;
        // auto start_pose_stamped = agent_start_pose_vel.pose;
        // tf::poseTFToMsg(start_pose_tf, start_pose_stamped.pose);
        // auto start_path = setFixedPath(start_pose_stamped);

        tf2::Transform start_pose_tf;
        tf2::fromMsg(agent_start_pose_vel.pose.pose, start_pose_tf);
        start_pose_tf = agent_to_map_tf * start_pose_tf;
        auto start_pose_stamped = agent_start_pose_vel.pose;
        start_pose_stamped.pose = tf2::toMsg(start_pose_tf);
        auto start_path = setFixedPath(start_pose_stamped);

        get_plan_srv->start.header.frame_id = map_frame_id_;
        get_plan_srv->start.header.stamp = now;
        get_plan_srv->start.pose = start_path.poses.back().pose;
        front_pose_pub_.publish(start_path.poses.back());

        // calculate agent pose behind robot
        if (!check_path_) {
          check_path_ = true;
          // ROS 1
          // tf::Transform behind_tr;
          // behind_tr.setOrigin(tf::Vector3(-agent_dist_behind_robot_, 0.0, 0.0));
          // behind_tr.setRotation(tf::createQuaternionFromYaw(agent_angle_behind_robot_));
          // behind_tr = robot_to_map_tf * behind_tr;
          // tf::transformTFToMsg(behind_tr, behind_pose_);

          tf2::Transform behind_tr;
          behind_tr.setOrigin(tf2::Vector3(-agent_dist_behind_robot_, 0.0, 0.0));
          behind_tr.setRotation(tf2::createQuaternionFromYaw(agent_angle_behind_robot_));
          behind_tr = robot_to_map_tf * behind_tr;
          behind_pose_ = tf2::toMsg(behind_tr);
        }
        get_plan_srv->goal.header.frame_id = map_frame_id_;
        get_plan_srv->goal.header.stamp = now;
        get_plan_srv->goal.pose.position.x = behind_pose_.translation.x;
        get_plan_srv->goal.pose.position.y = behind_pose_.translation.y;
        get_plan_srv->goal.pose.position.z = behind_pose_.translation.z;
        get_plan_srv->goal.pose.orientation = behind_pose_.rotation;

        // ROS_DEBUG_NAMED(NODE_NAME,
        //                 "agent start: x=%.2f, y=%.2f, theta=%.2f, "
        //                 "goal: x=%.2f, y=%.2f, theta=%.2f",
        //                 get_plan_srv.request.start.pose.position.x, get_plan_srv.request.start.pose.position.y, tf::getYaw(get_plan_srv.request.start.pose.orientation),
        //                 get_plan_srv.request.goal.pose.position.x, get_plan_srv.request.goal.pose.position.y, tf::getYaw(get_plan_srv.request.goal.pose.orientation));

        RCLCPP_DEBUG(node->get_logger(), "agent start: x=%.2f, y=%.2f, theta=%.2f, goal: x=%.2f, y=%.2f, theta=%.2f", get_plan_srv->start.pose.position.x, get_plan_srv->start.pose.position.y,
                     tf2::getYaw(get_plan_srv->start.pose.orientation), get_plan_srv->goal.pose.position.x, get_plan_srv->goal.pose.position.y, tf2::getYaw(get_plan_srv->goal.pose.orientation));

        // make plan for agent
        if (get_plan_client_) {
          // if (get_plan_client_.call(get_plan_srv)) {  // fix these
          //   if (!get_plan_srv.response.plan.poses.empty()) {
          auto future = get_plan_client_->async_send_request(get_plan_srv);
          if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (!response->plan.poses.empty()) {
              AgentPathVel agent_path_vel;
              agent_path_vel.id = agent_start_pose_vel.id;
              agent_path_vel.path = get_plan_srv.response.plan;
              agent_path_vel.start_vel = agent_start_pose_vel.vel;
              path_vels_.push_back(agent_path_vel);
              got_new_agent_paths_ = true;
            } else {
              RCLCPP_WARN(node->get_logger(), "Got empty path for agent, start or goal position is probably invalid");
            }
          } else {
            RCLCPP_WARN(node->get_logger(), "Failed to call %s service", get_plan_srv_name_.c_str());
          }
        } else {
          RCLCPP_WARN(node->get_logger(), "%s service does not exist, re-trying to subscribe", get_plan_srv_name_.c_str());
          get_plan_client_ = node_->create_client<nav_msgs::srv::GetPlan>("get_plan");
        }
      }
    }
  }

  return predictAgentsFromPaths(req, res);
}

// bool AgentPathPrediction::predictAgentsGoal(agent_path_prediction::srv::AgentPosePredict::Request &req, agent_path_prediction::srv::AgentPosePredict::Response &res) {
void predictAgentsGoal(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) {
  // auto now = ros::Time::now();
  auto now = node_->now();
  auto tracked_agents = tracked_agents_;
  std::map<int, geometry_msgs::msg::Pose> predicted_goals;

  for (auto &goal : predicted_goals_.goals) {
    predicted_goals[goal.id] = goal.goal;
  }

  // get robot pose
  // tf::StampedTransform robot_to_map_tf;
  // tf::StampedTransform agent_to_map_tf;
  // bool transforms_found = false;
  // try {
  //   tf_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0), robot_to_map_tf);
  //   std::string agents_frame = "map";
  //   if (!tracked_agents.header.frame_id.empty()) {
  //     agents_frame = tracked_agents.header.frame_id;
  //   }
  //   tf_.lookupTransform(map_frame_id_, agents_frame, ros::Time(0), agent_to_map_tf);

  //   transforms_found = true;
  // } catch (tf::LookupException &ex) {
  //   ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n", ex.what());
  // } catch (tf::ConnectivityException &ex) {
  //   ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
  // } catch (tf::ExtrapolationException &ex) {
  //   ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
  // }

  geometry_msgs::msg::TransformStamped agent_to_map_tf_msg;
  tf2::Transform agent_to_map_tf;
  bool transforms_found = false;
  try {
    std::string agents_frame = "map";
    if (!tracked_agents.header.frame_id.empty()) {
      agents_frame = tracked_agents.header.frame_id;
    }
    agent_to_map_tf_msg = tf_buffer_->lookupTransform(map_frame_id_, agents_frame, tf2::TimePointZero);
    tf2::fromMsg(agent_to_map_tf_msg.transform, agent_to_map_tf);

    transforms_found = true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(node->get_logger(), "Transform error: %s", ex.what());
  }

  // first check if path calculation is needed, and for whom
  std::vector<AgentStartPoseVel> agent_start_pose_vels;
  std::vector<bool> start_poses_far;
  int idx_order = 0;

  for (auto &agent : tracked_agents.agents) {
    path_vels_pos_.push_back(-1);
    if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) == req.ids.end()) {
      continue;
    }
    bool path_exist = false;
    for (const auto &path_vel : path_vels_) {
      if (path_vel.id == agent.track_id) {
        path_exist = true;
        break;
      }
    }

    // get agent pose
    for (auto &segment : agent.segments) {
      if (segment.type == default_agent_part_) {
        geometry_msgs::msg::PoseStamped agent_start;
        agent_start.header.frame_id = tracked_agents.header.frame_id;
        agent_start.header.stamp = now;
        agent_start.pose = segment.pose.pose;

        // tf::Pose start_pose_tf;
        // start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        // geometry_msgs::msg::Pose start_pose;
        // start_pose.orientation.w = 1.0;
        // tf::poseMsgToTF(agent_start.pose, start_pose_tf);
        // start_pose_tf = agent_to_map_tf * start_pose_tf;
        // tf::poseTFToMsg(start_pose_tf, start_pose);

        geometry_msgs::msg::Pose start_pose;
        start_pose.orientation.w = 1.0;
        tf2::Transform start_pose_tf;
        tf2::fromMsg(agent_start.pose, start_pose_tf);
        start_pose_tf = agent_to_map_tf * start_pose_tf;
        start_pose = tf2::toMsg(start_pose_tf);

        if (!path_exist || predicted_goals_.header.stamp.toSec() < 1) {
          AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
          agent_start_pose_vels.push_back(agent_start_pose_vel);
          path_vels_pos_[agent.track_id - 1] = idx_order;
        } else {
          if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) != req.ids.end()) {
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
      for (auto &agent_start_pose_vel : agent_start_pose_vels) {
        // nav_msgs::srv::GetPlan get_plan_srv;
        auto get_plan_srv = std::make_shared<nav_msgs::srv::GetPlan::Request>();

        // get agent pose in map frame
        // tf::Pose start_pose_tf;
        // start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        // tf::poseMsgToTF(agent_start_pose_vel.pose.pose, start_pose_tf);
        // start_pose_tf = agent_to_map_tf * start_pose_tf;
        // auto start_pose_stamped = agent_start_pose_vel.pose;
        // tf::poseTFToMsg(start_pose_tf, start_pose_stamped.pose);
        // auto start_path = setFixedPath(start_pose_stamped);

        tf2::Transform start_pose_tf;
        tf2::fromMsg(agent_start_pose_vel.pose.pose, start_pose_tf);
        start_pose_tf = agent_to_map_tf * start_pose_tf;
        auto start_pose_stamped = agent_start_pose_vel.pose;
        start_pose_stamped.pose = tf2::toMsg(start_pose_tf);
        auto start_path = setFixedPath(start_pose_stamped);

        get_plan_srv->start.header.frame_id = map_frame_id_;
        get_plan_srv->start.header.stamp = now;
        get_plan_srv->start.pose = start_path.poses.back().pose;
        front_pose_pub_.publish(start_path.poses.back());

        get_plan_srv->goal.header.frame_id = map_frame_id_;
        get_plan_srv->goal.header.stamp = now;
        get_plan_srv->goal.pose = predicted_goals[agent_start_pose_vel.id];

        // ROS_DEBUG_NAMED(NODE_NAME,
        //                 "agent start: x=%.2f, y=%.2f, theta=%.2f, "
        //                 "goal: x=%.2f, y=%.2f, theta=%.2f",
        //                 get_plan_srv->start.pose.position.x, get_plan_srv->start.pose.position.y, tf::getYaw(get_plan_srv->start.pose.orientation), get_plan_srv->goal.pose.position.x,
        //                 get_plan_srv->goal.pose.position.y, tf::getYaw(get_plan_srv->goal.pose.orientation));

        RCLCPP_DEBUG(node_->get_logger(), "agent start: x=%.2f, y=%.2f, theta=%.2f, goal: x=%.2f, y=%.2f, theta=%.2f", get_plan_srv->start.pose.position.x, get_plan_srv->start.pose.position.y,
                     tf2::getYaw(get_plan_srv->start.pose.orientation), get_plan_srv->goal.pose.position.x, get_plan_srv->goal.pose.position.y, tf2::getYaw(get_plan_srv->goal.pose.orientation));

        // make plan for agent
        if (get_plan_client_) {
          // if (get_plan_client_.call(get_plan_srv)) {
          //   if (!get_plan_srv.response.plan.poses.empty()) {
          auto future = get_plan_client_->async_send_request(get_plan_srv);
          if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (!response->plan.poses.empty()) {
              AgentPathVel agent_path_vel;
              agent_path_vel.id = agent_start_pose_vel.id;
              agent_path_vel.path = get_plan_srv.response.plan;
              agent_path_vel.start_vel = agent_start_pose_vel.vel;
              path_vels_.push_back(agent_path_vel);
              got_new_agent_paths_ = true;
            } else {
              RCLCPP_WARN(node_->get_logger(), "Got empty path for agent, start or goal position is probably invalid");
            }
          } else {
            RCLCPP_WARN(node_->get_logger(), "Failed to call %s service", get_plan_srv_name_.c_str());
          }
        } else {
          RCLCPP_WARN(node_->get_logger(), "%s service does not exist, re-trying to subscribe", get_plan_srv_name_.c_str());
          get_plan_client_ = node_->create_client<nav_msgs::srv::GetPlan>("get_plan");
        }
      }
    }
  }

  return predictAgentsFromPaths(req, res);
}

void predictAgentsFromPaths(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) {
  auto tracked_agents = tracked_agents_;

  if (got_new_agent_paths_) {
    for (auto agent_path_vel : path_vels_) {
      auto &poses = agent_path_vel.path.poses;
      if (!poses.empty()) {
        agent_path_prediction::msg::PredictedPoses predicted_poses;
        predicted_poses.id = agent_path_vel.id;

        auto lin_vel = std::hypot(agent_path_vel.start_vel.twist.linear.x, agent_path_vel.start_vel.twist.linear.y);
        // auto now = ros::Time::now();
        auto now = node_->now();

        predicted_poses.poses.resize(poses.size());
        for (size_t i = 0; i < poses.size(); ++i) {
          auto &pose = poses[i];
          geometry_msgs::msg::PoseWithCovarianceStamped predicted_pose;
          if (i == 0 || lin_vel == 0.0) {
            predicted_pose.header.stamp = now;
          } else {
            auto &last_pose = poses[i - 1];
            auto dist = std::hypot(pose.pose.position.x - last_pose.pose.position.x, pose.pose.position.y - last_pose.pose.position.y);
            // predicted_pose.header.stamp = predicted_poses.poses[i - 1].header.stamp + ros::Duration(dist / lin_vel);
            predicted_pose.header.stamp = predicted_poses.poses[i - 1].header.stamp + rclcpp::Duration::from_seconds(dist / lin_vel);
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

        for (auto it = tracked_agents.agents.begin(); it != tracked_agents.agents.end(); ++it) {  // TODD: Check this
          if (it->track_id == predicted_poses.id) {
            tracked_agents.agents.erase(it);
            break;
          }
        }
        // ROS_DEBUG_NAMED(NODE_NAME, "Processed new path for agent %ld with %ld poses in frame %s", agent_path_vel.id, predicted_poses.poses.size(),
        //                 predicted_poses.poses.front().header.frame_id.c_str());
        RCLCPP_DEBUG(node_->get_logger(), "Processed new path for agent %ld with %ld poses in frame %s", agent_path_vel.id, predicted_poses.poses.size(),
                     predicted_poses.poses.front().header.frame_id.c_str());
      }
    }
  }
  got_new_agent_paths_ = false;

  for (auto &poses : last_predicted_poses_) {
    if (!poses.poses.empty()) {
      geometry_msgs::msg::PoseStamped start_pose;
      geometry_msgs::msg::TwistStamped start_twist;
      if (transformPoseTwist(tracked_agents, poses.id, poses.poses.front().header.frame_id, start_pose, start_twist)) {
        auto last_prune_index_it = last_prune_indices_.find(poses.id);
        auto begin_index = (last_prune_index_it != last_prune_indices_.end()) ? last_prune_index_it->second : 0;
        auto prune_index = prunePath(begin_index, start_pose.pose, poses.poses);
        last_prune_indices_[poses.id] = prune_index;
        if (prune_index < 0 || prune_index > poses.poses.size()) {
          // ROS_ERROR_NAMED(NODE_NAME, "Logical error, cannot prune path");
          RCLCPP_ERROR(node_->get_logger(), "Logical error, cannot prune path");
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
          // auto now = ros::Time::now();
          auto now = node_->now();
          for (size_t i = 0; i < pruned_path.size(); i++) {
            if (i == 0 || lin_vel == 0) {
              pruned_path[i].header.stamp = now;
            } else {
              auto &pose = pruned_path[i].pose.pose;
              auto &last_pose = pruned_path[i - 1].pose.pose;
              auto dist = std::hypot(pose.position.x - last_pose.position.x, pose.position.y - last_pose.position.y);
              pruned_path[i].header.stamp = pruned_path[i - 1].header.stamp + rclcpp::Duration::from_seconds(dist / lin_vel);
            }
          }

          agent_path_prediction::msg::PredictedPoses predicted_poses;
          predicted_poses.id = poses.id;
          predicted_poses.start_velocity = start_twist;
          predicted_poses.poses = pruned_path;

          res.predicted_agents_poses.push_back(predicted_poses);
          // ROS_INFO("Pushed the poses");
          RCLCPP_DEBUG(node_->get_logger(), "Giving path of %ld points from %ld points for agent %d", predicted_poses.poses.size(), poses.poses.size(), poses.id);
        }
      }
    }
  }

  return true;
}

// Remove this and make it a subscriber
void setGoal(const std::shared_ptr<agent_path_prediction::srv::AgentGoal::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentGoal::Response> res) {
  RCLCPP_DEBUG(node_->get_logger(), "Received new agent goal");
  got_external_goal_ = true;
  external_goals_.clear();
  path_vels_.clear();
  for (auto &goal : req.goals) {
    external_goals_.push_back(goal);
  }
  res.success = true;
  res.message = "Goal has been set.";
}

void resetPredictionSrvs(const std::shared_ptr<std_srvs::Empty::Request> req, std::shared_ptr<std_srvs::Empty::Response> res) {
  got_new_agent_paths_ = false;
  got_external_goal_ = false;
  last_predicted_poses_.clear();
  path_vels_.clear();
  check_path_ = false;
  behind_pose_ = geometry_msgs::msg::Transform();
  return true;
}

void AgentPathPrediction::trackedAgentsCB(const cohan_msgs::msg::TrackedAgents &tracked_agents) { tracked_agents_ = tracked_agents; }

void AgentPathPrediction::externalPathsCB(const cohan_msgs::msg::AgentPathArray::ConstPtr &external_paths) {
  external_paths_ = external_paths;
  got_new_agent_paths_ = true;
}

void AgentPathPrediction::predictedGoalCB(const agent_path_prediction::msg::PredictedGoals::ConstPtr &predicted_goals) {
  // get goals
  predicted_goals_ = *predicted_goals;
}

nav_msgs::msg::Path AgentPathPrediction::setFixedPath(const geometry_msgs::msg::PoseStamped &start_pose) {
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

size_t AgentPathPrediction::prunePath(size_t begin_index, const geometry_msgs::msg::Pose &pose, const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> &path) {
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

bool AgentPathPrediction::transformPoseTwist(const cohan_msgs::msg::TrackedAgents &tracked_agents, const uint64_t &agent_id, const std::string &to_frame, geometry_msgs::msg::PoseStamped &pose,
                                             geometry_msgs::msg::TwistStamped &twist) const {
  for (const auto &agent : tracked_agents.agents) {
    if (agent.track_id == agent_id) {
      for (const auto &segment : agent.segments) {
        if (segment.type == default_agent_part_) {
          geometry_msgs::msg::PoseStamped pose_ut;
          pose_ut.header.stamp = tracked_agents.header.stamp;
          pose_ut.header.frame_id = tracked_agents.header.frame_id;
          pose_ut.pose = segment.pose.pose;
          twist.header.stamp = tracked_agents.header.stamp;
          twist.header.frame_id = tracked_agents.header.frame_id;
          twist.twist = segment.twist.twist;
          try {
            // ROS1
            //  tf::Stamped<tf::Pose> pose_tf;
            //  tf::poseStampedMsgToTF(pose_ut, pose_tf);
            //  tf::StampedTransform start_pose_to_plan_transform;

            tf2::Stamped<tf2::Transform> pose_tf;
            tf2::fromMsg(pose_ut, pose_tf);
            geometry_msgs::msg::TransformStamped start_pose_to_plan_transform;

            if (to_frame.empty() || pose_ut.header.frame_id.empty() || twist.header.frame_id.empty()) {
              continue;
            }

            // tf_.waitForTransform(to_frame, pose_ut.header.frame_id, ros::Time(0), ros::Duration(0.5));
            // tf_.lookupTransform(to_frame, pose_ut.header.frame_id, ros::Time(0), start_pose_to_plan_transform);
            // pose_tf.setData(start_pose_to_plan_transform * pose_tf);
            // pose_tf.frame_id_ = to_frame;
            // tf::poseStampedTFToMsg(pose_tf, pose);
            // geometry_msgs::msg::Twist start_twist_to_plan_transform;
            // tf_.lookupTwist(to_frame, twist.header.frame_id, ros::Time::now(), ros::Duration(0.1), start_twist_to_plan_transform);

            // Wait for transform
            if (!tf_buffer_->canTransform(to_frame, pose_ut.header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(500))) {
              RCLCPP_WARN(node_->get_logger(), "Transform not available");
            }
            // Lookup transform
            try {
              start_pose_to_plan_transform = tf_buffer_->lookupTransform(to_frame, pose_ut.header.frame_id, tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
              RCLCPP_ERROR(node_->get_logger(), "Transform lookup failed: %s", ex.what());
            }

            tf2::Transform transform;
            tf2::fromMsg(start_pose_to_plan_transform.transform, transform);
            pose_tf.setData(transform * pose_tf);
            pose_tf.frame_id_ = to_frame;
            pose = tf2::toMsg(pose_tf);

            geometry_msgs::msg::Twist start_twist_to_plan_transform;
            // tf_.lookupTwist(to_frame, twist.header.frame_id, ros::Time::now(), ros::Duration(0.1), start_twist_to_plan_transform);
            start_twist_to_plan_transform = getRelativeTwist(to_frame, twist.header.frame_id, node_->now(), rclcpp::Duration::from_seconds(0.1));
            twist.twist.linear.x -= start_twist_to_plan_transform.linear.x;
            twist.twist.linear.y -= start_twist_to_plan_transform.linear.y;
            twist.twist.angular.z -= start_twist_to_plan_transform.angular.z;
            twist.header.frame_id = to_frame;
            return true;
          } catch (const tf2::LookupException &ex) {
            // ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n", ex.what());
            RCLCPP_ERROR(node_->get_logger(), "No Transform available Error: %s", ex.what());
          } catch (const tf2::ConnectivityException &ex) {
            // ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
            RCLCPP_ERROR(node_->get_logger(), "Connectivity Error: %s", ex.what());
          } catch (const tf2::ExtrapolationException &ex) {
            // ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
            RCLCPP_ERROR(node_->get_logger(), "Extrapolation Error: %s", ex.what());
          }
          break;
        }
      }
      break;
    }
  }
  return false;
}

geometry_msgs::msg::Twist AgentPathPrediction::getRelativeTwist(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &time_now,
                                                                const rclcpp::Duration &duration) const {
  geometry_msgs::msg::Twist relative_twist;
  try {
    // Get transforms at current time and past time (time_now - duration)
    auto tf_now = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePoint(std::chrono::nanoseconds(time_now.nanoseconds())));
    auto tf_past = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePoint(std::chrono::nanoseconds((time_now - duration).nanoseconds())));

    // Positions
    tf2::Vector3 pos_now(tf_now.transform.translation.x, tf_now.transform.translation.y, tf_now.transform.translation.z);
    tf2::Vector3 pos_past(tf_past.transform.translation.x, tf_past.transform.translation.y, tf_past.transform.translation.z);

    // Calculate linear velocity (delta_pos / delta_time)
    double dt = duration.seconds();
    tf2::Vector3 linear_vel = (pos_now - pos_past) / dt;

    // Orientations
    tf2::Quaternion rot_now(tf_now.transform.rotation.x, tf_now.transform.rotation.y, tf_now.transform.rotation.z, tf_now.transform.rotation.w);
    tf2::Quaternion rot_past(tf_past.transform.rotation.x, tf_past.transform.rotation.y, tf_past.transform.rotation.z, tf_past.transform.rotation.w);

    // Calculate angular velocity
    tf2::Quaternion rot_delta = rot_past.inverse() * rot_now;

    // Convert quaternion to angle-axis to get angular velocity
    double angle = rot_delta.getAngle();
    tf2::Vector3 axis = rot_delta.getAxis();

    if (angle > M_PI) {
      angle -= 2 * M_PI;
    }

    tf2::Vector3 angular_vel = axis * (angle / dt);

    relative_twist.linear.x = linear_vel.x();
    relative_twist.linear.y = linear_vel.y();
    relative_twist.linear.z = linear_vel.z();

    relative_twist.angular.x = angular_vel.x();
    relative_twist.angular.y = angular_vel.y();
    relative_twist.angular.z = angular_vel.z();

  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to compute relative twist: %s", ex.what());
  }

  return relative_twist;
}

}  // namespace agents

// the main method starts a rosnode and initializes the optotrack_person class
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // initializing agent_path_prediction class
  auto ros2_node = std::make_shared<rclcpp::Node>(NODE_NAME);
  RCLCPP_DEBUG(ros2_node->get_logger(), "started %s node", NODE_NAME);

  agents::AgentPathPrediction agent_path_prediction(ros2_node);
  agent_path_prediction.initialize();

  rclcpp::spin(ros2_node);
  rclcpp::shutdown();

  return 0;
}