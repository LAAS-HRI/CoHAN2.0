/*********************************************************************
 * Majorly modified by Phani Teja Singamaneni from 2020-2025
 * Additional changes licensed under the MIT License. See LICENSE file.
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <hateb_local_planner/hateb_local_planner_ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <ros2_helpers/utils.hpp>

// pluginlib macros
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <utility>

// register this planner both as a Nav2 Controller
PLUGINLIB_EXPORT_CLASS(hateb_local_planner::HATebLocalPlannerROS, nav2_core::Controller)

namespace hateb_local_planner {

void HATebLocalPlannerROS::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                                     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  // Lock the node
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  // Create logger and clock
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  RCLCPP_INFO(logger_, "Configuring HATeb local planner: %s", plugin_name_.c_str());

  // Load configuration parameters from ROS 2 parameter server
  cfg_ = std::make_shared<HATebConfig>();
  cfg_->initialize(node, name);
  cfg_->setupParameterCallback();

  // Initialize intra process nodes
  intra_node_costmap_converter_ = std::make_shared<rclcpp::Node>("costmap_converter", node->get_namespace(), rclcpp::NodeOptions().use_intra_process_comms(true));
  intra_node_btree_ = std::make_shared<rclcpp::Node>("behavior_tree_cohan", node->get_namespace(), rclcpp::NodeOptions().use_intra_process_comms(true));

  // Reserve some memory for obstacles
  obstacles_.reserve(500);

  // Create the costmap model for collision checking
  // costmap_model_ = std::make_shared<nav2_costmap_2d::CostmapModel>(*costmap_);
  collision_checker_ = std::make_shared<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(costmap_);

  // Get frame IDs from costmap
  global_frame_ = costmap_ros_->getGlobalFrameID();
  cfg_->map_frame = global_frame_;
  robot_base_frame_ = costmap_ros_->getBaseFrameID();

  // Create visualization instance
  visualization_ = std::make_shared<TebVisualization>(node, cfg_);

  // Create robot footprint/contour model for optimization
  cfg_->robot_model = getRobotFootprintFromParamServer(node);

  // Create human footprint/contour model for optimization
  auto agent_radius = cfg_->agent.radius;
  if (agent_radius < 0.0) {
    RCLCPP_WARN(logger_, "agent radius is set to negative, using 0.0");
    agent_radius = 0.0;
  }
  cfg_->human_model = std::make_shared<CircularFootprint>(agent_radius);

  // Create the planner interface
  planner_ = PlannerInterfacePtr(std::make_shared<HATebOptimalPlanner>(node, *cfg_, &obstacles_, cfg_->robot_model, visualization_, &via_points_, cfg_->human_model, &agents_via_points_map_));
  planner_->local_weight_optimaltime_ = cfg_->optim.weight_optimaltime;

  // Initialize a costmap to polygon converter
  if (!cfg_->obstacles.costmap_converter_plugin.empty()) {
    try {
      costmap_converter_loader_ = std::make_unique<pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons>>("costmap_converter", "costmap_converter::BaseCostmapToPolygons");
      costmap_converter_ = costmap_converter_loader_->createSharedInstance(cfg_->obstacles.costmap_converter_plugin);
      std::string converter_name = costmap_converter_loader_->getName(cfg_->obstacles.costmap_converter_plugin);
      // Replace '::' by '/' for parameter namespace
      boost::replace_all(converter_name, "::", "/");

      costmap_converter_->setOdomTopic(cfg_->odom_topic);
      costmap_converter_->initialize(intra_node_costmap_converter_);
      costmap_converter_->setCostmap2D(costmap_);
      const auto rate = std::make_shared<rclcpp::Rate>((double)cfg_->obstacles.costmap_converter_rate);
      costmap_converter_->startWorker(rate, costmap_, cfg_->obstacles.costmap_converter_spin_thread);
      RCLCPP_INFO_STREAM(logger_, "Costmap conversion plugin " << cfg_->obstacles.costmap_converter_plugin << " loaded.");

    } catch (pluginlib::PluginlibException& ex) {
      RCLCPP_WARN(logger_, "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treated as point obstacles. Error message: %s", ex.what());
      costmap_converter_.reset();
    }
  } else {
    RCLCPP_INFO(logger_, "No costmap conversion plugin specified. All occupied costmap cells are treated as point obstacles.");
  }

  // Get footprint of the robot and minimum and maximum distance from the
  // center of the robot to its footprint vertices.
  footprint_spec_ = costmap_ros_->getRobotFootprint();

  // The radii are updated in the function below
  nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);

  // Create collision checker
  collision_checker_ = std::make_shared<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(costmap_);
  collision_checker_->setCostmap(costmap_);

  /*
  /
  / May not be needed ?
  /
  /
  */
  odom_helper_ = std::make_shared<nav2_util::OdomSmoother>(node, 0.3, cfg_->odom_topic);
  /////

  // Validate optimization footprint and costmap footprint
  validateFootprints(cfg_->robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_->obstacles.min_obstacle_dist);

  // Initialize failure detector
  double controller_frequency = 10.0;
  node->get_parameter_or("controller_frequency", controller_frequency, controller_frequency);
  failure_detector_.setBufferLength(std::round(cfg_->recovery.oscillation_filter_duration * controller_frequency));

  // Setup callback for custom obstacles
  custom_obst_sub_ = node->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>("obstacles", 1, std::bind(&HATebLocalPlannerROS::customObstacleCB, this, std::placeholders::_1));

  // Setup callback for custom via-points
  via_points_sub_ = node->create_subscription<nav_msgs::msg::Path>("via_points", 1, std::bind(&HATebLocalPlannerROS::customViaPointsCB, this, std::placeholders::_1));

  // Fix the namespace for some topics and services
  invisible_humans_sub_topic_ = std::string(cfg_->invisible_humans_sub_topic);
  predict_srv_name_ = std::string(cfg_->predict_srv_name);
  reset_prediction_srv_name_ = std::string(cfg_->reset_prediction_srv_name);

  if (!cfg_->ns.empty()) {
    invisible_humans_sub_topic_ = "/" + cfg_->ns + std::string(cfg_->invisible_humans_sub_topic);
    predict_srv_name_ = "/" + cfg_->ns + std::string(cfg_->predict_srv_name);
    reset_prediction_srv_name_ = "/" + cfg_->ns + std::string(cfg_->reset_prediction_srv_name);
  }

  // Setup callback for invisible humans
  inv_humans_sub_ =
      node->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(invisible_humans_sub_topic_, 1, std::bind(&HATebLocalPlannerROS::InvHumansCB, this, std::placeholders::_1));

  // Setup agent prediction clients
  predict_agents_client_ = node->create_client<agent_path_prediction::srv::AgentPosePredict>(predict_srv_name_);
  reset_agents_prediction_client_ = node->create_client<std_srvs::srv::Trigger>(reset_prediction_srv_name_);

  // Service servers and publishers
  optimize_server_ = node->create_service<cohan_msgs::srv::Optimize>(OPTIMIZE_SRV_NAME, std::bind(&HATebLocalPlannerROS::optimizeStandalone, this, std::placeholders::_1, std::placeholders::_2));
  log_pub_ = node->create_publisher<std_msgs::msg::String>(HATEB_LOG, 1);

  // Initialize the pointer to agents, backoff and mode switch
  agents_ptr_ = std::make_shared<hateb_local_planner::Agents>(node, tf_, costmap_ros, cfg_);
  backoff_ptr_ = std::make_shared<Backoff>(node, costmap_ros, cfg_);
  backoff_ptr_->initializeOffsets(robot_circumscribed_radius_);

  // Get behavior tree XML path
  if (cfg_->bt_xml_path.empty()) {
    RCLCPP_ERROR(logger_, "Please provide the xml path by setting the bt_xml_path param");
  }
  bt_mode_switch_.initialize(node, cfg_->bt_xml_path, agents_ptr_, backoff_ptr_);

  // Initialize timers and properties
  last_call_time_ = clock_->now() - rclcpp::Duration::from_seconds(cfg_->hateb.pose_prediction_reset_time);
  last_omega_sign_change_ = clock_->now() - rclcpp::Duration::from_seconds(cfg_->optim.omega_chage_time_seperation);
  last_omega_ = 0.0;
  isMode_ = 0;
  goal_ctrl_ = true;
  reset_states_ = true;
  goal_reached_ = false;
  custom_via_points_active_ = false;
  no_infeasible_plans_ = 0;
  last_preferred_rotdir_ = RotType::none;
  horizon_reduced_ = false;
  time_last_oscillation_ = clock_->now();
  time_last_infeasible_plan_ = clock_->now();

  initialized_ = true;
  RCLCPP_INFO(logger_, "HATeb local planner plugin initialized.");
}

/// Check these methods carefully later
////
////
///
///
///
///
void HATebLocalPlannerROS::cleanup() {
  RCLCPP_INFO(logger_, "Cleaning up HATeb local planner");

  // Reset all shared pointers and release resources
  planner_.reset();
  visualization_.reset();
  costmap_converter_->stopWorker();
  collision_checker_.reset();
  agents_ptr_.reset();
  backoff_ptr_.reset();

  // Reset subscribers, publishers, and service clients/servers
  custom_obst_sub_.reset();
  inv_humans_sub_.reset();
  via_points_sub_.reset();
  predict_agents_client_.reset();
  reset_agents_prediction_client_.reset();
  optimize_server_.reset();
  log_pub_.reset();

  initialized_ = false;
  return;
}

void HATebLocalPlannerROS::activate() {
  RCLCPP_INFO(logger_, "Activating HATeb local planner");
  // Reset state for new navigation task
  goal_reached_ = false;
  horizon_reduced_ = false;
  no_infeasible_plans_ = 0;
  return;
}

void HATebLocalPlannerROS::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating HATeb local planner");
  return;
}
////
////
////
////
////

void HATebLocalPlannerROS::setPlan(const nav_msgs::msg::Path& path) {
  // Check if plugin is initialized
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "HATeb local planner has not been initialized, please call configure() before using this planner");
    return;
  }

  // Store the global plan
  global_plan_ = path;

  // we do not clear the local planner here, since setPlan is called frequently
  // whenever the global planner updates the plan. the local planner checks
  // whether it is required to reinitialize the trajectory or not within each
  // velocity computation step.

  // reset goal_reached_ flag
  goal_reached_ = false;
}

geometry_msgs::msg::TwistStamped HATebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
                                                                               nav2_core::GoalChecker* goal_checker) {
  auto node = node_.lock();
  geometry_msgs::msg::TwistStamped cmd_vel;
  auto start_time = node->now();
  if ((start_time - last_call_time_).seconds() > cfg_->hateb.pose_prediction_reset_time) {
    resetAgentsPrediction();
  }
  last_call_time_ = start_time;

  // check if plugin initialized
  logs_.clear();
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "hateb_local_planner has not been initialized, please call configure() before using this planner");
    throw std::runtime_error("hateb_local_planner has not been configured!");
  }

  if (reset_states_) {
    agents_ptr_->resetAgents();  // Pass it to BT
    reset_states_ = false;
  }

  cmd_vel.header.stamp = node->now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;

  // Use robot pose from parameter
  robot_pose_ = PoseSE2(pose.pose);

  // Use robot velocity from parameter
  robot_vel_ = velocity;
  logs_ += "velocity: " + std::to_string(robot_vel_.linear.x) + " " + std::to_string(robot_vel_.linear.y) + "; ";

  // prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(pose, global_plan_, cfg_->trajectory.global_plan_prune_distance);

  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  auto transform_start_time = node->now();
  PlanCombined transformed_plan_combined;
  int goal_idx;
  geometry_msgs::msg::TransformStamped tf_plan_to_global;

  if (!transformGlobalPlan(global_plan_, pose, *costmap_, global_frame_, cfg_->trajectory.max_global_plan_lookahead_dist, transformed_plan_combined, &goal_idx, &tf_plan_to_global)) {
    RCLCPP_WARN(logger_, "Could not transform the global plan to the frame of the controller");
    throw std::runtime_error("Could not transform the global plan to the frame of the controller");
  }
  auto& transformed_plan = transformed_plan_combined.plan_to_optimize;

  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);

  // Check if we have reached the goal
  geometry_msgs::msg::PoseStamped global_goal;
  tf2::doTransform(global_plan_.poses.back(), global_goal, tf_plan_to_global);
  if (!goal_reached_ && goal_checker->isGoalReached(pose.pose, global_goal.pose, velocity)) {
    goal_reached_ = true;  // prevent multiple calls
    onGoalReached();
    return cmd_vel;
  }

  // Return false if the transformed global plan is empty
  if (transformed_plan.empty()) {
    RCLCPP_WARN(logger_, "Transformed plan is empty. Cannot determine a local plan.");
    throw std::runtime_error("Transformed plan is empty");
  }

  // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  // Overwrite goal orientation if needed

  if (cfg_->trajectory.global_plan_overwrite_orientation) {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_.poses, transformed_plan.back(), goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual
    // goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  } else {
    robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }

  // overwrite/update start of the transformed plan with the actual robot
  // position (allows using the plan as initial trajectory)
  if (transformed_plan.size() == 1)  // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::msg::PoseStamped());  // insert start (not yet initialized)
  }
  transformed_plan.front() = pose;  // update start

  // clear currently existing obstacles
  obstacles_.clear();

  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  if (costmap_converter_) {
    updateObstacleContainerWithCostmapConverter();
  } else {
    updateObstacleContainerWithCostmap();
  }

  // Also consider custom obstacles (must be called after other updates, since the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
  updateObstacleContainerWithInvHumans();

  // Do not allow config changes during the following optimization step
  std::scoped_lock cfg_lock(cfg_->configMutex());

  //************************************************************************
  //      The NEW BT Automation
  std::vector<AgentPlanCombined> transformed_agent_plans;
  AgentPlanVelMap transformed_agent_plan_vel_map;
  tickTreeAndUpdatePlans(pose, transformed_agent_plans, transformed_agent_plan_vel_map);
  updateAgentViaPointsContainers(transformed_agent_plan_vel_map, cfg_->trajectory.global_plan_viapoint_sep);

  //************************************************************************

  std::string mode;
  if (isMode_ == 0) {
    mode = "DualBand";
  } else if (isMode_ == 1) {
    mode = "VelObs";
  } else if (isMode_ == 2) {
    mode = "Backoff";
  } else if (isMode_ == 3) {
    mode = "Passing through";
  } else if (isMode_ == 4) {
    mode = "Approaching Pillar";
  } else if (isMode_ == 5) {
    mode = "Approaching Goal";
  } else {
    mode = "SingleBand";
  }
  logs_ += "Mode: " + mode + ", ";

  std_msgs::msg::String log_msg;
  log_msg.data = logs_;
  log_pub_->publish(log_msg);

  // update via-points container
  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  transformed_plan.front() = pose;
  if (!custom_via_points_active_) {
    updateViaPointsContainer(transformed_plan, cfg_->trajectory.global_plan_viapoint_sep);
  }

  // Now perform the actual optimization
  hateb_local_planner::msg::OptimizationCostArray op_costs;

  double dt_resize = cfg_->trajectory.dt_ref;
  double dt_hyst_resize = cfg_->trajectory.dt_hysteresis;
  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_->goal_tolerance.free_goal_vel, &transformed_agent_plan_vel_map, &op_costs, dt_resize, dt_hyst_resize, isMode_);

  if (!success) {
    planner_->clearPlanner();  // force reinitialization for next time
    RCLCPP_WARN(logger_, "hateb_local_planner was not able to obtain a local plan for the current setting.");

    ++no_infeasible_plans_;  // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = node->now();
    last_cmd_ = cmd_vel.twist;
    throw std::runtime_error("hateb_local_planner was not able to obtain a local plan");
  }

  PlanTrajCombined plan_traj_combined;
  plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
  plan_traj_combined.optimized_trajectory = planner_->getFullTrajectory().points;
  plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
  visualization_->publishTrajectory(plan_traj_combined);

  if (cfg_->planning_mode == 1) {
    visualization_->publishAgentGlobalPlans(transformed_agent_plans);
    std::vector<AgentPlanTrajCombined> agent_plans_traj_array;
    for (auto& agent_plan_combined : transformed_agent_plans) {
      AgentPlanTrajCombined agent_plan_traj_combined;
      agent_plan_traj_combined.id = agent_plan_combined.id;
      agent_plan_traj_combined.plan_before = agent_plan_combined.plan_before;
      agent_plan_traj_combined.optimized_trajectory = planner_->getFullAgentTrajectory(agent_plan_traj_combined.id).points;

      agent_plan_traj_combined.plan_after = agent_plan_combined.plan_after;
      agent_plans_traj_array.push_back(agent_plan_traj_combined);
    }
    visualization_->publishAgentTrajectories(agent_plans_traj_array);
  }

  double ttg = std::hypot(transformed_plan.back().pose.position.x - transformed_plan.front().pose.position.x, transformed_plan.back().pose.position.y - transformed_plan.front().pose.position.y) /
               std::hypot(robot_vel_.linear.x, robot_vel_.linear.y);

  // Check feasibility (but within the first few states only)
  if (cfg_->robot.is_footprint_dynamic) {
    // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    // The radii are updated in the function below
    nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);
    // backoff_ptr_->initializeOffsets(robot_circumscribed_radius_);
  }

  bool feasible = planner_->isTrajectoryFeasible(collision_checker_, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_, cfg_->trajectory.feasibility_check_no_poses);
  if (!feasible) {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    // now we reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    RCLCPP_WARN(logger_, "HATebLocalPlannerROS: trajectory is not feasible. Resetting planner...");

    ++no_infeasible_plans_;  // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = node->now();
    last_cmd_ = cmd_vel.twist;

    throw std::runtime_error("hateb_local_planner trajectory is not feasible");
  }

  // Get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->trajectory.control_look_ahead_poses, dt_resize)) {
    planner_->clearPlanner();
    RCLCPP_WARN(logger_, "HATebLocalPlannerROS: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_;  // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = node->now();
    last_cmd_ = cmd_vel.twist;
    throw std::runtime_error("hateb_local_planner velocity command invalid");
  }

  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->robot.max_vel_x, cfg_->robot.max_vel_y, cfg_->robot.max_vel_theta, cfg_->robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a
  // soft-constraint and opposed to the other constraints not affected by
  // penalty_epsilon. The user might add a safety margin to the parameter
  // itself.
  if (cfg_->robot.cmd_angle_instead_rotvel) {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, cfg_->robot.wheelbase, 0.95 * cfg_->robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z)) {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();
      RCLCPP_WARN(logger_, "HATebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
      ++no_infeasible_plans_;  // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = node->now();
      throw std::runtime_error("hateb_local_planner steering angle is not finite");
    }
  }

  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;

  if (backoff_ptr_->isBackoffGoalReached() && !goal_ctrl_) {
    cmd_vel.twist = geometry_msgs::msg::Twist();
  }

  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;

  // Now visualize everything
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_.poses);
  visualization_->publishMode(isMode_);

  return cmd_vel;
}

bool HATebLocalPlannerROS::onGoalReached() {
  if (goal_reached_) {
    bt_mode_switch_.resetBT();
    planner_->clearPlanner();
    resetAgentsPrediction();
    agents_ptr_->resetAgents();
    isMode_ = 0;
    goal_ctrl_ = true;
    reset_states_ = true;
    return true;
  }
  return false;
}

bool HATebLocalPlannerROS::tickTreeAndUpdatePlans(const geometry_msgs::msg::PoseStamped& robot_pose, std::vector<AgentPlanCombined>& transformed_agent_plans,
                                                  AgentPlanVelMap& transformed_agent_plan_vel_map) {
  // Ticks the tree once and returns the current planning mode
  auto mode_info = bt_mode_switch_.tickAndGetMode();
  // hateb_local_planner::msg::PlanningMode mode_info;

  // TODO(sphanit): Update this globally across the package
  isMode_ = static_cast<int>(mode_info.plan_mode) - 1;

  if (mode_info.plan_mode == PLAN::BACKOFF) {
    // Stopping the planner from the setting the goal to complete to do the Backoff Recovery
    goal_ctrl_ = false;
    return true;
  }

  goal_ctrl_ = true;

  // Return if there are no moving visible humans
  if (mode_info.moving_humans.empty()) {
    // Check and add static humans
    if (!mode_info.still_humans.empty()) {
      for (auto& static_agent : mode_info.still_humans) {
        geometry_msgs::msg::Twist empty_vel;
        geometry_msgs::msg::PoseStamped current_hpose;
        current_hpose.header.frame_id = "static";
        current_hpose.pose = agents_ptr_->getAgents()[static_agent];
        PlanStartVelGoalVel plan_start_vel_goal_vel;
        plan_start_vel_goal_vel.plan.push_back(current_hpose);
        plan_start_vel_goal_vel.start_vel = empty_vel;
        plan_start_vel_goal_vel.nominal_vel = 0;
        plan_start_vel_goal_vel.isMode = isMode_;
        transformed_agent_plan_vel_map[static_agent] = plan_start_vel_goal_vel;
      }
    }
    return true;
  }

  // Check and add static humans
  if (!mode_info.still_humans.empty()) {
    for (auto& static_agent : mode_info.still_humans) {
      geometry_msgs::msg::Twist empty_vel;
      geometry_msgs::msg::PoseStamped current_hpose;
      current_hpose.header.frame_id = "static";
      current_hpose.pose = agents_ptr_->getAgents()[static_agent];

      PlanStartVelGoalVel plan_start_vel_goal_vel;
      plan_start_vel_goal_vel.plan.push_back(current_hpose);
      plan_start_vel_goal_vel.start_vel = empty_vel;
      plan_start_vel_goal_vel.nominal_vel = 0;
      plan_start_vel_goal_vel.isMode = isMode_;
      transformed_agent_plan_vel_map[static_agent] = plan_start_vel_goal_vel;
    }
  }

  // Define the prediction service
  auto predict_srv = std::make_shared<agent_path_prediction::srv::AgentPosePredict::Request>();

  // Add the moving agent ids to the prediction service
  int num_agents = 0;
  for (auto& moving_agent : mode_info.moving_humans) {
    predict_srv->ids.push_back(moving_agent);
    num_agents++;

    // TODO(sphanit): Make this configurable
    if (num_agents == 2) {
      break;
    }
  }
  RCLCPP_INFO(logger_, "Requesting prediction for %d agents", num_agents);

  // Set the prediction method based on the mode
  switch (mode_info.predict_mode) {
    case PREDICTION::CONST_VEL: {
      double traj_size = 10;
      double predict_time = 5.0;  // TODO(sphanit): make these values configurable
      for (double i = 1.0; i <= traj_size; i++) {
        predict_srv->predict_times.push_back(predict_time * (i / traj_size));
      }
      predict_srv->type = agent_path_prediction::srv::AgentPosePredict::Request::VELOCITY_OBSTACLE;
    } break;

    case PREDICTION::BEHIND:
      predict_srv->type = agent_path_prediction::srv::AgentPosePredict::Request::BEHIND_ROBOT;
      break;

    case PREDICTION::PREDICT:
      predict_srv->type = agent_path_prediction::srv::AgentPosePredict::Request::PREDICTED_GOAL;
      break;

    case PREDICTION::EXTERNAL:
      predict_srv->type = agent_path_prediction::srv::AgentPosePredict::Request::EXTERNAL;
      break;

    default:
      break;
  }

  // Define the transfrom plans for visualization
  transformed_agent_plans.clear();

  // Call the predict agents service and update the agents plans
  if (predict_agents_client_) {
    auto future = predict_agents_client_->async_send_request(predict_srv);
    RCLCPP_INFO(logger_, "Waiting for agent prediction service response...");
    if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
      RCLCPP_WARN(logger_, "Agent prediction service call timed out");
      auto response = future.get();
      tf2::Stamped<tf2::Transform> tf_agent_plan_to_global;

      for (auto predicted_agents_poses : response->predicted_agents_poses) {
        // Transform agent plans
        AgentPlanCombined agent_plan_combined;
        auto& transformed_vel = predicted_agents_poses.start_velocity;
        if (!transformAgentPlan(robot_pose, *costmap_, global_frame_, predicted_agents_poses.poses, agent_plan_combined, transformed_vel, &tf_agent_plan_to_global)) {
          RCLCPP_WARN(logger_, "Could not transform the agent %d plan to the frame of the controller", predicted_agents_poses.id);
          continue;
        }

        agent_plan_combined.id = predicted_agents_poses.id;
        transformed_agent_plans.push_back(agent_plan_combined);  // Only used for visualization.. remove?

        PlanStartVelGoalVel plan_start_vel_goal_vel;
        plan_start_vel_goal_vel.plan = agent_plan_combined.plan_to_optimize;
        plan_start_vel_goal_vel.start_vel = transformed_vel.twist;
        plan_start_vel_goal_vel.nominal_vel = std::max(0.3, agents_ptr_->getNominalVels()[predicted_agents_poses.id]);  // update this
        if (agent_plan_combined.plan_after.size() > 0) {
          plan_start_vel_goal_vel.goal_vel = transformed_vel.twist;
        }
        transformed_agent_plan_vel_map[agent_plan_combined.id] = plan_start_vel_goal_vel;
      }
    } else {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, THROTTLE_RATE * 1000, "Failed to call %s service, is agent prediction server running?", predict_srv_name_.c_str());
    }
  } else {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, THROTTLE_RATE * 1000, "Predict agents client not initialized");
  }

  return true;
}

void HATebLocalPlannerROS::updateObstacleContainerWithCostmap() {
  // Add costmap obstacles if desired
  if (cfg_->obstacles.include_costmap_obstacles) {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

    for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i) {
      for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j) {
        if (costmap_->getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs - robot_pose_.position();
          if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_->obstacles.costmap_obstacles_behind_robot_dist) {
            continue;
          }

          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void HATebLocalPlannerROS::updateObstacleContainerWithCostmapConverter() {
  if (!costmap_converter_) {
    return;
  }
  // Get obstacles from costmap converter
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles) {
    return;
  }

  for (const auto& i : obstacles->obstacles) {
    const costmap_converter_msgs::msg::ObstacleMsg* obstacle = &i;
    const geometry_msgs::msg::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size() == 1 && obstacle->radius > 0)  // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    } else if (polygon->points.size() == 1)  // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    } else if (polygon->points.size() == 2)  // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y, polygon->points[1].x, polygon->points[1].y)));
    } else if (polygon->points.size() > 2)  // Real polygon
    {
      auto* polyobst = new PolygonObstacle;
      for (auto point : polygon->points) {
        polyobst->pushBackVertex(point.x, point.y);
      }
      polyobst->finalizePolygon();
      obstacles_.emplace_back(polyobst);
    }

    // Set velocity, if obstacle is moving
    if (!obstacles_.empty()) {
      obstacles_.back()->setCentroidVelocity(i.velocities, i.orientation);
    }
  }
}

void HATebLocalPlannerROS::updateObstacleContainerWithCustomObstacles() {
  // Add custom obstacles obtained via message
  std::scoped_lock l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty()) {
    // We only use the global header to specify the obstacle coordinate system
    // instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try {
      geometry_msgs::msg::TransformStamped obstacle_to_map = tf_->lookupTransform(global_frame_, custom_obstacle_msg_.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.8));
      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
    } catch (tf2::TransformException ex) {
      RCLCPP_ERROR(logger_, "%s", ex.what());
      obstacle_to_map_eig.setIdentity();
    }

    for (auto& obstacle : custom_obstacle_msg_.obstacles) {
      if (obstacle.polygon.points.size() == 1 && obstacle.radius > 0)  // circle
      {
        Eigen::Vector3d pos(obstacle.polygon.points.front().x, obstacle.polygon.points.front().y, obstacle.polygon.points.front().z);
        obstacles_.push_back(ObstaclePtr(new CircularObstacle((obstacle_to_map_eig * pos).head(2), obstacle.radius)));
      } else if (obstacle.polygon.points.size() == 1)  // point
      {
        Eigen::Vector3d pos(obstacle.polygon.points.front().x, obstacle.polygon.points.front().y, obstacle.polygon.points.front().z);
        obstacles_.push_back(ObstaclePtr(new PointObstacle((obstacle_to_map_eig * pos).head(2))));
      } else if (obstacle.polygon.points.size() == 2)  // line
      {
        Eigen::Vector3d line_start(obstacle.polygon.points.front().x, obstacle.polygon.points.front().y, obstacle.polygon.points.front().z);
        Eigen::Vector3d line_end(obstacle.polygon.points.back().x, obstacle.polygon.points.back().y, obstacle.polygon.points.back().z);
        obstacles_.push_back(ObstaclePtr(new LineObstacle((obstacle_to_map_eig * line_start).head(2), (obstacle_to_map_eig * line_end).head(2))));
      } else if (obstacle.polygon.points.empty()) {
        RCLCPP_WARN(logger_, "Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
        continue;
      } else  // polygon
      {
        auto* polyobst = new PolygonObstacle;
        for (auto& point : obstacle.polygon.points) {
          Eigen::Vector3d pos(point.x, point.y, point.z);
          polyobst->pushBackVertex((obstacle_to_map_eig * pos).head(2));
        }
        polyobst->finalizePolygon();
        obstacles_.emplace_back(polyobst);
      }

      // Set velocity, if obstacle is moving
      if (!obstacles_.empty()) obstacles_.back()->setCentroidVelocity(obstacle.velocities, obstacle.orientation);
    }
  }
}

void HATebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation) {
  via_points_.clear();

  if (min_separation <= 0) {
    return;
  }

  std::size_t prev_idx = 0;
  for (std::size_t i = 1; i < transformed_plan.size(); ++i)  // skip first one, since we do not need any point before the first
  // min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d(transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position) < min_separation) {
      continue;
    }

    // add via-point
    via_points_.emplace_back(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y);
    prev_idx = i;
  }
}

bool HATebLocalPlannerROS::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, nav_msgs::msg::Path& global_plan, double dist_behind_robot) {
  if (global_plan.poses.empty()) {
    return true;
  }

  try {
    // transform robot pose into the plan frame (we do not wait here, since
    // pruning not crucial, if missed a few times)
    geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(global_plan.header.frame_id, global_pose.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);

    double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

    // iterate plan until a pose close the robot is found
    auto it = global_plan.poses.begin();
    auto erase_end = it;
    while (it != global_plan.poses.end()) {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = (dx * dx) + (dy * dy);
      if (dist_sq < dist_thresh_sq) {
        erase_end = it;
        break;
      }
      ++it;
    }
    if (erase_end == global_plan.poses.end()) {
      return false;
    }

    if (erase_end != global_plan.poses.begin()) global_plan.poses.erase(global_plan.poses.begin(), erase_end);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_DEBUG(logger_, "Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}

bool HATebLocalPlannerROS::transformGlobalPlan(const nav_msgs::msg::Path& global_plan, const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap,
                                               const std::string& global_frame, double max_plan_length, PlanCombined& transformed_plan_combined, int* current_goal_idx,
                                               geometry_msgs::msg::TransformStamped* tf_plan_to_global) const {
  const geometry_msgs::msg::PoseStamped& plan_pose = global_plan.poses[0];

  transformed_plan_combined.plan_to_optimize.clear();

  try {
    if (global_plan.poses.empty()) {
      RCLCPP_ERROR(logger_, "Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::msg::TransformStamped plan_to_global_transform = tf_->lookupTransform(global_frame, global_plan.header.frame_id, global_plan.header.stamp, tf2::durationFromSec(0.5));

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    tf2::doTransform(global_pose, robot_pose, tf_->lookupTransform(global_plan.header.frame_id, global_pose.header.frame_id, plan_pose.header.stamp, tf2::durationFromSec(0.05)));

    // we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) * 1.0;
    // dist_threshold *= 0.90; // just consider 90% of the costmap size to better incorporate point obstacle that are located on the border of the local costmap
    // Planning radius should be within this range (can be adjusted from local costmap params)
    dist_threshold *= 0.9;

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;

    tf2::Stamped<tf2::Transform> tf_pose;
    geometry_msgs::msg::PoseStamped newer_pose;
    // we need to loop to a point on the plan that is within a certain distance
    // of the robot
    for (int j = 0; j < static_cast<int>(global_plan.poses.size()); ++j) {
      double x_diff = robot_pose.pose.position.x - global_plan.poses[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan.poses[j].pose.position.y;
      double new_sq_dist = (x_diff * x_diff) + (y_diff * y_diff);
      if (new_sq_dist > sq_dist_threshold) {
        break;
      }  // force stop if we have reached the costmap border

      if (new_sq_dist < sq_dist)  // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
      }

      const geometry_msgs::msg::PoseStamped& pose = global_plan.poses[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan_combined.plan_before.push_back(newer_pose);
    }
    double plan_length = 0;  // check cumulative Euclidean distance along the plan

    // now we'll transform until points are outside of our distance threshold
    while (i < static_cast<int>(global_plan.poses.size()) && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length)) {
      const geometry_msgs::msg::PoseStamped& pose = global_plan.poses[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan_combined.plan_to_optimize.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan.poses[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan.poses[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      // caclulate distance to previous pose
      if (i > 0 && max_plan_length > 0) {
        plan_length += distance_points2d(global_plan.poses[i - 1].pose.position, global_plan.poses[i].pose.position);
      }
      ++i;
    }

    // // // Modification for hateb_local_planner:
    // // // Return the index of the current goal point (inside the distance
    // // // threshold)
    if (current_goal_idx) {  // minus 1, since i was increased once before leaving the loop
      *current_goal_idx = i - 1;
    }

    while (i < global_plan.poses.size()) {
      const geometry_msgs::msg::PoseStamped& pose = global_plan.poses[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);
      transformed_plan_combined.plan_after.push_back(newer_pose);
      ++i;
    }

    // if we are really close to the goal (<sq_dist_threshold) and the goal is
    // not yet reached (e.g. orientation error >>0) the resulting transformed
    // plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan_combined.plan_after.empty()) {
      tf2::doTransform(global_plan.poses.back(), newer_pose, plan_to_global_transform);

      transformed_plan_combined.plan_after.push_back(newer_pose);

      // Return the index of the current goal point (inside the distance
      // threshold)
      if (current_goal_idx) {
        *current_goal_idx = static_cast<int>(global_plan.poses.size()) - 1;
      }
    } else {
      // Return the index of the current goal point (inside the distance
      // threshold)
      if (current_goal_idx) {
        *current_goal_idx = i - 1;
      }  // subtract 1, since i was increased once
         // before leaving the loop
    }

    // Return the transformation from the global plan to the global planning
    // frame if desired
    if (tf_plan_to_global) {
      *tf_plan_to_global = plan_to_global_transform;
    }
  } catch (tf2::LookupException& ex) {
    RCLCPP_ERROR(logger_, "No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException& ex) {
    RCLCPP_ERROR(logger_, "Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException& ex) {
    RCLCPP_ERROR(logger_, "Extrapolation Error: %s\n", ex.what());
    if (global_plan.poses.size() > 0) {
      RCLCPP_ERROR(logger_, "Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.poses.size(), global_plan.poses[0].header.frame_id.c_str());
    }

    return false;
  }

  return true;
}

double HATebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal, int current_goal_idx,
                                                          const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length) {
  int n = static_cast<int>(global_plan.size());

  // check if we are near the global goal already
  if (current_goal_idx > n - moving_average_length - 2) {
    if (current_goal_idx >= n - 1)  // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    tf2::Quaternion global_orientation;
    tf2::convert(global_plan.back().pose.orientation, global_orientation);
    tf2::Quaternion rotation;
    tf2::convert(tf_plan_to_global.transform.rotation, rotation);
    // TODO(roesmann): avoid conversion to tf2::Quaternion
    return tf2::getYaw(rotation * global_orientation);
  }

  // reduce number of poses taken into account if the desired number of poses is
  // not available
  moving_average_length = std::min(moving_average_length,
                                   n - current_goal_idx - 1);  // maybe redundant, since we have checked the
  // vicinity of the goal before

  std::vector<double> candidates;
  geometry_msgs::msg::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::msg::PoseStamped tf_pose_kp1;

  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i) {
    // Transform pose of the global plan to the planning frame
    tf2::doTransform(global_plan.at(i + 1), tf_pose_kp1, tf_plan_to_global);

    // calculate yaw angle
    candidates.push_back(std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y, tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x));

    if (i < range_end - 1) {
      tf_pose_k = tf_pose_kp1;
    }
  }
  return average_angles(candidates);
}

void HATebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) {
  auto node = node_.lock();
  // Limit translational velocity for forward driving
  vx = std::min(vx, max_vel_x);

  // limit strafing velocity
  if (vy > max_vel_y) {
    vy = max_vel_y;
  } else if (vy < -max_vel_y) {
    vy = -max_vel_y;
  }

  // Limit angular velocity
  if (omega > max_vel_theta) {
    omega = max_vel_theta;
  } else if (omega < -max_vel_theta) {
    omega = -max_vel_theta;
  }

  // Limit backwards velocity
  if (max_vel_x_backwards <= 0) {
    RCLCPP_WARN_ONCE(logger_, "HATebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalizing backwards driving.");
  } else if (vx < -max_vel_x_backwards) {
    vx = -max_vel_x_backwards;
  }

  // slow change of direction in angular velocity
  double min_vel_theta = 0.02;
  if (cfg_->optim.disable_rapid_omega_chage) {
    if (std::signbit(omega) != std::signbit(last_omega_)) {
      // signs are changed
      auto now = node->now();
      std::cout << "Time since last sign change: " << (now - last_omega_sign_change_).seconds() << " seconds." << std::endl;
      if ((now - last_omega_sign_change_).seconds() < cfg_->optim.omega_chage_time_seperation) {
        // do not allow sign change
        omega = std::copysign(min_vel_theta, omega);
      }
      last_omega_sign_change_ = now;
      last_omega_ = omega;
    }
  }
}

double HATebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) {
  if (omega == 0 || v == 0) {
    return 0;
  }
  double radius = v / omega;

  if (fabs(radius) < min_turning_radius) {
    radius = static_cast<double>(g2o::sign(radius)) * min_turning_radius;
  }
  return std::atan(wheelbase / radius);
}

void HATebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist) {
  RCLCPP_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius, logger_,
                   "The inscribed radius of the footprint specified for HATEB optimization (%f) + min_obstacle_dist (%f) are smaller than the inscribed radius of the robot's footprint in the costmap "
                   "parameters (%f, including 'footprint_padding'). Infeasible optimization results might occur frequently!",
                   opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}

void HATebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int& goal_idx) {
  auto node = node_.lock();

  // reduced horizon backup mode
  if (cfg_->recovery.shrink_horizon_backup && goal_idx < static_cast<int>(transformed_plan.size()) - 1 &&
      (no_infeasible_plans_ > 0 || (node->now() - time_last_infeasible_plan_).seconds() < cfg_->recovery.shrink_horizon_min_duration))
  // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations) keep short horizon for   at least a few seconds
  {
    RCLCPP_INFO_COND(no_infeasible_plans_ == 1, logger_, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_->recovery.shrink_horizon_min_duration);

    // Shorten horizon if requested
    // reduce to 50 percent:
    int horizon_reduction = goal_idx / 2;

    if (no_infeasible_plans_ > 9) {
      RCLCPP_INFO_COND(no_infeasible_plans_ == 10, logger_, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
      horizon_reduction /= 2;
    }

    // we have a small overhead here, since we already transformed 50% more of
    // the trajectory. But that's ok for now, since we do not need to make
    // transformGlobalPlan more complex and a reduced horizon should occur just
    // rarely.
    int new_goal_idx_transformed_plan = static_cast<int>(transformed_plan.size()) - horizon_reduction - 1;
    goal_idx -= horizon_reduction;
    if (new_goal_idx_transformed_plan > 0 && goal_idx >= 0) {
      transformed_plan.erase(transformed_plan.begin() + new_goal_idx_transformed_plan, transformed_plan.end());
    } else {
      goal_idx += horizon_reduction;  // this should not happen, but safety first ;-)
    }
  }

  // detect and resolve oscillations
  if (cfg_->recovery.oscillation_recovery) {
    double max_vel_theta;
    double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_->robot.max_vel_x : cfg_->robot.max_vel_x_backwards;
    if (cfg_->robot.min_turning_radius != 0 && max_vel_current > 0) {
      max_vel_theta = std::max(max_vel_current / std::abs(cfg_->robot.min_turning_radius), cfg_->robot.max_vel_theta);
    } else {
      max_vel_theta = cfg_->robot.max_vel_theta;
    }

    failure_detector_.update(last_cmd_, cfg_->robot.max_vel_x, cfg_->robot.max_vel_x_backwards, max_vel_theta, cfg_->recovery.oscillation_v_eps, cfg_->recovery.oscillation_omega_eps);

    bool oscillating = failure_detector_.isOscillating();
    bool recently_oscillated = (node->now() - time_last_oscillation_).seconds() < cfg_->recovery.oscillation_recovery_min_duration;  // check if we have already detected an oscillation recently

    if (oscillating) {
      if (!recently_oscillated) {
        // save current turning direction
        if (robot_vel_.angular.z > 0)
          last_preferred_rotdir_ = RotType::left;
        else
          last_preferred_rotdir_ = RotType::right;
        RCLCPP_WARN(logger_,
                    "HATebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
      }
      time_last_oscillation_ = node->now();
      planner_->setPreferredTurningDir(last_preferred_rotdir_);
    } else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none)  // clear recovery behavior
    {
      last_preferred_rotdir_ = RotType::none;
      planner_->setPreferredTurningDir(last_preferred_rotdir_);
      RCLCPP_INFO(logger_, "HATebLocalPlannerROS: oscillation recovery disabled/expired.");
    }
  }
}

void HATebLocalPlannerROS::customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr obst_msg) {
  std::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;
}

void HATebLocalPlannerROS::customViaPointsCB(const nav_msgs::msg::Path::SharedPtr via_points_msg) {
  RCLCPP_INFO_ONCE(logger_, "Via-points received. This message is printed once.");
  if (cfg_->trajectory.global_plan_viapoint_sep > 0) {
    RCLCPP_WARN(logger_, "Via-points are already obtained from the global plan (global_plan_viapoint_sep>0). Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  std::scoped_lock l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::msg::PoseStamped& pose : via_points_msg->poses) {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  custom_via_points_active_ = !via_points_.empty();
}

FootprintModelPtr HATebLocalPlannerROS::getRobotFootprintFromParamServer(const rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
  std::string model_name;
  model_name = cfg_->robot_footprint.type;
  if (model_name.empty()) {
    RCLCPP_INFO(logger_, "No robot footprint model specified for trajectory optimization. Using point-shaped model.");
    return std::make_shared<PointFootprint>();
  }

  // point
  if (model_name == "point") {
    RCLCPP_INFO(logger_, "Footprint model 'point' loaded for trajectory optimization.");
    return std::make_shared<PointFootprint>(cfg_->obstacles.min_obstacle_dist);
  }

  // circular
  if (model_name == "circular") {
    // get radius
    double radius;
    if (cfg_->robot_footprint.radius <= 0) {
      RCLCPP_ERROR_STREAM(logger_, "Footprint model 'circular' cannot be loaded for trajectory optimization, since param 'footprint_model.radius' is not valid. Using point-model instead.");
      return std::make_shared<PointFootprint>();
    }
    radius = cfg_->robot_footprint.radius;
    RCLCPP_INFO_STREAM(logger_, "Footprint model 'circular' (radius: " << radius << "m) loaded for trajectory optimization.");
    return std::make_shared<CircularFootprint>(radius);
  }

  // line
  if (model_name == "line") {
    // check parameters
    if (cfg_->robot_footprint.line_start.empty() || !cfg_->robot_footprint.line_end.empty()) {
      RCLCPP_ERROR_STREAM(
          logger_,
          "Footprint model 'line' cannot be loaded for trajectory optimization, since param 'footprint_model.line_start' and/or 'footprint_model.line_end' do not exist. Using point-model instead.");
      return std::make_shared<PointFootprint>();
    }
    // get line coordinates
    std::vector<double> line_start = cfg_->robot_footprint.line_start;
    std::vector<double> line_end = cfg_->robot_footprint.line_end;

    if (line_start.size() != 2 || line_end.size() != 2) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Footprint model 'line' cannot be loaded for trajectory optimization, since param 'footprint_model.line_start' and/or 'footprint_model.line_end' do not contain x and y "
                          "coordinates (2D). Using point-model instead.");
      return std::make_shared<PointFootprint>();
    }

    RCLCPP_INFO_STREAM(logger_, "Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] << "]m, line_end: [" << line_end[0] << "," << line_end[1]
                                                                        << "]m) loaded for trajectory optimization.");
    return std::make_shared<LineFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()), cfg_->obstacles.min_obstacle_dist);
  }

  // two circles
  if (model_name == "two_circles") {
    // check parameters
    if (cfg_->robot_footprint.front_offset <= 0 || cfg_->robot_footprint.front_radius <= 0 || cfg_->robot_footprint.rear_offset <= 0 || cfg_->robot_footprint.rear_radius <= 0) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params 'footprint_model.front_offset', 'footprint_model.front_radius', "
                          "'footprint_model.rear_offset' and '>footprint_model.rear_radius' do not exist. Using point-model instead.");
      return std::make_shared<PointFootprint>();
    }
    double front_offset = cfg_->robot_footprint.front_offset;
    double front_radius = cfg_->robot_footprint.front_radius;
    double rear_offset = cfg_->robot_footprint.rear_offset;
    double rear_radius = cfg_->robot_footprint.rear_radius;

    RCLCPP_INFO_STREAM(logger_, "Footprint model 'two_circles' (front_offset: " << front_offset << "m, front_radius: " << front_radius << "m, rear_offset: " << rear_offset
                                                                                << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return std::make_shared<TwoCirclesFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // polygon
  if (model_name == "polygon") {
    // check parameters
    std::string footprint_string = cfg_->robot_footprint.vertices;
    if (footprint_string.empty()) {
      RCLCPP_ERROR_STREAM(logger_, "Footprint model 'polygon' cannot be loaded for trajectory optimization, since param 'footprint_model.vertices' does not exist. Using point-model instead.");
      return std::make_shared<PointFootprint>();
    }

    std::string error;
    std::vector<std::vector<float>> footprint_vertices = nav2_costmap_2d::parseVVF(footprint_string, error);

    if (error != "") {
      RCLCPP_ERROR(logger_, "Error parsing footprint parameter: '%s'. Using point-model instead.", error.c_str());
      RCLCPP_ERROR(logger_, "  Footprint string was '%s'.", footprint_string.c_str());
      return std::make_shared<PointFootprint>();
    }

    // get vertices - expect flat array of [x1, y1, x2, y2, ...]
    if (footprint_vertices.size() >= 3) {
      try {
        Point2dContainer polygon;
        for (auto& vertex : footprint_vertices) {
          polygon.emplace_back(vertex[0], vertex[1]);
        }
        RCLCPP_INFO_STREAM(logger_, "Footprint model 'polygon' loaded for trajectory optimization.");
        return std::make_shared<PolygonFootprint>(polygon);
      } catch (const std::exception& ex) {
        RCLCPP_ERROR_STREAM(logger_, "Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
        return std::make_shared<PointFootprint>();
      }
    } else {
      RCLCPP_ERROR_STREAM(logger_,
                          "Footprint model 'polygon' cannot be loaded for trajectory optimization, since param 'footprint_model.vertices' does not define a valid array of coordinates (needs at least "
                          "6 values in pairs). Using point-model instead.");
      return std::make_shared<PointFootprint>();
    }
  }

  // otherwise
  RCLCPP_WARN_STREAM(logger_, "Unknown robot footprint model specified with parameter 'footprint_model.type'. Using point model instead.");
  return std::make_shared<PointFootprint>();
}

/*************************************************************************************************
 * Humans (or agents) Part of the code from here
 *************************************************************************************************/

void HATebLocalPlannerROS::updateObstacleContainerWithInvHumans() {
  auto node = node_.lock();
  if (!cfg_->hateb.add_invisible_humans) {
    return;
  }

  // Add custom obstacles obtained via message
  std::scoped_lock l(inv_human_mutex_);

  if (!inv_humans_msg_.obstacles.empty()) {
    // We only use the global header to specify the obstacle coordinate system
    // instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    double robot_x;
    double robot_y;
    double robot_yaw;
    Eigen::Vector2d robot_vec;
    std::vector<std::pair<double, int>> dist_idx;

    try {
      geometry_msgs::msg::TransformStamped obstacle_to_map = tf_->lookupTransform(global_frame_, inv_humans_msg_.header.frame_id, node->now(), tf2::durationFromSec(0.8));

      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);

      geometry_msgs::msg::TransformStamped transform_stamped;
      std::string base_link = "base_link";
      if (!cfg_->ns.empty()) {
        base_link = cfg_->ns + "/" + base_link;
      }
      transform_stamped = tf_->lookupTransform("map", base_link, tf2::TimePointZero, tf2::durationFromSec(0.5));

      robot_x = transform_stamped.transform.translation.x;
      robot_y = transform_stamped.transform.translation.y;
      robot_yaw = tf2::getYaw(transform_stamped.transform.rotation);
      robot_vec(std::cos(robot_yaw), std::sin(robot_yaw));
    } catch (tf2::TransformException ex) {
      RCLCPP_ERROR(logger_, "%s", ex.what());
      obstacle_to_map_eig.setIdentity();
    }

    for (auto& obstacle : inv_humans_msg_.obstacles) {
      if (obstacle.polygon.points.size() == 1 && obstacle.radius > 0)  // circle
      {
        Eigen::Vector3d pos(obstacle.polygon.points.front().x, obstacle.polygon.points.front().y, obstacle.polygon.points.front().z);
        obstacles_.push_back(ObstaclePtr(new CircularObstacle((obstacle_to_map_eig * pos).head(2), obstacle.radius)));
      } else if (obstacle.polygon.points.empty()) {
        RCLCPP_WARN(logger_, "Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
        continue;
      } else  // polygon
      {
        auto* polyobst = new PolygonObstacle;
        for (auto& point : obstacle.polygon.points) {
          Eigen::Vector3d pos(point.x, point.y, point.z);
          polyobst->pushBackVertex((obstacle_to_map_eig * pos).head(2));
        }
        polyobst->finalizePolygon();
        obstacles_.emplace_back(polyobst);
      }

      // Set velocity, if obstacle is moving
      if (!obstacles_.empty()) {
        obstacles_.back()->setCentroidVelocity(obstacle.velocities, obstacle.orientation);
        obstacles_.back()->setHuman();
      }
    }
  }
}

void HATebLocalPlannerROS::updateAgentViaPointsContainers(const AgentPlanVelMap& transformed_agent_plan_vel_map, double min_separation) {
  if (min_separation < 0) {
    return;
  }

  // reset via-points for known agents, create via-points for new agents
  for (const auto& transformed_agent_plan_vel_kv : transformed_agent_plan_vel_map) {
    const auto& agent_id = transformed_agent_plan_vel_kv.first;
    const auto& initial_agent_plan = transformed_agent_plan_vel_kv.second.plan;
    if (initial_agent_plan.size() == 1) {
      if (initial_agent_plan[0].header.frame_id == "static") {
        continue;  // Skip this static agent but continue processing others
      }
    }

    if (agents_via_points_map_.find(agent_id) != agents_via_points_map_.end()) {
      agents_via_points_map_[agent_id].clear();
    } else {
      agents_via_points_map_[agent_id] = ViaPointContainer();
    }
  }

  // remove agent via-points for vanished agents
  auto itr = agents_via_points_map_.begin();
  while (itr != agents_via_points_map_.end()) {
    if (transformed_agent_plan_vel_map.count(itr->first) == 0 || agents_ptr_->agentState(itr->first) == hateb_local_planner::AgentState::STOPPED) {
      itr = agents_via_points_map_.erase(itr);
    } else {
      ++itr;
    }
  }

  std::size_t prev_idx;
  for (const auto& transformed_agent_plan_vel_kv : transformed_agent_plan_vel_map) {
    prev_idx = 0;
    const auto& agent_id = transformed_agent_plan_vel_kv.first;
    const auto& transformed_agent_plan = transformed_agent_plan_vel_kv.second.plan;
    for (std::size_t i = 1; i < transformed_agent_plan.size(); ++i) {
      if (distance_points2d(transformed_agent_plan[prev_idx].pose.position, transformed_agent_plan[i].pose.position) < min_separation) {
        continue;
      }
      agents_via_points_map_[agent_id].emplace_back(transformed_agent_plan[i].pose.position.x, transformed_agent_plan[i].pose.position.y);

      prev_idx = i;
    }
  }
}

bool HATebLocalPlannerROS::transformAgentPlan(const geometry_msgs::msg::PoseStamped& robot_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame,
                                              const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& agent_plan, AgentPlanCombined& transformed_agent_plan_combined,
                                              geometry_msgs::msg::TwistStamped& transformed_agent_twist, tf2::Stamped<tf2::Transform>* tf_agent_plan_to_global) const {
  auto node = node_.lock();
  try {
    if (agent_plan.empty()) {
      RCLCPP_ERROR(logger_, "Received agent plan with zero length");
      return false;
    }

    // get agent_plan_to_global_transform from plan frame to global_frame
    geometry_msgs::msg::TransformStamped agent_plan_to_global_transform;
    agent_plan_to_global_transform = tf_->lookupTransform(global_frame, agent_plan.front().header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));
    tf2::Stamped<tf2::Transform> agent_plan_to_global_transform_;
    tf2::fromMsg(agent_plan_to_global_transform, agent_plan_to_global_transform_);

    // transform the full plan to local planning frame
    std::vector<geometry_msgs::msg::PoseStamped> transformed_agent_plan;
    tf2::Stamped<tf2::Transform> tf_pose_stamped;
    geometry_msgs::msg::PoseStamped transformed_pose;
    tf2::Transform tf_pose;
    const auto& agent_start_pose = agent_plan[0];
    for (const auto& agent_pose : agent_plan) {
      if (isMode_ >= 1 && isMode_ < 3) {
        if (std::hypot(agent_pose.pose.pose.position.x - agent_start_pose.pose.pose.position.x, agent_pose.pose.pose.position.y - agent_start_pose.pose.pose.position.y) > (cfg_->agent.radius)) {
          unsigned int mx;
          unsigned int my;
          if (costmap_->worldToMap(agent_pose.pose.pose.position.x, agent_pose.pose.pose.position.y, mx, my)) {
            if (costmap_->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
              break;
            }
          }
        }
      }
      tf2::fromMsg(agent_pose.pose.pose, tf_pose);
      tf_pose_stamped.setData(agent_plan_to_global_transform_ * tf_pose);
      tf_pose_stamped.stamp_ = agent_plan_to_global_transform_.stamp_;
      tf_pose_stamped.frame_id_ = global_frame;
      auto trans_stamped = tf2::toMsg(tf_pose_stamped);

      // ROS does not provide a direct method to transform PoseWithCovarianceStamped, so we only transform the pose part here
      transformed_pose.header = trans_stamped.header;
      transformed_pose.pose.position.x = trans_stamped.transform.translation.x;
      transformed_pose.pose.position.y = trans_stamped.transform.translation.y;
      transformed_pose.pose.position.z = trans_stamped.transform.translation.z;
      transformed_pose.pose.orientation = trans_stamped.transform.rotation;

      transformed_agent_plan.push_back(transformed_pose);
    }

    // transform agent twist to local planning frame
    geometry_msgs::msg::Twist agent_to_global_twist;
    // Use ROS2 implementation from ros2_helpers/utils.hpp
    lookupTwist(global_frame, transformed_agent_twist.header.frame_id, node->now(), tf2::durationFromSec(0.5), agent_to_global_twist, tf_);
    transformed_agent_twist.twist.linear.x -= agent_to_global_twist.linear.x;
    transformed_agent_twist.twist.linear.y -= agent_to_global_twist.linear.y;
    transformed_agent_twist.twist.angular.z -= agent_to_global_twist.angular.z;

    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) * 2.0;
    dist_threshold *= 0.9;

    double sq_dist_threshold = dist_threshold * dist_threshold;
    double x_diff;
    double y_diff;
    double sq_dist;

    // get first point of agent plan within threshold distance from robot
    int start_index = transformed_agent_plan.size();
    int end_index = 0;
    for (int i = 0; i < transformed_agent_plan.size(); i++) {
      x_diff = robot_pose.pose.position.x - transformed_agent_plan[i].pose.position.x;
      y_diff = robot_pose.pose.position.y - transformed_agent_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist < sq_dist_threshold) {
        start_index = i;
        break;
      }
    }
    // now get last point of agent plan withing threshold distance from robot
    for (int i = (transformed_agent_plan.size() - 1); i >= 0; i--) {
      x_diff = robot_pose.pose.position.x - transformed_agent_plan[i].pose.position.x;
      y_diff = robot_pose.pose.position.y - transformed_agent_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist < sq_dist_threshold) {
        end_index = i;
        break;
      }
    }

    transformed_agent_plan_combined.plan_before.clear();
    transformed_agent_plan_combined.plan_to_optimize.clear();
    transformed_agent_plan_combined.plan_after.clear();
    for (int i = 0; i < transformed_agent_plan.size(); i++) {
      if (i < start_index) {
        transformed_agent_plan_combined.plan_before.push_back(transformed_agent_plan[i]);
      } else if (i >= start_index && i <= end_index) {
        transformed_agent_plan_combined.plan_to_optimize.push_back(transformed_agent_plan[i]);
      } else if (i > end_index) {
        transformed_agent_plan_combined.plan_after.push_back(transformed_agent_plan[i]);
      } else {
        RCLCPP_ERROR(logger_, "Transform agent plan indexing error");
      }
    }

    if (tf_agent_plan_to_global) {
      *tf_agent_plan_to_global = agent_plan_to_global_transform_;
    }
  } catch (tf2::LookupException& ex) {
    RCLCPP_ERROR(logger_, "No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException& ex) {
    RCLCPP_ERROR(logger_, "Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException& ex) {
    RCLCPP_ERROR(logger_, "Extrapolation Error: %s\n", ex.what());
    if (!agent_plan.empty()) {
      RCLCPP_ERROR(logger_, "Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)agent_plan.size(), agent_plan.front().header.frame_id.c_str());
    }

    return false;
  }

  return true;
}

void HATebLocalPlannerROS::InvHumansCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr obst_msg) {
  std::scoped_lock l(inv_human_mutex_);
  inv_humans_msg_ = *obst_msg;
}

void HATebLocalPlannerROS::resetAgentsPrediction() {
  auto node = node_.lock();
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  RCLCPP_INFO(logger_, "Resetting agent pose prediction");

  if (!reset_agents_prediction_client_) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, THROTTLE_RATE * 1000, "Reset agents prediction client not initialized");
    return;
  }

  auto future = reset_agents_prediction_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, THROTTLE_RATE * 1000, "Failed to call %s service, is agent prediction server running?", reset_prediction_srv_name_.c_str());
  }
}

bool HATebLocalPlannerROS::optimizeStandalone(const std::shared_ptr<cohan_msgs::srv::Optimize::Request> req, std::shared_ptr<cohan_msgs::srv::Optimize::Response> res) {
  auto node = node_.lock();
  // check if plugin initialized
  if (!initialized_) {
    res->success = false;
    res->message = "planner has not been initialized";
    return true;
  }

  // get robot pose from the costmap
  geometry_msgs::msg::PoseStamped robot_pose_tf;
  costmap_ros_->getRobotPose(robot_pose_tf);

  // transform global plan to the frame of local costmap
  // RCLCPP_INFO(logger_, "transforming robot global plans");
  PlanCombined transformed_plan_combined;
  int goal_idx;
  geometry_msgs::msg::TransformStamped tf_robot_plan_to_global;

  if (!transformGlobalPlan(req->robot_plan, robot_pose_tf, *costmap_, global_frame_, cfg_->trajectory.max_global_plan_lookahead_dist, transformed_plan_combined, &goal_idx, &tf_robot_plan_to_global)) {
    res->success = false;
    res->message = "Could not transform the global plan to the local frame";
    return true;
  }
  auto& transformed_plan = transformed_plan_combined.plan_to_optimize;

  // check if the transformed robot plan is empty
  if (transformed_plan.empty()) {
    res->success = false;
    res->message = "Robot's transformed plan is empty";
    return true;
  }

  // update obstacles container
  obstacles_.clear();
  if (costmap_converter_) {
    updateObstacleContainerWithCostmapConverter();
  } else {
    updateObstacleContainerWithCostmap();
  }
  updateObstacleContainerWithCustomObstacles();
  updateObstacleContainerWithInvHumans();

  // update via-points container
  updateViaPointsContainer(transformed_plan, cfg_->trajectory.global_plan_viapoint_sep);

  // do not allow config changes from now until end of optimization
  std::scoped_lock cfg_lock(cfg_->configMutex());

  // update agents
  AgentPlanVelMap transformed_agent_plan_vel_map;
  std::vector<AgentPlanCombined> transformed_agent_plans;
  tf2::Stamped<tf2::Transform> tf_agent_plan_to_global;

  if (!req->agent_plan_array.paths.empty()) {
    for (const auto& agent_path : req->agent_plan_array.paths) {
      AgentPlanCombined agent_plan_combined;
      geometry_msgs::msg::TwistStamped transformed_vel;
      transformed_vel.header.frame_id = global_frame_;
      std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> agent_path_cov;
      for (const auto& agent_pose : agent_path.path.poses) {
        geometry_msgs::msg::PoseWithCovarianceStamped agent_pos_cov;
        agent_pos_cov.header = agent_pose.header;
        agent_pos_cov.pose.pose = agent_pose.pose;
        agent_path_cov.push_back(agent_pos_cov);
      }
      if (!transformAgentPlan(robot_pose_tf, *costmap_, global_frame_, agent_path_cov, agent_plan_combined, transformed_vel, &tf_agent_plan_to_global)) {
        res->success = false;
        res->message = "could not transform agent" + std::to_string(agent_path.id) + " plan to the local frame";
        return true;
      }
      agent_plan_combined.id = agent_path.id;
      transformed_agent_plans.push_back(agent_plan_combined);

      PlanStartVelGoalVel plan_start_vel_goal_vel;
      plan_start_vel_goal_vel.plan = agent_plan_combined.plan_to_optimize;
      plan_start_vel_goal_vel.start_vel = transformed_vel.twist;
      plan_start_vel_goal_vel.nominal_vel = std::max(0.3, agents_ptr_->getNominalVels()[agent_plan_combined.id]);  // update this
      if (agent_plan_combined.plan_after.size() > 0) {
        plan_start_vel_goal_vel.goal_vel = transformed_vel.twist;
      }
      transformed_agent_plan_vel_map[agent_plan_combined.id] = plan_start_vel_goal_vel;
    }
  } else if (!req->agents_ids.empty()) {
    auto predict_srv = std::make_shared<agent_path_prediction::srv::AgentPosePredict::Request>();

    predict_srv->ids = req->agents_ids;
    double traj_size = 10;
    double predict_time = 5.0;  // TODO(unknown): make these values configurable
    for (double i = 1.0; i <= traj_size; i++) {
      predict_srv->predict_times.push_back(predict_time * (i / traj_size));
    }
    predict_srv->type = agent_path_prediction::srv::AgentPosePredict::Request::VELOCITY_OBSTACLE;

    if (predict_agents_client_) {
      auto future = predict_agents_client_->async_send_request(predict_srv);
      if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        auto response = future.get();

        for (auto predicted_agents_poses : response->predicted_agents_poses) {
          // Transform agent plans
          AgentPlanCombined agent_plan_combined;
          auto& transformed_vel = predicted_agents_poses.start_velocity;

          if (!transformAgentPlan(robot_pose_tf, *costmap_, global_frame_, predicted_agents_poses.poses, agent_plan_combined, transformed_vel, &tf_agent_plan_to_global)) {
            res->success = false;
            res->message = "could not transform agent" + std::to_string(predicted_agents_poses.id) + " plan to the local frame";
            return true;
          }

          agent_plan_combined.id = predicted_agents_poses.id;
          transformed_agent_plans.push_back(agent_plan_combined);

          PlanStartVelGoalVel plan_start_vel_goal_vel;
          plan_start_vel_goal_vel.plan = agent_plan_combined.plan_to_optimize;
          plan_start_vel_goal_vel.start_vel = transformed_vel.twist;
          plan_start_vel_goal_vel.nominal_vel = std::max(0.3, agents_ptr_->getNominalVels()[agent_plan_combined.id]);  // update this
          if (agent_plan_combined.plan_after.size() > 0) {
            plan_start_vel_goal_vel.goal_vel = transformed_vel.twist;
          }
          transformed_agent_plan_vel_map[agent_plan_combined.id] = plan_start_vel_goal_vel;
        }
      } else {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, THROTTLE_RATE * 1000, "Failed to call %s service, is agent prediction server running?", predict_srv_name_.c_str());
      }
    } else {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, THROTTLE_RATE * 1000, "Predict agents client not initialized");
    }
  }

  updateAgentViaPointsContainers(transformed_agent_plan_vel_map, cfg_->trajectory.global_plan_viapoint_sep);

  // now perform the actual planning
  robot_vel_ = odom_helper_->getTwist();
  hateb_local_planner::msg::OptimizationCostArray op_costs;

  double dt_resize = cfg_->trajectory.dt_ref;
  double dt_hyst_resize = cfg_->trajectory.dt_hysteresis;

  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_->goal_tolerance.free_goal_vel, &transformed_agent_plan_vel_map, &op_costs, dt_resize, dt_hyst_resize, isMode_);
  if (!success) {
    planner_->clearPlanner();
    res->success = false;
    res->message = "planner was not able to obtain a local plan for the current setting";
    return true;
  }

  PlanTrajCombined plan_traj_combined;
  plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
  auto robot_trajectory = planner_->getFullTrajectory();
  plan_traj_combined.optimized_trajectory = robot_trajectory.points;
  plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
  visualization_->publishTrajectory(plan_traj_combined);

  // Add robot trajectory to result
  res->robot_trajectory = robot_trajectory;

  std::vector<AgentPlanTrajCombined> agent_plans_traj_array;
  for (auto& agent_plan_combined : transformed_agent_plans) {
    AgentPlanTrajCombined agent_plan_traj_combined;
    cohan_msgs::msg::AgentTrajectory agent_trajectory;
    auto trajectory = planner_->getFullAgentTrajectory(agent_plan_combined.id);
    agent_plan_traj_combined.id = agent_plan_combined.id;
    agent_plan_traj_combined.plan_before = agent_plan_combined.plan_before;
    agent_plan_traj_combined.optimized_trajectory = trajectory.points;
    agent_plan_traj_combined.plan_after = agent_plan_combined.plan_after;
    agent_plans_traj_array.push_back(agent_plan_traj_combined);

    // Add human trajectories to the result
    agent_trajectory.id = agent_plan_combined.id;
    agent_trajectory.type = cohan_msgs::msg::AgentType::HUMAN;
    agent_trajectory.trajectory = trajectory;
    res->human_trajectories.trajectories.push_back(agent_trajectory);
  }
  visualization_->publishAgentTrajectories(agent_plans_traj_array);

  // now visualize everything
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_.poses);
  visualization_->publishAgentGlobalPlans(transformed_agent_plans);
  // Note: Do not call this before publishAgentTrajectories --> will lead to segFault
  planner_->visualize();

  res->success = true;
  res->message = "planning successful";

  // check feasibility of robot plan
  bool feasible = planner_->isTrajectoryFeasible(collision_checker_, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_, cfg_->trajectory.feasibility_check_no_poses);
  if (!feasible) {
    res->message += "\nhowever, trajectory is not feasible";
  }

  // get the velocity command for this sampling interval
  geometry_msgs::msg::Twist cmd_vel;
  if (!planner_->getVelocityCommand(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_->trajectory.control_look_ahead_poses, dt_resize)) {
    res->message += feasible ? "\nhowever," : "\nand";
    res->message += " velocity command is invalid";
  }
  // saturate velocity
  saturateVelocity(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_->robot.max_vel_x, cfg_->robot.max_vel_y, cfg_->robot.max_vel_theta, cfg_->robot.max_vel_x_backwards);
  res->cmd_vel = cmd_vel;

  // clear the planner only after getting the velocity command
  planner_->clearPlanner();

  return true;
}

void hateb_local_planner::HATebLocalPlannerROS::setSpeedLimit(const double& speed_limit, const bool& percentage) {
  // TODO: Implement speed limit logic if needed
  // For now, just ignore the parameters
}

}  // end namespace hateb_local_planner
