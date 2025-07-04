/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Copyright (c) 2020 LAAS/CNRS
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
 * Authors: Christoph Rösmann, Phani Teja Singamaneni
 *********************************************************************/
#include <hateb_local_planner/hateb_local_planner_ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <boost/algorithm/string.hpp>
// MBF return codes
#include <mbf_msgs/ExePathResult.h>
#include <std_msgs/Float64.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

#include <string>
#include <utility>

#define PREDICT_SERVICE_NAME "/agent_path_predict/predict_agent_poses"
#define RESET_PREDICTION_SERVICE_NAME "/agent_path_predict/reset_prediction_services"
#define OPTIMIZE_SRV_NAME "optimize"
#define APPROACH_SRV_NAME "set_approach_id"
#define PLANNING_SRV_NAME "set_planning_mode"
#define GET_PLANNING_SRV_NAME "get_planning_mode"
#define HATEB_LOG "hateb_log"
#define INVISIBLE_HUMANS_TOPIC "/map_scanner/invisible_humans_obs"
#define DEFAULT_AGENT_SEGMENT cohan_msgs::TrackedSegmentType::TORSO
#define THROTTLE_RATE 5.0  // seconds

// register this planner both as a BaseLocalPlanner and as a MBF's
// CostmapController plugin
PLUGINLIB_EXPORT_CLASS(hateb_local_planner::HATebLocalPlannerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(hateb_local_planner::HATebLocalPlannerROS, mbf_costmap_core::CostmapController)

namespace hateb_local_planner {

HATebLocalPlannerROS::HATebLocalPlannerROS()
    : costmap_ros_(nullptr),
      tf_(nullptr),
      costmap_model_(nullptr),
      costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
      dynamic_recfg_(nullptr),
      custom_via_points_active_(false),
      goal_reached_(false),
      no_infeasible_plans_(0),
      last_preferred_rotdir_(RotType::none),
      horizon_reduced_(false),
      initialized_(false) {}

HATebLocalPlannerROS::HATebLocalPlannerROS(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_ros_(nullptr),
      tf_(nullptr),
      costmap_model_(nullptr),
      costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
      dynamic_recfg_(nullptr),
      custom_via_points_active_(false),
      goal_reached_(false),
      no_infeasible_plans_(0),
      last_preferred_rotdir_(RotType::none),
      horizon_reduced_(false),
      initialized_(false) {
  // initialize the planner
  initialize(std::move(name), tf, costmap_ros);
}

HATebLocalPlannerROS::~HATebLocalPlannerROS() = default;

void HATebLocalPlannerROS::reconfigureCB(HATebLocalPlannerReconfigureConfig &config, uint32_t level) {
  cfg_.reconfigure(config);
  config_ = config;
}

void HATebLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) {
  // check if the plugin is already initialized
  if (!initialized_) {
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle nh("~/" + name);

    // get parameters of HaTebConfig via the nodehandle and override the default config
    cfg_.loadRosParamFromNodeHandle(nh);

    // reserve some memory for obstacles
    obstacles_.reserve(500);

    // init some variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();  // locking should be done in MoveBase.

    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);
    global_frame_ = costmap_ros_->getGlobalFrameID();
    cfg_.map_frame = global_frame_;
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    // create visualization instance
    visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_));

    // create robot footprint/contour model for optimization
    cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);

    // create human footprint/contour model for optimization
    auto agent_radius = cfg_.agent.radius;
    if (agent_radius < 0.0) {
      ROS_WARN("agent radius is set to negative, using 0.0");
      agent_radius = 0.0;
    }
    cfg_.human_model = boost::make_shared<CircularFootprint>(agent_radius);

    // create the planner interface
    planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, cfg_.robot_model, visualization_, &via_points_, cfg_.human_model, &agents_via_points_map_));
    planner_->local_weight_optimaltime_ = cfg_.optim.weight_optimaltime;
    ROS_INFO("Parallel planning in distinctive topologies disabled.");

    // Initialize a costmap to polygon converter
    if (!cfg_.obstacles.costmap_converter_plugin.empty()) {
      try {
        costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle
        // namespace
        boost::replace_all(converter_name, "::", "/");
        costmap_converter_->setOdomTopic(cfg_.odom_topic);
        costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
        costmap_converter_->setCostmap2D(costmap_);

        costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
        ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");
      } catch (pluginlib::PluginlibException &ex) {
        ROS_WARN(
            "The specified costmap converter plugin cannot be loaded. All "
            "occupied costmap cells are treaten as point obstacles. Error "
            "message: %s",
            ex.what());
        costmap_converter_.reset();
      }
    } else {
      ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
    }

    // Get footprint of the robot and minimum and maximum distance from the
    // center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();

    // The radii are updated in the function below
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);

    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // setup dynamic reconfigure
    dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<HATebLocalPlannerReconfigureConfig>>(nh);
    dynamic_reconfigure::Server<HATebLocalPlannerReconfigureConfig>::CallbackType cb = [this](auto &&PH1, auto &&PH2) {
      reconfigureCB(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_recfg_->setCallback(cb);

    // validate optimization footprint and costmap footprint
    validateFootprints(cfg_.robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);

    // initialize failure detector
    ros::NodeHandle nh_move_base("~");
    double controller_frequency = 10;
    nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
    failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration * controller_frequency));

    // setup callback for custom obstacles
    custom_obst_sub_ = nh.subscribe("obstacles", 1, &HATebLocalPlannerROS::customObstacleCB, this);

    // setup callback for custom via-points
    via_points_sub_ = nh.subscribe("via_points", 1, &HATebLocalPlannerROS::customViaPointsCB, this);

    // Get the namespace from the parameter server (different from the cfg server)
    if (!ros::param::get("~ns", ns_)) {
      ns_ = std::string("");
    }

    // Fix the namespace for some topics and services
    invisible_humans_sub_topic_ = std::string(INVISIBLE_HUMANS_TOPIC);
    predict_srv_name_ = std::string(PREDICT_SERVICE_NAME);
    reset_prediction_srv_name_ = std::string(RESET_PREDICTION_SERVICE_NAME);
    if (!ns_.empty()) {
      invisible_humans_sub_topic_ = "/" + ns_ + std::string(INVISIBLE_HUMANS_TOPIC);
      predict_srv_name_ = "/" + ns_ + std::string(PREDICT_SERVICE_NAME);
      reset_prediction_srv_name_ = "/" + ns_ + std::string(RESET_PREDICTION_SERVICE_NAME);
    }

    // setup callback for invisible humans
    inv_humans_sub_ = nh.subscribe(invisible_humans_sub_topic_, 1, &HATebLocalPlannerROS::InvHumansCB, this);

    // setup agent prediction client with persistent connection
    predict_agents_client_ = nh.serviceClient<agent_path_prediction::AgentPosePredict>(predict_srv_name_, true);
    reset_agents_prediction_client_ = nh.serviceClient<std_srvs::Empty>(reset_prediction_srv_name_, true);

    // Service servers and publishers
    optimize_server_ = nh.advertiseService(OPTIMIZE_SRV_NAME, &HATebLocalPlannerROS::optimizeStandalone, this);
    log_pub_ = nh.advertise<std_msgs::String>(HATEB_LOG, 1);

    // Initialize the pointer to agents, backoff and mode switch
    agents_ptr_ = std::make_shared<agents::Agents>(tf, costmap_ros);
    backoff_ptr_ = std::make_shared<Backoff>(costmap_ros);
    backoff_ptr_->initializeOffsets(robot_circumscribed_radius_);

    std::string xml_path;
    ros::param::get("~bt_xml_path", xml_path);
    if (xml_path == "") {
      ROS_ERROR("Please provide the xml path by setting the bt_xml_path param");
    }
    bt_mode_switch_.initialize(nh, xml_path, agents_ptr_, backoff_ptr_);  // pass costmap_ros, tf and is_real to this and eliminate other pointers

    // Initialize timers and properties
    last_call_time_ = ros::Time::now() - ros::Duration(cfg_.hateb.pose_prediction_reset_time);
    last_omega_sign_change_ = ros::Time::now() - ros::Duration(cfg_.optim.omega_chage_time_seperation);
    last_omega_ = 0.0;
    isMode_ = 0;
    goal_ctrl_ = true;
    reset_states_ = true;

    // set initialized flag
    initialized_ = true;
    ROS_INFO("hateb_local_planner plugin initialized.");

  } else {
    ROS_WARN("hateb_local_planner has already been initialized, doing nothing.");
  }
}

bool HATebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  // check if plugin is initialized
  if (!initialized_) {
    ROS_ERROR(
        "hateb_local_planner has not been initialized, please call "
        "initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // we do not clear the local planner here, since setPlan is called frequently
  // whenever the global planner updates the plan. the local planner checks
  // whether it is required to reinitialize the trajectory or not within each
  // velocity computation step.

  // reset goal_reached_ flag
  goal_reached_ = false;

  return true;
}

bool HATebLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity;
  geometry_msgs::TwistStamped cmd_vel_stamped;
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;
  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t HATebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped &pose, const geometry_msgs::TwistStamped &velocity, geometry_msgs::TwistStamped &cmd_vel,
                                                       std::string &message) {
  auto start_time = ros::Time::now();
  if ((start_time - last_call_time_).toSec() > cfg_.hateb.pose_prediction_reset_time) {
    resetAgentsPrediction();
  }
  last_call_time_ = start_time;

  // check if plugin initialized
  logs_.clear();
  if (!initialized_) {
    ROS_ERROR(
        "hateb_local_planner has not been initialized, please call "
        "initialize() before using this planner");
    message = "hateb_local_planner has not been initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  if (reset_states_) {
    agents_ptr_->resetAgents();  // Pass it to BT
    reset_states_ = false;
  }

  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;

  // Get robot pose
  auto pose_get_start_time = ros::Time::now();
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  robot_pose_ = PoseSE2(robot_pose.pose);

  auto pose_get_time = ros::Time::now() - pose_get_start_time;

  // Get robot velocity
  auto vel_get_start_time = ros::Time::now();
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
  auto vel_get_time = ros::Time::now() - vel_get_start_time;
  logs_ += "velocity: " + std::to_string(robot_vel_.linear.x) + " " + std::to_string(robot_vel_.linear.y) + "; ";

  // prune global plan to cut off parts of the past (spatially before the robot)
  auto prune_start_time = ros::Time::now();
  pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);
  auto prune_time = ros::Time::now() - prune_start_time;

  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  auto transform_start_time = ros::Time::now();
  PlanCombined transformed_plan_combined;
  int goal_idx;
  geometry_msgs::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist, transformed_plan_combined, &goal_idx, &tf_plan_to_global)) {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    message = "Could not transform the global plan to the frame of the controller";
    return mbf_msgs::ExePathResult::INTERNAL_ERROR;
  }
  auto &transformed_plan = transformed_plan_combined.plan_to_optimize;
  auto transform_time = ros::Time::now() - transform_start_time;

  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);

  auto other_start_time = ros::Time::now();
  // check if global goal is reached
  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
  double dx = global_goal.pose.position.x - robot_pose_.x();
  double dy = global_goal.pose.position.y - robot_pose_.y();
  // goal_ctrl = false;
  double delta_orient = g2o::normalize_theta(tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta());
  if (fabs(std::sqrt((dx * dx) + (dy * dy))) < cfg_.goal_tolerance.xy_goal_tolerance && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance &&
      (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0) && goal_ctrl_) {
    goal_reached_ = true;
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  // Return false if the transformed global plan is empty
  if (transformed_plan.empty()) {
    ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
    message = "Transformed plan is empty";
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }

  // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  // Overwrite goal orientation if needed
  if (cfg_.trajectory.global_plan_overwrite_orientation) {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
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
    transformed_plan.insert(transformed_plan.begin(),
                            geometry_msgs::PoseStamped());  // insert start (not yet initialized)
  }
  transformed_plan.front() = robot_pose;  // update start

  // clear currently existing obstacles
  obstacles_.clear();
  auto other_time = ros::Time::now() - other_start_time;

  // Update obstacle container with costmap information or polygons provided by
  // a costmap_converter plugin
  auto cc_start_time = ros::Time::now();
  if (costmap_converter_) {
    updateObstacleContainerWithCostmapConverter();
  } else {
    updateObstacleContainerWithCostmap();
  }

  // also consider custom obstacles (must be called after other updates, since
  // the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
  updateObstacleContainerWithInvHumans();
  auto cc_time = ros::Time::now() - cc_start_time;

  // Do not allow config changes during the following optimization step
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

  //************************************************************************
  //      The NEW BT Automation
  std::vector<AgentPlanCombined> transformed_agent_plans;
  AgentPlanVelMap transformed_agent_plan_vel_map;
  tickTreeAndUpdatePlans(robot_pose, transformed_agent_plans, transformed_agent_plan_vel_map);
  updateAgentViaPointsContainers(transformed_agent_plan_vel_map, cfg_.trajectory.global_plan_viapoint_sep);
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

  std_msgs::String log_msg;
  log_msg.data = logs_;
  log_pub_.publish(log_msg);

  // update via-points container
  auto via_start_time = ros::Time::now();
  // overwrite/update start of the transformed plan with the actual robot
  // position (allows using the plan as initial trajectory)
  transformed_plan.front() = robot_pose;
  if (!custom_via_points_active_) {
    updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);
  }
  auto via_time = ros::Time::now() - via_start_time;

  // Now perform the actual
  auto plan_start_time = ros::Time::now();
  hateb_local_planner::OptimizationCostArray op_costs;

  double dt_resize = cfg_.trajectory.dt_ref;
  double dt_hyst_resize = cfg_.trajectory.dt_hysteresis;
  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel, &transformed_agent_plan_vel_map, &op_costs, dt_resize, dt_hyst_resize, isMode_);

  if (!success) {
    planner_->clearPlanner();  // force reinitialization for next time
    ROS_WARN(
        "hateb_local_planner was not able to obtain a local plan for the "
        "current setting.");

    ++no_infeasible_plans_;  // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "hateb_local_planner was not able to obtain a local plan";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  // op_costs_pub_.publish(op_costs);
  auto plan_time = ros::Time::now() - plan_start_time;

  PlanTrajCombined plan_traj_combined;
  plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
  plan_traj_combined.optimized_trajectory = planner_->getFullTrajectory().points;
  plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
  visualization_->publishTrajectory(plan_traj_combined);

  if (cfg_.planning_mode == 1) {
    visualization_->publishAgentGlobalPlans(transformed_agent_plans);
    std::vector<AgentPlanTrajCombined> agent_plans_traj_array;
    for (auto &agent_plan_combined : transformed_agent_plans) {
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

  // Undo temporary horizon reduction
  auto hr2_start_time = ros::Time::now();

  auto hr2_time = ros::Time::now() - hr2_start_time;

  // Check feasibility (but within the first few states only)
  auto fsb_start_time = ros::Time::now();
  if (cfg_.robot.is_footprint_dynamic) {
    // Update footprint of the robot and minimum and maximum distance from the
    // center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    // The radii are updated in the function below
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);
    // backoff_ptr_->initializeOffsets(robot_circumscribed_radius_);
  }

  bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_, cfg_.trajectory.feasibility_check_no_poses);
  if (!feasible) {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    // now we reset everything to start again with the initialization of new
    // trajectories.
    planner_->clearPlanner();
    ROS_WARN(
        "HATebLocalPlannerROS: trajectory is not feasible. Resetting "
        "planner...");

    ++no_infeasible_plans_;  // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;

    message = "hateb_local_planner trajectory is not feasible";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  auto fsb_time = ros::Time::now() - fsb_start_time;

  // Get the velocity command for this sampling interval
  auto vel_start_time = ros::Time::now();
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.trajectory.control_look_ahead_poses, dt_resize)) {
    planner_->clearPlanner();
    ROS_WARN("HATebLocalPlannerROS: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_;  // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "hateb_local_planner velocity command invalid";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // Saturate velocity, if the optimization results violates the constraints
  // (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a
  // soft-constraint and opposed to the other constraints not affected by
  // penalty_epsilon. The user might add a safety margin to the parameter
  // itself.
  if (cfg_.robot.cmd_angle_instead_rotvel) {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, cfg_.robot.wheelbase, 0.95 * cfg_.robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z)) {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();
      ROS_WARN(
          "HATebLocalPlannerROS: Resulting steering angle is not finite. "
          "Resetting planner...");
      ++no_infeasible_plans_;  // increase number of infeasible solutions in a
                               // row
      time_last_infeasible_plan_ = ros::Time::now();

      message = "hateb_local_planner steering angle is not finite";
      return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }
  }
  auto vel_time = ros::Time::now() - vel_start_time;

  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;

  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;

  // Now visualize everything
  auto viz_start_time = ros::Time::now();
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  visualization_->publishMode(isMode_);

  auto viz_time = ros::Time::now() - viz_start_time;
  auto total_time = ros::Time::now() - start_time;

  ROS_DEBUG_STREAM_COND(total_time.toSec() > 0.1, "\tcompute velocity times:\n"
                                                      << "\t\ttotal time                   " << std::to_string(total_time.toSec()) << "\n"
                                                      << "\t\tpose get time                " << std::to_string(pose_get_time.toSec()) << "\n"
                                                      << "\t\tvel get time                 " << std::to_string(vel_get_time.toSec()) << "\n"
                                                      << "\t\tprune time                   " << std::to_string(prune_time.toSec()) << "\n"
                                                      << "\t\ttransform time               " << std::to_string(transform_time.toSec())
                                                      << "\n"
                                                      // << "\t\thorizon setup time           "
                                                      // << std::to_string((hr1_time + hr2_time).toSec()) << "\n"
                                                      << "\t\tother time                   " << std::to_string(other_time.toSec()) << "\n"
                                                      << "\t\tcostmap convert time         " << std::to_string(cc_time.toSec()) << "\n"
                                                      << "\t\tvia points time              " << std::to_string(via_time.toSec())
                                                      << "\n"
                                                      // << "\t\tagent time                   " << std::to_string(agent_time.toSec()) << "\n"
                                                      << "\t\tplanning time                " << std::to_string(plan_time.toSec()) << "\n"
                                                      << "\t\tplan feasibility check time  " << std::to_string(fsb_time.toSec()) << "\n"
                                                      << "\t\tvelocity extract time        " << std::to_string(vel_time.toSec()) << "\n"
                                                      << "\t\tvisualization publish time   " << std::to_string(viz_time.toSec()) << "\n=========================");
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool HATebLocalPlannerROS::isGoalReached() {
  if (goal_reached_) {
    bt_mode_switch_.resetBT();
    ROS_INFO("GOAL Reached!");
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

bool HATebLocalPlannerROS::tickTreeAndUpdatePlans(const geometry_msgs::PoseStamped &robot_pose, std::vector<AgentPlanCombined> &transformed_agent_plans,
                                                  AgentPlanVelMap &transformed_agent_plan_vel_map) {
  // Ticks the tree once and returns the current planning mode
  auto mode_info = bt_mode_switch_.tickAndGetMode();

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
      for (auto &static_agent : mode_info.still_humans) {
        geometry_msgs::Twist empty_vel;
        geometry_msgs::PoseStamped current_hpose;
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
    for (auto &static_agent : mode_info.still_humans) {
      geometry_msgs::Twist empty_vel;
      geometry_msgs::PoseStamped current_hpose;
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
  agent_path_prediction::AgentPosePredict predict_srv;

  // Add the moving agent ids to the prediction service
  int num_agents = 0;
  for (auto &moving_agent : mode_info.moving_humans) {
    predict_srv.request.ids.push_back(moving_agent);
    num_agents++;

    // TODO(sphanit): : Make this configurable
    if (num_agents == 2) {
      break;
    }
  }

  // Set the prediction method based on the mode
  switch (mode_info.predict_mode) {
    case PREDICTION::CONST_VEL: {
      double traj_size = 10;
      double predict_time = 5.0;  // TODO(sphanit): make these values configurable
      for (double i = 1.0; i <= traj_size; i++) {
        predict_srv.request.predict_times.push_back(predict_time * (i / traj_size));
      }
      predict_srv.request.type = agent_path_prediction::AgentPosePredictRequest::VELOCITY_OBSTACLE;
    } break;

    case PREDICTION::BEHIND:
      predict_srv.request.type = agent_path_prediction::AgentPosePredictRequest::BEHIND_ROBOT;
      break;

    case PREDICTION::PREDICT:
      predict_srv.request.type = agent_path_prediction::AgentPosePredictRequest::PREDICTED_GOAL;
      break;

    case PREDICTION::EXTERNAL:
      predict_srv.request.type = agent_path_prediction::AgentPosePredictRequest::EXTERNAL;
      break;

    default:
      break;
  }

  // Define the transfrom plans for visualization
  transformed_agent_plans.clear();

  // Call the predict agents service and update the agents plans
  if (predict_agents_client_ && predict_agents_client_.call(predict_srv)) {
    tf2::Stamped<tf2::Transform> tf_agent_plan_to_global;

    for (auto predicted_agents_poses : predict_srv.response.predicted_agents_poses) {
      // Transform agent plans
      AgentPlanCombined agent_plan_combined;
      auto &transformed_vel = predicted_agents_poses.start_velocity;

      if (!transformAgentPlan(*tf_, robot_pose, *costmap_, global_frame_, predicted_agents_poses.poses, agent_plan_combined, transformed_vel, &tf_agent_plan_to_global)) {
        ROS_WARN("Could not transform the agent %d plan to the frame of the controller", predicted_agents_poses.id);
        continue;
      }

      agent_plan_combined.id = predicted_agents_poses.id;
      transformed_agent_plans.push_back(agent_plan_combined);  // Only used for visualization.. remove?

      PlanStartVelGoalVel plan_start_vel_goal_vel;
      plan_start_vel_goal_vel.plan = agent_plan_combined.plan_to_optimize;
      plan_start_vel_goal_vel.start_vel = transformed_vel.twist;
      plan_start_vel_goal_vel.nominal_vel = std::max(0.3, agents_ptr_->getNominalVels()[predicted_agents_poses.id]);  // update this
      plan_start_vel_goal_vel.isMode = isMode_;
      if (agent_plan_combined.plan_after.size() > 0) {
        plan_start_vel_goal_vel.goal_vel = transformed_vel.twist;
      }
      transformed_agent_plan_vel_map[agent_plan_combined.id] = plan_start_vel_goal_vel;
    }

  } else {
    ROS_WARN_THROTTLE(THROTTLE_RATE, "Failed to call %s service, is agent prediction server running?", predict_srv_name_.c_str());
  }

  return true;
}

void HATebLocalPlannerROS::updateObstacleContainerWithCostmap() {
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles) {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

    for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i) {
      for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j) {
        if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE) {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs - robot_pose_.position();
          if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist) {
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

  for (const auto &i : obstacles->obstacles) {
    const costmap_converter::ObstacleMsg *obstacle = &i;
    const geometry_msgs::Polygon *polygon = &obstacle->polygon;

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
      auto *polyobst = new PolygonObstacle;
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
  boost::mutex::scoped_lock l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty()) {
    // We only use the global header to specify the obstacle coordinate system
    // instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try {
      geometry_msgs::TransformStamped obstacle_to_map =
          tf_->lookupTransform(global_frame_, ros::Time::now(), custom_obstacle_msg_.header.frame_id, ros::Time::now(), custom_obstacle_msg_.header.frame_id, ros::Duration(0.8));
      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      obstacle_to_map_eig.setIdentity();
    }

    for (auto &obstacle : custom_obstacle_msg_.obstacles) {
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
        ROS_WARN(
            "Invalid custom obstacle received. List of polygon vertices "
            "is empty. Skipping...");
        continue;
      } else  // polygon
      {
        auto *polyobst = new PolygonObstacle;
        for (auto &point : obstacle.polygon.points) {
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

void HATebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double min_separation) {
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

bool HATebLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose, std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot) {
  if (global_plan.empty()) {
    return true;
  }

  try {
    // transform robot pose into the plan frame (we do not wait here, since
    // pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);

    double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

    // iterate plan until a pose close the robot is found
    auto it = global_plan.begin();
    auto erase_end = it;
    while (it != global_plan.end()) {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = (dx * dx) + (dy * dy);
      if (dist_sq < dist_thresh_sq) {
        erase_end = it;
        break;
      }
      ++it;
    }
    if (erase_end == global_plan.end()) {
      return false;
    }

    if (erase_end != global_plan.begin()) global_plan.erase(global_plan.begin(), erase_end);
  } catch (const tf::TransformException &ex) {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}

bool HATebLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan, const geometry_msgs::PoseStamped &global_pose,
                                               const costmap_2d::Costmap2D &costmap, const std::string &global_frame, double max_plan_length, PlanCombined &transformed_plan_combined,
                                               int *current_goal_idx, geometry_msgs::TransformStamped *tf_plan_to_global) const {
  // this method is a slightly modified version of
  // base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped &plan_pose = global_plan[0];

  transformed_plan_combined.plan_to_optimize.clear();

  try {
    if (global_plan.empty()) {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform =
        tf.lookupTransform(global_frame, ros::Time(0), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id, ros::Duration(0.05));

    // we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) * 1.0;
    // dist_threshold *= 0.90; // just consider 90% of the costmap size to better incorporate point obstacle that are located on the border of the local costmap
    // Planning radius should be within this range (can be adjusted from local costmap params)
    dist_threshold *= 0.9;

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;

    tf2::Stamped<tf2::Transform> tf_pose;
    geometry_msgs::PoseStamped newer_pose;
    // we need to loop to a point on the plan that is within a certain distance
    // of the robot
    for (int j = 0; j < static_cast<int>(global_plan.size()); ++j) {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = (x_diff * x_diff) + (y_diff * y_diff);
      if (new_sq_dist > sq_dist_threshold) {
        break;
      }  // force stop if we have reached the costmap border

      if (new_sq_dist < sq_dist)  // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
      }

      const geometry_msgs::PoseStamped &pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan_combined.plan_before.push_back(newer_pose);
    }
    double plan_length = 0;  // check cumulative Euclidean distance along the plan

    // now we'll transform until points are outside of our distance threshold
    while (i < static_cast<int>(global_plan.size()) && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length)) {
      const geometry_msgs::PoseStamped &pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan_combined.plan_to_optimize.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      // caclulate distance to previous pose
      if (i > 0 && max_plan_length > 0) {
        plan_length += distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);
      }
      ++i;
    }

    // // // Modification for hateb_local_planner:
    // // // Return the index of the current goal point (inside the distance
    // // // threshold)
    if (current_goal_idx) {  // minus 1, since i was increased once before leaving the loop
      *current_goal_idx = i - 1;
    }

    while (i < global_plan.size()) {
      const geometry_msgs::PoseStamped &pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);
      transformed_plan_combined.plan_after.push_back(newer_pose);
      ++i;
    }

    // if we are really close to the goal (<sq_dist_threshold) and the goal is
    // not yet reached (e.g. orientation error >>0) the resulting transformed
    // plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan_combined.plan_after.empty()) {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan_combined.plan_after.push_back(newer_pose);

      // Return the index of the current goal point (inside the distance
      // threshold)
      if (current_goal_idx) {
        *current_goal_idx = static_cast<int>(global_plan.size()) - 1;
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
  } catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0) {
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());
    }

    return false;
  }

  return true;
}

double HATebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped> &global_plan, const geometry_msgs::PoseStamped &local_goal, int current_goal_idx,
                                                          const geometry_msgs::TransformStamped &tf_plan_to_global, int moving_average_length) {
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
  geometry_msgs::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::PoseStamped tf_pose_kp1;

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

void HATebLocalPlannerROS::saturateVelocity(double &vx, double &vy, double &omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) {
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
    ROS_WARN_ONCE(
        "HATebLocalPlannerROS(): Do not choose max_vel_x_backwards "
        "to be <=0. Disable backwards driving by increasing the "
        "optimization weight for penalyzing backwards driving.");
  } else if (vx < -max_vel_x_backwards) {
    vx = -max_vel_x_backwards;
  }

  // slow change of direction in angular velocity
  double min_vel_theta = 0.02;
  if (cfg_.optim.disable_rapid_omega_chage) {
    if (std::signbit(omega) != std::signbit(last_omega_)) {
      // signs are changed
      auto now = ros::Time::now();
      if ((now - last_omega_sign_change_).toSec() < cfg_.optim.omega_chage_time_seperation) {
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
  ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                "The inscribed radius of the footprint specified for TEB "
                "optimization (%f) + min_obstacle_dist (%f) are smaller "
                "than the inscribed radius of the robot's footprint in the "
                "costmap parameters (%f, including 'footprint_padding'). "
                "Infeasible optimziation results might occur frequently!",
                opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}

void HATebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::PoseStamped> &transformed_plan, int &goal_idx) {
  ros::Time current_time = ros::Time::now();

  // reduced horizon backup mode
  if (cfg_.recovery.shrink_horizon_backup && goal_idx < static_cast<int>(transformed_plan.size()) - 1 &&
      (no_infeasible_plans_ > 0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration))
  // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations) keep short horizon for   at least a few seconds
  {
    ROS_INFO_COND(no_infeasible_plans_ == 1,
                  "Activating reduced horizon backup mode for at least %.2f "
                  "sec (infeasible trajectory detected).",
                  cfg_.recovery.shrink_horizon_min_duration);

    // Shorten horizon if requested
    // reduce to 50 percent:
    int horizon_reduction = goal_idx / 2;

    if (no_infeasible_plans_ > 9) {
      ROS_INFO_COND(no_infeasible_plans_ == 10,
                    "Infeasible trajectory detected 10 times in a row: further "
                    "reducing horizon...");
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
  if (cfg_.recovery.oscillation_recovery) {
    double max_vel_theta;
    double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
    if (cfg_.robot.min_turning_radius != 0 && max_vel_current > 0) {
      max_vel_theta = std::max(max_vel_current / std::abs(cfg_.robot.min_turning_radius), cfg_.robot.max_vel_theta);
    } else {
      max_vel_theta = cfg_.robot.max_vel_theta;
    }

    failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta, cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);

    bool oscillating = failure_detector_.isOscillating();
    bool recently_oscillated = (ros::Time::now() - time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration;  // check if we have
                                                                                                                                       // already detected an
                                                                                                                                       // oscillation recently

    if (oscillating) {
      if (!recently_oscillated) {
        // save current turning direction
        if (robot_vel_.angular.z > 0)
          last_preferred_rotdir_ = RotType::left;
        else
          last_preferred_rotdir_ = RotType::right;
        ROS_WARN(
            "HATebLocalPlannerROS: possible oscillation (of the robot or "
            "its local plan) detected. Activating recovery strategy "
            "(prefer current turning direction during optimization).");
      }
      time_last_oscillation_ = ros::Time::now();
      planner_->setPreferredTurningDir(last_preferred_rotdir_);
    } else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none)  // clear recovery behavior
    {
      last_preferred_rotdir_ = RotType::none;
      planner_->setPreferredTurningDir(last_preferred_rotdir_);
      ROS_INFO("HATebLocalPlannerROS: oscillation recovery disabled/expired.");
    }
  }
}

void HATebLocalPlannerROS::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg) {
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;
}

void HATebLocalPlannerROS::customViaPointsCB(const nav_msgs::Path::ConstPtr &via_points_msg) {
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  if (cfg_.trajectory.global_plan_viapoint_sep > 0) {
    ROS_WARN(
        "Via-points are already obtained from the global plan "
        "(global_plan_viapoint_sep>0)."
        "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  boost::mutex::scoped_lock l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::PoseStamped &pose : via_points_msg->poses) {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  custom_via_points_active_ = !via_points_.empty();
}

FootprintModelPtr HATebLocalPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle &nh, const HATebConfig &config) {
  std::string model_name;
  if (!nh.getParam("footprint_model/type", model_name)) {
    ROS_INFO(
        "No robot footprint model specified for trajectory optimization. Using point-shaped "
        "model.");
    return boost::make_shared<PointFootprint>();
  }

  // point
  if (model_name == "point") {
    ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
    return boost::make_shared<PointFootprint>(config.obstacles.min_obstacle_dist);
  }

  // circular
  if (model_name == "circular") {
    // get radius
    double radius;
    if (!nh.getParam("footprint_model/radius", radius)) {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                                << "/footprint_model/radius' does not exist. Using point-model instead.");
      return boost::make_shared<PointFootprint>();
    }
    ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<CircularFootprint>(radius);
  }

  // line
  if (model_name == "line") {
    // check parameters
    if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end")) {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                            << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model "
                                                                                                               "instead.");
      return boost::make_shared<PointFootprint>();
    }
    // get line coordinates
    std::vector<double> line_start;
    std::vector<double> line_end;
    nh.getParam("footprint_model/line_start", line_start);
    nh.getParam("footprint_model/line_end", line_end);
    if (line_start.size() != 2 || line_end.size() != 2) {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                            << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y "
                                                                                                               "coordinates (2D). Using point-model instead.");
      return boost::make_shared<PointFootprint>();
    }

    ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] << "]m, line_end: [" << line_end[0] << "," << line_end[1]
                                                            << "]m) loaded for trajectory optimization.");
    return boost::make_shared<LineFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()), config.obstacles.min_obstacle_dist);
  }

  // two circles
  if (model_name == "two_circles") {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius")) {
      ROS_ERROR_STREAM(
          "Footprint model 'two_circles' cannot be loaded for trajectory optimization, since "
          "params '"
          << nh.getNamespace()
          << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and "
             "'.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointFootprint>();
    }
    double front_offset;
    double front_radius;
    double rear_offset;
    double rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset << "m, front_radius: " << front_radius << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius
                                                                    << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // polygon
  if (model_name == "polygon") {
    // check parameters
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc)) {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                               << "/footprint_model/vertices' does not exist. Using point-model instead.");
      return boost::make_shared<PointFootprint>();
    }
    // get vertices
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      try {
        Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
        ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
        return boost::make_shared<PolygonFootprint>(polygon);
      } catch (const std::exception &ex) {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
        return boost::make_shared<PointFootprint>();
      }
    } else {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                               << "/footprint_model/vertices' does not define an array of coordinates. Using "
                                                                                                                  "point-model instead.");
      return boost::make_shared<PointFootprint>();
    }
  }

  // otherwise
  ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
  return boost::make_shared<PointFootprint>();
}

Point2dContainer HATebLocalPlannerROS::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name) {
  // Make sure we have an array of at least 3 elements.
  if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xmlrpc.size() < 3) {
    ROS_FATAL(
        "The footprint must be specified as list of lists on the "
        "parameter server, %s was specified as %s",
        full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
    throw std::runtime_error(
        "The footprint must be specified as list of lists on the parameter "
        "server with at least "
        "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  Point2dContainer footprint;
  Eigen::Vector2d pt;

  for (int i = 0; i < footprint_xmlrpc.size(); ++i) {
    // Make sure each element of the list is an array of size 2. (x and y
    // coordinates)
    XmlRpc::XmlRpcValue point = footprint_xmlrpc[i];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 2) {
      ROS_FATAL(
          "The footprint (parameter %s) must be specified as list of "
          "lists on the parameter server eg: "
          "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of "
          "that form.",
          full_param_name.c_str());
      throw std::runtime_error(
          "The footprint must be specified as list of "
          "lists on the parameter server eg: "
          "[[x1, y1], [x2, y2], ..., [xn, yn]], but this "
          "spec is not of that form");
    }

    pt.x() = getNumberFromXMLRPC(point[0], full_param_name);
    pt.y() = getNumberFromXMLRPC(point[1], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double HATebLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name) {
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt && value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
    std::string &value_string = value;
    ROS_FATAL(
        "Values in the footprint specification (param %s) must be "
        "numbers. Found value %s.",
        full_param_name.c_str(), value_string.c_str());
    throw std::runtime_error("Values in the footprint specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

/*************************************************************************************************
 * Humans (or agents) Part of the code from here
 *************************************************************************************************/

void HATebLocalPlannerROS::updateObstacleContainerWithInvHumans() {
  if (!cfg_.hateb.add_invisible_humans) {
    return;
  }

  // Add custom obstacles obtained via message
  boost::mutex::scoped_lock l(inv_human_mutex_);

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
      geometry_msgs::TransformStamped obstacle_to_map =
          tf_->lookupTransform(global_frame_, ros::Time::now(), inv_humans_msg_.header.frame_id, ros::Time::now(), inv_humans_msg_.header.frame_id, ros::Duration(0.8));

      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);

      geometry_msgs::TransformStamped transform_stamped;
      std::string base_link = "base_link";
      if (!ns_.empty()) {
        base_link = ns_ + "/" + base_link;
      }
      transform_stamped = tf_->lookupTransform("map", base_link, ros::Time(0), ros::Duration(0.5));

      robot_x = transform_stamped.transform.translation.x;
      robot_y = transform_stamped.transform.translation.y;
      robot_yaw = tf2::getYaw(transform_stamped.transform.rotation);
      robot_vec(std::cos(robot_yaw), std::sin(robot_yaw));
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      obstacle_to_map_eig.setIdentity();
    }

    for (auto &obstacle : inv_humans_msg_.obstacles) {
      if (obstacle.polygon.points.size() == 1 && obstacle.radius > 0)  // circle
      {
        Eigen::Vector3d pos(obstacle.polygon.points.front().x, obstacle.polygon.points.front().y, obstacle.polygon.points.front().z);
        obstacles_.push_back(ObstaclePtr(new CircularObstacle((obstacle_to_map_eig * pos).head(2), obstacle.radius)));
      } else if (obstacle.polygon.points.empty()) {
        ROS_WARN(
            "Invalid custom obstacle received. List of polygon vertices "
            "is empty. Skipping...");
        continue;
      } else  // polygon
      {
        auto *polyobst = new PolygonObstacle;
        for (auto &point : obstacle.polygon.points) {
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

void HATebLocalPlannerROS::updateAgentViaPointsContainers(const AgentPlanVelMap &transformed_agent_plan_vel_map, double min_separation) {
  if (min_separation < 0) {
    return;
  }

  // reset via-points for known agents, create via-points for new agents
  for (const auto &transformed_agent_plan_vel_kv : transformed_agent_plan_vel_map) {
    const auto &agent_id = transformed_agent_plan_vel_kv.first;
    const auto &initial_agent_plan = transformed_agent_plan_vel_kv.second.plan;
    if (initial_agent_plan.size() == 1) {
      if (initial_agent_plan[0].header.frame_id == "static") {
        continue; // Skip this static agent but continue processing others
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
    if (transformed_agent_plan_vel_map.count(itr->first) == 0 || agents_ptr_->agentState(itr->first) == agents::AgentState::STOPPED) {
      itr = agents_via_points_map_.erase(itr);
    } else {
      ++itr;
    }
  }

  std::size_t prev_idx;
  for (const auto &transformed_agent_plan_vel_kv : transformed_agent_plan_vel_map) {
    prev_idx = 0;
    const auto &agent_id = transformed_agent_plan_vel_kv.first;
    const auto &transformed_agent_plan = transformed_agent_plan_vel_kv.second.plan;
    for (std::size_t i = 1; i < transformed_agent_plan.size(); ++i) {
      if (distance_points2d(transformed_agent_plan[prev_idx].pose.position, transformed_agent_plan[i].pose.position) < min_separation) {
        continue;
      }
      agents_via_points_map_[agent_id].emplace_back(transformed_agent_plan[i].pose.position.x, transformed_agent_plan[i].pose.position.y);

      prev_idx = i;
    }
  }
}

bool HATebLocalPlannerROS::transformAgentPlan(const tf2_ros::Buffer &tf2, const geometry_msgs::PoseStamped &robot_pose, const costmap_2d::Costmap2D &costmap, const std::string &global_frame,
                                              const std::vector<geometry_msgs::PoseWithCovarianceStamped> &agent_plan, AgentPlanCombined &transformed_agent_plan_combined,
                                              geometry_msgs::TwistStamped &transformed_agent_twist, tf2::Stamped<tf2::Transform> *tf_agent_plan_to_global) const {
  try {
    if (agent_plan.empty()) {
      ROS_ERROR("Received agent plan with zero length");
      return false;
    }

    // get agent_plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped agent_plan_to_global_transform;
    // tf.waitForTransform(global_frame, agent_plan.front().header.frame_id,
    // ros::Time(0), ros::Duration(0.5));
    agent_plan_to_global_transform = tf2.lookupTransform(global_frame, agent_plan.front().header.frame_id, ros::Time(0), ros::Duration(0.5));
    tf2::Stamped<tf2::Transform> agent_plan_to_global_transform_;
    tf2::fromMsg(agent_plan_to_global_transform, agent_plan_to_global_transform_);

    // transform the full plan to local planning frame
    std::vector<geometry_msgs::PoseStamped> transformed_agent_plan;
    tf2::Stamped<tf2::Transform> tf_pose_stamped;
    geometry_msgs::PoseStamped transformed_pose;
    tf2::Transform tf_pose;
    const auto &agent_start_pose = agent_plan[0];
    for (const auto &agent_pose : agent_plan) {
      if (isMode_ >= 1 && isMode_ < 3) {
        if (std::hypot(agent_pose.pose.pose.position.x - agent_start_pose.pose.pose.position.x, agent_pose.pose.pose.position.y - agent_start_pose.pose.pose.position.y) > (cfg_.agent.radius)) {
          unsigned int mx;
          unsigned int my;
          if (costmap_->worldToMap(agent_pose.pose.pose.position.x, agent_pose.pose.pose.position.y, mx, my)) {
            if (costmap_->getCost(mx, my) >= costmap_2d::LETHAL_OBSTACLE) {
              break;
            }
          }
        }
      }
      tf2::fromMsg(agent_pose.pose.pose, tf_pose);
      tf_pose_stamped.setData(agent_plan_to_global_transform_ * tf_pose);
      tf_pose_stamped.stamp_ = agent_plan_to_global_transform_.stamp_;
      tf_pose_stamped.frame_id_ = global_frame;
      tf2::toMsg(tf_pose_stamped, transformed_pose);

      transformed_agent_plan.push_back(transformed_pose);
    }

    // transform agent twist to local planning frame
    geometry_msgs::Twist agent_to_global_twist;
    lookupTwist(global_frame, transformed_agent_twist.header.frame_id, ros::Time(0), ros::Duration(0.5), agent_to_global_twist);
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

    // ROS_INFO("start: %d, end: %d, full: %ld", start_index, end_index,
    // transformed_agent_plan.size());
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
        ROS_ERROR("Transform agent plan indexing error");
      }
    }

    if (tf_agent_plan_to_global) {
      *tf_agent_plan_to_global = agent_plan_to_global_transform_;
    }
  } catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (!agent_plan.empty()) {
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)agent_plan.size(), agent_plan.front().header.frame_id.c_str());
    }

    return false;
  }

  return true;
}

void HATebLocalPlannerROS::InvHumansCB(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg) {
  boost::mutex::scoped_lock l(inv_human_mutex_);
  inv_humans_msg_ = *obst_msg;
}

void HATebLocalPlannerROS::resetAgentsPrediction() {
  std_srvs::Empty empty_service;
  ROS_INFO("Resetting agent pose prediction");
  if (!reset_agents_prediction_client_ || !reset_agents_prediction_client_.call(empty_service)) {
    ROS_WARN_THROTTLE(THROTTLE_RATE, "Failed to call %s service, is agent prediction server running?", predict_srv_name_.c_str());
    // re-initialize the service
    // reset_agents_prediction_client_ =
    //     nh.serviceClient<std_srvs::Empty>(RESET_PREDICTION_SERVICE_NAME,
    //     true);
  }
}

bool HATebLocalPlannerROS::optimizeStandalone(cohan_msgs::Optimize::Request &req, cohan_msgs::Optimize::Response &res) {
  // check if plugin initialized
  if (!initialized_) {
    res.success = false;
    res.message = "planner has not been initialized";
    return true;
  }

  // get robot pose from the costmap
  geometry_msgs::PoseStamped robot_pose_tf;
  costmap_ros_->getRobotPose(robot_pose_tf);

  // transform global plan to the frame of local costmap
  // ROS_INFO("transforming robot global plans");
  PlanCombined transformed_plan_combined;
  int goal_idx;
  geometry_msgs::TransformStamped tf_robot_plan_to_global;

  if (!transformGlobalPlan(*tf_, req.robot_plan.poses, robot_pose_tf, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist, transformed_plan_combined, &goal_idx,
                           &tf_robot_plan_to_global)) {
    res.success = false;
    res.message = "Could not transform the global plan to the local frame";
    return true;
  }
  auto &transformed_plan = transformed_plan_combined.plan_to_optimize;

  // check if the transformed robot plan is empty
  if (transformed_plan.empty()) {
    res.success = false;
    res.message = "Robot's transformed plan is empty";
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
  updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);

  // do not allow config changes from now until end of optimization
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

  // update agents
  AgentPlanVelMap transformed_agent_plan_vel_map;
  std::vector<AgentPlanCombined> transformed_agent_plans;
  tf2::Stamped<tf2::Transform> tf_agent_plan_to_global;

  if (!req.agent_plan_array.paths.empty()) {
    for (const auto &agent_path : req.agent_plan_array.paths) {
      AgentPlanCombined agent_plan_combined;
      geometry_msgs::TwistStamped transformed_vel;
      transformed_vel.header.frame_id = global_frame_;
      std::vector<geometry_msgs::PoseWithCovarianceStamped> agent_path_cov;
      for (const auto &agent_pose : agent_path.path.poses) {
        geometry_msgs::PoseWithCovarianceStamped agent_pos_cov;
        agent_pos_cov.header = agent_pose.header;
        agent_pos_cov.pose.pose = agent_pose.pose;
        agent_path_cov.push_back(agent_pos_cov);
      }
      if (!transformAgentPlan(*tf_, robot_pose_tf, *costmap_, global_frame_, agent_path_cov, agent_plan_combined, transformed_vel, &tf_agent_plan_to_global)) {
        res.success = false;
        res.message = "could not transform agent" + std::to_string(agent_path.id) + " plan to the local frame";
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
  } else if (!req.agents_ids.empty()) {
    agent_path_prediction::AgentPosePredict predict_srv;
    predict_srv.request.ids = req.agents_ids;
    double traj_size = 10;
    double predict_time = 5.0;  // TODO(unknown): make these values configurable
    for (double i = 1.0; i <= traj_size; i++) {
      predict_srv.request.predict_times.push_back(predict_time * (i / traj_size));
    }
    predict_srv.request.type = agent_path_prediction::AgentPosePredictRequest::VELOCITY_OBSTACLE;

    if (predict_agents_client_ && predict_agents_client_.call(predict_srv)) {
      for (auto predicted_agents_poses : predict_srv.response.predicted_agents_poses) {
        // Transform agent plans
        AgentPlanCombined agent_plan_combined;
        auto &transformed_vel = predicted_agents_poses.start_velocity;

        if (!transformAgentPlan(*tf_, robot_pose_tf, *costmap_, global_frame_, predicted_agents_poses.poses, agent_plan_combined, transformed_vel, &tf_agent_plan_to_global)) {
          res.success = false;
          res.message = "could not transform agent" + std::to_string(predicted_agents_poses.id) + " plan to the local frame";
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
      ROS_WARN_THROTTLE(THROTTLE_RATE, "Failed to call %s service, is agent prediction server running?", predict_srv_name_.c_str());
    }
  }

  updateAgentViaPointsContainers(transformed_agent_plan_vel_map, cfg_.trajectory.global_plan_viapoint_sep);

  // now perform the actual planning
  geometry_msgs::Twist robot_vel_twist;
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
  hateb_local_planner::OptimizationCostArray op_costs;

  double dt_resize = cfg_.trajectory.dt_ref;
  double dt_hyst_resize = cfg_.trajectory.dt_hysteresis;

  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel, &transformed_agent_plan_vel_map, &op_costs, dt_resize, dt_hyst_resize, isMode_);
  if (!success) {
    planner_->clearPlanner();
    res.success = false;
    res.message = "planner was not able to obtain a local plan for the current setting";
    return true;
  }

  PlanTrajCombined plan_traj_combined;
  plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
  auto robot_trajectory = planner_->getFullTrajectory();
  plan_traj_combined.optimized_trajectory = robot_trajectory.points;
  plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
  visualization_->publishTrajectory(plan_traj_combined);

  // Add robot trajectory to result
  res.robot_trajectory = robot_trajectory;

  std::vector<AgentPlanTrajCombined> agent_plans_traj_array;
  for (auto &agent_plan_combined : transformed_agent_plans) {
    AgentPlanTrajCombined agent_plan_traj_combined;
    cohan_msgs::AgentTrajectory agent_trajectory;
    auto trajectory = planner_->getFullAgentTrajectory(agent_plan_combined.id);
    agent_plan_traj_combined.id = agent_plan_combined.id;
    agent_plan_traj_combined.plan_before = agent_plan_combined.plan_before;
    agent_plan_traj_combined.optimized_trajectory = trajectory.points;
    agent_plan_traj_combined.plan_after = agent_plan_combined.plan_after;
    agent_plans_traj_array.push_back(agent_plan_traj_combined);

    // Add human trajectories to the result
    agent_trajectory.id = agent_plan_combined.id;
    agent_trajectory.type = cohan_msgs::AgentType::HUMAN;
    agent_trajectory.trajectory = trajectory;
    res.human_trajectories.trajectories.push_back(agent_trajectory);
  }
  visualization_->publishAgentTrajectories(agent_plans_traj_array);

  // now visualize everything
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  visualization_->publishAgentGlobalPlans(transformed_agent_plans);
  // Note: Do not call this before publishAgentTrajectories --> will lead to segFault
  planner_->visualize();

  res.success = true;
  res.message = "planning successful";

  // check feasibility of robot plan
  bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_, cfg_.trajectory.feasibility_check_no_poses);
  if (!feasible) {
    res.message += "\nhowever, trajectory is not feasible";
  }

  // get the velocity command for this sampling interval
  geometry_msgs::Twist cmd_vel;
  if (!planner_->getVelocityCommand(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.trajectory.control_look_ahead_poses, dt_resize)) {
    res.message += feasible ? "\nhowever," : "\nand";
    res.message += " velocity command is invalid";
  }
  // saturate velocity
  saturateVelocity(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);
  res.cmd_vel = cmd_vel;

  // clear the planner only after getting the velocity command
  planner_->clearPlanner();

  return true;
}

}  // end namespace hateb_local_planner
