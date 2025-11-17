/*********************************************************************
 * Majorly modified by Phani Teja Singamaneni in 2021-2025
 * Additional changes licensed under the MIT License. See LICENSE file.
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
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

#include <hateb_local_planner/hateb_config.hpp>

namespace hateb_local_planner {

void HATebConfig::setupParameterCallback() {
  // Bind all parameters with automatic updates
  bindParameters();

  // Set up parameter change callback with custom validation and thread safety
  param_helper_.setupParameterCallback([this](const std::vector<rclcpp::Parameter>& params) -> bool {
    std::lock_guard<std::mutex> lock(config_mutex_);  // Thread-safe parameter updates
    // Parameters are automatically updated by ParameterHelper bindings
    // Just do validation here
    checkParameters();
    return true;
  });

  // Load initial parameter values
  param_helper_.loadBoundParameters();
}

void HATebConfig::bindParameters() {
  // Helper lambda to add plugin name prefix to parameter names
  auto param_name = [this](const std::string& name) -> std::string { return plugin_name_.empty() ? name : plugin_name_ + "." + name; };

  // General parameters
  param_helper_.bindStringParam(param_name("ns"), ns, "Namespace of the planner");
  param_helper_.bindStringParam(param_name("odom_topic"), odom_topic, "Odometry topic");
  param_helper_.bindStringParam(param_name("global_frame"), global_frame, "Global frame ID");
  param_helper_.bindStringParam(param_name("map_frame"), map_frame, "Map frame ID");
  param_helper_.bindStringParam(param_name("base_frame"), base_frame, "Base frame ID");
  param_helper_.bindStringParam(param_name("footprint_frame"), footprint_frame, "Footprint frame ID");
  param_helper_.bindIntParam(param_name("planning_mode"), planning_mode, 0, 10, "Planning mode");
  param_helper_.bindStringParam(param_name("bt_xml_path"), bt_xml_path, "Behavior tree XML file path");
  param_helper_.bindStringParam(param_name("predict_srv_name"), predict_srv_name, "Agent prediction service name");
  param_helper_.bindStringParam(param_name("reset_prediction_srv_name"), reset_prediction_srv_name, "Agent Prediction reset service name");
  param_helper_.bindStringParam(param_name("invisible_humans_sub_topic"), invisible_humans_sub_topic, "Invisible humans subscription topic name");

  // Trajectory parameters
  param_helper_.bindBoolParam(param_name("teb_autosize"), trajectory.teb_autosize, "Enable TEB autosize");
  param_helper_.bindFloatParam(param_name("dt_ref"), trajectory.dt_ref, 0.01, 10.0, "Reference time step");
  param_helper_.bindFloatParam(param_name("dt_hysteresis"), trajectory.dt_hysteresis, 0.0, 1.0, "Time step hysteresis");
  param_helper_.bindIntParam(param_name("min_samples"), trajectory.min_samples, 2, 1000, "Minimum trajectory samples");
  param_helper_.bindIntParam(param_name("max_samples"), trajectory.max_samples, 3, 10000, "Maximum trajectory samples");
  param_helper_.bindIntParam(param_name("agent_min_samples"), trajectory.agent_min_samples, 2, 1000, "Minimum agent trajectory samples");
  param_helper_.bindBoolParam(param_name("global_plan_overwrite_orientation"), trajectory.global_plan_overwrite_orientation, "Overwrite orientation from global plan");
  param_helper_.bindBoolParam(param_name("allow_init_with_backwards_motion"), trajectory.allow_init_with_backwards_motion, "Allow initialization with backwards motion");
  param_helper_.bindFloatParam(param_name("global_plan_viapoint_sep"), trajectory.global_plan_viapoint_sep, -1.0, 100.0, "Global plan viapoint separation");
  param_helper_.bindBoolParam(param_name("via_points_ordered"), trajectory.via_points_ordered, "Respect viapoint order");
  param_helper_.bindFloatParam(param_name("max_global_plan_lookahead_dist"), trajectory.max_global_plan_lookahead_dist, 0.0, 100.0, "Max global plan lookahead distance");
  param_helper_.bindFloatParam(param_name("global_plan_prune_distance"), trajectory.global_plan_prune_distance, 0.0, 10.0, "Global plan prune distance");
  param_helper_.bindBoolParam(param_name("exact_arc_length"), trajectory.exact_arc_length, "Use exact arc length");
  param_helper_.bindFloatParam(param_name("force_reinit_new_goal_dist"), trajectory.force_reinit_new_goal_dist, 0.0, 10.0, "Force reinit distance threshold");
  param_helper_.bindFloatParam(param_name("force_reinit_new_goal_angular"), trajectory.force_reinit_new_goal_angular, 0.0, 4.0, "Force reinit angular threshold");
  param_helper_.bindIntParam(param_name("feasibility_check_no_poses"), trajectory.feasibility_check_no_poses, 0, 100, "Number of poses for feasibility check");
  param_helper_.bindBoolParam(param_name("publish_feedback"), trajectory.publish_feedback, "Publish planner feedback");
  param_helper_.bindFloatParam(param_name("min_resolution_collision_check_angular"), trajectory.min_resolution_collision_check_angular, 0.0, 6.3, "Min angular resolution for collision check");
  param_helper_.bindIntParam(param_name("control_look_ahead_poses"), trajectory.control_look_ahead_poses, 0, 100, "Control look ahead poses");
  param_helper_.bindFloatParam(param_name("teb_init_skip_dist"), trajectory.teb_init_skip_dist, 0.0, 10.0, "TEB init skip distance");
  param_helper_.bindFloatParam(param_name("visualize_with_time_as_z_axis_scale"), trajectory.visualize_with_time_as_z_axis_scale, 0.0, 100.0, "Visualization time as z-axis scale");

  // Robot parameters
  param_helper_.bindFloatParam(param_name("max_vel_x"), robot.max_vel_x, 0.0, 10.0, "Max forward velocity");
  param_helper_.bindFloatParam(param_name("max_vel_x_backwards"), robot.max_vel_x_backwards, 0.0, 10.0, "Max backward velocity");
  param_helper_.bindFloatParam(param_name("max_vel_y"), robot.max_vel_y, 0.0, 10.0, "Max strafing velocity");
  param_helper_.bindFloatParam(param_name("max_vel_theta"), robot.max_vel_theta, 0.0, 10.0, "Max angular velocity");
  param_helper_.bindFloatParam(param_name("acc_lim_x"), robot.acc_lim_x, 0.0, 10.0, "Max forward acceleration");
  param_helper_.bindFloatParam(param_name("acc_lim_y"), robot.acc_lim_y, 0.0, 10.0, "Max strafing acceleration");
  param_helper_.bindFloatParam(param_name("acc_lim_theta"), robot.acc_lim_theta, 0.0, 10.0, "Max angular acceleration");
  param_helper_.bindFloatParam(param_name("min_turning_radius"), robot.min_turning_radius, 0.0, 100.0, "Min turning radius");
  param_helper_.bindFloatParam(param_name("wheelbase"), robot.wheelbase, 0.0, 10.0, "Wheelbase length");
  param_helper_.bindBoolParam(param_name("cmd_angle_instead_rotvel"), robot.cmd_angle_instead_rotvel, "Command angle instead of rotational velocity");
  param_helper_.bindBoolParam(param_name("is_footprint_dynamic"), robot.is_footprint_dynamic, "Is footprint dynamic");
  param_helper_.bindBoolParam(param_name("use_simulated_fov"), robot.use_simulated_fov, "Use simulated field of view for human-aware planning");

  // Agent parameters
  param_helper_.bindFloatParam(param_name("agent_radius"), agent.radius, 0.0, 10.0, "Agent radius");
  param_helper_.bindFloatParam(param_name("max_agent_vel_x"), agent.max_vel_x, 0.0, 10.0, "Max agent forward velocity");
  param_helper_.bindFloatParam(param_name("max_agent_vel_y"), agent.max_vel_y, 0.0, 10.0, "Max agent strafing velocity");
  param_helper_.bindFloatParam(param_name("max_agent_vel_x_backwards"), agent.max_vel_x_backwards, -10.0, 10.0, "Max agent backward velocity");
  param_helper_.bindFloatParam(param_name("max_agent_vel_theta"), agent.max_vel_theta, 0.0, 10.0, "Max agent angular velocity");
  param_helper_.bindFloatParam(param_name("agent_acc_lim_x"), agent.acc_lim_x, 0.0, 10.0, "Max agent forward acceleration");
  param_helper_.bindFloatParam(param_name("agent_acc_lim_y"), agent.acc_lim_y, 0.0, 10.0, "Max agent strafing acceleration");
  param_helper_.bindFloatParam(param_name("agent_acc_lim_theta"), agent.acc_lim_theta, 0.0, 10.0, "Max agent angular acceleration");
  param_helper_.bindFloatParam(param_name("agent_fov"), agent.fov, 0.0, 6.3, "Agent field of view");
  param_helper_.bindIntParam(param_name("num_moving_avg"), agent.num_moving_avg, 1, 100, "Number of samples for moving average");

  // Goal tolerance parameters
  param_helper_.bindFloatParam(param_name("xy_goal_tolerance"), goal_tolerance.xy_goal_tolerance, 0.0, 10.0, "XY goal tolerance");
  param_helper_.bindFloatParam(param_name("yaw_goal_tolerance"), goal_tolerance.yaw_goal_tolerance, 0.0, 6.3, "Yaw goal tolerance");
  param_helper_.bindBoolParam(param_name("free_goal_vel"), goal_tolerance.free_goal_vel, "Allow non-zero goal velocity");
  param_helper_.bindBoolParam(param_name("complete_global_plan"), goal_tolerance.complete_global_plan, "Complete global plan");

  // Obstacle parameters
  param_helper_.bindFloatParam(param_name("min_obstacle_dist"), obstacles.min_obstacle_dist, 0.0, 10.0, "Minimum obstacle distance");
  param_helper_.bindFloatParam(param_name("inflation_dist"), obstacles.inflation_dist, 0.0, 10.0, "Inflation distance");
  param_helper_.bindBoolParam(param_name("use_nonlinear_obstacle_penalty"), obstacles.use_nonlinear_obstacle_penalty, "Use nonlinear obstacle penalty");
  param_helper_.bindFloatParam(param_name("obstacle_cost_mult"), obstacles.obstacle_cost_mult, 0.0, 100.0, "Obstacle cost multiplier");
  param_helper_.bindFloatParam(param_name("dynamic_obstacle_inflation_dist"), obstacles.dynamic_obstacle_inflation_dist, 0.0, 10.0, "Dynamic obstacle inflation distance");
  param_helper_.bindBoolParam(param_name("include_dynamic_obstacles"), obstacles.include_dynamic_obstacles, "Include dynamic obstacles");
  param_helper_.bindBoolParam(param_name("include_costmap_obstacles"), obstacles.include_costmap_obstacles, "Include costmap obstacles");
  param_helper_.bindFloatParam(param_name("costmap_obstacles_behind_robot_dist"), obstacles.costmap_obstacles_behind_robot_dist, 0.0, 10.0, "Costmap obstacles behind robot distance");
  param_helper_.bindIntParam(param_name("obstacle_poses_affected"), obstacles.obstacle_poses_affected, 0, 100, "Obstacle poses affected");
  param_helper_.bindBoolParam(param_name("legacy_obstacle_association"), obstacles.legacy_obstacle_association, "Use legacy obstacle association");
  param_helper_.bindFloatParam(param_name("obstacle_association_force_inclusion_factor"), obstacles.obstacle_association_force_inclusion_factor, 0.0, 100.0,
                               "Obstacle association force inclusion factor");
  param_helper_.bindFloatParam(param_name("obstacle_association_cutoff_factor"), obstacles.obstacle_association_cutoff_factor, 0.0, 100.0, "Obstacle association cutoff factor");
  param_helper_.bindStringParam(param_name("costmap_converter_plugin"), obstacles.costmap_converter_plugin, "Costmap converter plugin");
  param_helper_.bindBoolParam(param_name("costmap_converter_spin_thread"), obstacles.costmap_converter_spin_thread, "Costmap converter spin thread");
  param_helper_.bindIntParam(param_name("costmap_converter_rate"), obstacles.costmap_converter_rate, 1, 100, "Costmap converter rate");

  // Optimization parameters
  param_helper_.bindIntParam(param_name("no_inner_iterations"), optim.no_inner_iterations, 1, 100, "Number of inner iterations");
  param_helper_.bindIntParam(param_name("no_outer_iterations"), optim.no_outer_iterations, 1, 100, "Number of outer iterations");
  param_helper_.bindBoolParam(param_name("optimization_activate"), optim.optimization_activate, "Activate optimization");
  param_helper_.bindBoolParam(param_name("optimization_verbose"), optim.optimization_verbose, "Verbose optimization output");
  param_helper_.bindFloatParam(param_name("penalty_epsilon"), optim.penalty_epsilon, 0.0, 1.0, "Penalty epsilon");
  param_helper_.bindFloatParam(param_name("time_penalty_epsilon"), optim.time_penalty_epsilon, 0.0, 1.0, "Time penalty epsilon");
  param_helper_.bindBoolParam(param_name("cap_optimaltime_penalty"), optim.cap_optimaltime_penalty, "Cap optimal time penalty");
  param_helper_.bindFloatParam(param_name("weight_max_vel_x"), optim.weight_max_vel_x, 0.0, 1000.0, "Weight max vel x");
  param_helper_.bindFloatParam(param_name("weight_max_vel_y"), optim.weight_max_vel_y, 0.0, 1000.0, "Weight max vel y");
  param_helper_.bindFloatParam(param_name("weight_max_vel_theta"), optim.weight_max_vel_theta, 0.0, 1000.0, "Weight max vel theta");
  param_helper_.bindFloatParam(param_name("weight_acc_lim_x"), optim.weight_acc_lim_x, 0.0, 1000.0, "Weight acc lim x");
  param_helper_.bindFloatParam(param_name("weight_acc_lim_y"), optim.weight_acc_lim_y, 0.0, 1000.0, "Weight acc lim y");
  param_helper_.bindFloatParam(param_name("weight_acc_lim_theta"), optim.weight_acc_lim_theta, 0.0, 1000.0, "Weight acc lim theta");
  param_helper_.bindFloatParam(param_name("weight_kinematics_nh"), optim.weight_kinematics_nh, 0.0, 10000.0, "Weight kinematics non-holonomic");
  param_helper_.bindFloatParam(param_name("weight_kinematics_forward_drive"), optim.weight_kinematics_forward_drive, 0.0, 1000.0, "Weight kinematics forward drive");
  param_helper_.bindFloatParam(param_name("weight_kinematics_turning_radius"), optim.weight_kinematics_turning_radius, 0.0, 1000.0, "Weight kinematics turning radius");
  param_helper_.bindFloatParam(param_name("weight_optimaltime"), optim.weight_optimaltime, 0.0, 1000.0, "Weight optimal time");
  param_helper_.bindFloatParam(param_name("weight_shortest_path"), optim.weight_shortest_path, 0.0, 1000.0, "Weight shortest path");
  param_helper_.bindFloatParam(param_name("weight_obstacle"), optim.weight_obstacle, 0.0, 1000.0, "Weight obstacle");
  param_helper_.bindFloatParam(param_name("weight_inflation"), optim.weight_inflation, 0.0, 100.0, "Weight inflation");
  param_helper_.bindFloatParam(param_name("weight_dynamic_obstacle"), optim.weight_dynamic_obstacle, 0.0, 1000.0, "Weight dynamic obstacle");
  param_helper_.bindFloatParam(param_name("weight_dynamic_obstacle_inflation"), optim.weight_dynamic_obstacle_inflation, 0.0, 100.0, "Weight dynamic obstacle inflation");
  param_helper_.bindFloatParam(param_name("weight_viapoint"), optim.weight_viapoint, 0.0, 1000.0, "Weight viapoint");
  param_helper_.bindFloatParam(param_name("weight_prefer_rotdir"), optim.weight_prefer_rotdir, 0.0, 1000.0, "Weight prefer rotation direction");
  param_helper_.bindFloatParam(param_name("weight_adapt_factor"), optim.weight_adapt_factor, 1.0, 10.0, "Weight adaptation factor");
  param_helper_.bindFloatParam(param_name("obstacle_cost_exponent"), optim.obstacle_cost_exponent, 0.1, 10.0, "Obstacle cost exponent");
  param_helper_.bindFloatParam(param_name("weight_max_agent_vel_x"), optim.weight_max_agent_vel_x, 0.0, 1000.0, "Weight max agent vel x");
  param_helper_.bindFloatParam(param_name("weight_max_agent_vel_y"), optim.weight_max_agent_vel_y, 0.0, 1000.0, "Weight max agent vel y");
  param_helper_.bindFloatParam(param_name("weight_nominal_agent_vel_x"), optim.weight_nominal_agent_vel_x, 0.0, 1000.0, "Weight nominal agent vel x");
  param_helper_.bindFloatParam(param_name("weight_max_agent_vel_theta"), optim.weight_max_agent_vel_theta, 0.0, 1000.0, "Weight max agent vel theta");
  param_helper_.bindFloatParam(param_name("weight_agent_acc_lim_x"), optim.weight_agent_acc_lim_x, 0.0, 1000.0, "Weight agent acc lim x");
  param_helper_.bindFloatParam(param_name("weight_agent_acc_lim_y"), optim.weight_agent_acc_lim_y, 0.0, 1000.0, "Weight agent acc lim y");
  param_helper_.bindFloatParam(param_name("weight_agent_acc_lim_theta"), optim.weight_agent_acc_lim_theta, 0.0, 1000.0, "Weight agent acc lim theta");
  param_helper_.bindFloatParam(param_name("weight_agent_optimaltime"), optim.weight_agent_optimaltime, 0.0, 1000.0, "Weight agent optimal time");
  param_helper_.bindFloatParam(param_name("weight_agent_viapoint"), optim.weight_agent_viapoint, 0.0, 1000.0, "Weight agent viapoint");
  param_helper_.bindFloatParam(param_name("weight_invisible_human"), optim.weight_invisible_human, 0.0, 1000.0, "Weight invisible human");
  param_helper_.bindFloatParam(param_name("weight_agent_robot_safety"), optim.weight_agent_robot_safety, 0.0, 1000.0, "Weight agent robot safety");
  param_helper_.bindFloatParam(param_name("weight_agent_agent_safety"), optim.weight_agent_agent_safety, 0.0, 1000.0, "Weight agent agent safety");
  param_helper_.bindFloatParam(param_name("weight_agent_robot_rel_vel"), optim.weight_agent_robot_rel_vel, 0.0, 1000.0, "Weight agent robot relative velocity");
  param_helper_.bindFloatParam(param_name("weight_agent_robot_visibility"), optim.weight_agent_robot_visibility, 0.0, 1000.0, "Weight agent robot visibility");
  param_helper_.bindBoolParam(param_name("disable_warm_start"), optim.disable_warm_start, "Disable warm start");
  param_helper_.bindBoolParam(param_name("disable_rapid_omega_chage"), optim.disable_rapid_omega_chage, "Disable rapid omega change");
  param_helper_.bindFloatParam(param_name("omega_chage_time_seperation"), optim.omega_chage_time_seperation, 0.0, 10.0, "Omega change time separation");

  // HATEB parameters
  param_helper_.bindBoolParam(param_name("use_agent_robot_safety_c"), hateb.use_agent_robot_safety_c, "Use agent robot safety constraint");
  param_helper_.bindBoolParam(param_name("use_agent_agent_safety_c"), hateb.use_agent_agent_safety_c, "Use agent agent safety constraint");
  param_helper_.bindBoolParam(param_name("use_agent_robot_rel_vel_c"), hateb.use_agent_robot_rel_vel_c, "Use agent robot relative velocity constraint");
  param_helper_.bindBoolParam(param_name("add_invisible_humans"), hateb.add_invisible_humans, "Add invisible humans");
  param_helper_.bindBoolParam(param_name("use_agent_robot_visi_c"), hateb.use_agent_robot_visi_c, "Use agent robot visibility constraint");
  param_helper_.bindBoolParam(param_name("use_agent_elastic_vel"), hateb.use_agent_elastic_vel, "Use agent elastic velocity");
  param_helper_.bindFloatParam(param_name("pose_prediction_reset_time"), hateb.pose_prediction_reset_time, 0.0, 100.0, "Pose prediction reset time");
  param_helper_.bindFloatParam(param_name("min_agent_robot_dist"), hateb.min_agent_robot_dist, 0.0, 10.0, "Minimum agent robot distance");
  param_helper_.bindFloatParam(param_name("min_agent_agent_dist"), hateb.min_agent_agent_dist, 0.0, 10.0, "Minimum agent agent distance");
  param_helper_.bindFloatParam(param_name("rel_vel_cost_threshold"), hateb.rel_vel_cost_threshold, 0.0, 100.0, "Relative velocity cost threshold");
  param_helper_.bindFloatParam(param_name("invisible_human_threshold"), hateb.invisible_human_threshold, 0.0, 100.0, "Invisible human threshold");
  param_helper_.bindFloatParam(param_name("visibility_cost_threshold"), hateb.visibility_cost_threshold, 0.0, 100.0, "Visibility cost threshold");

  // Recovery parameters
  param_helper_.bindBoolParam(param_name("shrink_horizon_backup"), recovery.shrink_horizon_backup, "Shrink horizon backup");
  param_helper_.bindFloatParam(param_name("shrink_horizon_min_duration"), recovery.shrink_horizon_min_duration, 0.0, 100.0, "Shrink horizon min duration");
  param_helper_.bindBoolParam(param_name("oscillation_recovery"), recovery.oscillation_recovery, "Oscillation recovery");
  param_helper_.bindFloatParam(param_name("oscillation_v_eps"), recovery.oscillation_v_eps, 0.0, 10.0, "Oscillation velocity epsilon");
  param_helper_.bindFloatParam(param_name("oscillation_omega_eps"), recovery.oscillation_omega_eps, 0.0, 10.0, "Oscillation omega epsilon");
  param_helper_.bindFloatParam(param_name("oscillation_recovery_min_duration"), recovery.oscillation_recovery_min_duration, 0.0, 100.0, "Oscillation recovery min duration");
  param_helper_.bindFloatParam(param_name("oscillation_filter_duration"), recovery.oscillation_filter_duration, 0.0, 100.0, "Oscillation filter duration");

  // Backoff Recovery parameters
  param_helper_.bindStringParam(param_name("publish_goal_topic"), backoff.publish_goal_topic, "Backoff publish goal topic");
  param_helper_.bindStringParam(param_name("get_plan_srv_name"), backoff.get_plan_srv_name, "Backoff get plan service name");
  param_helper_.bindBoolParam(param_name("visualize"), backoff.visualize, "Visualize backoff poses");
  param_helper_.bindFloatParam(param_name("timeout"), backoff.timeout, 0.0, 1800.0, "Backoff timeout");

  // Visualization parameters
  param_helper_.bindBoolParam(param_name("publish_robot_global_plan"), visualization.publish_robot_global_plan, "Publish robot global plan");
  param_helper_.bindBoolParam(param_name("publish_robot_local_plan"), visualization.publish_robot_local_plan, "Publish robot local plan");
  param_helper_.bindBoolParam(param_name("publish_robot_local_plan_poses"), visualization.publish_robot_local_plan_poses, "Publish robot local plan poses");
  param_helper_.bindBoolParam(param_name("publish_robot_local_plan_fp_poses"), visualization.publish_robot_local_plan_fp_poses, "Publish robot local plan footprint poses");
  param_helper_.bindBoolParam(param_name("publish_agents_global_plans"), visualization.publish_agents_global_plans, "Publish agents global plans");
  param_helper_.bindBoolParam(param_name("publish_agents_local_plans"), visualization.publish_agents_local_plans, "Publish agents local plans");
  param_helper_.bindBoolParam(param_name("publish_agents_local_plan_poses"), visualization.publish_agents_local_plan_poses, "Publish agents local plan poses");
  param_helper_.bindBoolParam(param_name("publish_agents_local_plan_fp_poses"), visualization.publish_agents_local_plan_fp_poses, "Publish agents local plan footprint poses");
  param_helper_.bindFloatParam(param_name("pose_array_z_scale"), visualization.pose_array_z_scale, 0.0, 100.0, "Pose array z-axis scale");
}

void HATebConfig::checkParameters() const {
  auto node = node_.lock();
  if (!node) {
    return;
  }

  // positive backward velocity?
  if (robot.max_vel_x_backwards <= 0) {
    RCLCPP_WARN(node->get_logger(),
                "HATebLocalPlanner Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalizing backwards driving.");
  }
  // bounds smaller than penalty epsilon
  if (robot.max_vel_x <= optim.penalty_epsilon) {
    RCLCPP_WARN(node->get_logger(), "HATebLocalPlanner Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  }
  if (robot.max_vel_x_backwards <= optim.penalty_epsilon) {
    RCLCPP_WARN(node->get_logger(), "HATebLocalPlanner Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  }
  if (robot.max_vel_theta <= optim.penalty_epsilon) {
    RCLCPP_WARN(node->get_logger(), "HATebLocalPlanner Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  }
  if (robot.acc_lim_x <= optim.penalty_epsilon) {
    RCLCPP_WARN(node->get_logger(), "HATebLocalPlanner Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  }
  if (robot.acc_lim_theta <= optim.penalty_epsilon) {
    RCLCPP_WARN(node->get_logger(), "HATebLocalPlanner Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  }
  // dt_ref and dt_hyst
  if (trajectory.dt_ref <= trajectory.dt_hysteresis) {
    RCLCPP_WARN(node->get_logger(),
                "HATebLocalPlanner Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");
  }
  // min number of samples
  if (trajectory.min_samples < 3) {
    RCLCPP_WARN(node->get_logger(),
                "HATebLocalPlanner Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");
  }
  // costmap obstacle behind robot
  if (obstacles.costmap_obstacles_behind_robot_dist < 0) {
    RCLCPP_WARN(node->get_logger(), "HATebLocalPlanner Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");
  }

  // carlike
  if (robot.cmd_angle_instead_rotvel && robot.wheelbase == 0) {
    RCLCPP_WARN(node->get_logger(), "HATebLocalPlanner Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");
  }
  if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius == 0) {
    RCLCPP_WARN(
        node->get_logger(),
        "HATebLocalPlanner Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");
  }

  // positive weight_adapt_factor
  if (optim.weight_adapt_factor < 1.0) {
    RCLCPP_WARN(node->get_logger(), "HATebLocalPlanner Param Warning: parameter weight_adapt_factor should be >= 1.0");
  }

  if (recovery.oscillation_filter_duration < 0) {
    RCLCPP_WARN(node->get_logger(), "HATebLocalPlanner Param Warning: parameter oscillation_filter_duration must be >= 0");
  }

  // weights
  if (optim.weight_optimaltime <= 0) {
    RCLCPP_WARN(node->get_logger(), "HATebLocalPlanner Param Warning: parameter weight_optimaltime should be > 0 (even if weight_shortest_path is in use)");
  }
}

}  // namespace hateb_local_planner
