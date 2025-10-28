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

#ifndef TEB_CONFIG_H_
#define TEB_CONFIG_H_

#include <hateb_local_planner/footprint_model.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <ros2_helpers/parameters.hpp>

// Definitions
#define USE_ANALYTIC_JACOBI  // if available for a specific edge, use analytic jacobi

namespace hateb_local_planner {

/**
 * @class HATebConfig
 * @brief Config class for the hateb_local_planner and its components.
 */
class HATebConfig {
 public:
  std::string ns_;         //!< Namespace of the planner, used for topic, service and parameter names
  std::string odom_topic;  //!< Topic name of the odometry message, provided by the robot driver or simulator
  std::string map_frame;   //!< Global planning frame

  FootprintModelPtr robot_model;     //!< model of the robot's footprint
  CircularFootprintPtr human_model;  //!< model of the agent's footprint

  int planning_mode;  //!< Planning mode (autonomous vs human-aware)

  //! Trajectory related parameters
  struct Trajectory {
    bool teb_autosize;     //!< Enable automatic resizing of the trajectory w.r.t to the temporal resolution (recommended)
    double dt_ref;         //!< Desired temporal resolution of the trajectory (should be in the magniture of the underlying control rate)
    double dt_hysteresis;  //!< Hysteresis for automatic resizing depending on the current temporal resolution (dt): usually 10% of dt_ref
    int min_samples;       //!< Minimum number of samples (should be always greater than 2)
    int max_samples;  //!< Maximum number of samples; Warning: if too small the discretization/resolution might not be sufficient for the given robot model or obstacle avoidance does not work anymore.
    int agent_min_samples;                   //!< Minimum number of samples for agent's teb (should be always greater than 2)
    bool global_plan_overwrite_orientation;  //!< Overwrite orientation of local subgoals provided by the global planner
    bool allow_init_with_backwards_motion;  //!< If true, the underlying trajectories might be initialized with backwards motions in case the goal is behind the start within the local costmap (this is
                                            //!< only recommended if the robot is equipped with rear sensors)
    double global_plan_viapoint_sep;        //!< Min. separation between each two consecutive via-points extracted from the global plan (if negative: disabled)
    bool via_points_ordered;                //!< If true, the planner adheres to the order of via-points in the storage container
    double max_global_plan_lookahead_dist;  //!< Specify maximum length (cumulative Euclidean distances) of the subset of the global plan taken into account for optimization [if <=0: disabled; the
                                            //!< length is also bounded by the local costmap size!]
    double global_plan_prune_distance;      //!< Distance between robot and via_points of global plan which is used for pruning
    bool exact_arc_length;  //!< If true, the planner uses the exact arc length in velocity, acceleration and turning rate computations [-> increased cpu time], otherwise the euclidean approximation
                            //!< is used.
    double force_reinit_new_goal_dist;     //!< Reinitialize the trajectory if a previous goal is updated with a seperation of more than the specified value in meters (skip hot-starting)
    double force_reinit_new_goal_angular;  //!< Reinitialize the trajectory if a previous goal is updated with an angular difference of more than the specified value in radians (skip hot-starting)
    int feasibility_check_no_poses;        //!< Specify up to which pose on the predicted plan the feasibility should be checked each sampling interval.
    bool publish_feedback;                 //!< Publish planner feedback containing the full trajectory and a list of active obstacles (should be enabled only for evaluation or debugging purposes)
    double min_resolution_collision_check_angular;  //!< Min angular resolution used during the costmap collision check. If not respected, intermediate samples are added. [rad]
    int control_look_ahead_poses;                   //!< Index of the pose used to extract the velocity command
    double teb_init_skip_dist;                      //!< How much distance needs to be skipped before calculating the plan
    double visualize_with_time_as_z_axis_scale;     //!< Scaling factor for visualizing the trajectory in 3D with time as z-axis
  } trajectory;

  //! Robot related parameters
  struct Robot {
    int type;
    double max_vel_x;            //!< Maximum translational velocity of the robot
    double max_vel_x_backwards;  //!< Maximum translational velocity of the robot for driving backwards
    double max_vel_y;            //!< Maximum strafing velocity of the robot (should be zero for non-holonomic robots!)
    double max_vel_theta;        //!< Maximum angular velocity of the robot
    double acc_lim_x;            //!< Maximum translational acceleration of the robot
    double acc_lim_y;            //!< Maximum strafing acceleration of the robot
    double acc_lim_theta;        //!< Maximum angular acceleration of the robot
    double min_turning_radius;   //!< Minimum turning radius of a carlike robot (diff-drive robot: zero);
    double wheelbase;  //!< The distance between the drive shaft and steering axle (only required for a carlike robot with 'cmd_angle_instead_rotvel' enabled); The value might be negative for
                       //!< back-wheeled robots!
    bool cmd_angle_instead_rotvel;  //!< Substitute the rotational velocity in the commanded velocity message by the corresponding steering angle (check 'axles_distance')
    bool is_footprint_dynamic;      //<! If true, updated the footprint before checking trajectory feasibility
    bool is_real;                   //<! Check if the robot is real (or from gazebo) and uncheck if any other simulator
  } robot;

  //! Agent related parameters
  struct Agent {
    double radius;               //!< Radius of the agent for collision checking
    double max_vel_x;            //!< Maximum translational velocity of the agent
    double max_vel_y;            //!< Maximum strafing velocity of the agent
    double max_vel_x_backwards;  //!< Maximum backwards velocity of the agent
    double min_vel_x_backwards;  //!< Minimum backwards velocity of the agent
    double max_vel_theta;        //!< Maximum angular velocity of the agent
    double min_vel_theta;        //!< Minimum angular velocity of the agent
    double acc_lim_x;            //!< Maximum translational acceleration of the agent
    double acc_lim_y;            //!< Maximum strafing acceleration of the agent
    double acc_lim_theta;        //!< Maximum angular acceleration of the agent
    double fov;                  //!< Field of view angle of the agent in radians
    int num_moving_avg;          //!< Number of samples for moving average calculation
  } agent;

  //! Goal tolerance related parameters
  struct GoalTolerance {
    double yaw_goal_tolerance;  //!< Allowed final orientation error
    double xy_goal_tolerance;   //!< Allowed final euclidean distance to the goal position
    bool free_goal_vel;         //!< Allow the robot's velocity to be nonzero (usally max_vel) for planning purposes
    bool complete_global_plan;  // true prevents the robot from ending the path early when it cross the end goal
  } goal_tolerance;

  //! Obstacle related parameters
  struct Obstacles {
    double min_obstacle_dist;  //!< Minimum desired separation from obstacles
    double inflation_dist;     //!< buffer zone around obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    bool use_nonlinear_obstacle_penalty;
    double obstacle_cost_mult;
    double
        dynamic_obstacle_inflation_dist;  //!< Buffer zone around predicted locations of dynamic obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    bool include_dynamic_obstacles;  //!< Specify whether the movement of dynamic obstacles should be predicted by a constant velocity model (this also effects homotopy class planning); If false, all
                                     //!< obstacles are considered to be static.
    bool include_costmap_obstacles;  //!< Specify whether the obstacles in the costmap should be taken into account directly
    double costmap_obstacles_behind_robot_dist;  //!< Limit the occupied local costmap obstacles taken into account for planning behind the robot (specify distance in meters)
    int obstacle_poses_affected;       //!< The obstacle position is attached to the closest pose on the trajectory to reduce computational effort, but take a number of neighbors into account as well
    bool legacy_obstacle_association;  //!< If true, the old association strategy is used (for each obstacle, find the nearest TEB pose), otherwise the new one (for each teb pose, find only "relevant"
                                       //!< obstacles).
    double obstacle_association_force_inclusion_factor;  //!< The non-legacy obstacle association technique tries to connect only relevant obstacles with the discretized trajectory during
                                                         //!< optimization, all obstacles within a specifed distance are forced to be included (as a multiple of min_obstacle_dist), e.g. choose 2.0 in
                                                         //!< order to consider obstacles within a radius of 2.0*min_obstacle_dist.
    double obstacle_association_cutoff_factor;  //!< See obstacle_association_force_inclusion_factor, but beyond a multiple of [value]*min_obstacle_dist all obstacles are ignored during optimization.
                                                //!< obstacle_association_force_inclusion_factor is processed first.
    std::string costmap_converter_plugin;       //!< Define a plugin name of the costmap_converter package (costmap cells are converted to points/lines/polygons)
    bool costmap_converter_spin_thread;         //!< If \c true, the costmap converter invokes its callback queue in a different thread
    int costmap_converter_rate;  //!< The rate that defines how often the costmap_converter plugin processes the current costmap (the value should not be much higher than the costmap update rate)
  } obstacles;                   //!< Obstacle related parameters

  //! Optimization related parameters
  struct Optimization {
    int no_inner_iterations;  //!< Number of solver iterations called in each outerloop iteration
    int no_outer_iterations;  //!< Each outerloop iteration automatically resizes the trajectory and invokes the internal optimizer with no_inner_iterations

    bool optimization_activate;  //!< Activate the optimization
    bool optimization_verbose;   //!< Print verbose information

    double penalty_epsilon;                    //!< Add a small safety margin to penalty functions for hard-constraint approximations
    double time_penalty_epsilon;               //!< Time safety margin for penalty functions
    bool cap_optimaltime_penalty;              //!< Whether to cap the optimal time penalty
    double weight_max_vel_x;                   //!< Optimization weight for satisfying the maximum allowed translational velocity
    double weight_max_vel_y;                   //!< Optimization weight for satisfying the maximum allowed strafing velocity (in use only for holonomic robots)
    double weight_max_vel_theta;               //!< Optimization weight for satisfying the maximum allowed angular velocity
    double weight_acc_lim_x;                   //!< Optimization weight for satisfying the maximum allowed translational acceleration
    double weight_acc_lim_y;                   //!< Optimization weight for satisfying the maximum allowed strafing acceleration (in use only for holonomic robots)
    double weight_acc_lim_theta;               //!< Optimization weight for satisfying the maximum allowed angular acceleration
    double weight_kinematics_nh;               //!< Optimization weight for satisfying the non-holonomic kinematics
    double weight_kinematics_forward_drive;    //!< Optimization weight for forcing the robot to choose only forward directions (positive transl. velocities, only diffdrive robot)
    double weight_kinematics_turning_radius;   //!< Optimization weight for enforcing a minimum turning radius (carlike robots)
    double weight_optimaltime;                 //!< Optimization weight for contracting the trajectory w.r.t. transition time
    double weight_shortest_path;               //!< Optimization weight for contracting the trajectory w.r.t. path length
    double weight_obstacle;                    //!< Optimization weight for satisfying a minimum separation from obstacles
    double weight_inflation;                   //!< Optimization weight for the inflation penalty (should be small)
    double weight_dynamic_obstacle;            //!< Optimization weight for satisfying a minimum separation from dynamic obstacles
    double weight_dynamic_obstacle_inflation;  //!< Optimization weight for the inflation penalty of dynamic obstacles (should be small)
    double weight_viapoint;                    //!< Optimization weight for minimizing the distance to via-points
    double weight_prefer_rotdir;               //!< Optimization weight for preferring a specific turning direction
    double weight_adapt_factor;  //!< Some special weights (currently 'weight_obstacle') are repeatedly scaled by this factor in each outer TEB iteration (weight_new = weight_old*factor); Increasing
                                 //!< weights iteratively instead of setting a huge value a-priori leads to better numerical conditions of the underlying optimization problem.
    double obstacle_cost_exponent;         //!< Exponent for nonlinear obstacle cost (cost = linear_cost * obstacle_cost_exponent). Set to 1 to disable nonlinear cost (default)
    double weight_max_agent_vel_x;         //!< Optimization weight for satisfying maximum agent translational velocity
    double weight_max_agent_vel_y;         //!< Optimization weight for satisfying maximum agent straffing velocity
    double weight_nominal_agent_vel_x;     //!< Optimization weight for maintaining nominal agent velocity
    double weight_max_agent_vel_theta;     //!< Optimization weight for satisfying maximum agent angular velocity
    double weight_agent_acc_lim_x;         //!< Optimization weight for satisfying agent translational acceleration limits
    double weight_agent_acc_lim_y;         //!< Optimization weight for satisfying agent straffing acceleration limits
    double weight_agent_acc_lim_theta;     //!< Optimization weight for satisfying agent angular acceleration limits
    double weight_agent_optimaltime;       //!< Optimization weight for agent trajectory transition time
    double weight_agent_viapoint;          //!< Optimization weight for agent via-points
    double weight_invisible_human;         //!< Optimization weight for invisible human penalties
    double weight_agent_robot_safety;      //!< Optimization weight for agent-robot safety constraints
    double weight_agent_agent_safety;      //!< Optimization weight for agent-agent safety constraints
    double weight_agent_robot_rel_vel;     //!< Optimization weight for relative velocity constraint between agent and robot
    double weight_agent_robot_visibility;  //!< Optimization weight for agent visibility to robot
    bool disable_warm_start;               //!< If true, disables warm start in optimization
    bool disable_rapid_omega_chage;        //!< If true, prevents rapid changes in angular velocity
    double omega_chage_time_seperation;    //!< Minimum time separation for angular velocity changes
  } optim;                                 //!< Optimization related parameters

  struct Hateb {
    int planning_mode;                  //!< Mode of planning (autonomous vs human-aware)
    bool use_agent_robot_safety_c;      //!< Enable agent-robot safety constraints
    bool use_agent_agent_safety_c;      //!< Enable agent-agent safety constraints
    bool use_agent_robot_rel_vel_c;     //!< Enable relative velocity constraints
    bool add_invisible_humans;          //!< Include invisible humans in planning
    bool use_agent_robot_visi_c;        //!< Enable visibility constraints
    bool use_agent_elastic_vel;         //!< Enable elastic velocity constraints for agents
    double pose_prediction_reset_time;  //!< Time after which pose prediction is reset
    double min_agent_robot_dist;        //!< Minimum distance that should be maintained agent and robot
    double min_agent_agent_dist;        //!< Minimum distance that should be maintained between agents
    double rel_vel_cost_threshold;      //!< Threshold value for Relative Velocity Constraint (lower means higher reaction)
    double invisible_human_threshold;   //!< Threshold value for Invisible Humans Constraint (higher means higher reaction)
    double visibility_cost_threshold;   //!< Threshold value for Visibility Constraint (lower means higher reaction)
  } hateb;

  //! Recovery/backup related parameters
  struct Recovery {
    bool shrink_horizon_backup;          //!< Allows the planner to shrink the horizon temporary (50%) in case of automatically detected issues.
    double shrink_horizon_min_duration;  //!< Specify minimum duration for the reduced horizon in case an infeasible trajectory is detected.
    bool oscillation_recovery;     //!< Try to detect and resolve oscillations between multiple solutions in the same equivalence class (robot frequently switches between left/right/forward/backwards)
    double oscillation_v_eps;      //!< Threshold for the average normalized linear velocity: if oscillation_v_eps and oscillation_omega_eps are not exceeded both, a possible oscillation is detected
    double oscillation_omega_eps;  //!< Threshold for the average normalized angular velocity: if oscillation_v_eps and oscillation_omega_eps are not exceeded both, a possible oscillation is detected
    double oscillation_recovery_min_duration;  //!< Minumum duration [sec] for which the recovery mode is activated after an oscillation is detected.
    double oscillation_filter_duration;        //!< Filter length/duration [sec] for the detection of oscillations
  } recovery;                                  //!< Parameters related to recovery and backup strategies

  //! Visualization
  struct Visualization {
    bool publish_robot_global_plan;           //!< If true, publish the global plan for the robot
    bool publish_robot_local_plan;            //!< If true, publish the local plan for the robot
    bool publish_robot_local_plan_poses;      //!< If true, publish poses from the robot's local plan
    bool publish_robot_local_plan_fp_poses;   //!< If true, publish footprint (time colored and vel scaled) poses from the robot's local plan
    bool publish_agents_global_plans;         //!< If true, publish the global plans for all agents
    bool publish_agents_local_plans;          //!< If true, publish the local plans for all agents
    bool publish_agents_local_plan_poses;     //!< If true, publish poses from agents' local plans
    bool publish_agents_local_plan_fp_poses;  //!< If true, publish footprint (time colored and vel scaled) poses from agents' local plans
    double pose_array_z_scale;                //!< Multiplier to show time on z value of pose array for agents and robot
  } visualization;

  /**
   * @brief Construct the HATebConfig using default values.
   * @warning If the \b rosparam server or/and \b dynamic_reconfigure (rqt_reconfigure) node are used,
   *	     the default variables will be overwritten: \n
   *	     E.g. if \e base_local_planner is utilized as plugin for the navigation stack, the initialize() method will register a
   * 	     dynamic_reconfigure server. A subset (not all but most) of the parameters are considered for dynamic modifications.
   * 	     All parameters considered by the dynamic_reconfigure server (and their \b default values) are
   * 	     set in \e PROJECT_SRC/cfg/HATebLocalPlannerReconfigure.cfg. \n
   * 	     In addition the rosparam server can be queried to get parameters e.g. defiend in a launch file.
   * 	     The plugin source (or a possible binary source) can call loadRosParamFromNodeHandle() to update the parameters.
   * 	     In \e summary, default parameters are loaded in the following order (the right one overrides the left ones): \n
   * 		<b>HATebConfig Constructor defaults << dynamic_reconfigure defaults << rosparam server defaults</b>
   */
  HATebConfig() {
    odom_topic = "odom";
    map_frame = "odom";

    planning_mode = 1;  // Agent-Aware planning by default

    // Trajectory

    trajectory.teb_autosize = true;
    trajectory.dt_ref = 0.3;
    trajectory.dt_hysteresis = 0.1;
    trajectory.min_samples = 3;
    trajectory.agent_min_samples = 3;
    trajectory.max_samples = 500;
    trajectory.global_plan_overwrite_orientation = true;
    trajectory.allow_init_with_backwards_motion = false;
    trajectory.global_plan_viapoint_sep = -1;
    trajectory.via_points_ordered = false;
    trajectory.max_global_plan_lookahead_dist = 1;
    trajectory.global_plan_prune_distance = 1;
    trajectory.exact_arc_length = false;
    trajectory.force_reinit_new_goal_dist = 1;
    trajectory.force_reinit_new_goal_angular = 0.5 * M_PI;
    trajectory.feasibility_check_no_poses = 5;
    trajectory.publish_feedback = false;
    trajectory.min_resolution_collision_check_angular = M_PI;
    trajectory.control_look_ahead_poses = 1;
    trajectory.teb_init_skip_dist = 0.4;
    trajectory.visualize_with_time_as_z_axis_scale = 0.0;

    // Robot

    robot.type = 0;
    robot.max_vel_x = 0.4;
    robot.max_vel_x_backwards = 0.2;
    robot.max_vel_y = 0.0;
    robot.max_vel_theta = 0.3;
    robot.acc_lim_x = 0.5;
    robot.acc_lim_y = 0.5;
    robot.acc_lim_theta = 0.5;
    robot.min_turning_radius = 0;
    robot.wheelbase = 1.0;
    robot.cmd_angle_instead_rotvel = false;
    robot.is_footprint_dynamic = false;
    robot.is_real = false;

    // Agent
    agent.radius = 0.35;
    agent.max_vel_x = 1.3;
    agent.max_vel_y = 0.4;
    agent.max_vel_x_backwards = 0.0;
    agent.max_vel_theta = 1.1;
    agent.acc_lim_x = 0.6;
    agent.acc_lim_theta = 0.8;
    agent.num_moving_avg = 5;

    // GoalTolerance

    goal_tolerance.xy_goal_tolerance = 0.2;
    goal_tolerance.yaw_goal_tolerance = 0.2;
    goal_tolerance.free_goal_vel = false;
    goal_tolerance.complete_global_plan = true;

    // Obstacles

    obstacles.min_obstacle_dist = 0.5;
    obstacles.inflation_dist = 0.6;
    obstacles.use_nonlinear_obstacle_penalty = true;
    obstacles.obstacle_cost_mult = 1.0;
    obstacles.dynamic_obstacle_inflation_dist = 0.6;
    obstacles.include_dynamic_obstacles = true;
    obstacles.include_costmap_obstacles = true;
    obstacles.costmap_obstacles_behind_robot_dist = 1.5;
    obstacles.obstacle_poses_affected = 25;
    obstacles.legacy_obstacle_association = false;
    obstacles.obstacle_association_force_inclusion_factor = 1.5;
    obstacles.obstacle_association_cutoff_factor = 5;
    obstacles.costmap_converter_plugin = "";
    obstacles.costmap_converter_spin_thread = true;
    obstacles.costmap_converter_rate = 5;

    // Optimization

    optim.no_inner_iterations = 8;
    optim.no_outer_iterations = 4;
    optim.optimization_activate = true;
    optim.optimization_verbose = false;
    optim.penalty_epsilon = 0.1;
    optim.time_penalty_epsilon = 0.1;
    optim.cap_optimaltime_penalty = true;
    optim.weight_max_vel_x = 2;  // 1
    optim.weight_max_vel_y = 2;
    optim.weight_max_vel_theta = 1;
    optim.weight_acc_lim_x = 1;
    optim.weight_acc_lim_y = 1;
    optim.weight_acc_lim_theta = 1;
    optim.weight_kinematics_nh = 1000;
    optim.weight_kinematics_forward_drive = 1;
    optim.weight_kinematics_turning_radius = 1;
    optim.weight_optimaltime = 1;
    optim.weight_shortest_path = 0;
    optim.weight_obstacle = 50;
    optim.weight_inflation = 0.1;
    optim.weight_dynamic_obstacle = 50;
    optim.weight_dynamic_obstacle_inflation = 0.1;
    optim.weight_viapoint = 1;
    optim.weight_prefer_rotdir = 50;

    optim.weight_adapt_factor = 2.0;
    optim.obstacle_cost_exponent = 1.0;

    optim.weight_max_agent_vel_x = 2.0;
    optim.weight_nominal_agent_vel_x = 2.0;
    optim.weight_max_agent_vel_theta = 2.0;
    optim.weight_agent_acc_lim_x = 1;
    optim.weight_agent_acc_lim_theta = 1;
    optim.weight_agent_optimaltime = 1;
    optim.weight_agent_viapoint = 1;
    optim.weight_invisible_human = 1;
    optim.weight_agent_robot_safety = 20;
    optim.weight_agent_agent_safety = 20;
    optim.weight_agent_robot_rel_vel = 20;
    optim.weight_agent_robot_visibility = 20;
    optim.disable_warm_start = false;
    optim.disable_rapid_omega_chage = true;
    optim.omega_chage_time_seperation = 1.0;

    // Hateb
    hateb.use_agent_robot_safety_c = true;
    hateb.use_agent_agent_safety_c = true;
    hateb.use_agent_robot_rel_vel_c = true;
    hateb.add_invisible_humans = true;
    hateb.use_agent_robot_visi_c = true;
    hateb.use_agent_elastic_vel = true;
    hateb.pose_prediction_reset_time = 2.0;
    hateb.min_agent_robot_dist = 0.6;
    hateb.min_agent_agent_dist = 0.2;

    // Recovery
    recovery.shrink_horizon_backup = true;
    recovery.shrink_horizon_min_duration = 10;
    recovery.oscillation_recovery = true;
    recovery.oscillation_v_eps = 0.1;
    recovery.oscillation_omega_eps = 0.1;
    recovery.oscillation_recovery_min_duration = 10;
    recovery.oscillation_filter_duration = 10;

    // Visualization
    visualization.publish_robot_global_plan = true;
    visualization.publish_robot_local_plan = true;
    visualization.publish_robot_local_plan_poses = false;
    visualization.publish_robot_local_plan_fp_poses = false;
    visualization.publish_agents_global_plans = false;
    visualization.publish_agents_local_plans = true;
    visualization.publish_agents_local_plan_poses = false;
    visualization.publish_agents_local_plan_fp_poses = false;
    visualization.pose_array_z_scale = 1.0;
  }

  /**
   * @brief Initializes the parameter helper for this config
   * @param node ROS 2 node shared pointer
   * @param plugin_name Name of the plugin for parameter namespacing
   */
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string& plugin_name = "") {
    node_ = node;
    plugin_name_ = plugin_name;
    param_helper_.initialize(node);
    RCLCPP_INFO(node->get_logger(), "Initializing HATebConfig parameters for plugin: %s", plugin_name_.empty() ? "default" : plugin_name_.c_str());
  }

  /**
   * @brief Sets up parameter declarations and callback for parameter updates
   */
  void setupParameterCallback();

  /**
   * @brief Check parameters and print warnings in case of discrepancies
   *
   * Call this method whenever parameters are changed using public interfaces to inform the user
   * about some improper uses.
   */
  void checkParameters() const;

  /**
   * @brief Return the internal config mutex
   */
  std::mutex& configMutex() { return config_mutex_; }

 private:
  /**
   * @brief Binds all configuration variables to parameters for auto-update
   */
  void bindParameters();

  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node_;  //!< ROS 2 node weak pointer (to avoid circular dependencies)
  std::string plugin_name_;                              //!< Plugin name for parameter namespacing
  parameters::ParameterHelper param_helper_;             //!< Parameter helper for managing ROS2 parameters
  std::mutex config_mutex_;                              //!< Mutex for config accesses and changes
};

}  // namespace hateb_local_planner

#endif
