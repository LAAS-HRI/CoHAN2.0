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

#ifndef HATEB_LOCAL_PLANNER_ROS_H_
#define HATEB_LOCAL_PLANNER_ROS_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ros2_helpers/utils.hpp>

// nav2 base class and utilities
#include <nav2_core/controller.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_util/odometry_utils.hpp>
#include <nav2_util/robot_utils.hpp>

// timed-elastic-band related classes
#include <hateb_local_planner/optimal_planner.h>
#include <hateb_local_planner/recovery_behaviors.h>

#include <hateb_local_planner/visualization.hpp>

// message types
#include <cohan_msgs/srv/optimize.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// agent data
#include <agent_path_prediction/agent_path_prediction.hpp>
#include <agent_path_prediction/msg/predicted_goal.hpp>
#include <agent_path_prediction/srv/agent_pose_predict.hpp>
#include <cohan_msgs/msg/state_array.hpp>
#include <hateb_local_planner/agents_class.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// transforms
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// costmap converter
#include <costmap_converter/costmap_converter_interface.h>

#include <pluginlib/class_loader.hpp>

// Backoff recovery
#include <hateb_local_planner/backoff.h>

// Behavior Tree and Mode Switch
#include <hateb_local_planner/mode_switch.h>

// Standard library
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#define OPTIMIZE_SRV_NAME "optimize"
#define HATEB_LOG "hateb_log"
#define THROTTLE_RATE 5.0  // seconds

namespace hateb_local_planner {
// enum class AgentState : std::uint8_t { NO_STATE, STATIC, MOVING, STOPPED, BLOCKED };

/**
 * @class HATebLocalPlannerROS
 * @brief Implements the nav2_core::Controller interface for Nav2,
 * providing human-aware trajectory planning with the Timed Elastic Band approach.
 */
class HATebLocalPlannerROS : public nav2_core::Controller {
 public:
  /**
   * @brief Default constructor of the hateb plugin
   */
  HATebLocalPlannerROS() = default;

  /**
   * @brief  Destructor of the plugin
   */
  ~HATebLocalPlannerROS() override = default;

  /**
   * @brief Configure the controller
   * @param parent WeakPtr to the lifecycle node
   * @param name Name of the plugin
   * @param tf Shared pointer to a TF buffer
   * @param costmap_ros Shared pointer to the costmap
   */
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup the controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate the controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate the controller state machine
   */
  void deactivate() override;

  /**
   * @brief Set the plan that the hateb local planner is following
   * @param path The plan to pass to the local planner
   */
  void setPlan(const nav_msgs::msg::Path& path) override;

  /**
   * @brief Nav2 controller computeVelocityCommands - computes the velocity command
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the goal checker for awareness if completed
   * @return TwistStamped with velocity command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity, nav2_core::GoalChecker* goal_checker) override;

  /**
   * @brief Set the speed limit
   * @param speed_limit Speed limit to set
   * @param percentage Whether the speed limit is a percentage
   */
  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

  /**
   * @brief Cancel the current planning operation
   * @remarks Method is called when the controller server receives a cancel request.
   * This allows for graceful stopping behavior. If unimplemented, the controller
   * will immediately stop when receiving a cancel request.
   * @return True if cancel was successfully requested, false otherwise
   */
  bool cancel() { return false; };

  /**
   * @brief Check if the goal pose has been achieved
   * @remarks This is an internal method for HATeb's goal tracking and cleanup logic.
   * Nav2 uses the GoalChecker passed to computeVelocityCommands() for actual goal checking.
   * This method handles internal state management when goal is reached.
   * @return True if goal has been achieved, false otherwise
   */
  bool onGoalReached();

  /** @name Public utility functions/methods */
  //@{

  /**
   * @brief Get the current robot footprint/contour model
   * @param node Shared pointer to the lifecycle node
   * @param config Configuration parameters
   * @return Robot footprint model used for optimization
   */
  FootprintModelPtr getRobotFootprintFromParamServer(const rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::shared_ptr<HATebConfig> config);

  /**
   * @brief Set the footprint from the given parameter value
   * @remarks This method parses footprint parameters for ROS 2
   * @remarks It returns a container of Eigen::Vector2d instead of geometry_msgs::Point
   * @param footprint_param Parameter value containing footprint array
   * @param full_param_name Full name of the parameter (for error reporting)
   * @return container of vertices describing the polygon
   */
  Point2dContainer makeFootprintFromParams(const rclcpp::Parameter& footprint_param, const std::string& full_param_name);

  //@}

 protected:
  /**
   * @brief Update internal obstacle vector based on occupied costmap cells
   * @remarks All occupied cells will be added as point obstacles.
   * @remarks All previous obstacles are cleared.
   * @sa updateObstacleContainerWithCostmapConverter
   * velocity model)
   */
  void updateObstacleContainerWithCostmap();

  /**
   * @brief Update internal obstacle vector based on polygons provided by a
   * costmap_converter plugin
   * @remarks Requires a loaded costmap_converter plugin.
   * @remarks All previous obstacles are cleared.
   * @sa updateObstacleContainerWithCostmap
   */
  void updateObstacleContainerWithCostmapConverter();

  /**
   * @brief Update internal obstacle vector based on custom messages received
   * via subscriber
   * @remarks All previous obstacles are NOT cleared. Call this method after
   * other update methods.
   * @sa updateObstacleContainerWithCostmap,
   * updateObstacleContainerWithCostmapConverter
   */
  void updateObstacleContainerWithCustomObstacles();

  /**
   * @brief Update internal obstacle vector based on invisible human messages received
   * via subscriber
   * @remarks All previous obstacles are NOT cleared. Call this method after
   * other update methods.
   * @sa updateObstacleContainerWithCostmap,
   * updateObstacleContainerWithCostmapConverter
   */
  void updateObstacleContainerWithInvHumans();

  /**
   * @brief Update internal via-point container based on the current reference
   * plan
   * @remarks All previous via-points will be cleared.
   * @param transformed_plan (local) portion of the global plan (which is
   * already transformed to the planning frame)
   * @param min_separation minimum separation between two consecutive via-points
   */
  void updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation);

  /**
   * @brief Update internal via-point container for human based on the current reference plan
   * @remarks All previous via-points will be cleared.
   * @param transformed_plan (local) portion of the global plan (which is
   * already transformed to the planning frame)
   * @param min_separation minimum separation between two consecutive via-points
   */
  void updateAgentViaPointsContainers(const AgentPlanVelMap& transformed_agent_plan_vel_map, double min_separation);

  /**
   * @brief Callback for custom obstacles that are not obtained from the costmap
   * @param obst_msg pointer to the message containing a list of polygon shaped
   * obstacles
   */
  void customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr obst_msg);

  /**
   * @brief Callback for invisible humans
   * @param obst_msg pointer to the message containing a list of circular obstacles
   */
  void InvHumansCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr obst_msg);

  /**
   * @brief Callback for custom via-points
   * @param via_points_msg pointer to the message containing a list of
   * via-points
   */
  void customViaPointsCB(const nav_msgs::msg::Path::SharedPtr via_points_msg);

  /**
   * @brief Prune global plan such that already passed poses are cut off
   *
   * The pose of the robot is transformed into the frame of the global plan by
   * taking the most recent tf transform. If no valid transformation can be
   * found, the method returns \c false. The global plan is pruned until the
   * distance to the robot is at least \c dist_behind_robot. If no pose within
   * the specified treshold \c dist_behind_robot can be found, nothing will be
   * pruned and the method returns \c false.
   * @remarks Do not choose \c dist_behind_robot too small (not smaller the
   * cellsize of the map), otherwise nothing will be pruned.
   * @param tf A reference to a tf buffer
   * @param global_pose The global pose of the robot
   * @param[in,out] global_plan The plan to be transformed
   * @param dist_behind_robot Distance behind the robot that should be kept
   * [meters]
   * @return \c true if the plan is pruned, \c false in case of a transform
   * exception or if no pose cannot be found inside the threshold
   */
  bool pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, nav_msgs::msg::Path& global_plan, double dist_behind_robot = 1);

  /**
   * @brief  Transforms the global plan of the robot from the planner frame to
   * the local frame (modified).
   *
   * The method replaces transformGlobalPlan as defined in
   * base_local_planner/goal_functions.h such that the index of the current goal
   * pose is returned as well as the transformation between the global plan and
   * the planning frame.
   * @param tf A reference to a tf buffer
   * @param global_plan The plan to be transformed
   * @param global_pose The global pose of the robot
   * @param costmap A reference to the costmap being used so the window size for
   * transforming can be computed
   * @param global_frame The frame to transform the plan to
   * @param max_plan_length Specify maximum length (cumulative Euclidean
   * distances) of the transformed plan [if <=0: disabled; the length is also
   * bounded by the local costmap size!]
   * @param[out] transformed_plan Populated with the transformed plan
   * @param[out] current_goal_idx Index of the current (local) goal pose in the
   * global plan
   * @param[out] tf_plan_to_global Transformation between the global plan and
   * the global planning frame
   * @return \c true if the global plan is transformed, \c false otherwise
   */
  bool transformGlobalPlan(const nav_msgs::msg::Path& global_plan, const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame,
                           double max_plan_length, PlanCombined& transformed_plan_combined, int* current_goal_idx = nullptr, geometry_msgs::msg::TransformStamped* tf_plan_to_global = nullptr) const;

  /**
   * @brief  Transforms the agent plan from the tracker frame to the local
   * frame.
   *
   * @param tf A reference to a transform listener
   * @param agent_plan The plan to be transformed
   * @param global_pose The global pose of the robot
   * @param costmap A reference to the costmap being used so the window size
   * for transforming can be computed
   * @param global_frame The frame to transform the plan to
   * @param[out] transformed_agent_plan Populated with the transformed plan
   * @param[out] tf_agent_plan_to_global Transformation between the agent plan
   * and the local planning frame
   * @return \c true if the global plan is transformed, \c false otherwise
   */
  bool transformAgentPlan(const geometry_msgs::msg::PoseStamped& robot_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame,
                          const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& agent_plan, AgentPlanCombined& transformed_agent_plan_combined,
                          geometry_msgs::msg::TwistStamped& transformed_agent_twist, tf2::Stamped<tf2::Transform>* tf_agent_plan_to_global = nullptr) const;

  /**
   * @brief Estimate the orientation of a pose from the global_plan that is
   * treated as a local goal for the local planner.
   *
   * If the current (local) goal point is not the final one (global)
   * substitute the goal orientation by the angle of the direction vector
   * between the local goal and the subsequent pose of the global plan. This is
   * often helpful, if the global planner does not consider orientations. \n A
   * moving average filter is utilized to smooth the orientation.
   * @param global_plan The global plan
   * @param local_goal Current local goal
   * @param current_goal_idx Index of the current (local) goal pose in the
   * global plan
   * @param[out] tf_plan_to_global Transformation between the global plan and
   * the global planning frame
   * @param moving_average_length number of future poses of the global plan to
   * be taken into account
   * @return orientation (yaw-angle) estimate
   */
  double estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal, int current_goal_idx,
                                      const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length = 3);

  /**
   * @brief Saturate the translational and angular velocity to given limits.
   *
   * The limit of the translational velocity for backwards driving can be
   * changed independently. Do not choose max_vel_x_backwards <= 0. If no
   * backward driving is desired, change the optimization weight for penalizing
   * backwards driving instead.
   * @param[in,out] vx The translational velocity that should be saturated.
   * @param[in,out] vy Strafing velocity which can be nonzero for holonomic
   * robots
   * @param[in,out] omega The angular velocity that should be saturated.
   * @param max_vel_x Maximum translational velocity for forward driving
   * @param max_vel_y Maximum strafing velocity (for holonomic robots)
   * @param max_vel_theta Maximum (absolute) angular velocity
   * @param max_vel_x_backwards Maximum translational velocity for backwards
   * driving
   */
  void saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards);

  /**
   * @brief Convert translational and rotational velocities to a steering angle
   * of a carlike robot
   *
   * The conversion is based on the following equations:
   * - The turning radius is defined by \f$ R = v/omega \f$
   * - For a car like robot withe a distance L between both axles, the relation
   * is: \f$ tan(\phi) = L/R \f$
   * - phi denotes the steering angle.
   * @remarks You might provide distances instead of velocities, since the
   * temporal information is not required.
   * @param v translational velocity [m/s]
   * @param omega rotational velocity [rad/s]
   * @param wheelbase distance between both axles (drive shaft and steering
   * axle), the value might be negative for back_wheeled robots
   * @param min_turning_radius Specify a lower bound on the turning radius
   * @return Resulting steering angle in [rad] inbetween [-pi/2, pi/2]
   */
  double convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius = 0);

  /**
   * @brief Validate current parameter values of the footprint for optimization,
   * obstacle distance and the costmap footprint
   *
   * This method prints warnings if validation fails.
   * @remarks Currently, we only validate the inscribed radius of the footprints
   * @param opt_inscribed_radius Inscribed radius of the RobotFootprintModel for
   * optimization
   * @param costmap_inscribed_radius Inscribed radius of the footprint model
   * used for the costmap
   * @param min_obst_dist desired distance to obstacles
   */
  void configureBackupModes(std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int& goal_idx);

  void validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist);

  // Agent Prediction reset
  /**
   * @brief Reset the prediction state for all tracked agents
   *
   * This method resets the prediction states for all agents being tracked by the
   * planner. It calls the prediction reset service to clear any existing prediction
   * data and prepare for new predictions.
   */
  void resetAgentsPrediction();

  /**
   * @brief Process the behavior tree and update agent plans
   *
   * This method ticks the behavior tree to evaluate the current planning mode and
   * updates the transformed plans for all agents in the environment.
   * @param robot_pose Current pose of the robot
   * @param transformed_agent_plans Vector containing the transformed plans for all agents
   * @param transformed_agent_plan_vel_map Map containing agent plans with their velocities
   * @return True if plans were successfully updated, false otherwise
   */
  bool tickTreeAndUpdatePlans(const geometry_msgs::msg::PoseStamped& robot_pose, std::vector<AgentPlanCombined>& transformed_agent_plans, AgentPlanVelMap& transformed_agent_plan_vel_map);

  /**
   * @brief Perform standalone trajectory optimization
   *
   * This method provides a service interface for performing trajectory optimization
   * independently of the main planning loop. Useful for analysis and debugging.
   * @param req Service request containing optimization parameters
   * @param res Service response containing optimization results
   * @return True if optimization was successful, false otherwise
   */
  bool optimizeStandalone(const std::shared_ptr<cohan_msgs::srv::Optimize::Request> req, std::shared_ptr<cohan_msgs::srv::Optimize::Response> res);

 private:
  // Definition of member variables

  // ROS 2 node
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;  //!< Weak pointer to the lifecycle node
  std::string plugin_name_;                        //!< Name of this plugin instance

  // external objects (store shared pointers for ROS 2)
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  //!< Pointer to the costmap ros wrapper
  nav2_costmap_2d::Costmap2D* costmap_;                         //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
  std::shared_ptr<tf2_ros::Buffer> tf_;                         //!< pointer to tf buffer
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;     //!< TF listener

  // internal objects (memory management owned)
  PlannerInterfacePtr planner_;                                  //!< Instance of the underlying optimal planner class
  ObstContainer obstacles_;                                      //!< Obstacle vector that should be considered during local trajectory optimization
  ViaPointContainer via_points_;                                 //!< Container of via-points that should be considered during local trajectory optimization
  std::map<uint64_t, ViaPointContainer> agents_via_points_map_;  //!< Map storing via points for each agent
  TebVisualizationPtr visualization_;                            //!< Instance of the visualization class (local/global plan, obstacles, ...)
  std::shared_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>> collision_checker_;  //!< Collision checker for footprint
  std::shared_ptr<HATebConfig> cfg_;                                                                            //!< Config class that stores and manages all related parameters
  FailureDetector failure_detector_;                                                                            //!< Detect if the robot got stucked

  nav_msgs::msg::Path global_plan_;  //!< Store the current global plan
  std::shared_ptr<nav2_util::OdomSmoother> odom_helper_;

  std::unique_ptr<pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons>> costmap_converter_loader_;  //!< Load costmap converter plugins at runtime
  std::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;                                 //!< Store the current costmap_converter

  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr custom_obst_sub_;  //!< Subscriber for custom obstacles received via ObstacleMsg
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr inv_humans_sub_;   //!< Subscriber for invisible humans data
  std::mutex custom_obst_mutex_;                                                                    //!< Mutex that locks the obstacle array (multi-threaded)
  std::mutex inv_human_mutex_;                                                                      //!< Mutex that locks the invisible humans array
  costmap_converter_msgs::msg::ObstacleArrayMsg custom_obstacle_msg_;                               //!< Copy of the most recent obstacle message
  costmap_converter_msgs::msg::ObstacleArrayMsg inv_humans_msg_;                                    //!< Copy of the most recent invisible humans message

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr via_points_sub_;  //!< Subscriber for custom via-points received via Path msg
  bool custom_via_points_active_;                                        //!< Keep track whether valid via-points have been received
  std::mutex via_point_mutex_;                                           //!< Mutex that locks the via_points container (multi-threaded)

  PoseSE2 robot_pose_;                      //!< Store current robot pose
  PoseSE2 robot_goal_;                      //!< Store current robot goal
  geometry_msgs::msg::Twist robot_vel_;     //!< Store current robot velocities (vx, vy, omega)
  bool goal_reached_;                       //!< Store whether the goal is reached or not
  bool horizon_reduced_;                    //!< Flag indicating if the planning horizon was temporarily reduced
  rclcpp::Time time_last_infeasible_plan_;  //!< Time stamp of last infeasible plan detection
  int no_infeasible_plans_;                 //!< Number of consecutive infeasible plans
  rclcpp::Time time_last_oscillation_;      //!< Time stamp of last oscillation detection
  RotType last_preferred_rotdir_;           //!< Store recent preferred turning direction
  geometry_msgs::msg::Twist last_cmd_;      //!< Store the last control command generated

  std::vector<geometry_msgs::msg::Point> footprint_spec_;  //!< Store the footprint of the robot
  double robot_inscribed_radius_;                          //!< The radius of the inscribed circle of the robot
  double robot_circumscribed_radius_;                      //!< The radius of the circumscribed circle of the robot

  std::string global_frame_;      //!< The frame in which the controller will run
  std::string robot_base_frame_;  //!< Used as the base frame id of the robot

  bool initialized_;  //!< Flag to track proper initialization of this class

  // Agent prediction services and related variables
  rclcpp::Client<agent_path_prediction::srv::AgentPosePredict>::SharedPtr predict_agents_client_;  //!< Client for predicting agent trajectories
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_agents_prediction_client_;               //!< Client for resetting agent predictions
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr publish_predicted_markers_client_;             //!< Client for publishing prediction markers

  std::string predict_srv_name_;           //!< Name of the prediction service
  std::string reset_prediction_srv_name_;  //!< Name of the reset prediction service
  std::string publish_makers_srv_name_;    //!< Name of the marker publishing service

  rclcpp::Service<cohan_msgs::srv::Optimize>::SharedPtr optimize_server_;  //!< optimize service for calling standalone
  rclcpp::Time last_call_time_;                                            //!< Time of last service call
  rclcpp::Time last_omega_sign_change_;                                    //!< Time of last angular velocity sign change
  double last_omega_;                                                      //!< Last angular velocity value

  // Planning control flags
  bool goal_ctrl_;     //!< Flag for goal control
  bool reset_states_;  //!< Flag for state reset

  int isMode_;  //!< Current planning mode

  std::string logs_;                                                         //!< System log messages
  rclcpp::Subscription<cohan_msgs::msg::StateArray>::SharedPtr agents_sub_;  //!< Subscriber for tracked agents
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;              //!< Publisher for system logs
  std::string invisible_humans_sub_topic_;                                   //!< Name for invisible humans topic

  // Helper class instances
  std::shared_ptr<hateb_local_planner::Agents> agents_ptr_;  //!< Pointer to agents management class
  std::shared_ptr<Backoff> backoff_ptr_;                     //!< Pointer to backoff behavior class
  ModeSwitch bt_mode_switch_;                                //!< Behavior tree mode switch handler

  // ROS 2 logging and time
  rclcpp::Logger logger_{rclcpp::get_logger("HATebLocalPlanner")};  //!< Logger for this plugin
  rclcpp::Clock::SharedPtr clock_;                                  //!< ROS 2 clock
  rclcpp::Node::SharedPtr intra_node_costmap_converter_;            //!< Shared pointer to node for costmap converter
  rclcpp::Node::SharedPtr intra_node_btree_;                        //!< Shared pointer to node for behavior tree

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};  // end namespace hateb_local_planner

#endif  // HATEB_LOCAL_PLANNER_ROS_H_
