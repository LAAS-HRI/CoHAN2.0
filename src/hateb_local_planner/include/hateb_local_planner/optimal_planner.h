/*********************************************************************
 * Majorly modified by Phani Teja Singamaneni from 2020-2025
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

#ifndef OPTIMAL_PLANNER_H_
#define OPTIMAL_PLANNER_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ros2_helpers/utils.hpp>
#include <utility>

// teb stuff
#include <hateb_local_planner/footprint_model.h>
#include <hateb_local_planner/hateb_config.h>
#include <hateb_local_planner/misc.h>
#include <hateb_local_planner/planner_interface.h>
#include <hateb_local_planner/timed_elastic_band.h>
#include <hateb_local_planner/visualization.h>

// g2o lib stuff
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

// g2o custom edges and vertices for the HATEB planner
#include <hateb_local_planner/g2o_types/edge_acceleration.h>
#include <hateb_local_planner/g2o_types/edge_agent_agent_safety.h>
#include <hateb_local_planner/g2o_types/edge_agent_robot_rel_velocity.h>
#include <hateb_local_planner/g2o_types/edge_agent_robot_safety.h>
#include <hateb_local_planner/g2o_types/edge_agent_robot_visibility.h>
#include <hateb_local_planner/g2o_types/edge_dynamic_obstacle.h>
#include <hateb_local_planner/g2o_types/edge_invisible_human.h>
#include <hateb_local_planner/g2o_types/edge_kinematics.h>
#include <hateb_local_planner/g2o_types/edge_obstacle.h>
#include <hateb_local_planner/g2o_types/edge_prefer_rotdir.h>
#include <hateb_local_planner/g2o_types/edge_shortest_path.h>
#include <hateb_local_planner/g2o_types/edge_static_agent_visibility.h>
#include <hateb_local_planner/g2o_types/edge_time_optimal.h>
#include <hateb_local_planner/g2o_types/edge_velocity.h>
#include <hateb_local_planner/g2o_types/edge_via_point.h>

// messages
#include <climits>
#include <cohan_msgs/msg/agent_type.hpp>
#include <cohan_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

namespace hateb_local_planner {

//! Typedef for the block solver utilized for optimization
using HATEBBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;

//! Typedef for the linear solver utilized for optimization
using HATEBLinearSolver = g2o::LinearSolverCholmod<HATEBBlockSolver::PoseMatrixType>;

//! Typedef for a container storing via-points
using ViaPointContainer = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

/**
 * @class HATebOptimalPlanner
 * @brief This class optimizes an internal Human Aware Timed Elastic Band trajectory using the g2o-framework.
 *
 * For an introduction and further details about the TEB optimization problem refer to:
 * 	- C. Rösmann et al.: Trajectory modification considering dynamic constraints of autonomous robots, ROBOTIK, 2012.
 * 	- C. Rösmann et al.: Efficient trajectory optimization using a sparse model, ECMR, 2013.
 * 	- R. Kümmerle et al.: G2o: A general framework for graph optimization, ICRA, 2011.
 */
class HATebOptimalPlanner : public PlannerInterface {
 public:
  /**
   * @brief Default constructor
   */
  HATebOptimalPlanner();

  /**
   * @brief Construct and initialize the TEB optimal planner.
   * @param node Shared pointer to the lifecycle node for ROS2 operations
   * @param cfg Const reference to the HATebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param robot_model Shared pointer to the robot shape model used for optimization (optional)
   * @param visual Shared pointer to the TebVisualization class (optional)
   * @param via_points Container storing via-points (optional)
   */
  explicit HATebOptimalPlanner(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const HATebConfig& cfg, ObstContainer* obstacles = nullptr,
                               FootprintModelPtr robot_model = std::make_shared<PointFootprint>(), TebVisualizationPtr visual = TebVisualizationPtr(), const ViaPointContainer* via_points = nullptr,
                               CircularFootprintPtr agent_model = std::make_shared<CircularFootprint>(), const std::map<uint64_t, ViaPointContainer>* agents_via_points_map = nullptr);

  /**
   * @brief Destruct the optimal planner.
   */
  ~HATebOptimalPlanner() override;

  /**
   * @brief Initializes the optimal planner
   * @param node Shared pointer to the lifecycle node for ROS2 operations
   * @param cfg Const reference to the HATebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param robot_model Shared pointer to the robot shape model used for optimization (optional)
   * @param visual Shared pointer to the TebVisualization class (optional)
   * @param via_points Container storing via-points (optional)
   */
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const HATebConfig& cfg, ObstContainer* obstacles = nullptr, FootprintModelPtr robot_model = std::make_shared<PointFootprint>(),
                  TebVisualizationPtr visual = TebVisualizationPtr(), const ViaPointContainer* via_points = nullptr, CircularFootprintPtr agent_model = std::make_shared<CircularFootprint>(),
                  const std::map<uint64_t, ViaPointContainer>* agents_via_points_map = nullptr);

  /** @name Plan a trajectory  */
  //@{

  /**
   * @brief Plan a trajectory based on an initial reference plan.
   *
   * Call this method to create and optimize a trajectory that is initialized
   * according to an initial reference plan (given as a container of poses). \n
   * The method supports hot-starting from previous solutions, if avaiable: \n
   * 	- If no trajectory exist yet, a new trajectory is initialized based on the initial plan,
   *	  see TimedElasticBand::initTEBtoGoal
   * 	- If a previous solution is avaiable, update the trajectory based on the initial plan,
   * 	  see bool TimedElasticBand::updateAndPruneTEB
   * 	- Afterwards optimize the recently initialized or updated trajectory by calling optimizeTEB() and invoking g2o
   * @param initial_plan vector of geometry_msgs::msg::PoseStamped
   * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x, linear.y (holonomic) and angular.z are used)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  bool plan(const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, const geometry_msgs::msg::Twist* start_vel = nullptr, bool free_goal_vel = false,
            const AgentPlanVelMap* initial_agent_plan_vel_map = nullptr, hateb_local_planner::OptimizationCostArray* op_costs = nullptr, double dt_ref = 0.4, double dt_hyst = 0.1,
            int Mode = 0) override;

  /**
   * @brief Plan a trajectory between a given start and goal pose
   *
   * Call this method to create and optimize a trajectory that is initialized between a given start and goal pose. \n
   * The method supports hot-starting from previous solutions, if avaiable: \n
   * 	- If no trajectory exist yet, a new trajectory is initialized between start and goal poses
   *	  @see TimedElasticBand::initTEBtoGoal
   * 	- If a previous solution is avaiable, update the trajectory @see bool TimedElasticBand::updateAndPruneTEB
   * 	- Afterwards optimize the recently initialized or updated trajectory by calling optimizeTEB() and invoking g2o
   * @param start PoseSE2 containing the start pose of the trajectory
   * @param goal PoseSE2 containing the goal pose of the trajectory
   * @param start_vel Initial velocity at the start pose (twist message containing the translational and angular velocity).
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  bool plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::msg::Twist* start_vel = nullptr, bool free_goal_vel = false, double pre_plan_time = 0.0,
            hateb_local_planner::OptimizationCostArray* op_costs = nullptr, double dt_ref = 0.4, double dt_hyst = 0.1, int Mode = 0) override;

  /**
   * @brief Get the velocity command from a previously optimized plan to control the robot at the current sampling interval.
   * @warning Call plan() first and check if the generated plan is feasible.
   * @param[out] vx translational velocity [m/s]
   * @param[out] vy strafing velocity which can be nonzero for holonomic robots[m/s]
   * @param[out] omega rotational velocity [rad/s]
   * @return \c true if command is valid, \c false otherwise
   */
  bool getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses, double dt_ref) const override;

  /**
   * @brief Optimize a previously initialized trajectory (actual TEB optimization loop).
   *
   * optimizeTEB implements the main optimization loop. \n
   * It consist of two nested loops:
   * 	- The outer loop resizes the trajectory according to the temporal resolution by invoking TimedElasticBand::autoResize().
   * 	  Afterwards the internal method optimizeGraph() is called that constitutes the innerloop.
   * 	- The inner loop calls the solver (g2o framework, resp. sparse Levenberg-Marquardt) and iterates a specified
   * 	  number of optimization calls (\c iterations_innerloop).
   *
   * The outer loop is repeated \c iterations_outerloop times. \n
   * The ratio of inner and outer loop iterations significantly defines the contraction behavior
   * and convergence rate of the trajectory optimization. Based on our experiences, 2-6 innerloop iterations are sufficient. \n
   * The number of outer loop iterations should be determined by considering the maximum CPU time required to match the control rate. \n
   * Optionally, the cost vector can be calculated by specifying \c compute_cost_afterwards, see computeCurrentCost().
   * @remarks This method is usually called from a plan() method
   * @param iterations_innerloop Number of iterations for the actual solver loop
   * @param iterations_outerloop Specifies how often the trajectory should be resized followed by the inner solver loop.
   * @param compute_cost_afterwards if \c true Calculate the cost vector according to computeCurrentCost(),
   *         the vector can be accessed afterwards using getCurrentCost().
   * @param obst_cost_scale Specify extra scaling for obstacle costs (only used if \c compute_cost_afterwards is true)
   * @param viapoint_cost_scale Specify extra scaling for via-point costs (only used if \c compute_cost_afterwards is true)
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time
   *          (only used if \c compute_cost_afterwards is true).
   * @return \c true if the optimization terminates successfully, \c false
   * otherwise
   */
  bool optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards = true, double obst_cost_scale = 1.0, double viapoint_cost_scale = 1.0,
                   bool alternative_time_cost = false, hateb_local_planner::OptimizationCostArray* op_costs = nullptr, double dt_ref = 0.4, double dt_hyst = 0.1);

  bool optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards = true, double obst_cost_scale = 1.0, double viapoint_cost_scale = 1.0,
                   bool alternative_time_cost = false, hateb_local_planner::OptimizationCostArray* op_costs = nullptr);

  //@}

  /** @name Desired initial and final velocity */
  //@{

  /**
   * @brief Set the initial velocity at the trajectory's start pose (e.g. the robot's velocity) [twist overload].
   * @remarks Calling this function is not neccessary if the initial velocity is passed via the plan() method
   * @param vel_start Current start velocity (e.g. the velocity of the robot, only linear.x and angular.z are used,
   *                  for holonomic robots also linear.y)
   */
  void setVelocityStart(const geometry_msgs::msg::Twist& vel_start);

  /**
   * @brief Set the desired final velocity at the trajectory's goal pose.
   * @remarks Call this function only if a non-zero velocity is desired and if \c free_goal_vel is set to \c false in plan()
   * @param vel_goal twist message containing the translational and angular final velocity
   */
  void setVelocityGoal(const geometry_msgs::msg::Twist& vel_goal);

  /**
   * @brief Set the desired final velocity at the trajectory's goal pose to be the maximum velocity limit
   * @remarks Calling this function is not neccessary if \c free_goal_vel is set to \c false in plan()
   */
  void setVelocityGoalFree() { vel_goal_.first = false; }

  //@}

  /** @name Take obstacles into account */
  //@{

  /**
   * @brief Assign a new set of obstacles
   * @param obst_vector pointer to an obstacle container (can also be a nullptr)
   * @remarks This method overrids the obstacle container optinally assigned in the constructor.
   */
  void setObstVector(ObstContainer* obst_vector) { obstacles_ = obst_vector; }

  /**
   * @brief Access the internal obstacle container.
   * @return Const reference to the obstacle container
   */
  const ObstContainer& getObstVector() const { return *obstacles_; }

  //@}

  /** @name Take via-points into account */
  //@{

  /**
   * @brief Assign a new set of via-points
   * @param via_points pointer to a via_point container (can also be a nullptr)
   * @details Any previously set container will be overwritten.
   */
  void setViaPoints(const ViaPointContainer* via_points) { via_points_ = via_points; }

  /**
   * @brief Access the internal via-point container.
   * @return Const reference to the via-point container
   */
  const ViaPointContainer& getViaPoints() const { return *via_points_; }

  //@}

  /** @name Visualization */
  //@{

  /**
   * @brief Register a TebVisualization class to enable visiualization routines (e.g. publish the local plan and pose sequence)
   * @param visualization shared pointer to a TebVisualization instance
   * @see visualize
   */
  void setVisualization(TebVisualizationPtr visualization);

  /**
   * @brief Publish the local plan and pose sequence via ros topics (e.g. subscribe with rviz).
   *
   * Make sure to register a TebVisualization instance before using setVisualization() or an overlaoded constructor.
   * @see setVisualization
   */
  void visualize() override;

  //@}

  /** @name Utility methods and more */
  //@{

  /**
   * @brief Reset the planner by clearing the internal graph and trajectory.
   */
  void clearPlanner() override {
    clearGraph();
    teb_.clearTimedElasticBand();
    for (auto& agent_teb : agents_tebs_map_) {
      agent_teb.second.clearTimedElasticBand();
    }
  }

  /**
   * @brief Prefer a desired initial turning direction (by penalizing the opposing one)
   *
   * A desired (initial) turning direction might be specified in case the planned trajectory oscillates between two
   * solutions (in the same equivalence class!) with similar cost. Check the parameters in order to adjust the weight of the penalty.
   * Initial means that the penalty is applied only to the first few poses of the trajectory.
   * @param dir This parameter might be RotType::left (prefer left), RotType::right (prefer right) or RotType::none (prefer none)
   */
  void setPreferredTurningDir(RotType dir) override { prefer_rotdir_ = dir; }

  /**
   * @brief Register the vertices and edges defined for the TEB to the g2o::Factory.
   *
   * This allows the user to export the internal graph to a text file for instance.
   * Access the optimizer() for more details.
   */
  static void registerG2OTypes();

  /**
   * @brief Access the internal TimedElasticBand trajectory.
   * @warning In general, the underlying teb must not be modified directly. Use with care...
   * @return reference to the teb
   */
  TimedElasticBand& teb() { return teb_; };

  /**
   * @brief Access the internal TimedElasticBand trajectory (read-only).
   * @return const reference to the teb
   */
  const TimedElasticBand& teb() const { return teb_; };

  /**
   * @brief Access the internal g2o optimizer.
   * @warning In general, the underlying optimizer must not be modified directly. Use with care...
   * @return const shared pointer to the g2o sparse optimizer
   */
  std::shared_ptr<g2o::SparseOptimizer> optimizer() { return optimizer_; };

  /**
   * @brief Access the internal g2o optimizer (read-only).
   * @return const shared pointer to the g2o sparse optimizer
   */
  std::shared_ptr<const g2o::SparseOptimizer> optimizer() const { return optimizer_; };

  /**
   * @brief Check if last optimization was successful
   * @return \c true if the last optimization returned without errors,
   *         otherwise \c false (also if no optimization has been called before).
   */
  bool isOptimized() const { return optimized_; };

  /**
   * @brief Compute the cost vector of a given optimization problen (hyper-graph must exist).
   *
   * Use this method to obtain information about the current edge errors / costs (local cost functions). \n
   * The vector of cost values is composed according to the different edge types (time_optimal, obstacles, ...). \n
   * Refer to the method declaration for the detailed composition. \n
   * The cost for the edges that minimize time differences (EdgeTimeOptimal) corresponds to the sum of all single
   * squared time differneces: \f$ \sum_i \Delta T_i^2 \f$. Sometimes, the user may want to get a value that is proportional
   * or identical to the actual trajectory transition time \f$ \sum_i \Delta T_i \f$. \n
   * Set \c alternative_time_cost to true in order to get the cost calculated using the latter equation, but check the
   * implemented definition, if the value is scaled to match the magnitude of other cost values.
   * @see getCurrentCost
   * @see optimizeTEB
   * @param obst_cost_scale Specify extra scaling for obstacle costs.
   * @param viapoint_cost_scale Specify extra scaling for via points.
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time.
   * @return TebCostVec containing the cost values
   */
  void computeCurrentCost(double obst_cost_scale = 1.0, double viapoint_cost_scale = 1.0, bool alternative_time_cost = false, hateb_local_planner::OptimizationCostArray* op_costs = NULL);

  /**
   * Compute and return the cost of the current optimization graph (supports multiple trajectories)
   * @param[out] cost current cost value for each trajectory
   *                  [for a planner with just a single trajectory: size=1, vector will not be cleared]
   * @param obst_cost_scale Specify extra scaling for obstacle costs
   * @param viapoint_cost_scale Specify extra scaling for via points.
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time
   */
  virtual void computeCurrentCost(std::vector<double>& cost, double obst_cost_scale = 1.0, double viapoint_cost_scale = 1.0, bool alternative_time_cost = false) {
    computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
    cost.push_back(getCurrentCost());
  }

  /**
   * @brief Access the cost vector.
   *
   * The accumulated cost value previously calculated using computeCurrentCost
   * or by calling optimizeTEB with enabled cost flag.
   * @return const reference to the TebCostVec.
   */
  double getCurrentCost() const { return cost_; }

  /**
   * @brief Extract the velocity from consecutive poses and a time difference (including strafing velocity for holonomic robots)
   *
   * The velocity is extracted using finite differences.
   * The direction of the translational velocity is also determined.
   * @param pose1 pose at time k
   * @param pose2 consecutive pose at time k+1
   * @param dt actual time difference between k and k+1 (must be >0 !!!)
   * @param[out] vx translational velocity
   * @param[out] vy strafing velocity which can be nonzero for holonomic robots
   * @param[out] omega rotational velocity
   */
  inline void extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const;

  /**
   * @brief Compute the velocity profile of the trajectory
   *
   * This method computes the translational and rotational velocity for the complete
   * planned trajectory.
   * The first velocity is the one that is provided as initial velocity (fixed).
   * Velocities at index k=2...end-1 are related to the transition from pose_{k-1} to pose_k.
   * The last velocity is the final velocity (fixed).
   * The number of Twist objects is therefore sizePoses()+1;
   * In summary:
   *     v[0] = v_start,
   *     v[1,...end-1] = +-(pose_{k+1}-pose{k})/dt,
   *     v(end) = v_goal
   * It can be used for evaluation and debugging purposes or
   * for open-loop control. For computing the velocity required for controlling the robot
   * to the next step refer to getVelocityCommand().
   * @param[out] velocity_profile velocity profile will be written to this vector (after clearing any existing content) with the size=no_poses+1
   */
  void getVelocityProfile(std::vector<geometry_msgs::msg::Twist>& velocity_profile) const;

  /**
   * @brief Return the complete trajectory including poses, velocity profiles and temporal information
   *
   * It is useful for evaluation and debugging purposes or for open-loop control.
   * Since the velocity obtained using difference quotients is the mean velocity between consecutive poses,
   * the velocity at each pose at time stamp k is obtained by taking the average between both velocities.
   * The velocity of the first pose is v_start (provided initial value) and the last one is v_goal (usually zero, if free_goal_vel is off).
   * See getVelocityProfile() for the list of velocities between consecutive points.
   * @param[out] trajectory the resulting trajectory
   */
  cohan_msgs::msg::Trajectory getFullTrajectory() const override;

  /**
   * @brief Return the complete trajectory for a specific agent including poses, velocity profiles and temporal information
   *
   * Similar to getFullTrajectory() but specifically for retrieving an agent's planned trajectory.
   * The trajectory includes pose sequence, velocity profile, and timing information useful for
   * evaluation and debugging of agent behavior.
   * @param agent_id The unique identifier of the agent whose trajectory should be retrieved
   * @return The complete trajectory message for the specified agent
   */
  cohan_msgs::msg::Trajectory getFullAgentTrajectory(uint64_t agent_id) override;

  /**
   * @brief Check whether the planned trajectory is feasible or not.
   *
   * This method currently checks only that the trajectory, or a part of the trajectory is collision free.
   * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
   * @param costmap_model Pointer to the costmap model
   * @param footprint_spec The specification of the footprint of the robot in world coordinates
   * @param inscribed_radius The radius of the inscribed circle of the robot
   * @param circumscribed_radius The radius of the circumscribed circle of the robot
   * @param look_ahead_idx Number of poses along the trajectory that should be verified, if -1, the complete trajectory will be checked.
   * @return \c true, if the robot footprint along the first part of the trajectory intersects with
   *         any obstacle in the costmap, \c false otherwise.
   */
  bool isTrajectoryFeasible(nav2_costmap_2d::CostmapModel* costmap_model, const std::vector<geometry_msgs::msg::Point>& footprint_spec, double inscribed_radius = 0.0,
                            double circumscribed_radius = 0.0, int look_ahead_idx = -1) override;

  //@}

 protected:
  /** @name Hyper-Graph creation and optimization */
  //@{

  /**
   * @brief Build the hyper-graph representing the TEB optimization problem.
   *
   * This method creates the optimization problem according to the hyper-graph formulation. \n
   * For more details refer to the literature cited in the HATebOptimalPlanner class description.
   * @see optimizeGraph
   * @see clearGraph
   * @param weight_multiplier Specify a weight multipler for selected weights in optimizeGraph
   *                          This might be used for weight adapation strategies.
   *                          Currently, only obstacle collision weights are considered.
   * @return \c true, if the graph was created successfully, \c false otherwise.
   */
  bool buildGraph(double weight_multiplier = 1.0);

  /**
   * @brief Optimize the previously constructed hyper-graph to deform / optimize the TEB.
   *
   * This method invokes the g2o framework to solve the optimization problem considering dedicated sparsity patterns. \n
   * The current implementation calls a non-constrained sparse Levenberg-Marquardt algorithm. Constraints are considered
   * by utilizing penalty approximations. Refer to the literature cited in the HATebOptimalPlanner class description.
   * @see buildGraph
   * @see clearGraph
   * @param no_iterations Number of solver iterations
   * @param clear_after Clear the graph after optimization.
   * @return \c true, if optimization terminates successfully, \c false otherwise.
   */
  bool optimizeGraph(int no_iterations, bool clear_after = true);

  /**
   * @brief Clear an existing internal hyper-graph.
   * @see buildGraph
   * @see optimizeGraph
   */
  void clearGraph();

  /**
   * @brief Add all relevant vertices to the hyper-graph as optimizable variables.
   *
   * Vertices (if unfixed) represent the variables that will be optimized. \n
   * In case of the Timed-Elastic-Band poses and time differences form the vertices of the hyper-graph. \n
   * The order of insertion of vertices (to the graph) is important for efficiency,
   * since it affect the sparsity pattern of the underlying hessian computed for optimization.
   * @see VertexPose
   * @see VertexTimeDiff
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddTEBVertices();

  /**
   * @brief Add all edges (local cost functions) for limiting the translational and angular velocity.
   * @see EdgeVelocity
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesVelocity();

  /**
   * @brief Add all edges (local cost functions) for limiting the translational and angular velocity for agents.
   * @see EdgeVelocity
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesVelocityForAgents();

  /**
   * @brief Add all edges (local cost functions) for limiting the translational and angular acceleration.
   * @see EdgeAcceleration
   * @see EdgeAccelerationStart
   * @see EdgeAccelerationGoal
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesAcceleration();

  /**
   * @brief Add all edges (local cost functions) for limiting the translational and angular acceleration for agents.
   * @see EdgeAcceleration
   * @see EdgeAccelerationStart
   * @see EdgeAccelerationGoal
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesAccelerationForAgents();

  /**
   * @brief Add all edges (local cost functions) for minimizing the transition time (resp. minimize time differences)
   * @see EdgeTimeOptimal
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesTimeOptimal();

  /**
   * @brief Add all edges (local cost functions) for minimizing the transition time (resp. minimize time differences) for agents
   * @see EdgeTimeOptimal
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesTimeOptimalForAgents();

  /**
   * @brief Add all edges (local cost functions) for minimizing the path length
   * @see EdgeShortestPath
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesShortestPath();

  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from static obstacles
   * @warning do not combine with AddEdgesInflatedObstacles
   * @see EdgeObstacle
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Specify an additional weight multipler (in addition to the the config weight)
   */
  void AddEdgesObstacles(double weight_multiplier = 1.0);

  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from static obstacles (legacy association strategy)
   * @warning do not combine with AddEdgesInflatedObstacles
   * @see EdgeObstacle
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Specify an additional weight multipler (in addition to the the config weight)
   */
  void AddEdgesObstaclesLegacy(double weight_multiplier = 1.0);

  /**
   * @brief Add edges to the graph to maintain distance between agents and static obstacles
   *
   * This method adds edges (cost functions) that help maintain safe distances between
   * each agent and static obstacles in the environment. Similar to AddEdgesObstacles()
   * but specifically handles the agent trajectories from the agents_tebs_map_.
   * @see EdgeObstacle
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesObstaclesForAgents();

  /**
   * @brief Add all edges (local cost functions) related to minimizing the distance to via-points
   * @see EdgeViaPoint
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesViaPoints();

  /**
   * @brief Add all edges (local cost functions) related to minimizing the distance to via-points for agents
   * @see EdgeViaPoint
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesViaPointsForAgents();

  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from dynamic (moving) obstacles.
   * @see EdgeDynamicObstacle
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Specify an additional weight multipler (in addition to the the config weight)
   */
  void AddEdgesDynamicObstacles(double weight_multiplier = 1.0);

  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from dynamic (moving) obstacles in case of humans
   * @see EdgeDynamicObstacle
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Specify an additional weight multipler (in addition to the the config weight)
   */
  void AddEdgesDynamicObstaclesForAgents(double weight_multiplier = 1.0);

  /**
   * @brief Add edges to consider invisible humans in trajectory optimization
   *
   * Adds cost function edges to handle regions where humans might be present but not directly
   * visible to the robot's sensors. This helps create safer trajectories in areas with
   * potential occlusions or blind spots.
   * @see EdgeInvisibleHuman
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Scaling factor for the invisible human cost weights
   */
  void AddEdgesInvisibleHumans(double weight_multiplier = 1.0);

  /**
   * @brief Add edges to consider visibility constraints for static agents.
   * Penelizes robot for entering a human fov from behind and very closely.
   * @see EdgeStaticAgentVisibility
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesStaticAgentVisibility();

  /**
   * @brief Add all edges (local cost functions) for satisfying kinematic constraints of a differential drive robot
   * @warning do not combine with AddEdgesKinematicsCarlike()
   * @see AddEdgesKinematicsCarlike
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesKinematicsDiffDrive();

  /**
   * @brief Add all edges (local cost functions) for satisfying kinematic constraints of a differential drive agents
   * @warning do not combine with AddEdgesKinematicsCarlike()
   * @see AddEdgesKinematicsCarlike
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesKinematicsDiffDriveForAgents();

  /**
   * @brief Add all edges (local cost functions) for satisfying kinematic constraints of a carlike robot
   * @warning do not combine with AddEdgesKinematicsDiffDrive()
   * @see AddEdgesKinematicsDiffDrive
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesKinematicsCarlike();

  /**
   * @brief Add all edges (local cost functions) for satisfying kinematic constraints of a carlike agent
   * @warning do not combine with AddEdgesKinematicsDiffDrive()
   * @see AddEdgesKinematicsDiffDrive
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesKinematicsCarlikeForAgents();
  /**
   * @brief Add all edges (local cost functions) for prefering a specifiy turning direction (by penalizing the other one)
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesPreferRotDir();

  /**
   * @brief Add edges to maintain safety distance between agents and robot
   *
   * Adds cost function edges that ensure a minimum safety distance is maintained
   * between the robot and all agents during trajectory optimization.
   * @see EdgeAgentRobotSafety
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesAgentRobotSafety();

  /**
   * @brief Add edges to maintain safety distances between different agents
   *
   * Adds cost function edges that ensure minimum safety distances are maintained
   * between different agents during trajectory optimization to avoid inter-agent collisions.
   * @see EdgeAgentAgentSafety
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesAgentAgentSafety();

  /**
   * @brief Add edges to control relative velocities between agents and robot
   *
   * Adds cost function edges that help the robot to exploit the existing space
   * and robot's velocity to increase human comfort. This makes the robot reduce
   * its velocity in constrained spaces and move farther away with maximum velocity
   * in open spaces.
   * @see EdgeAgentRobotRelVelocity
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesAgentRobotRelVelocity();

  /**
   * @brief Add edges to maintain visibility constraints between agents and robot
   *
   * Adds cost function edges that encourage the robot to enter the human's FOV
   * at an appropriate angle and distance, helping ensure that agents (especially
   * humans) can see and predict the robot's movements.
   * @see EdgeAgentRobotVisibility
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesAgentRobotVisibility();

  //@}

  /**
   * @brief Initialize and configure the g2o sparse optimizer.
   * @return shared pointer to the g2o::SparseOptimizer instance
   */
  static std::shared_ptr<g2o::SparseOptimizer> initOptimizer();

  // external objects (store weak pointers)
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;                     //!< Shared pointer to lifecycle node for ROS2 operations
  const HATebConfig* cfg_;                                              //!< Config class that stores and manages all related parameters
  ObstContainer* obstacles_;                                            //!< Store obstacles that are relevant for planning
  const ViaPointContainer* via_points_;                                 //!< Store via points for planning
  const std::map<uint64_t, ViaPointContainer>* agents_via_points_map_;  //!< Store via points for multiple agents, where each agent is identified by a unique ID

  // internal objects (memory management owned)
  TebVisualizationPtr visualization_;                     //!< Instance of the visualization class
  TimedElasticBand teb_;                                  //!< Actual trajectory object
  std::map<uint64_t, TimedElasticBand> agents_tebs_map_;  //!< Map of TEBs of agents
  FootprintModelPtr robot_model_;                         //!< Robot model
  CircularFootprintPtr agent_model_;                      //!< Agent Footprint model
  std::shared_ptr<g2o::SparseOptimizer> optimizer_;       //!< g2o optimizer for trajectory optimization

  std::map<uint64_t, std::pair<bool, geometry_msgs::msg::Twist>> agents_vel_start_;  //!< Initial velocities for each agent
  std::map<uint64_t, std::pair<bool, geometry_msgs::msg::Twist>> agents_vel_goal_;   //!< Goal velocities for each agent
  std::pair<bool, geometry_msgs::msg::Twist> vel_start_;                             //!< Store the initial velocity at the start pose
  std::pair<bool, geometry_msgs::msg::Twist> vel_goal_;                              //!< Store the final velocity at the goal pose
  std::vector<geometry_msgs::msg::Pose> static_agents_;  //!< Store poses of static agents in the environment that do not move but need consideration for visibility and safety
  bool initialized_;                                     //!< Keeps track about the correct initialization of this class
  bool optimized_;                                       //!< This variable is \c true as long as the last optimization has been completed successful
  double agent_radius_;                                  //!< Radius of the circular footprint used for agent collision checking and safety distances
  double robot_radius_;                                  //!< Radius of the robot's circular footprint used for collision checking and safety calculations
  int isMode_;                                           //!< Planning Mode
  std::vector<double> agent_nominal_vels_;               //!< Nominal agent velocities calculated using moving average filter
  double current_agent_robot_min_dist_;                  //!< Controls addition of edges
  double cost_;                                          //!< Store cost value of the current hyper-graph
  RotType prefer_rotdir_;                                //!< Store whether to prefer a specific initial rotation in optimization (might be activated in case the robot oscillates)

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of the HATebOptimalPlanner
using HATebOptimalPlannerPtr = std::shared_ptr<HATebOptimalPlanner>;
//! Abbrev. for shared const HATebOptimalPlanner pointers
using HATebOptimalPlannerConstPtr = std::shared_ptr<const HATebOptimalPlanner>;
//! Abbrev. for containers storing multiple teb optimal planners
using TebOptPlannerContainer = std::vector<HATebOptimalPlannerPtr>;

}  // namespace hateb_local_planner

#endif /* OPTIMAL_PLANNER_H_ */
