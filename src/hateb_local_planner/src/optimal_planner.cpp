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

#include <hateb_local_planner/optimal_planner.h>

#define THROTTLE_RATE 1.0  // seconds

namespace hateb_local_planner {

// ============== Implementation ===================

HATebOptimalPlanner::HATebOptimalPlanner()
    : node_(nullptr),
      cfg_(nullptr),
      obstacles_(nullptr),
      via_points_(nullptr),
      cost_(HUGE_VAL),
      prefer_rotdir_(RotType::none),
      robot_model_(new PointFootprint()),
      agent_model_(new CircularFootprint()),
      initialized_(false),
      optimized_(false) {}

HATebOptimalPlanner::HATebOptimalPlanner(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const HATebConfig& cfg, ObstContainer* obstacles, FootprintModelPtr robot_model, TebVisualizationPtr visual,
                                         const ViaPointContainer* via_points, CircularFootprintPtr agent_model, const std::map<uint64_t, ViaPointContainer>* agents_via_points_map) {
  initialize(node, cfg, obstacles, std::move(robot_model), std::move(visual), via_points, std::move(agent_model), agents_via_points_map);
}

HATebOptimalPlanner::~HATebOptimalPlanner() {
  clearGraph();
  // free dynamically allocated memory
  // if (optimizer_)
  //  g2o::Factory::destroy();
  // g2o::OptimizationAlgorithmFactory::destroy();
  // g2o::HyperGraphActionLibrary::destroy();
}

void HATebOptimalPlanner::initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const HATebConfig& cfg, ObstContainer* obstacles, FootprintModelPtr robot_model, TebVisualizationPtr visual,
                                     const ViaPointContainer* via_points, CircularFootprintPtr agent_model, const std::map<uint64_t, ViaPointContainer>* agents_via_points_map) {
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();

  node_ = node;
  cfg_ = &cfg;
  obstacles_ = obstacles;
  robot_model_ = std::move(robot_model);
  agent_model_ = std::move(agent_model);
  via_points_ = via_points;
  agents_via_points_map_ = agents_via_points_map;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;
  setVisualization(std::move(visual));

  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;

  initialized_ = true;
  isMode_ = 0;
}

void HATebOptimalPlanner::setVisualization(TebVisualizationPtr visualization) { visualization_ = std::move(visualization); }

void HATebOptimalPlanner::visualize() {
  if (!visualization_) {
    return;
  }

  double fp_size = 0.0;
  fp_size = teb_.sizePoses();

  for (auto& agent_teb_kv : agents_tebs_map_) {
    auto& agent_teb = agent_teb_kv.second;
    fp_size = teb_.sizePoses() > agent_teb.sizePoses() ? teb_.sizePoses() : agent_teb.sizePoses();
  }
  visualization_->publishLocalPlanAndPoses(teb_, *robot_model_, fp_size);
  visualization_->publishAgentLocalPlansAndPoses(agents_tebs_map_, *agent_model_, fp_size);
  visualization_->publishCrossingPoses(teb_, agents_tebs_map_);

  if (teb_.sizePoses() > 0) {
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *robot_model_);
  }

  if (cfg_->trajectory.publish_feedback) {
    visualization_->publishFeedbackMessage(*this, *obstacles_);
  }
}

/*
 * registers custom vertices and edges in g2o framework
 */
void HATebOptimalPlanner::registerG2OTypes() {
  g2o::Factory* factory = g2o::Factory::instance();

  factory->registerType("VERTEX_POSE", std::make_shared<g2o::HyperGraphElementCreator<VertexPose>>());
  factory->registerType("VERTEX_TIMEDIFF", std::make_shared<g2o::HyperGraphElementCreator<VertexTimeDiff>>());
  factory->registerType("EDGE_TIME_OPTIMAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeTimeOptimal>>());
  factory->registerType("EDGE_SHORTEST_PATH", std::make_shared<g2o::HyperGraphElementCreator<EdgeShortestPath>>());
  factory->registerType("EDGE_VELOCITY", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocity>>());
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>>());
  factory->registerType("EDGE_ACCELERATION", std::make_shared<g2o::HyperGraphElementCreator<EdgeAcceleration>>());
  factory->registerType("EDGE_ACCELERATION_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationStart>>());
  factory->registerType("EDGE_ACCELERATION_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationGoal>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>>());
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>>());
  factory->registerType("EDGE_KINEMATICS_CARLIKE", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>>());
  factory->registerType("EDGE_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeObstacle>>());
  factory->registerType("EDGE_INFLATED_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeInflatedObstacle>>());
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeDynamicObstacle>>());
  factory->registerType("EDGE_INVISIBLE_HUMAN", std::make_shared<g2o::HyperGraphElementCreator<EdgeInvisibleHuman>>());
  factory->registerType("EDGE_VIA_POINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeViaPoint>>());
  factory->registerType("EDGE_PREFER_ROTDIR", std::make_shared<g2o::HyperGraphElementCreator<EdgePreferRotDir>>());

  // Agents
  factory->registerType("EDGE_VELOCITY_AGENT", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityAgent>>());
  factory->registerType("EDGE_VELOCITY_HOLONOMIC_AGENT", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomicAgent>>());
  factory->registerType("EDGE_AGENT_ROBOT_SAFETY", std::make_shared<g2o::HyperGraphElementCreator<EdgeAgentRobotSafety>>());
  factory->registerType("EDGE_AGENT_AGENT_SAFETY", std::make_shared<g2o::HyperGraphElementCreator<EdgeAgentAgentSafety>>());
  factory->registerType("EDGE_AGENT_ROBOT_REL_VELOCITy", std::make_shared<g2o::HyperGraphElementCreator<EdgeAgentRobotRelVelocity>>());
  factory->registerType("EDGE_AGENT_ROBOT_VISIBILITY", std::make_shared<g2o::HyperGraphElementCreator<EdgeAgentRobotVisibility>>());
  factory->registerType("EDGE_STATIC_AGENT_VISIBILITY", std::make_shared<g2o::HyperGraphElementCreator<EdgeStaticAgentVisibility>>());
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
std::shared_ptr<g2o::SparseOptimizer> HATebOptimalPlanner::initOptimizer() {
  // Call register_g2o_types once, even for multiple HATebOptimalPlanner instances
  // (thread-safe)
  static std::once_flag flag;
  std::call_once(flag, &registerG2OTypes);

  // allocating the optimizer
  std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();
  std::unique_ptr<TEBLinearSolver> linear_solver(new TEBLinearSolver());  // see typedef in optimization.h
  linear_solver->setBlockOrdering(true);
  std::unique_ptr<TEBBlockSolver> block_solver(new TEBBlockSolver(std::move(linear_solver)));
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  optimizer->setAlgorithm(solver);

  g2o::SparseOptimizer::initMultiThreading();  // required for >Eigen 3.1

  return optimizer;
}

bool HATebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards, double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost,
                                      hateb_local_planner::OptimizationCostArray* op_costs) {
  optimizeTEB(iterations_innerloop, iterations_outerloop, compute_cost_afterwards, obst_cost_scale, viapoint_cost_scale, alternative_time_cost, op_costs, cfg_->trajectory.dt_ref,
              cfg_->trajectory.dt_hysteresis);
  return true;
}

bool HATebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards, double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost,
                                      hateb_local_planner::OptimizationCostArray* op_costs, double dt_ref, double dt_hyst) {
  if (!cfg_->optim.optimization_activate) {
    return false;
  }

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  for (int i = 0; i < iterations_outerloop; ++i) {
    if (cfg_->trajectory.teb_autosize) {
      teb_.autoResize(dt_ref, dt_hyst, cfg_->trajectory.min_samples);
      for (auto& agent_teb_kv : agents_tebs_map_) agent_teb_kv.second.autoResize(dt_ref, dt_hyst, cfg_->trajectory.min_samples);
    }

    success = buildGraph(weight_multiplier);
    if (!success) {
      clearGraph();
      return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success) {
      clearGraph();
      return false;
    }
    optimized_ = true;

    if (compute_cost_afterwards && i == iterations_outerloop - 1)  // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost, op_costs);

    clearGraph();

    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  return true;
}

void HATebOptimalPlanner::setVelocityStart(const geometry_msgs::msg::Twist& vel_start) {
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void HATebOptimalPlanner::setVelocityGoal(const geometry_msgs::msg::Twist& vel_goal) {
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool HATebOptimalPlanner::plan(const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel,
                               const AgentPlanVelMap* initial_agent_plan_vel_map, hateb_local_planner::OptimizationCostArray* op_costs, double dt_ref, double dt_hyst, int Mode) {
  isMode_ = Mode;
  RCLCPP_ASSERT_MSG(initialized_, "Call initialize() first.");
  auto prep_start_time = node_->now();
  if (!teb_.isInit()) {
    // init trajectory
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples,
                              cfg_->trajectory.allow_init_with_backwards_motion, cfg_->trajectory.teb_init_skip_dist);
  } else if (cfg_->optim.disable_warm_start) {
    teb_.clearTimedElasticBand();
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples,
                              cfg_->trajectory.allow_init_with_backwards_motion, cfg_->trajectory.teb_init_skip_dist);

  } else  // warm start
  {
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_(initial_plan.back().pose);
    if (teb_.sizePoses() > 0 && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist &&
        fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) {  // actual warm start!
      teb_.updateAndPruneTEB(start_, goal_,
                             cfg_->trajectory.min_samples);  // update TEB
    } else                                                   // goal too far away -> reinit
    {
      RCLCPP_DEBUG(node_->get_logger(), "New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, true, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion,
                                cfg_->trajectory.teb_init_skip_dist);
    }
  }
  if (start_vel) {
    setVelocityStart(*start_vel);
  }
  if (free_goal_vel) {
    setVelocityGoalFree();
  } else {
    vel_goal_.first = true;
  }  // we just reactivate and use the previously set velocity (should
     // be zero if nothing was modified)
  auto prep_time = node_->now() - prep_start_time;

  auto agent_prep_time_start = node_->now();
  agents_vel_start_.clear();
  agents_vel_goal_.clear();
  agent_nominal_vels_.clear();

  current_agent_robot_min_dist_ = std::numeric_limits<double>::max();

  switch (cfg_->planning_mode) {
    case 0:
      agents_tebs_map_.clear();
      break;
    case 1: {
      auto itr = agents_tebs_map_.begin();
      while (itr != agents_tebs_map_.end()) {
        if (initial_agent_plan_vel_map->find(itr->first) == initial_agent_plan_vel_map->end()) {
          itr = agents_tebs_map_.erase(itr);
        } else {
          ++itr;
        }
      }

      static_agents_.clear();

      const auto& rp = initial_plan.front().pose.position;

      for (const auto& initial_agent_plan_vel_kv : *initial_agent_plan_vel_map) {
        const auto& agent_id = initial_agent_plan_vel_kv.first;
        const auto& initial_agent_plan = initial_agent_plan_vel_kv.second.plan;

        // isMode = initial_agent_plan_vel_kv.second.isMode;
        // erase agent-teb if agent plan is empty
        if (initial_agent_plan.empty()) {
          auto itr = agents_tebs_map_.find(agent_id);
          if (itr != agents_tebs_map_.end()) {
            RCLCPP_DEBUG(node_->get_logger(), "New plan: new agent plan is empty. Removing agent trajectories.");
            agents_tebs_map_.erase(itr);
          }

          continue;
        } else if (!initial_agent_plan.empty()) {
          if (initial_agent_plan[0].header.frame_id == "static") {
            auto itr = agents_tebs_map_.find(agent_id);
            if (itr != agents_tebs_map_.end()) {
              RCLCPP_DEBUG(node_->get_logger(), "New plan: new agent plan is empty. Removing agent trajectories.");
              agents_tebs_map_.erase(itr);
            }
            static_agents_.push_back(initial_agent_plan[0].pose);
            continue;
          }
        }

        agent_nominal_vels_.push_back(initial_agent_plan_vel_kv.second.nominal_vel);

        const auto& hp = initial_agent_plan.front().pose.position;
        auto dist = std::hypot(rp.x - hp.x, rp.y - hp.y);
        current_agent_robot_min_dist_ = std::min(dist, current_agent_robot_min_dist_);
        if (agents_tebs_map_.find(agent_id) == agents_tebs_map_.end()) {
          // create new agent-teb for new agent
          agents_tebs_map_[agent_id] = TimedElasticBand();
          agents_tebs_map_[agent_id].initTrajectoryToGoal(initial_agent_plan, cfg_->agent.max_vel_x, cfg_->agent.max_vel_theta, true, cfg_->trajectory.agent_min_samples,
                                                          cfg_->trajectory.allow_init_with_backwards_motion, cfg_->trajectory.teb_init_skip_dist);

        } else if (cfg_->optim.disable_warm_start) {
          auto& agent_teb = agents_tebs_map_[agent_id];
          agent_teb.clearTimedElasticBand();
          agent_teb.initTrajectoryToGoal(initial_agent_plan, cfg_->agent.max_vel_x, cfg_->agent.max_vel_theta, true, cfg_->trajectory.agent_min_samples,
                                         cfg_->trajectory.allow_init_with_backwards_motion, cfg_->trajectory.teb_init_skip_dist);

        }

        else {
          // modify agent-teb for existing agent
          PoseSE2 agent_start_(initial_agent_plan.front().pose);
          PoseSE2 agent_goal_(initial_agent_plan.back().pose);
          auto& agent_teb = agents_tebs_map_[agent_id];
          if (agent_teb.sizePoses() > 0 && (agent_goal_.position() - agent_teb.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist) {
            agent_teb.updateAndPruneTEB(agent_start_, agent_goal_, cfg_->trajectory.agent_min_samples);
          } else {
            RCLCPP_DEBUG(node_->get_logger(), "New goal: distance to existing goal is higher than the specified threshold. Reinitializing agent trajectories.");
            agent_teb.clearTimedElasticBand();
            agent_teb.initTrajectoryToGoal(initial_agent_plan, cfg_->agent.max_vel_x, cfg_->agent.max_vel_theta, true, cfg_->trajectory.agent_min_samples, false, cfg_->trajectory.teb_init_skip_dist);
          }
        }

        // give start velocity for agents
        std::pair<bool, geometry_msgs::msg::Twist> agent_start_vel;
        agent_start_vel.first = true;
        agent_start_vel.second.linear.x = initial_agent_plan_vel_kv.second.start_vel.linear.x;
        agent_start_vel.second.linear.y = initial_agent_plan_vel_kv.second.start_vel.linear.y;
        agent_start_vel.second.angular.z = initial_agent_plan_vel_kv.second.start_vel.angular.z;
        agents_vel_start_[agent_id] = agent_start_vel;

        // do not set goal velocity for agents
        std::pair<bool, geometry_msgs::msg::Twist> agent_goal_vel;
        agent_goal_vel.first = false;
      }
      break;
    }
    default:
      agents_tebs_map_.clear();
  }
  auto agent_prep_time = node_->now() - agent_prep_time_start;

  // now optimize
  auto opt_start_time = node_->now();
  bool teb_opt_result = optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations, true, 1.0, 1.0, false, op_costs, dt_ref, dt_hyst);

  if (op_costs) {
    hateb_local_planner::OptimizationCost op_cost;
    op_cost.type = hateb_local_planner::OptimizationCost::AGENT_ROBOT_MIN_DIST;
    op_cost.cost = current_agent_robot_min_dist_;
    op_costs->costs.push_back(op_cost);
  }

  auto opt_time = node_->now() - opt_start_time;

  auto total_time = node_->now() - prep_start_time;

  RCLCPP_DEBUG_STREAM_COND(total_time.seconds() > 0.1, node_->get_logger(),
                           "\nteb optimal plan times:\n"
                               << "\ttotal plan time                " << std::to_string(total_time.seconds()) << "\n"
                               << "\toptimizatoin preparation time  " << std::to_string(prep_time.seconds()) << "\n"
                               << "\tagent preparation time         " << std::to_string(prep_time.seconds()) << "\n"
                               << "\tteb optimize time              " << std::to_string(opt_time.seconds()) << "\n-------------------------");

  return teb_opt_result;
}

bool HATebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel, double pre_plan_time,
                               hateb_local_planner::OptimizationCostArray* op_costs, double dt_ref, double dt_hyst, int Mode) {
  isMode_ = Mode;
  RCLCPP_ASSERT_MSG(initialized_, "Call initialize() first.");
  auto prep_start_time = node_->now();
  if (!teb_.isInit()) {
    // init trajectory
    teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples,
                              cfg_->trajectory.allow_init_with_backwards_motion);  // 0 intermediate
    // samples, but dt=1 -> autoResize will add more samples before calling
    // first optimization
    // teb_.initTEBtoGoal(start, goal, 0, 1, cfg_->trajectory.min_samples);
  } else  // warm start
  {
    if (teb_.sizePoses() > 0 && (goal.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist &&
        fabs(g2o::normalize_theta(goal.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular)  // actual warm start!
    {
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    } else  // goal too far away -> reinit
    {
      RCLCPP_DEBUG(node_->get_logger(), "New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
      // teb_.initTEBtoGoal(start, goal, 0, 1, cfg_->trajectory.min_samples);
    }
  }
  if (start_vel) {
    setVelocityStart(*start_vel);
  }
  if (free_goal_vel) {
    setVelocityGoalFree();
  } else {
    // we just reactivate and use the previously set velocity (should
    // be zero if nothing was modified)
    vel_goal_.first = true;
  }
  auto prep_time = node_->now() - prep_start_time;
  // now optimize

  auto opt_start_time = node_->now();
  bool teb_opt_result = optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations, true, 1.0, 1.0, false, op_costs, dt_ref, dt_hyst);
  auto opt_time = node_->now() - opt_start_time;

  auto total_time = node_->now() - prep_start_time;
  RCLCPP_INFO_STREAM_COND((total_time.seconds() + pre_plan_time) > 0.05, node_->get_logger(),
                          "\nteb optimal plan times:\n"
                              << "\ttotal plan time                " << std::to_string(total_time.seconds() + pre_plan_time) << "\n"
                              << "\tpre-plan time                  " << std::to_string(pre_plan_time) << "\n"
                              << "\toptimizatoin preparation time  " << std::to_string(prep_time.seconds()) << "\n"
                              << "\tteb optimize time              " << std::to_string(opt_time.seconds()) << "\n-------------------------");

  return teb_opt_result;
}

bool HATebOptimalPlanner::buildGraph(double weight_multiplier) {
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty()) {
    RCLCPP_WARN(node_->get_logger(), "Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  // add TEB vertices
  AddTEBVertices();

  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association) {
    AddEdgesObstaclesLegacy(weight_multiplier);
  } else {
    AddEdgesObstacles(weight_multiplier);
  }

  if (cfg_->obstacles.include_dynamic_obstacles) {
    AddEdgesDynamicObstacles();
  }

  AddEdgesViaPoints();

  AddEdgesVelocity();

  AddEdgesAcceleration();

  AddEdgesTimeOptimal();

  AddEdgesShortestPath();

  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0) {
    AddEdgesKinematicsDiffDrive();
  }  // we have a differential drive robot
  else {
    AddEdgesKinematicsCarlike();
  }  // we have a carlike robot since the turning
     // radius is bounded from below.

  AddEdgesPreferRotDir();

  switch (cfg_->planning_mode) {
    case 0:
      break;
    case 1:
      AddEdgesObstaclesForAgents();
      AddEdgesDynamicObstaclesForAgents();

      AddEdgesViaPointsForAgents();

      AddEdgesVelocityForAgents();
      AddEdgesAccelerationForAgents();

      AddEdgesTimeOptimalForAgents();

      AddEdgesKinematicsDiffDriveForAgents();
      // AddEdgesKinematicsCarlikeForAgents();

      if (cfg_->hateb.use_agent_robot_safety_c) {
        AddEdgesAgentRobotSafety();
      }

      if (cfg_->hateb.use_agent_agent_safety_c) {
        AddEdgesAgentAgentSafety();
      }
      if (cfg_->hateb.use_agent_robot_rel_vel_c) {
        AddEdgesAgentRobotRelVelocity();
      }
      if (cfg_->hateb.use_agent_robot_visi_c) {
        AddEdgesAgentRobotVisibility();
        AddEdgesStaticAgentVisibility();
      }
      if (cfg_->hateb.add_invisible_humans) {
        AddEdgesInvisibleHumans();
      }
      break;
    default:
      break;
  }

  return true;
}

bool HATebOptimalPlanner::optimizeGraph(int no_iterations, bool clear_after) {
  if (cfg_->robot.max_vel_x < 0.01) {
    RCLCPP_WARN(node_->get_logger(), "optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) {
      clearGraph();
    }
    return false;
  }

  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples) {
    RCLCPP_WARN(node_->get_logger(), "optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) {
      clearGraph();
    }
    return false;
  }

  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm =
  //  dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if (!iter) {
    RCLCPP_ERROR(node_->get_logger(), "optimizeGraph(): Optimization failed! iter=%i", iter);
    return false;
  }

  if (clear_after) clearGraph();

  return true;
}

void HATebOptimalPlanner::clearGraph() {
  // clear optimizer states

  if (optimizer_) {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto& vertices = optimizer_->vertices();
    for (auto& v : vertices) v.second->edges().clear();

    optimizer_->vertices().clear();  // necessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}

void HATebOptimalPlanner::AddTEBVertices() {
  // add vertices to graph
  RCLCPP_DEBUG_COND(cfg_->optim.optimization_verbose, node_->get_logger(), "Adding TEB vertices ...");
  unsigned int id_counter = 0;  // used for vertices ids
  for (int i = 0; i < teb_.sizePoses(); ++i) {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));
    if (teb_.sizeTimeDiffs() != 0 && i < teb_.sizeTimeDiffs()) {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));
    }
  }

  switch (cfg_->planning_mode) {
    case 0:
      break;
    case 1: {
      for (auto& agent_teb_kv : agents_tebs_map_) {
        auto& agent_teb = agent_teb_kv.second;
        for (int i = 0; i < agent_teb.sizePoses(); ++i) {
          agent_teb.PoseVertex(i)->setId(id_counter++);
          optimizer_->addVertex(agent_teb.PoseVertex(i));
          if (teb_.sizeTimeDiffs() != 0 && i < agent_teb.sizeTimeDiffs()) {
            agent_teb.TimeDiffVertex(i)->setId(id_counter++);
            optimizer_->addVertex(agent_teb.TimeDiffVertex(i));
          }
        }
      }
      break;
    }
    default:
      break;
  }
}

void HATebOptimalPlanner::AddEdgesObstacles(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 || obstacles_ == nullptr) {
    return;
  }  // if weight equals zero skip adding edges!

  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double, 2, 2> information_inflated;
  information_inflated(0, 0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1, 1) = cfg_->optim.weight_inflation;
  information_inflated(0, 1) = information_inflated(1, 0) = 0;

  // iterate all teb points (skip first and last)
  for (int i = 1; i < teb_.sizePoses() - 1; ++i) {
    double left_min_dist = std::numeric_limits<double>::max();
    double right_min_dist = std::numeric_limits<double>::max();
    Obstacle* left_obstacle = nullptr;
    Obstacle* right_obstacle = nullptr;

    std::vector<Obstacle*> relevant_obstacles;

    const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

    // iterate obstacles
    for (const ObstaclePtr& obst : *obstacles_) {
      // we handle dynamic obstacles differently below
      if (cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic()) {
        continue;
      }

      // calculate distance to robot model
      double dist = robot_model_->calculateDistance(teb_.Pose(i), obst.get());

      // force considering obstacle if really close to the current pose
      if (dist < cfg_->obstacles.min_obstacle_dist * cfg_->obstacles.obstacle_association_force_inclusion_factor) {
        relevant_obstacles.push_back(obst.get());
        continue;
      }
      // cut-off distance
      if (dist > cfg_->obstacles.min_obstacle_dist * cfg_->obstacles.obstacle_association_cutoff_factor) {
        continue;
      }

      // determine side (left or right) and assign obstacle if closer than the
      // previous one
      if (cross2d(pose_orient, obst->getCentroid()) > 0)  // left
      {
        if (dist < left_min_dist) {
          left_min_dist = dist;
          left_obstacle = obst.get();
        }
      } else {
        if (dist < right_min_dist) {
          right_min_dist = dist;
          right_obstacle = obst.get();
        }
      }
    }

    // create obstacle edges
    if (left_obstacle) {
      if (inflated) {
        auto* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, left_obstacle, cohan_msgs::msg::AgentType::ROBOT);
        optimizer_->addEdge(dist_bandpt_obst);
      } else {
        auto* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, left_obstacle, cohan_msgs::msg::AgentType::ROBOT);
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }

    if (right_obstacle) {
      if (inflated) {
        auto* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, right_obstacle, cohan_msgs::msg::AgentType::ROBOT);
        optimizer_->addEdge(dist_bandpt_obst);
      } else {
        auto* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, right_obstacle, cohan_msgs::msg::AgentType::ROBOT);
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }

    for (const Obstacle* obst : relevant_obstacles) {
      if (inflated) {
        auto* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, obst, cohan_msgs::msg::AgentType::ROBOT);
        optimizer_->addEdge(dist_bandpt_obst);
      } else {
        auto* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, obst, cohan_msgs::msg::AgentType::ROBOT);
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }
  }
}

void HATebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 || obstacles_ == nullptr) {
    return;
  }  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double, 2, 2> information_inflated;
  information_inflated(0, 0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1, 1) = cfg_->optim.weight_inflation;
  information_inflated(0, 1) = information_inflated(1, 0) = 0;

  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  for (const auto& obstacle : *obstacles_) {
    if (cfg_->obstacles.include_dynamic_obstacles && obstacle->isDynamic()) {  // we handle dynamic obstacles differently below

      continue;
    }

    int index;

    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses()) {
      index = teb_.sizePoses() / 2;
    } else {
      index = teb_.findClosestTrajectoryPose(*(obstacle));
    }
    // check if obstacle is outside index-range between start and goal
    if ((index <= 1) || (index > teb_.sizePoses() - 2))  // start and goal are fixed and findNearestBandpoint
                                                         // finds first or last conf if intersection point is
                                                         // outside the range
    {
      continue;
    }

    if (inflated) {
      auto* dist_bandpt_obst = new EdgeInflatedObstacle;
      dist_bandpt_obst->setVertex(0, teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::ROBOT);
      optimizer_->addEdge(dist_bandpt_obst);
    } else {
      auto* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0, teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::ROBOT);
      optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbour_idx = 0; neighbour_idx < floor(cfg_->obstacles.obstacle_poses_affected / 2); neighbour_idx++) {
      if (index + neighbour_idx < teb_.sizePoses()) {
        if (inflated) {
          auto* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
          dist_bandpt_obst_n_r->setVertex(0, teb_.PoseVertex(index + neighbour_idx));
          dist_bandpt_obst_n_r->setInformation(information_inflated);
          dist_bandpt_obst_n_r->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::ROBOT);
          optimizer_->addEdge(dist_bandpt_obst_n_r);
        } else {
          auto* dist_bandpt_obst_n_r = new EdgeObstacle;
          dist_bandpt_obst_n_r->setVertex(0, teb_.PoseVertex(index + neighbour_idx));
          dist_bandpt_obst_n_r->setInformation(information);
          dist_bandpt_obst_n_r->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::ROBOT);
          optimizer_->addEdge(dist_bandpt_obst_n_r);
        }
      }
      if (index - neighbour_idx >= 0)  // needs to be casted to int to allow negative values
      {
        if (inflated) {
          auto* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
          dist_bandpt_obst_n_l->setVertex(0, teb_.PoseVertex(index - neighbour_idx));
          dist_bandpt_obst_n_l->setInformation(information_inflated);
          dist_bandpt_obst_n_l->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::ROBOT);
          optimizer_->addEdge(dist_bandpt_obst_n_l);
        } else {
          auto* dist_bandpt_obst_n_l = new EdgeObstacle;
          dist_bandpt_obst_n_l->setVertex(0, teb_.PoseVertex(index - neighbour_idx));
          dist_bandpt_obst_n_l->setInformation(information);
          dist_bandpt_obst_n_l->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::ROBOT);
          optimizer_->addEdge(dist_bandpt_obst_n_l);
        }
      }
    }
  }
}

// Needs an update --> please check
void HATebOptimalPlanner::AddEdgesObstaclesForAgents() {
  if (cfg_->optim.weight_obstacle == 0 || obstacles_ == nullptr) {
    return;
  }

  for (const auto& obstacle : *obstacles_) {
    if (obstacle->isDynamic())  // we handle dynamic obstacles differently below
    {
      continue;
    }

    unsigned int index;

    for (auto& agent_teb_kv : agents_tebs_map_) {
      auto& agent_teb = agent_teb_kv.second;

      if (cfg_->obstacles.obstacle_poses_affected >= agent_teb.sizePoses()) {
        index = agent_teb.sizePoses() / 2;
      } else {
        index = agent_teb.findClosestTrajectoryPose(*(obstacle));
      }

      if ((index <= 1) || (index > agent_teb.sizePoses() - 1)) {
        continue;
      }

      Eigen::Matrix<double, 1, 1> information;
      information.fill(cfg_->optim.weight_obstacle);

      auto* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0, agent_teb.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::HUMAN);
      optimizer_->addEdge(dist_bandpt_obst);

      for (unsigned int neighbour_idx = 0; neighbour_idx < floor(cfg_->obstacles.obstacle_poses_affected / 2); neighbour_idx++) {
        if (index + neighbour_idx < agent_teb.sizePoses()) {
          auto* dist_bandpt_obst_n_r = new EdgeObstacle;
          dist_bandpt_obst_n_r->setVertex(0, agent_teb.PoseVertex(index + neighbour_idx));
          dist_bandpt_obst_n_r->setInformation(information);
          dist_bandpt_obst_n_r->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::HUMAN);
          optimizer_->addEdge(dist_bandpt_obst_n_r);
        }
        if (static_cast<int>(index) - static_cast<int>(neighbour_idx) >= 0) {
          auto* dist_bandpt_obst_n_l = new EdgeObstacle;
          dist_bandpt_obst_n_l->setVertex(0, agent_teb.PoseVertex(index - neighbour_idx));
          dist_bandpt_obst_n_l->setInformation(information);
          dist_bandpt_obst_n_l->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::HUMAN);
          optimizer_->addEdge(dist_bandpt_obst_n_l);
        }
      }
    }
  }
}

void HATebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 || obstacles_ == nullptr) {
    return;
  }  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 2, 2> information;
  information(0, 0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1, 1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0, 1) = information(1, 0) = 0;

  for (auto& obstacle : *obstacles_) {
    if (!obstacle->isDynamic() || obstacle->isHuman()) {
      continue;
    }

    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i = 1; i < teb_.sizePoses() - 1; ++i) {
      auto* dynobst_edge = new EdgeDynamicObstacle(time);
      dynobst_edge->setVertex(0, teb_.PoseVertex(i));
      dynobst_edge->setInformation(information);
      dynobst_edge->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::ROBOT);
      optimizer_->addEdge(dynobst_edge);
      time += teb_.TimeDiff(i);  // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
    }
  }
}

void HATebOptimalPlanner::AddEdgesInvisibleHumans(double weight_multiplier) {
  if (cfg_->optim.weight_invisible_human == 0 || weight_multiplier == 0 || obstacles_ == nullptr || isMode_ >= 3) {
    return;
  }  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information(0, 0) = cfg_->optim.weight_invisible_human * weight_multiplier;

  for (auto& obstacle : *obstacles_) {
    if (!obstacle->isDynamic() || !obstacle->isHuman()) {
      continue;
    }

    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i = 1; i < teb_.sizePoses() - 1; ++i) {
      auto* inv_human_edge = new EdgeInvisibleHuman(time);
      inv_human_edge->setVertex(0, teb_.PoseVertex(i));
      inv_human_edge->setVertex(1, teb_.PoseVertex(i + 1));
      inv_human_edge->setVertex(2, teb_.TimeDiffVertex(i));
      inv_human_edge->setInformation(information);
      inv_human_edge->setParameters(*cfg_, obstacle.get());
      optimizer_->addEdge(inv_human_edge);
      time += teb_.TimeDiff(i);  // we do not need to check the time diff bounds,
                                 // since we iterate to "< sizePoses()-1".
    }
  }
}

void HATebOptimalPlanner::AddEdgesDynamicObstaclesForAgents(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 || obstacles_ == nullptr) {
    return;
  }  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 2, 2> information;
  information(0, 0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1, 1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0, 1) = information(1, 0) = 0;

  for (const auto& obstacle : *obstacles_) {
    if (!obstacle->isDynamic()) {
      continue;
    }

    for (auto& agent_teb_kv : agents_tebs_map_) {
      auto& agent_teb = agent_teb_kv.second;

      for (std::size_t i = 1; i < agent_teb.sizePoses() - 1; ++i) {
        auto* dynobst_edge = new EdgeDynamicObstacle(i);
        dynobst_edge->setVertex(0, agent_teb.PoseVertex(i));
        dynobst_edge->setVertex(1, agent_teb.TimeDiffVertex(i));
        dynobst_edge->setInformation(information);
        dynobst_edge->setParameters(*cfg_, obstacle.get(), cohan_msgs::msg::AgentType::HUMAN);
        optimizer_->addEdge(dynobst_edge);
      }
    }
  }
}

void HATebOptimalPlanner::AddEdgesViaPoints() {
  if (cfg_->optim.weight_viapoint == 0 || via_points_ == nullptr || via_points_->empty()) {
    return;
  }  // if weight equals zero skip adding edges!

  int start_pose_idx = 0;

  int n = teb_.sizePoses();
  if (n < 3)  // we do not have any degrees of freedom for reaching via-points
  {
    return;
  }

  for (const auto& via_point : *via_points_) {
    int index = teb_.findClosestTrajectoryPose(via_point, nullptr, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered) start_pose_idx = index + 2;  // skip a point to have a DOF inbetween for further via-points

    // check if point conicides with goal or is located behind it
    index = std::min(index, n - 2);  // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if (index < 1) {
      if (cfg_->trajectory.via_points_ordered) {
        index = 1;  // try to connect the via point with the second (and
                    // non-fixed) pose. It is likely that autoresize adds new
                    // poses inbetween later.
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "HATebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
        continue;  // skip via points really close or behind the current robot
                   // pose
      }
    }
    Eigen::Matrix<double, 1, 1> information;
    information.fill(cfg_->optim.weight_viapoint);

    auto* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0, teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &via_point);
    optimizer_->addEdge(edge_viapoint);
  }
}

void HATebOptimalPlanner::AddEdgesViaPointsForAgents() {
  if (cfg_->optim.weight_agent_viapoint == 0 || via_points_ == nullptr || via_points_->empty()) {
    return;
  }

  int start_pose_idx = 0;

  int n = teb_.sizePoses();
  if (n < 3) {
    return;
  }

  for (const auto& agent_via_points_kv : *agents_via_points_map_) {
    if (agents_tebs_map_.find(agent_via_points_kv.first) == agents_tebs_map_.end()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), THROTTLE_RATE * 1000, "inconsistant data between agents_tebs_map and agents_via_points_map (for id %ld)",
                           agent_via_points_kv.first);
      continue;
    }

    const auto& agent_via_points = agent_via_points_kv.second;
    auto& agent_teb = agents_tebs_map_[agent_via_points_kv.first];

    for (const auto& agent_via_point : agent_via_points) {
      int index = agent_teb.findClosestTrajectoryPose(agent_via_point, nullptr, start_pose_idx);
      if (cfg_->trajectory.via_points_ordered) start_pose_idx = index + 2;

      index = std::min(index, n - 1);
      index = std::max(index, 1);

      Eigen::Matrix<double, 1, 1> information;
      information.fill(cfg_->optim.weight_agent_viapoint);

      auto* edge_viapoint = new EdgeViaPoint;
      edge_viapoint->setVertex(0, agent_teb.PoseVertex(index));
      edge_viapoint->setInformation(information);
      edge_viapoint->setParameters(*cfg_, &agent_via_point);
      optimizer_->addEdge(edge_viapoint);
    }
  }
}

void HATebOptimalPlanner::AddEdgesVelocity() {
  if (cfg_->robot.max_vel_y == 0)  // non-holonomic robot
  {
    if (cfg_->optim.weight_max_vel_x == 0 && cfg_->optim.weight_max_vel_theta == 0) {  // if weight equals zero skip adding edges!
      return;
    }

    int n = teb_.sizePoses();
    Eigen::Matrix<double, 2, 2> information;
    information(0, 0) = cfg_->optim.weight_max_vel_x;
    information(1, 1) = cfg_->optim.weight_max_vel_theta;
    information(0, 1) = 0.0;
    information(1, 0) = 0.0;

    for (int i = 0; i < n - 1; ++i) {
      auto* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0, teb_.PoseVertex(i));
      velocity_edge->setVertex(1, teb_.PoseVertex(i + 1));
      velocity_edge->setVertex(2, teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setParameters(*cfg_, robot_model_.get(), isMode_);
      optimizer_->addEdge(velocity_edge);
    }
  } else  // holonomic-robot
  {
    if (cfg_->optim.weight_max_vel_x == 0 && cfg_->optim.weight_max_vel_y == 0 && cfg_->optim.weight_max_vel_theta == 0) {  // if weight equals zero skip adding edges!
      return;
    }

    int n = teb_.sizePoses();
    Eigen::Matrix<double, 3, 3> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_max_vel_x;
    information(1, 1) = cfg_->optim.weight_max_vel_y;
    information(2, 2) = cfg_->optim.weight_max_vel_theta;

    for (int i = 0; i < n - 1; ++i) {
      auto* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0, teb_.PoseVertex(i));
      velocity_edge->setVertex(1, teb_.PoseVertex(i + 1));
      velocity_edge->setVertex(2, teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setParameters(*cfg_, robot_model_.get(), isMode_);
      optimizer_->addEdge(velocity_edge);
    }
  }
}

void HATebOptimalPlanner::AddEdgesVelocityForAgents() {
  if (cfg_->agent.max_vel_y == 0)  // non-holonomic robot
  {
    if (cfg_->optim.weight_max_agent_vel_x == 0 && cfg_->optim.weight_max_agent_vel_theta == 0 && cfg_->optim.weight_nominal_agent_vel_x == 0) {  // if weight equals zero skip adding edges!

      return;
    }
    Eigen::Matrix<double, 3, 3> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_max_agent_vel_x;
    information(1, 1) = cfg_->optim.weight_max_agent_vel_theta;
    information(2, 2) = cfg_->optim.weight_nominal_agent_vel_x;

    int itr_idx = 0;
    for (auto& agent_teb_kv : agents_tebs_map_) {
      auto& agent_teb = agent_teb_kv.second;

      int n = agent_teb.sizePoses();
      for (int i = 0; i < n - 1; ++i) {
        auto* agent_velocity_edge = new EdgeVelocityAgent;
        agent_velocity_edge->setVertex(0, agent_teb.PoseVertex(i));
        agent_velocity_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
        agent_velocity_edge->setVertex(2, agent_teb.TimeDiffVertex(i));
        agent_velocity_edge->setInformation(information);
        agent_velocity_edge->setParameters(*cfg_, agent_nominal_vels_[itr_idx]);
        optimizer_->addEdge(agent_velocity_edge);
      }
      itr_idx++;
    }
  } else  // holonomic-agent
  {
    if (cfg_->optim.weight_max_agent_vel_x == 0 && cfg_->optim.weight_max_agent_vel_y == 0 && cfg_->optim.weight_max_agent_vel_theta == 0 && cfg_->optim.weight_nominal_agent_vel_x == 0) {
      return;  // if weight equals zero skip adding edges!
    }

    Eigen::Matrix<double, 4, 4> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_max_agent_vel_x;
    information(1, 1) = cfg_->optim.weight_max_agent_vel_y;
    information(2, 2) = cfg_->optim.weight_max_agent_vel_theta;
    information(3, 3) = cfg_->optim.weight_nominal_agent_vel_x;

    int itr_idx = 0;
    for (auto& agent_teb_kv : agents_tebs_map_) {
      auto& agent_teb = agent_teb_kv.second;

      int n = agent_teb.sizePoses();
      for (int i = 0; i < n - 1; ++i) {
        auto* agent_velocity_edge = new EdgeVelocityHolonomicAgent;
        agent_velocity_edge->setVertex(0, agent_teb.PoseVertex(i));
        agent_velocity_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
        agent_velocity_edge->setVertex(2, agent_teb.TimeDiffVertex(i));
        agent_velocity_edge->setInformation(information);
        agent_velocity_edge->setParameters(*cfg_, agent_nominal_vels_[itr_idx]);
        optimizer_->addEdge(agent_velocity_edge);
      }
      itr_idx++;
    }
  }
}

void HATebOptimalPlanner::AddEdgesAcceleration() {
  if (cfg_->optim.weight_acc_lim_x == 0 && cfg_->optim.weight_acc_lim_theta == 0) {  // if weight equals zero skip adding edges!
    return;
  }

  int n = teb_.sizePoses();

  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0)  // non-holonomic robot
  {
    Eigen::Matrix<double, 2, 2> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_acc_lim_x;
    information(1, 1) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first) {
      auto* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0, teb_.PoseVertex(0));
      acceleration_edge->setVertex(1, teb_.PoseVertex(1));
      acceleration_edge->setVertex(2, teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i = 0; i < n - 2; ++i) {
      auto* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0, teb_.PoseVertex(i));
      acceleration_edge->setVertex(1, teb_.PoseVertex(i + 1));
      acceleration_edge->setVertex(2, teb_.PoseVertex(i + 2));
      acceleration_edge->setVertex(3, teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4, teb_.TimeDiffVertex(i + 1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first) {
      auto* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0, teb_.PoseVertex(n - 2));
      acceleration_edge->setVertex(1, teb_.PoseVertex(n - 1));
      acceleration_edge->setVertex(2, teb_.TimeDiffVertex(teb_.sizeTimeDiffs() - 1));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  } else  // holonomic robot
  {
    Eigen::Matrix<double, 3, 3> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_acc_lim_x;
    information(1, 1) = cfg_->optim.weight_acc_lim_y;
    information(2, 2) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first) {
      auto* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0, teb_.PoseVertex(0));
      acceleration_edge->setVertex(1, teb_.PoseVertex(1));
      acceleration_edge->setVertex(2, teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i = 0; i < n - 2; ++i) {
      auto* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0, teb_.PoseVertex(i));
      acceleration_edge->setVertex(1, teb_.PoseVertex(i + 1));
      acceleration_edge->setVertex(2, teb_.PoseVertex(i + 2));
      acceleration_edge->setVertex(3, teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4, teb_.TimeDiffVertex(i + 1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first) {
      auto* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0, teb_.PoseVertex(n - 2));
      acceleration_edge->setVertex(1, teb_.PoseVertex(n - 1));
      acceleration_edge->setVertex(2, teb_.TimeDiffVertex(teb_.sizeTimeDiffs() - 1));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
}

void HATebOptimalPlanner::AddEdgesAccelerationForAgents() {
  if (cfg_->optim.weight_agent_acc_lim_x == 0 && cfg_->optim.weight_agent_acc_lim_theta == 0) {  // if weight equals zero skip adding edges!
    return;
  }

  if (cfg_->agent.max_vel_y == 0 || cfg_->agent.acc_lim_y == 0)  // non-holonomic agent
  {
    Eigen::Matrix<double, 2, 2> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_agent_acc_lim_x;
    information(1, 1) = cfg_->optim.weight_agent_acc_lim_theta;
    for (auto& agent_teb_kv : agents_tebs_map_) {
      const auto& agent_it = agent_teb_kv.first;
      auto& agent_teb = agent_teb_kv.second;
      // check if an initial velocity should be taken into accound
      int n = agent_teb.sizePoses();

      if (agents_vel_start_[agent_it].first) {
        auto* agent_acceleration_edge = new EdgeAccelerationStart;
        agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(0));
        agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(1));
        agent_acceleration_edge->setVertex(2, agent_teb.TimeDiffVertex(0));
        agent_acceleration_edge->setInitialVelocity(agents_vel_start_[agent_it].second);
        agent_acceleration_edge->setInformation(information);
        agent_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(agent_acceleration_edge);
      }

      // now add the usual acceleration edge for each tuple of three teb poses
      for (int i = 0; i < n - 2; ++i) {
        auto* agent_acceleration_edge = new EdgeAcceleration;
        agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(i));
        agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
        agent_acceleration_edge->setVertex(2, agent_teb.PoseVertex(i + 2));
        agent_acceleration_edge->setVertex(3, agent_teb.TimeDiffVertex(i));
        agent_acceleration_edge->setVertex(4, agent_teb.TimeDiffVertex(i + 1));
        agent_acceleration_edge->setInformation(information);
        agent_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(agent_acceleration_edge);
      }

      // check if a goal velocity should be taken into accound
      if (agents_vel_goal_[agent_it].first) {
        auto* agent_acceleration_edge = new EdgeAccelerationGoal;
        agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(n - 2));
        agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(n - 1));
        agent_acceleration_edge->setVertex(2, agent_teb.TimeDiffVertex(agent_teb.sizeTimeDiffs() - 1));
        agent_acceleration_edge->setGoalVelocity(agents_vel_goal_[agent_it].second);
        agent_acceleration_edge->setInformation(information);
        agent_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(agent_acceleration_edge);
      }
    }
  } else  // holonomic robot
  {
    Eigen::Matrix<double, 3, 3> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_acc_lim_x;
    information(1, 1) = cfg_->optim.weight_acc_lim_y;
    information(2, 2) = cfg_->optim.weight_acc_lim_theta;
    for (auto& agent_teb_kv : agents_tebs_map_) {
      const auto& agent_it = agent_teb_kv.first;
      auto& agent_teb = agent_teb_kv.second;
      // check if an initial velocity should be taken into accound
      int n = agent_teb.sizePoses();
      // check if an initial velocity should be taken into accound
      if (vel_start_.first) {
        auto* agent_acceleration_edge = new EdgeAccelerationHolonomicStart;
        agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(0));
        agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(1));
        agent_acceleration_edge->setVertex(2, agent_teb.TimeDiffVertex(0));
        agent_acceleration_edge->setInitialVelocity(agents_vel_start_[agent_it].second);
        agent_acceleration_edge->setInformation(information);
        agent_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(agent_acceleration_edge);
      }

      // now add the usual acceleration edge for each tuple of three teb poses
      for (int i = 0; i < n - 2; ++i) {
        auto* agent_acceleration_edge = new EdgeAccelerationHolonomic;
        agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(i));
        agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
        agent_acceleration_edge->setVertex(2, agent_teb.PoseVertex(i + 2));
        agent_acceleration_edge->setVertex(3, agent_teb.TimeDiffVertex(i));
        agent_acceleration_edge->setVertex(4, agent_teb.TimeDiffVertex(i + 1));
        agent_acceleration_edge->setInformation(information);
        agent_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(agent_acceleration_edge);
      }

      // check if a goal velocity should be taken into accound
      if (vel_goal_.first) {
        auto* agent_acceleration_edge = new EdgeAccelerationHolonomicGoal;
        agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(n - 2));
        agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(n - 1));
        agent_acceleration_edge->setVertex(2, agent_teb.TimeDiffVertex(agent_teb.sizeTimeDiffs() - 1));
        agent_acceleration_edge->setGoalVelocity(agents_vel_goal_[agent_it].second);
        agent_acceleration_edge->setInformation(information);
        agent_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(agent_acceleration_edge);
      }
    }
  }
}

void HATebOptimalPlanner::AddEdgesTimeOptimal() {
  if (cfg_->optim.weight_optimaltime == 0) {  // if weight equals zero skip adding edges!
    return;
  }

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i = 0; i < teb_.sizeTimeDiffs(); ++i) {
    auto* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0, teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setHATebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void HATebOptimalPlanner::AddEdgesTimeOptimalForAgents() {
  if (cfg_->optim.weight_agent_optimaltime == 0) {
    return;
  }

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_agent_optimaltime);

  for (auto& agent_teb_kv : agents_tebs_map_) {
    auto& agent_teb = agent_teb_kv.second;

    std::size_t no_time_diffs(agent_teb.sizeTimeDiffs());
    for (std::size_t i = 0; i < no_time_diffs; ++i) {
      auto* timeoptimal_edge = new EdgeTimeOptimal;
      timeoptimal_edge->setVertex(0, agent_teb.TimeDiffVertex(i));
      timeoptimal_edge->setInformation(information);
      timeoptimal_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(timeoptimal_edge);
    }
  }
}

void HATebOptimalPlanner::AddEdgesShortestPath() {
  if (cfg_->optim.weight_shortest_path == 0) {  // if weight equals zero skip adding edges!
    return;
  }

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i = 0; i < teb_.sizePoses() - 1; ++i) {
    auto* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0, teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1, teb_.PoseVertex(i + 1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setHATebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}

void HATebOptimalPlanner::AddEdgesKinematicsDiffDrive() {
  if (cfg_->optim.weight_kinematics_nh == 0 && cfg_->optim.weight_kinematics_forward_drive == 0) {  // if weight equals zero skip adding edges!
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;

  for (int i = 0; i < teb_.sizePoses() - 1; i++)  // ignore twiced start only
  {
    auto* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0, teb_.PoseVertex(i));
    kinematics_edge->setVertex(1, teb_.PoseVertex(i + 1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setHATebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void HATebOptimalPlanner::AddEdgesKinematicsDiffDriveForAgents() {
  if (cfg_->optim.weight_kinematics_nh == 0 && cfg_->optim.weight_kinematics_forward_drive == 0) {  // if weight equals zero skip adding edges!
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;

  for (auto& agent_teb_kv : agents_tebs_map_) {
    auto& agent_teb = agent_teb_kv.second;
    for (unsigned int i = 0; i < agent_teb.sizePoses() - 1; i++) {
      auto* kinematics_edge = new EdgeKinematicsDiffDrive;
      kinematics_edge->setVertex(0, agent_teb.PoseVertex(i));
      kinematics_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
      kinematics_edge->setInformation(information_kinematics);
      kinematics_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(kinematics_edge);
    }
  }
}

void HATebOptimalPlanner::AddEdgesKinematicsCarlike() {
  if (cfg_->optim.weight_kinematics_nh == 0 && cfg_->optim.weight_kinematics_turning_radius == 0) {  // if weight equals zero skip adding edges!
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;

  for (int i = 0; i < teb_.sizePoses() - 1; i++)  // ignore twiced start only
  {
    auto* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0, teb_.PoseVertex(i));
    kinematics_edge->setVertex(1, teb_.PoseVertex(i + 1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setHATebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void HATebOptimalPlanner::AddEdgesKinematicsCarlikeForAgents() {
  if (cfg_->optim.weight_kinematics_nh == 0 && cfg_->optim.weight_kinematics_turning_radius == 0) {  // if weight equals zero skip adding edges!
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;

  for (auto& agent_teb_kv : agents_tebs_map_) {
    auto& agent_teb = agent_teb_kv.second;
    for (int i = 0; i < agent_teb.sizePoses() - 1; i++)  // ignore twiced start only
    {
      auto* kinematics_edge = new EdgeKinematicsCarlike;
      kinematics_edge->setVertex(0, agent_teb.PoseVertex(i));
      kinematics_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
      kinematics_edge->setInformation(information_kinematics);
      kinematics_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(kinematics_edge);
    }
  }
}

void HATebOptimalPlanner::AddEdgesPreferRotDir() {
  // TODO(roesmann): Note, these edges can result in odd predictions, in
  // particular
  //                we can observe a substantional mismatch between open- and
  //                closed-loop planning leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation
  //                recovery: Activating the edge for a short time period might
  //                not be crucial and could move the robot to a new
  //                oscillation-free state. This needs to be analyzed in more
  //                detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir == 0) {  // if weight equals zero skip adding edges!
    return;
  }

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left) {
    RCLCPP_WARN(node_->get_logger(), "HATebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 1, 1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);

  for (int i = 0; i < teb_.sizePoses() - 1 && i < 3; ++i)  // currently: apply to first 3 rotations
  {
    auto* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0, teb_.PoseVertex(i));
    rotdir_edge->setVertex(1, teb_.PoseVertex(i + 1));
    rotdir_edge->setInformation(information_rotdir);

    if (prefer_rotdir_ == RotType::left) {
      rotdir_edge->preferLeft();
    } else if (prefer_rotdir_ == RotType::right) {
      rotdir_edge->preferRight();
    }

    optimizer_->addEdge(rotdir_edge);
  }
}

void HATebOptimalPlanner::AddEdgesAgentRobotSafety() {
  auto robot_teb_size = teb_.sizePoses();

  if (current_agent_robot_min_dist_ < 2.0) {
    for (auto& agent_teb_kv : agents_tebs_map_) {
      auto& agent_teb = agent_teb_kv.second;

      for (unsigned int i = 0; (i < agent_teb.sizePoses()) && (i < robot_teb_size); i++) {
        Eigen::Matrix<double, 1, 1> information_agent_robot;
        information_agent_robot.fill(cfg_->optim.weight_agent_robot_safety);
        auto* agent_robot_safety_edge = new EdgeAgentRobotSafety;
        agent_robot_safety_edge->setVertex(0, teb_.PoseVertex(i));
        agent_robot_safety_edge->setVertex(1, agent_teb.PoseVertex(i));
        agent_robot_safety_edge->setInformation(information_agent_robot);
        agent_robot_safety_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(agent_robot_safety_edge);
      }
    }
  }
}

void HATebOptimalPlanner::AddEdgesAgentAgentSafety() {
  for (auto oi = agents_tebs_map_.begin(); oi != agents_tebs_map_.end();) {
    auto& agent1_teb = oi->second;
    for (auto ii = ++oi; ii != agents_tebs_map_.end(); ii++) {
      auto& agent2_teb = ii->second;

      for (unsigned int k = 0; (k < agent1_teb.sizePoses()) && (k < agent2_teb.sizePoses()); k++) {
        Eigen::Matrix<double, 1, 1> information_agent_agent;
        information_agent_agent.fill(cfg_->optim.weight_agent_agent_safety);

        auto* agent_agent_safety_edge = new EdgeAgentAgentSafety;
        agent_agent_safety_edge->setVertex(0, agent1_teb.PoseVertex(k));
        agent_agent_safety_edge->setVertex(1, agent2_teb.PoseVertex(k));
        agent_agent_safety_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(agent_agent_safety_edge);
      }
    }
  }
}

void HATebOptimalPlanner::AddEdgesAgentRobotRelVelocity() {
  Eigen::Matrix<double, 1, 1> information_agent_robot_rel_vel;
  information_agent_robot_rel_vel.fill(cfg_->optim.weight_agent_robot_rel_vel);

  auto robot_teb_size = teb_.sizePoses();
  for (auto& agent_teb_kv : agents_tebs_map_) {
    auto& agent_teb = agent_teb_kv.second;

    size_t agent_teb_size = agent_teb.sizePoses();
    for (unsigned int i = 0; (i < agent_teb_size - 1) && (i < robot_teb_size - 1); i++) {
      auto* agent_robot_rel_vel_edge = new EdgeAgentRobotRelVelocity;
      agent_robot_rel_vel_edge->setVertex(0, teb_.PoseVertex(i));
      agent_robot_rel_vel_edge->setVertex(1, teb_.PoseVertex(i + 1));
      agent_robot_rel_vel_edge->setVertex(2, teb_.TimeDiffVertex(i));
      agent_robot_rel_vel_edge->setVertex(3, agent_teb.PoseVertex(i));
      agent_robot_rel_vel_edge->setVertex(4, agent_teb.PoseVertex(i + 1));
      agent_robot_rel_vel_edge->setVertex(5, agent_teb.TimeDiffVertex(i));
      agent_robot_rel_vel_edge->setInformation(information_agent_robot_rel_vel);
      agent_robot_rel_vel_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(agent_robot_rel_vel_edge);
    }
  }
}

void HATebOptimalPlanner::AddEdgesAgentRobotVisibility() {
  auto robot_teb_size = teb_.sizePoses();

  for (auto& agent_teb_kv : agents_tebs_map_) {
    auto& agent_teb = agent_teb_kv.second;

    for (unsigned int i = 0; (i < agent_teb.sizePoses()) && (i < robot_teb_size); i++) {
      Eigen::Matrix<double, 1, 1> information_agent_robot;
      information_agent_robot.fill(cfg_->optim.weight_agent_robot_visibility);

      auto* agent_robot_visibility_edge = new EdgeAgentRobotVisibility;
      agent_robot_visibility_edge->setVertex(0, teb_.PoseVertex(i));
      agent_robot_visibility_edge->setVertex(1, agent_teb.PoseVertex(i));
      agent_robot_visibility_edge->setInformation(information_agent_robot);
      agent_robot_visibility_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(agent_robot_visibility_edge);
    }
  }
}

void HATebOptimalPlanner::AddEdgesStaticAgentVisibility() {
  auto robot_teb_size = teb_.sizePoses();

  // for (auto &agent_teb_kv : agents_tebs_map_) {
  //     auto &agent_teb = agent_teb_kv.second;
  for (auto& agent : static_agents_) {
    PoseSE2 agent_pose(agent);
    for (unsigned int i = 0; i < robot_teb_size; i++) {
      Eigen::Matrix<double, 1, 1> information_agent_robot;
      information_agent_robot.fill(cfg_->optim.weight_agent_robot_visibility);

      auto* static_agent_visibility_edge = new EdgeStaticAgentVisibility;
      static_agent_visibility_edge->setVertex(0, teb_.PoseVertex(i));
      static_agent_visibility_edge->setInformation(information_agent_robot);
      static_agent_visibility_edge->setParameters(*cfg_, agent_pose);
      optimizer_->addEdge(static_agent_visibility_edge);
    }
  }
}

void HATebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost, hateb_local_planner::OptimizationCostArray* op_costs) {
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);

  if (optimizer_->edges().empty() && optimizer_->vertices().empty()) {
    // here the graph is build again, for time efficiency make sure to call this
    // function between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();
    optimizer_->initializeOptimization();
  } else {
    graph_exist_flag = true;
  }

  optimizer_->computeInitialGuess();

  cost_ = 0;

  if (alternative_time_cost) {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for
    // similar TEBs, since we are using an AutoResize Function with hysteresis.
  }

  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  int i = 0;
  for (auto* it : optimizer_->activeEdges()) {
    i++;
    double cur_cost = it->chi2();

    if (dynamic_cast<EdgeObstacle*>(it) != nullptr || dynamic_cast<EdgeInflatedObstacle*>(it) != nullptr || dynamic_cast<EdgeDynamicObstacle*>(it) != nullptr) {
      cur_cost *= obst_cost_scale;
    } else if (dynamic_cast<EdgeViaPoint*>(it) != nullptr) {
      cur_cost *= viapoint_cost_scale;
    } else if (dynamic_cast<EdgeTimeOptimal*>(it) != nullptr && alternative_time_cost) {
      continue;  // skip these edges if alternative_time_cost is active
    }

    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag) {
    clearGraph();
  }
}

void HATebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const {
  if (dt == 0) {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }

  Eigen::Vector2d delta_s = pose2.position() - pose1.position();

  if (cfg_->robot.max_vel_y == 0)  // nonholonomic robot
  {
    Eigen::Vector2d conf1dir(cos(pose1.theta()), sin(pose1.theta()));
    // translational velocity
    double dir = delta_s.dot(conf1dir);
    vx = static_cast<double>(g2o::sign(dir)) * delta_s.norm() / dt;
    vy = 0;
  }

  else  // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx = (cos_theta1 * delta_s.x()) + (sin_theta1 * delta_s.y());
    double p1_dy = (-sin_theta1 * delta_s.x()) + (cos_theta1 * delta_s.y());
    vx = p1_dx / dt;
    vy = p1_dy / dt;
  }

  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff / dt;
}

bool HATebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses, double dt_ref) const {
  if (teb_.sizePoses() < 2) {
    RCLCPP_ERROR(node_->get_logger(), "HATebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  look_ahead_poses = std::max(1, std::min(look_ahead_poses, teb_.sizePoses() - 1));
  double dt = 0.0;
  for (int counter = 0; counter < look_ahead_poses; ++counter) {
    dt += teb_.TimeDiff(counter);
    if (dt >= dt_ref * look_ahead_poses) {
      look_ahead_poses = counter + 1;
      break;
    }
  }

  if (dt <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "HATebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  // Get velocity from the first two configurations
  extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);

  return true;
}

void HATebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::msg::Twist>& velocity_profile) const {
  int n = teb_.sizePoses();
  velocity_profile.resize(n + 1);

  // start velocity
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;

  for (int i = 1; i < n; ++i) {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i - 1), teb_.Pose(i), teb_.TimeDiff(i - 1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }

  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

cohan_msgs::msg::Trajectory HATebOptimalPlanner::getFullTrajectory() const {
  int n = teb_.sizePoses();
  cohan_msgs::msg::Trajectory trajectory;

  if (n == 0) {
    return trajectory;
  }

  double curr_time = 0;

  // start
  cohan_msgs::msg::TrajectoryPoint start;
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start = rclcpp::Duration::from_seconds(curr_time);
  trajectory.points.push_back(start);

  curr_time += teb_.TimeDiff(0);

  // intermediate points
  for (int i = 1; i < n - 1; ++i) {
    cohan_msgs::msg::TrajectoryPoint point;
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x;
    double vel1_y;
    double vel2_x;
    double vel2_y;
    double omega1;
    double omega2;
    extractVelocity(teb_.Pose(i - 1), teb_.Pose(i), teb_.TimeDiff(i - 1), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i + 1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5 * (vel1_x + vel2_x);
    point.velocity.linear.y = 0.5 * (vel1_y + vel2_y);
    point.velocity.angular.z = 0.5 * (omega1 + omega2);
    point.time_from_start = rclcpp::Duration::from_seconds(curr_time);
    trajectory.points.push_back(point);
    curr_time += teb_.TimeDiff(i);
  }

  // goal
  cohan_msgs::msg::TrajectoryPoint goal;
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start = rclcpp::Duration::from_seconds(curr_time);
  trajectory.points.push_back(goal);

  return trajectory;
}

cohan_msgs::msg::Trajectory HATebOptimalPlanner::getFullAgentTrajectory(const uint64_t agent_id) {
  cohan_msgs::msg::Trajectory agent_trajectory;
  auto agent_teb_it = agents_tebs_map_.find(agent_id);
  if (agent_teb_it != agents_tebs_map_.end()) {
    auto& agent_teb = agent_teb_it->second;
    auto agent_teb_size = agent_teb.sizePoses();
    if (agent_teb_size < 3) {
      RCLCPP_WARN(node_->get_logger(), "TEB size is %d for agent %ld", agent_teb_size, agent_id);
      return agent_trajectory;
    }
    double curr_time = 0;

    // start
    cohan_msgs::msg::TrajectoryPoint start;
    agent_teb.Pose(0).toPoseMsg(start.pose);
    start.velocity.linear.z = 0;
    start.velocity.angular.x = start.velocity.angular.y = 0;
    start.velocity.linear.x = agents_vel_start_[agent_id].second.linear.x;
    start.velocity.linear.y = agents_vel_start_[agent_id].second.linear.y;
    start.velocity.angular.z = agents_vel_start_[agent_id].second.angular.z;
    start.time_from_start = rclcpp::Duration::from_seconds(curr_time);
    agent_trajectory.points.push_back(start);

    curr_time += agent_teb.TimeDiff(0);

    // intermediate points
    for (int i = 1; i < agent_teb_size - 1; ++i) {
      cohan_msgs::msg::TrajectoryPoint point;
      agent_teb.Pose(i).toPoseMsg(point.pose);
      point.velocity.linear.z = 0;
      point.velocity.angular.x = point.velocity.angular.y = 0;
      double vel1_x;
      double vel1_y;
      double vel2_x;
      double vel2_y;
      double omega1;
      double omega2;
      extractVelocity(agent_teb.Pose(i - 1), agent_teb.Pose(i), agent_teb.TimeDiff(i - 1), vel1_x, vel1_y, omega1);
      extractVelocity(agent_teb.Pose(i), agent_teb.Pose(i + 1), agent_teb.TimeDiff(i), vel2_x, vel2_y, omega2);
      point.velocity.linear.x = 0.5 * (vel1_x + vel2_x);
      point.velocity.linear.y = 0.5 * (vel1_y + vel2_y);
      point.velocity.angular.z = 0.5 * (omega1 + omega2);
      point.time_from_start = rclcpp::Duration::from_seconds(curr_time);
      agent_trajectory.points.push_back(point);

      curr_time += agent_teb.TimeDiff(i);
    }
    // goal
    cohan_msgs::msg::TrajectoryPoint goal;
    agent_teb.BackPose().toPoseMsg(goal.pose);
    goal.velocity.linear.z = 0;
    goal.velocity.angular.x = goal.velocity.angular.y = 0;
    goal.velocity.linear.x = agents_vel_goal_[agent_id].second.linear.x;
    goal.velocity.linear.y = agents_vel_goal_[agent_id].second.linear.y;
    goal.velocity.angular.z = agents_vel_goal_[agent_id].second.angular.z;
    goal.time_from_start = rclcpp::Duration::from_seconds(curr_time);
    agent_trajectory.points.push_back(goal);
  }
  return agent_trajectory;
}
bool HATebOptimalPlanner::isTrajectoryFeasible(nav2_costmap_2d::CostmapModel* costmap_model, const std::vector<geometry_msgs::msg::Point>& footprint_spec, double inscribed_radius,
                                               double circumscribed_radius, int look_ahead_idx) {
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses()) look_ahead_idx = teb().sizePoses() - 1;

  for (int i = 0; i <= look_ahead_idx; ++i) {
    if (costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) == -1) {
      if (visualization_) {
        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *robot_model_);
      }
      return false;
    }
    // Checks if the distance between two poses is higher than the robot radius
    // or the orientation diff is bigger than the specified threshold and
    // interpolates in that case. (if obstacles are pushing two consecutive
    // poses away, the center between two consecutive poses might coincide with
    // the obstacle ;-)!
    if (i < look_ahead_idx) {
      double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i + 1).theta()) - g2o::normalize_theta(teb().Pose(i).theta()));
      Eigen::Vector2d delta_dist = teb().Pose(i + 1).position() - teb().Pose(i).position();
      if (fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius) {
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        PoseSE2 intermediate_pose = teb().Pose(i);
        for (int step = 0; step < n_additional_samples; ++step) {
          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() + (delta_rot / (n_additional_samples + 1.0)));
          if (costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(), footprint_spec, inscribed_radius, circumscribed_radius) == -1) {
            if (visualization_) {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *robot_model_);
            }
            return false;
          }
        }
      }
    }
  }
  return true;
}

}  // namespace hateb_local_planner
