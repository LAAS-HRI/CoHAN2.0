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
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
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
 * Modified by: Phani Teja Singamaneni
 *********************************************************************/

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

// teb stuff
#include <hateb_local_planner/footprint_model.h>
#include <hateb_local_planner/hateb_config.h>
#include <hateb_local_planner/timed_elastic_band.h>

// ros stuff
#include <base_local_planner/goal_functions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/utility.hpp>

// messages
#include <cohan_msgs/msg/agent_path_array.hpp>
#include <cohan_msgs/msg/agent_time_to_goal.hpp>
#include <cohan_msgs/msg/agent_time_to_goal_array.hpp>
#include <cohan_msgs/msg/agent_trajectory_array.hpp>
#include <cohan_msgs/msg/crossing_info.hpp>
#include <cohan_msgs/msg/tracked_agents.hpp>
#include <cohan_msgs/msg/tracked_segment_type.hpp>
#include <cohan_msgs/msg/trajectory_point.hpp>
#include <cohan_msgs/msg/trajectory_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#define GLOBAL_PLAN_TOPIC "global_plan"
#define LOCAL_PLAN_TOPIC "local_plan"
#define LOCAL_TRAJ_TOPIC "local_traj"
#define LOCAL_PLAN_POSES_TOPIC "local_plan_poses"
#define LOCAL_PLAN_FP_POSES_TOPIC "local_plan_fp_poses"
#define AGENT_GLOBAL_PLANS_TOPIC "agents_global_plans"
#define AGENT_LOCAL_PLANS_TOPIC "agents_local_plans"
#define AGENT_LOCAL_TRAJS_TOPIC "agents_local_trajs"
#define AGENT_LOCAL_PLANS_POSES_TOPIC "agents_local_plans_poses"
#define AGENT_LOCAL_PLANS_FP_POSES_TOPIC "agents_local_plans_fp_poses"
#define ROBOT_FP_POSES_NS "robot_fp_poses"
#define AGENT_FP_POSES_NS "agents_fp_poses"
#define ROBOT_TRAJ_TIME_TOPIC "traj_time"
#define ROBOT_PATH_TIME_TOPIC "plan_time"
#define AGENT_TRAJS_TIME_TOPIC "agents_trajs_time"
#define AGENT_PATHS_TIME_TOPIC "agents_plans_time"
#define AGENT_MARKER_TOPIC "agent_marker"
#define AGENT_ARROW_TOPIC "agent_arrow"
#define ROBOT_NEXT_POSE_TOPIC "robot_next_pose"
#define AGENT_NEXT_POSE_TOPIC "agent_next_pose"
#define FEEDBACK_TOPIC "teb_feedback"
#define TEB_MARKER_TOPIC "teb_markers"
#define MODE_TEXT_TOPIC "mode_text"
#define CROSSING_POINT_TOPIC "crossing_points"
#define CROSSING_INFO_TOPIC "crossing_info"
#define TTG_TOPIC "time_to_goal"
#define TRACKED_AGENTS_SUB "/tracked_agents"
#define CLEARING_TIMER_DURATION 1.0  // seconds

namespace hateb_local_planner {

typedef struct {
  std::vector<geometry_msgs::msg::PoseStamped> plan_before;
  std::vector<cohan_msgs::msg::TrajectoryPoint> optimized_trajectory;
  std::vector<geometry_msgs::msg::PoseStamped> plan_after;
} PlanTrajCombined;

typedef struct {
  std::vector<geometry_msgs::msg::PoseStamped> plan_before;
  std::vector<geometry_msgs::msg::PoseStamped> plan_to_optimize;
  std::vector<geometry_msgs::msg::PoseStamped> plan_after;
} PlanCombined;

typedef struct {
  uint64_t id;
  std::vector<geometry_msgs::msg::PoseStamped> plan_before;
  std::vector<cohan_msgs::msg::TrajectoryPoint> optimized_trajectory;
  std::vector<geometry_msgs::msg::PoseStamped> plan_after;
} AgentPlanTrajCombined;

typedef struct {
  uint64_t id;
  std::vector<geometry_msgs::msg::PoseStamped> plan_before;
  std::vector<geometry_msgs::msg::PoseStamped> plan_to_optimize;
  std::vector<geometry_msgs::msg::PoseStamped> plan_after;
} AgentPlanCombined;

class TebOptimalPlanner;  //!< Forward Declaration

/**
 * @class TebVisualization
 * @brief Visualize stuff from the hateb_local_planner
 */
class TebVisualization {
 public:
  /**
   * @brief Default constructor
   * @remarks do not forget to call initialize()
   */
  TebVisualization();

  /**
   * @brief Constructor that initializes the class and registers topics
   * @param node shared pointer to rclcpp_lifecycle::LifecycleNode
   * @param cfg const reference to the HATebConfig class for parameters
   */
  TebVisualization(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::shared_ptr<HATebConfig> cfg);

  /**
   * @brief Initializes the class and registers topics.
   *
   * Call this function if only the default constructor has been called before.
   * @param node shared pointer to rclcpp_lifecycle::LifecycleNode
   * @param cfg const reference to the HATebConfig class for parameters
   */
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::shared_ptr<HATebConfig> cfg);

  /** @name Publish to topics */
  //@{

  /**
   * @brief Publish a given global plan to the ros topic \e ../../global_plan
   * @param global_plan Pose array describing the global plan
   */
  void publishGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan) const;

  /**
   * @brief Publish a given agent's global plan to the ros topic
   * @param agents_plans Vector comntaining the AgentPlanCombined
   */
  void publishAgentGlobalPlans(const std::vector<AgentPlanCombined>& agents_plans) const;

  /**
   * @brief Publish a given local plan to the ros topic \e ../../local_plan
   * @param local_plan Pose array describing the local plan
   */
  void publishLocalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& local_plan) const;

  /**
   * @brief Publish visualization markers for tracked agents in the environment
   * @param agents Pointer to the tracked agents message containing agent positions, velocities, and properties
   */
  void publishTrackedAgents(cohan_msgs::msg::TrackedAgents::ConstSharedPtr agents);

  /**
   * @brief Publish Timed_Elastic_Band related stuff (local plan, pose sequence).
   *
   * Given a Timed_Elastic_Band instance, publish the local plan to  \e ../../local_plan
   * and the pose sequence to  \e ../../teb_poses.
   * @param teb const reference to a Timed_Elastic_Band
   */
  void publishLocalPlanAndPoses(const TimedElasticBand& teb, const BaseFootprintModel& robot_model, double fp_size, const std_msgs::msg::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));

  /**
   * @brief Publish local plans and pose sequences for multiple agents
   *
   * Similar to publishLocalPlanAndPoses but handles multiple agents. Publishes local plans and
   * pose sequences for each agent in the map to their respective ROS topics.
   * @param agents_tebs_map Map of agent IDs to their Timed Elastic Band trajectories
   * @param agent_model The footprint model used for all agents
   * @param fp_size Size of the footprint for visualization
   * @param color Color to use for visualization markers (RGBA)
   */
  void publishAgentLocalPlansAndPoses(const std::map<uint64_t, TimedElasticBand>& agents_tebs_map, const BaseFootprintModel& agent_model, double fp_size,
                                      const std_msgs::msg::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));

  /**
   * @brief Publish a complete trajectory including plan segments before and after optimization
   *
   * Publishes visualization of the complete trajectory which includes:
   * - The plan segment before optimization
   * - The optimized trajectory
   * - The plan segment after optimization
   * @param plan_traj_combined Combined plan and trajectory data structure containing all segments
   */
  void publishTrajectory(const PlanTrajCombined& plan_traj_combined);

  /**
   * @brief Publish trajectories for multiple agents
   *
   * Publishes visualization of complete trajectories for multiple agents, each including:
   * - The plan segment before optimization
   * - The optimized trajectory
   * - The plan segment after optimization
   * @param agents_plans_combined Vector of combined plan and trajectory data for each agent
   */
  void publishAgentTrajectories(const std::vector<AgentPlanTrajCombined>& agents_plans_combined);

  /**
   * @brief Publish the visualization of the robot model
   *
   * @param current_pose Current pose of the robot
   * @param robot_model Subclass of BaseFootprintModel
   * @param ns Namespace for the marker objects
   * @param color Color of the footprint
   */
  void publishRobotFootprintModel(const PoseSE2& current_pose, const BaseFootprintModel& robot_model, const std::string& ns = "RobotFootprintModel",
                                  const std_msgs::msg::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));

  /**
   * @brief Publish the robot footprints related to infeasible poses
   *
   * @param current_pose Current pose of the robot
   * @param robot_model Subclass of BaseFootprintModel
   */
  void publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseFootprintModel& robot_model);

  /**
   * @brief Publish obstacle positions to the ros topic \e ../../teb_markers
   * @param obstacles Obstacle container
   */
  void publishObstacles(const ObstContainer& obstacles) const;

  /**
   * @brief Publish via-points to the ros topic \e ../../teb_markers
   * @param via_points via-point container
   */
  void publishViaPoints(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns = "ViaPoints") const;

  /**
   * @brief Publish a boost::adjacency_list (boost's graph datatype) via markers.
   * @remarks Make sure that vertices of the graph contain a member \c pos as \c Eigen::Vector2d type
   *	      to query metric position values.
   * @param graph Const reference to the boost::adjacency_list (graph)
   * @param ns_prefix Namespace prefix for the marker objects (the strings "Edges" and "Vertices" will be appended)
   * @tparam GraphType boost::graph object in which vertices has the field/member \c pos.
   */
  template <typename GraphType>
  void publishGraph(const GraphType& graph, const std::string& ns_prefix = "Graph");

  /**
   * @brief Publish multiple 2D paths (each path given as a point sequence) from a container class.
   *
   * Provide a std::vector< std::vector< T > > in which T.x() and T.y() exist
   * and std::vector could be individually substituded by std::list / std::deque /...
   *
   * A common point-type for object T could be Eigen::Vector2d.
   *
   * T could be also a raw pointer std::vector< std::vector< T* > >.
   *
   * @code
   * 	typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > PathType; // could be a list or deque as well ...
   *    std::vector<PathType> path_container(2); // init 2 empty paths; the container could be a list or deque as well ...
   * 	// Fill path_container.at(0) with Eigen::Vector2d elements, we skip that here
   * 	// Fill path_container.at(1) with Eigen::Vector2d elements, we skip that here
   *    publishPathContainer( path_container.begin(), path_container.end() );
   * @endcode
   *
   * @remarks Actually the underlying path does not necessarily need to be a Eigen::Vector2d sequence.
   *          Eigen::Vector2d can be replaced with any datatype that implement public x() and y() methods.\n
   * @param first Bidirectional iterator pointing to the begin of the path
   * @param last Bidirectional iterator pointing to the end of the path
   * @param ns Namespace for the marker objects (the strings "Edges" and "Vertices" will be appended)
   * @tparam BidirIter Bidirectional iterator to a 2D path (sequence of Eigen::Vector2d elements) in a container
   */
  template <typename BidirIter>
  void publishPathContainer(BidirIter first, BidirIter last, const std::string& ns = "PathContainer");

  /**
   * @brief Publish multiple Tebs from a container class (publish as marker message).
   *
   * @param teb_planner Container of boost::shared_ptr< TebOptPlannerPtr >
   * @param ns Namespace for the marker objects
   */
  void publishTebContainer(const std::vector<boost::shared_ptr<TebOptimalPlanner> >& teb_planner, const std::string& ns = "TebContainer");

  /**
   * @brief Publish a feedback message (multiple trajectory version)
   *
   * The feedback message contains the all planned trajectory candidates (e.g. if planning in distinctive topologies is turned on).
   * Each trajectory is composed of the sequence of poses, the velocity profile and temporal information.
   * The feedback message also contains a list of active obstacles.
   * @param teb_planners container with multiple tebs (resp. their planner instances)
   * @param selected_trajectory_idx Idx of the currently selected trajectory in \c teb_planners
   * @param obstacles Container of obstacles
   */
  void publishFeedbackMessage(const std::vector<boost::shared_ptr<TebOptimalPlanner> >& teb_planners, unsigned int selected_trajectory_idx, const ObstContainer& obstacles);

  /**
   * @brief Publish a feedback message (single trajectory overload)
   *
   * The feedback message contains the planned trajectory
   * that is composed of the sequence of poses, the velocity profile and temporal information.
   * The feedback message also contains a list of active obstacles.
   * @param teb_planner the planning instance
   * @param obstacles Container of obstacles
   */
  void publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles);

  //@}

  /**
   * @brief Helper function to generate a color message from single values
   * @param a Alpha value
   * @param r Red value
   * @param g Green value
   * @param b Blue value
   * @return Color message
   */
  static std_msgs::msg::ColorRGBA toColorMsg(double a, double r, double g, double b);

  /**
   * @brief Publish crossing poses for the given robot and human TEBs
   * @param teb The TEB for the robot
   * @param agents_tebs_map A map of agent IDs to their respective TEBs
   */
  void publishCrossingPoses(const TimedElasticBand& teb, const std::map<uint64_t, TimedElasticBand>& agents_tebs_map);

  /**
   * @brief Set the color of a visualization marker (for fp_poses)
   * @param marker The marker to modify
   * @param itr The current index of the marker
   * @param n The total number of poses (length of trajectory)
   */
  static void setMarkerColour(visualization_msgs::msg::Marker& marker, double itr, double n);

  /**
   * @brief Publish the current mode of the planner
   * @param mode The current mode to publish
   */
  void publishMode(int mode);

 protected:
  /**
   * @brief Small helper function that checks if initialize() has been called and prints an error message if not.
   * @return \c true if not initialized, \c false if everything is ok
   */
  bool printErrorWhenNotInitialized() const;

  /**
   * @brief Callback function for the clearing timer that removes outdated visualization markers
   * @param event The timer event containing timing information about the callback
   * @details This protected method is called periodically to remove visualization markers that are
   *          no longer needed, preventing visualization clutter and memory buildup
   */
  void clearingTimerCB();

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;                                                               //!< Shared pointer to ROS2 node
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;                                             //!< Publisher for the global plan
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;                                              //!< Publisher for the local plan
  rclcpp::Publisher<cohan_msgs::msg::TrajectoryStamped>::SharedPtr local_traj_pub_;                               //!< Publisher for the local traj
  rclcpp::Publisher<cohan_msgs::msg::AgentPathArray>::SharedPtr agents_global_plans_pub_;                         //!< Publisher for the local plan
  rclcpp::Publisher<cohan_msgs::msg::AgentPathArray>::SharedPtr agents_local_plans_pub_;                          //!< Publisher for the local plan
  rclcpp::Publisher<cohan_msgs::msg::AgentTrajectoryArray>::SharedPtr agents_local_trajs_pub_;                    //!< Publisher for the agents local plans
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr teb_poses_pub_, teb_fp_poses_pub_;                  //!< Publisher for the trajectory pose sequence
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr agents_tebs_poses_pub_, agents_tebs_fp_poses_pub_;  //!< Publisher for the trajectory pose sequence
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr teb_marker_pub_;                                  //!< Publisher for visualization markers
  rclcpp::Publisher<hateb_local_planner::msg::FeedbackMsg>::SharedPtr feedback_pub_;                              //!< Publisher for optimization feedback messages
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mode_text_pub_;                                   //!< Publisher for current planner mode text visualization
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr robot_traj_time_pub_;                  //!< Publisher for robot's time to goal with optimized trajectory (until end of trajectory)
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr robot_path_time_pub_;                  //!< Publisher for robot's full time to goal (until goal, using path + traj)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_next_pose_pub_;         //!< Publisher for robot's predicted next pose
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr agent_next_pose_pub_;           //!< Publisher for agents' predicted next poses
  rclcpp::Publisher<cohan_msgs::msg::AgentTimeToGoalArray>::SharedPtr agent_trajs_time_pub_;  //!< Publisher for agents' time to goal with optimized trajectory (until end of trajectory)
  rclcpp::Publisher<cohan_msgs::msg::AgentTimeToGoalArray>::SharedPtr agent_paths_time_pub_;  //!< Publisher for agents' full time to goal (until goal, using path + traj)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr agent_marker_pub_;       //!< Publisher for agent visualization markers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr agent_arrow_pub_;        //!< Publisher for agent direction arrow markers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr crossing_point_pub_;
  rclcpp::Publisher<cohan_msgs::msg::CrossingInfo>::SharedPtr crossing_info_pub_;       //!< Publisher for crossing points and crossing info
  rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr tracked_agents_sub_;  //!< Subscriber for tracked agents data input
  std::vector<double> vel_robot_;                                                       //!< Store robot velocity history (for bar visualization)
  std::vector<double> vel_agent_;                                                       //!< Store agent velocity history (for bar visualization)
  tf2_ros::Buffer tf_;                                                                  // TF2 buffer for coordinate transformations
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;                             // TF2 transform listener for coordinate transformations
  rclcpp::Publisher<cohan_msgs::msg::AgentTimeToGoalArray>::SharedPtr ttg_pub_;         //!< Publisher for time-to-goal information
  std::string ns_;                                                                      //!< Name space of the robot
  std::string tracked_agents_sub_topic_;                                                //!< Tracked agents sub topic

  const std::shared_ptr<HATebConfig> cfg_;       //!< Config class that stores and manages all related parameters
  bool initialized_;                             //!< Keeps track about the correct initialization of this class
  rclcpp::TimerBase::SharedPtr clearing_timer_;  //!< Timer for periodically clearing old visualization markers

  bool last_publish_robot_global_plan_,          //!< Last publish state of robot's global plan
      last_publish_robot_local_plan_,            //!< Last publish state of robot's local plan
      last_publish_robot_local_plan_poses_,      //!< Last publish state of robot's local plan poses
      last_publish_robot_local_plan_fp_poses_,   //!< Last publish state of robot's local plan footprint poses
      last_publish_agents_global_plans_,         //!< Last publish state of agents' global plans
      last_publish_agents_local_plans_,          //!< Last publish state of agents' local plans
      last_publish_agents_local_plan_poses_,     //!< Last publish state of agents' local plan poses
      last_publish_agents_local_plan_fp_poses_;  //!< Last publish state of agents' local plan footprint poses

  mutable int last_robot_fp_poses_idx_,  //!< Index of last published robot footprint pose
      last_agent_fp_poses_idx_;          //!< Index of last published agent footprint pose

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of the TebVisualization
using TebVisualizationPtr = std::shared_ptr<TebVisualization>;

//! Abbrev. for shared instances of the TebVisualization (read-only)
using TebVisualizationConstPtr = std::shared_ptr<const TebVisualization>;

template <typename GraphType>
void TebVisualization::publishGraph(const GraphType& graph, const std::string& ns_prefix) {
  if (printErrorWhenNotInitialized()) return;

  using GraphVertexIterator = typename std::graph_traits<GraphType>::vertex_iterator;
  using GraphEdgeIterator = typename std::graph_traits<GraphType>::edge_iterator;

  // Visualize Edges
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = node_->now();
  marker.ns = ns_prefix + "Edges";
  marker.id = 0;
// #define TRIANGLE
#ifdef TRIANGLE
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
#else
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
#endif
  marker.action = visualization_msgs::msg::Marker::ADD;

  GraphEdgeIterator it_edge;
  GraphEdgeIterator end_edges;
  for (boost::tie(it_edge, end_edges) = boost::edges(graph); it_edge != end_edges; ++it_edge) {
#ifdef TRIANGLE
    geometry_msgs::msg::Point point_start1;
    point_start1.x = graph[boost::source(*it_edge, graph)].pos[0] + 0.05;
    point_start1.y = graph[boost::source(*it_edge, graph)].pos[1] - 0.05;
    point_start1.z = 0;
    marker.points.push_back(point_start1);
    geometry_msgs::msg::Point point_start2;
    point_start2.x = graph[boost::source(*it_edge, graph)].pos[0] - 0.05;
    point_start2.y = graph[boost::source(*it_edge, graph)].pos[1] + 0.05;
    point_start2.z = 0;
    marker.points.push_back(point_start2);

#else
    geometry_msgs::msg::Point point_start;
    point_start.x = graph[boost::source(*it_edge, graph)].pos[0];
    point_start.y = graph[boost::source(*it_edge, graph)].pos[1];
    point_start.z = 0;
    marker.points.push_back(point_start);
#endif
    geometry_msgs::msg::Point point_end;
    point_end.x = graph[boost::target(*it_edge, graph)].pos[0];
    point_end.y = graph[boost::target(*it_edge, graph)].pos[1];
    point_end.z = 0;
    marker.points.push_back(point_end);

    // add color
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = 0;
    color.g = 0;
    color.b = 1;
    marker.colors.push_back(color);
    marker.colors.push_back(color);
#ifdef TRIANGLE
    marker.colors.push_back(color);
#endif
  }

#ifdef TRIANGLE
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
#else
  marker.scale.x = 0.01;
#endif
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  // Now publish edge markers
  teb_marker_pub_->publish(marker);

  // Visualize vertices
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = node_->now();
  marker.ns = ns_prefix + "Vertices";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;

  GraphVertexIterator it_vert;
  GraphVertexIterator end_vert;
  for (boost::tie(it_vert, end_vert) = boost::vertices(graph); it_vert != end_vert; ++it_vert) {
    geometry_msgs::msg::Point point;
    point.x = graph[*it_vert].pos[0];
    point.y = graph[*it_vert].pos[1];
    point.z = 0;
    marker.points.push_back(point);
    // add color

    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    if (it_vert == end_vert - 1) {
      color.r = 1;
      color.g = 0;
      color.b = 0;
    } else {
      color.r = 0;
      color.g = 1;
      color.b = 0;
    }
    marker.colors.push_back(color);
  }
  // set first color (start vertix) to blue
  if (!marker.colors.empty()) {
    marker.colors.front().b = 1;
    marker.colors.front().g = 0;
  }

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  // Now publish vertex markers
  teb_marker_pub_->publish(marker);
}

template <typename BidirIter>
void TebVisualization::publishPathContainer(BidirIter first, BidirIter last, const std::string& ns) {
  if (printErrorWhenNotInitialized()) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = node_->now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

  using PathType = typename std::iterator_traits<BidirIter>::value_type;  // Get type of the path (point container)

  // Iterate through path container
  while (first != last) {
    // iterate single path points
    typename PathType::const_iterator it_point;
    typename PathType::const_iterator end_point;
    for (it_point = first->begin(), end_point = boost::prior(first->end()); it_point != end_point; ++it_point) {
      geometry_msgs::msg::Point point_start;
      point_start.x = get_const_reference(*it_point).x();
      point_start.y = get_const_reference(*it_point).y();
      point_start.z = 0;
      marker.points.push_back(point_start);

      geometry_msgs::msg::Point point_end;
      point_end.x = get_const_reference(*boost::next(it_point)).x();
      point_end.y = get_const_reference(*boost::next(it_point)).y();
      point_end.z = 0;
      marker.points.push_back(point_end);
    }
    ++first;
  }
  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  teb_marker_pub_->publish(marker);
}

}  // namespace hateb_local_planner

#endif /* VISUALIZATION_H_ */
