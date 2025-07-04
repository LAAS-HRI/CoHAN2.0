#include <agent_path_prediction/agents_class.h>

#include <numeric>

// Configuarable parameters
#define AGENTS_SUB_TOPIC "/tracked_agents"
#define WINDOW_MOVING_AVG 5
#define HUM_RADIUS 0.3
#define ROBOT_RADIUS 0.35
#define PLANNING_RADIUS 10.0
#define BASE_LINK_FRAME "base_link"
#define MAP_FRAME "map"
#define ODOM_FRAME "odom"

namespace agents {

Agents::Agents() : initialized_(false) {
  // Throw error on wrong initialization
  ROS_ERROR("The Agents class needs tf2_ros::Buffer* and costmap_2d::Costmap2DROS *, and are not passed!");
}

Agents::Agents(tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) : initialized_(false) {
  // call the initialize class to start things
  if (!initialized_) {
    ros::NodeHandle private_nh("~");

    // Get params
    loadRosParamFromNodeHandle(private_nh);

    // Initialize the publisher
    agents_info_pub_ = private_nh.advertise<agent_path_prediction::AgentsInfo>("agentsInfo", 10);

    // Need to remap subscriber properly
    if (!ns_.empty()) {
      tracked_agents_sub_topic_ = "/" + ns_ + tracked_agents_sub_topic_;
    }

    // Subscribers
    tracked_agents_sub_ = private_nh.subscribe(tracked_agents_sub_topic_, 1, &Agents::trackedAgentsCB, this);

    // Initialize variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    stuck_ = false;
    stuck_agent_id_ = -1;
    initialized_ = true;
  }
}

void Agents::trackedAgentsCB(const cohan_msgs::TrackedAgents &tracked_agents) {
  tracked_agents_ = tracked_agents;

  // Msg for publishing agents info
  agent_path_prediction::AgentsInfo agents_info;

  geometry_msgs::TransformStamped transform_stamped;
  auto base_link = base_link_frame_;
  if (!ns_.empty()) {
    base_link = ns_ + "/" + base_link_frame_;
  }

  // Get the robot pose
  transform_stamped = tf_->lookupTransform(map_frame_, base_link, ros::Time(0), ros::Duration(0.5));
  auto xpos = transform_stamped.transform.translation.x;
  auto ypos = transform_stamped.transform.translation.y;
  auto ryaw = tf2::getYaw(transform_stamped.transform.rotation);
  Eigen::Vector2d robot_vec(std::cos(ryaw), std::sin(ryaw));
  geometry_msgs::Pose2D robot_pose;
  robot_pose.x = xpos;
  robot_pose.y = ypos;
  robot_pose.theta = ryaw;

  // Add robot pose to the info message
  agents_info.robot_pose = robot_pose;

  agent_vels_.clear();
  visible_agent_ids_.clear();
  agent_still_.clear();

  std::map<double, int> agent_dist_id_map;
  std::map<int, double> agents_radii;
  std::map<int, agent_path_prediction::HumanInfo> humans_info;

  for (auto &agent : tracked_agents_.agents) {
    agent_path_prediction::HumanInfo human_info;
    auto h_id = agent.track_id;
    human_info.id = h_id;
    human_info.name = agent.name;
    if (agent.type == 1) {
      agents_radii[h_id] = human_radius_;
    } else {
      agents_radii[h_id] = robot_radius_;
    }

    if (agents_states_.size() < tracked_agents_.agents.size()) {
      // Add the agent id and state
      human_info.state = agents::AgentState::NO_STATE;

      // Update the class variable
      if (agents_states_.find(h_id) == agents_states_.end()) {
        agents_states_[h_id] = agents::AgentState::NO_STATE;
      }

      std::vector<double> h_vels;
      agent_vels_[h_id] = h_vels;
      agent_nominal_vels_[h_id] = 0.0;
      geometry_msgs::Pose h_pose;
      agents_[h_id] = h_pose;
    }
    // double h_xpos, h_ypos;
    for (auto &segment : agent.segments) {
      if (segment.type == DEFAULT_AGENT_SEGMENT) {
        agents_[h_id] = segment.pose.pose;
        double h_xpos = segment.pose.pose.position.x;
        double h_ypos = segment.pose.pose.position.y;

        Eigen::Vector2d rh_vec(h_xpos - xpos, h_ypos - ypos);
        auto hr_dist = rh_vec.norm();
        // Add the agent distance to robot
        human_info.dist = hr_dist;
        if (hr_dist < planning_radius_ && rh_vec.dot(robot_vec) >= 0) {
          // Update the dist map
          agent_dist_id_map[hr_dist] = h_id;
        }

        agent_vels_[agent.track_id].push_back(std::hypot(segment.twist.twist.linear.x, segment.twist.twist.linear.y));

        // Human state update --> MOVING
        if ((abs(segment.twist.twist.linear.x) + abs(segment.twist.twist.linear.y) + abs(segment.twist.twist.angular.z)) > CALC_EPS) {
          if (agents_states_.find(h_id) != agents_states_.end() && agents_states_[h_id] != agents::AgentState::BLOCKED) {
            agents_states_[h_id] = agents::AgentState::MOVING;
            // Update the state info message
            human_info.state = agents::AgentState::MOVING;
          }
        }

        auto n = agent_vels_[h_id].size();
        float average_vel = 0.0F;
        if (n != 0) {
          average_vel = accumulate(agent_vels_[h_id].begin(), agent_vels_[h_id].end(), 0.0) / n;
        }
        agent_nominal_vels_[h_id] = average_vel;

        if (n == window_moving_avg_) {
          agent_vels_[h_id].erase(agent_vels_[h_id].begin());
        }

        // Check if the human is still or halted
        if (prev_agents_.find(h_id) != prev_agents_.end()) {
          double human_move_dist = std::hypot(h_xpos - prev_agents_[h_id].position.x, h_ypos - prev_agents_[h_id].position.y);
          if (human_move_dist < CALC_EPS && agents_states_.find(h_id) != agents_states_.end()) {
            agent_still_[h_id] = true;

            // Human state update --> STOPPED
            if (agents_states_[h_id] == agents::AgentState::STOPPED || agents_states_[h_id] == agents::AgentState::MOVING) {
              // Update the state info message
              agents_states_[h_id] = agents::AgentState::STOPPED;
              human_info.state = agents::AgentState::STOPPED;
            }
          } else {
            agent_still_[h_id] = false;
            agents_states_[h_id] = agents::AgentState::MOVING;
            human_info.state = agents::AgentState::MOVING;
          }
        }

        // Check if the human is stuck and update the angle for backoff recovery
        if (h_id == stuck_agent_id_) {
          stuck_ = hr_dist <= planning_radius_ && rh_vec.dot(robot_vec) >= 0;
        }

        // agents_info.humans.push_back(human_info);
        humans_info[h_id] = human_info;
      }
    }
  }
  prev_agents_ = agents_;

  ROS_INFO_ONCE("Number of agents_agents_info, %d ", (int)agents_.size());

  // Get the distance sorted list of visible ids
  visible_agent_ids_.clear();

  for (auto &dist_id_map : agent_dist_id_map) {
    visible_agent_ids_.push_back(dist_id_map.second);
  }

  std::vector<int> sorted_ids;
  if (use_simulated_fov_) {
    /**************** for a centralised perception ***************/
    sorted_ids = filterVisibleAgents(agents_, visible_agent_ids_, agents_radii, robot_pose);
  } else {
    sorted_ids = visible_agent_ids_;
  }

  agents_info.visible = sorted_ids;

  for (auto &f_id : sorted_ids) {
    if (agents_states_[f_id] == agents::AgentState::NO_STATE || agents_states_[f_id] == agents::AgentState::STATIC) {
      agents_states_[f_id] = agents::AgentState::STATIC;
      humans_info[f_id].state = agents::AgentState::STATIC;
    }

    if (agent_still_.find(f_id) != agent_still_.end()) {
      if (agent_still_[f_id]) {
        agents_info.still.push_back(f_id);
      } else {
        agents_info.moving.push_back(f_id);
      }
    }

    agents_info.humans.push_back(humans_info[f_id]);
  }

  // Safety step for agents if agent_layers is not added in local costmap
  // Adds a temporary costmap around the agents to let planner plan safe
  // trajectories

  if (planning_mode_ > 0) {
    for (int i = 0; i < sorted_ids.size() && i < agents_.size(); i++) {
      geometry_msgs::Point v1;
      geometry_msgs::Point v2;
      geometry_msgs::Point v3;
      geometry_msgs::Point v4;
      auto idx = sorted_ids[i];
      auto agent_radius = agents_radii[idx];
      v1.x = agents_[idx].position.x - agent_radius, v1.y = agents_[idx].position.y - agent_radius, v1.z = 0.0;
      v2.x = agents_[idx].position.x - agent_radius, v2.y = agents_[idx].position.y + agent_radius, v2.z = 0.0;
      v3.x = agents_[idx].position.x + agent_radius, v3.y = agents_[idx].position.y + agent_radius, v3.z = 0.0;
      v4.x = agents_[idx].position.x + agent_radius, v4.y = agents_[idx].position.y - agent_radius, v4.z = 0.0;

      std::vector<geometry_msgs::Point> agent_pos_costmap;

      transform_stamped = tf_->lookupTransform(odom_frame_, map_frame_, ros::Time(0), ros::Duration(0.5));
      tf2::doTransform(v1, v1, transform_stamped);
      tf2::doTransform(v2, v2, transform_stamped);
      tf2::doTransform(v3, v3, transform_stamped);
      tf2::doTransform(v4, v4, transform_stamped);

      agent_pos_costmap.push_back(v1);
      agent_pos_costmap.push_back(v2);
      agent_pos_costmap.push_back(v3);
      agent_pos_costmap.push_back(v4);

      bool set_success = false;
      set_success = costmap_->setConvexPolygonCost(agent_pos_costmap, COST_OBS);
    }
  }

  agents_info_pub_.publish(agents_info);
};  // namespace agents

std::vector<int> Agents::filterVisibleAgents(std::map<int, geometry_msgs::Pose> tr_agents, std::vector<int> sorted_ids, std::map<int, double> agents_radii, geometry_msgs::Pose2D robot_pose) {
  std::vector<int> filtered_ids;
  auto xpos = robot_pose.x;
  auto ypos = robot_pose.y;

  if (!stuck_) {
    int n = MAX_PTS;
    if (sorted_ids.size() >= AGENT_NUM_TH) {
      n = MIN_PTS;
    }
    for (auto &it : sorted_ids) {
      // Ray Tracing
      double tm_x = tr_agents[it].position.x;
      double tm_y = tr_agents[it].position.y;
      // Get the difference between poses along x and y
      auto dx = (tm_x - xpos);
      auto dy = (tm_y - ypos);
      // Define step size in each direction
      dx = dx / n;
      dy = dy / n;

      // Checking using raytracing
      bool cell_collision = false;
      double x = xpos;
      double y = ypos;

      for (int j = 0; j < n; j++) {
        unsigned int mx;
        unsigned int my;

        double check_rad = agents_radii[it] + 0.1 + inflation_radius_;

        if (sqrt(((x - tm_x) * (x - tm_x)) + ((y - tm_y) * (y - tm_y))) <= check_rad) {
          break;
        }
        if (costmap_->worldToMap(x, y, mx, my)) {
          auto cellcost = costmap_->getCost(mx, my);
          if ((int)(cellcost) > COST_MIN && (int)(cellcost) < COST_OBS) {
            cell_collision = true;
            break;
          }
          x += dx;
          y += dy;
        }
      }

      if (!cell_collision) {
        filtered_ids.push_back(it);
      }
    }
    return filtered_ids;

  } else {
    for (int it = 0; it < 2 && it < sorted_ids.size(); it++) {
      if (sorted_ids[it] == stuck_agent_id_) {
        filtered_ids.push_back(sorted_ids[it]);
        break;
      }
    }
  }
  return filtered_ids;
}

void Agents::resetAgents() {
  // Reset the variables
  agents_states_.clear();
  agent_nominal_vels_.clear();
  stuck_agent_id_ = -1;
  stuck_ = false;
}

void Agents::loadRosParamFromNodeHandle(const ros::NodeHandle &private_nh) {
  private_nh.param("ns", ns_, std::string(""));
  private_nh.param("local_costmap/inflater/inflation_radius", inflation_radius_, 0.0);
  private_nh.param("planning_mode", planning_mode_, 0);
  private_nh.param("use_simulated_fov", use_simulated_fov_, false);
  private_nh.param("window_moving_avg", window_moving_avg_, WINDOW_MOVING_AVG);
  private_nh.param("human_radius", human_radius_, HUM_RADIUS);
  private_nh.param("robot_radius", robot_radius_, ROBOT_RADIUS);
  private_nh.param("tracked_agents_sub_topic", tracked_agents_sub_topic_, std::string(AGENTS_SUB_TOPIC));
  private_nh.param("base_link_frame", base_link_frame_, std::string(BASE_LINK_FRAME));
  private_nh.param("map_frame", map_frame_, std::string(MAP_FRAME));
  private_nh.param("odom_frame", odom_frame_, std::string(ODOM_FRAME));
  private_nh.param("planning_radius", planning_radius_, PLANNING_RADIUS);
}

}  // namespace agents
