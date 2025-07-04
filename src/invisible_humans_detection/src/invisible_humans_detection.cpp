/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2025 LAAS/CNRS
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
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 *********************************************************************/
#include <invisible_humans_detection/map_scanner.h>
#include <ros/console.h>
#define MAP_FRAME "map"
#define FOOTPRINT_FRAME "base_footprint"

namespace invisible_humans_detection {
MapScanner::MapScanner() { initialize(); }

MapScanner::~MapScanner() = default;

void MapScanner::initialize() {
  ros::NodeHandle nh("~/");
  // Load params
  loadRosParamFromNodeHandle(nh);
  // Initialize Tf listener
  tf2_ros::TransformListener tf_listener(tf_);

  // Timer for corner detection
  get_robot_pose_ = nh.createTimer(ros::Duration(0.02), &MapScanner::detectOccludedCorners, this);

  // Initialize Subscribers and Publishers
  map_sub_ = nh.subscribe("/map", 1, &MapScanner::mapCB, this);
  scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("map_scan", 1);
  pub_invis_human_viz_ = nh.advertise<visualization_msgs::MarkerArray>("invisible_humans_markers", 1);
  pub_invis_human_corners_ = nh.advertise<geometry_msgs::PoseArray>("invisible_humans_corners", 1);
  pub_invis_humans_pos_ = nh.advertise<geometry_msgs::PoseArray>("invisible_humans", 1);
  pub_invis_human_ = nh.advertise<costmap_converter::ObstacleArrayMsg>("invisible_humans_obs", 1);
  passage_detect_pub_ = nh.advertise<cohan_msgs::PassageType>("passage", 1);

  // Initialize laser scan msg
  scan_msg_.angle_min = angle_min_;
  scan_msg_.angle_max = angle_max_;
  scan_msg_.angle_increment = (angle_max_ - angle_min_) / samples_;
  scan_msg_.range_min = range_min_;
  scan_msg_.range_max = range_max_;

  ros::spin();
}

void MapScanner::loadRosParamFromNodeHandle(const ros::NodeHandle &private_nh) {
  // Name space param
  private_nh.param("ns", ns_, std::string(""));
  // Laser scan parameters (can be set using ros parameter server)
  private_nh.param("samples", samples_, 1081);
  private_nh.param("range_min", range_min_, 0.05);
  private_nh.param("range_max", range_max_, 7.0);
  private_nh.param("angle_min", angle_min_, -2.358);
  private_nh.param("angle_max", angle_max_, 2.358);
  private_nh.param("scan_resolution", scan_resolution_, 700);
  // Other params
  private_nh.param("publish_scan", publish_scan_, true);
  private_nh.param("human_radius", human_radius_, 0.31);
}

void MapScanner::mapCB(const nav_msgs::OccupancyGrid &grid) {
  // Get map data
  map_ = grid;
  origin_x_ = map_.info.origin.position.x;
  origin_y_ = map_.info.origin.position.y;
  resolution_ = map_.info.resolution;
  size_x_ = map_.info.width;
  size_y_ = map_.info.height;
}

void MapScanner::publishInvisibleHumans(const geometry_msgs::PoseArray &corners, const geometry_msgs::PoseArray &poses, std::vector<std::vector<double>> &inv_humans) {
  // Publish Poses
  pub_invis_humans_pos_.publish(poses);

  // Publish corners
  pub_invis_human_corners_.publish(corners);

  costmap_converter::ObstacleArrayMsg obstacle_msg;
  obstacle_msg.header.stamp = ros::Time::now();
  obstacle_msg.header.frame_id = MAP_FRAME;  // CHANGE HERE : odom / map
  int id = 0;
  for (const auto &human : inv_humans) {
    double yaw = atan2(human[3], human[2]);
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(0, 0, yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    double mid_scan = ranges_[ranges_.size() / 2];

    geometry_msgs::Point32 point;
    point.x = human[0];
    point.y = human[1];

    costmap_converter::ObstacleMsg obstacle;
    obstacle.radius = 0.07;
    obstacle.id = id;
    obstacle.polygon.points.push_back(point);
    obstacle.orientation = quaternion;
    obstacle.velocities.twist.linear.x = human[2];
    obstacle.velocities.twist.linear.y = human[3];
    obstacle_msg.obstacles.push_back(obstacle);
    id++;
  }
  // Publish Obstacle msg
  pub_invis_human_.publish(obstacle_msg);
}

void MapScanner::detectOccludedCorners(const ros::TimerEvent &event) {
  // Get Robot Pose
  geometry_msgs::TransformStamped footprint_transform;
  try {
    std::string base_frame = FOOTPRINT_FRAME;
    if (!ns_.empty()) {
      base_frame = ns_ + "/" + FOOTPRINT_FRAME;
    }
    scan_msg_.header.frame_id = base_frame;
    footprint_transform = tf_.lookupTransform(MAP_FRAME, base_frame, ros::Time(0), ros::Duration(1.0));

  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
  robot_pose_.header = footprint_transform.header;
  robot_pose_.pose.position.x = footprint_transform.transform.translation.x;
  robot_pose_.pose.position.y = footprint_transform.transform.translation.y;
  robot_pose_.pose.position.z = footprint_transform.transform.translation.z;
  robot_pose_.pose.orientation = footprint_transform.transform.rotation;
  auto theta = tf2::getYaw(robot_pose_.pose.orientation);

  double ang = angle_min_;
  double angle_increment = (angle_max_ - angle_min_) / samples_;
  double increment = range_max_ / scan_resolution_;

  robot_vec_ << cos(theta), sin(theta);
  ranges_.resize(samples_, 0.0);

  // Scan the map using a fake laser at robot's position
  for (int i = 0; i < samples_; i++) {
    if (map_.data.empty()) {
      continue;
    }
    double ray = range_min_;
    Eigen::Vector2d r_dir{(robot_vec_.x() * cos(ang)) - (robot_vec_.y() * sin(ang)), (+robot_vec_.x() * sin(ang)) + (robot_vec_.y() * cos(ang))};

    ranges_[i] = range_max_;

    for (int j = 0; j < scan_resolution_; j++) {
      auto rx = robot_pose_.pose.position.x + (ray * r_dir.x());
      auto ry = robot_pose_.pose.position.y + (ray * r_dir.y());
      int mx;
      int my;

      if (!worldToMap(rx, ry, mx, my)) {
        continue;
      }
      auto idx = getIndex(mx, my);

      if (static_cast<int>(map_.data[idx]) == 100) {
        ranges_[i] = ray;
        break;
      }
      ray += increment;
    }
    ang += angle_increment;
  }

  // Publish this scan if requried
  if (publish_scan_) {
    scan_msg_.ranges = ranges_;
    scan_pub_.publish(scan_msg_);
  }

  // The Corner detection part starts from here
  Coordinates corner_set1;
  Coordinates corner_set2;
  std::vector<char> dir;

  ang = angle_min_;
  for (int i = 0; i < samples_ - 1; i++) {
    if (fabs(ang) < M_PI / 2) {
      double current_x = ranges_[i] * cos(ang);
      double current_y = ranges_[i] * sin(ang);
      double next_x = ranges_[i + 1] * cos(ang + angle_increment);
      double next_y = ranges_[i + 1] * sin(ang + angle_increment);

      // Distance and range check
      double current_dist = std::hypot(current_x, current_y);
      double next_dist = std::hypot(next_x, next_y);

      double dist = std::hypot(next_x - current_x, next_y - current_y);
      double under_rad = std::min(current_dist, next_dist);

      // TODO: Update the magic numbers here --> make them parameters or fix them
      if (dist > 0.15 && under_rad <= 5.0 && (fabs(next_x - current_x) >= 0.5 || fabs(next_y - current_y) >= 0.5)) {
        if (current_dist < next_dist) {
          corner_set1.emplace_back(current_x, current_y);
          corner_set2.emplace_back(next_x, next_y);
          dir.push_back('p');
        } else {
          corner_set1.emplace_back(next_x, next_y);
          corner_set2.emplace_back(current_x, current_y);
          dir.push_back('n');
        }
        corner_ranges_.push_back(i);
      }
    }
    ang += angle_increment;
  }

  // Locate the invisible humans using the detected corners
  locateInvHumans(corner_set1, corner_set2, dir, footprint_transform);
}

bool MapScanner::locateInvHumans(Coordinates c1, Coordinates c2, std::vector<char> direction, geometry_msgs::TransformStamped &footprint_transform) {
  assert(c1.size() == c2.size());
  int n_corners = c1.size();

  double angle_increment = (angle_max_ - angle_min_) / samples_;

  // Initialize the necessary
  std::vector<Point> centers;
  std::vector<std::vector<double>> inv_humans;
  auto now = ros::Time::now();

  geometry_msgs::PoseArray corner_array;
  corner_array.header.stamp = now;
  corner_array.header.frame_id = MAP_FRAME;

  geometry_msgs::PoseArray inv_array;
  inv_array.header.stamp = now;
  inv_array.header.frame_id = MAP_FRAME;

  visualization_msgs::MarkerArray marker_array;
  int m_id = 0;

  tf2::Quaternion q(footprint_transform.transform.rotation.x, footprint_transform.transform.rotation.y, footprint_transform.transform.rotation.z, footprint_transform.transform.rotation.w);
  tf2::Vector3 p(footprint_transform.transform.translation.x, footprint_transform.transform.translation.y, footprint_transform.transform.translation.z);
  tf2::Transform transform(q, p);

  // Iterate through corners to find the invisible humans
  if (n_corners > 0) {
    for (int i = 0; i < n_corners; i++) {
      // Get the two vertices of the corners
      double x1 = c1[i].first;
      double y1 = c1[i].second;
      double x2 = c2[i].first;
      double y2 = c2[i].second;

      // Find the unit vector of two vertices
      double v_mag = std::hypot(x2 - x1, y2 - y1);
      double ux = (x2 - x1) / v_mag;
      double uy = (y2 - y1) / v_mag;

      // step size for increment
      double alp = 0.2;

      // Initialize variables and flags
      Point center = {0.0, 0.0};
      Point pt;
      tf2::Vector3 in_pose_l;
      tf2::Vector3 in_pose_r;
      tf2::Vector3 in_pose_mid;
      tf2::Vector3 robot_position;
      bool remove_detection = false;

      // Get the first point on \vec(X1X2)
      double xt = x1 + (human_radius_ * ux);
      double yt = y1 + (human_radius_ * uy);

      while (true) {
        // Find the point pt depeding on the direction of sequence of points
        if (direction[i] == 'p') {
          pt = getRightPoint(Point(x1, y1), Point(x2, y2), Point(xt, yt), human_radius_);
        } else if (direction[i] == 'n') {
          pt = getLeftPoint(Point(x1, y1), Point(x2, y2), Point(xt, yt), human_radius_);
        }

        // Calculate angle \beta
        auto angle = atan2(pt.second, pt.first);
        // Get the index of \beta in the laser ranges
        int ang_idx = static_cast<int>((angle - angle_min_) / angle_increment);

        // Check if the given point is inside or outside the polygon; continue if it is inside
        if (ranges_[ang_idx] > std::hypot(pt.first, pt.second)) {
          // increment the point
          xt = xt + alp * ux;
          yt = yt + alp * uy;
          continue;
        }

        center.first = (xt + pt.first) / 2;
        center.second = (yt + pt.second) / 2;
        bool overlap = false;
        int n_div = 10;

        for (int ri = 0; ri < n_div; ri++) {
          // Get left and right points of pt
          auto points = getTwoPoints(Point(xt, yt), pt, ((ri + 1) / n_div) * (1.5 * human_radius_));

          // Now do the transforms here
          in_pose_l = tf2::Vector3(points[0].first, points[0].second, 0.0);
          in_pose_l = transform * in_pose_l;
          in_pose_r = tf2::Vector3(points[1].first, points[1].second, 0.0);
          in_pose_r = transform * in_pose_r;
          in_pose_mid = tf2::Vector3(center.first, center.second, 0.0);
          in_pose_mid = transform * in_pose_mid;
          robot_position = tf2::Vector3(0., 0., 0.);
          robot_position = transform * robot_position;

          int mx_l;
          int my_l;
          int mx_r;
          int my_r;
          int mx_c;
          int my_c;
          worldToMap(in_pose_l.x(), in_pose_l.y(), mx_l, my_l);
          worldToMap(in_pose_r.x(), in_pose_r.y(), mx_r, my_r);
          worldToMap(in_pose_mid.x(), in_pose_mid.y(), mx_c, my_c);

          auto m_idx_l = getIndex(mx_l, my_l);
          auto m_idx_r = getIndex(mx_r, my_r);
          auto m_idx_c = getIndex(mx_c, my_c);
          int map_len = map_.data.size() - 1;

          // Check the index limits of the map
          if (m_idx_c > map_len || m_idx_l > map_len || m_idx_r > map_len) {
            remove_detection = true;
            break;
          }

          // Check if there is no overlap
          if (map_.data[m_idx_l] == 0 && map_.data[m_idx_r] == 0 && map_.data[m_idx_c] == 0) {
            continue;
          }
          // else overlap
          overlap = true;
          break;
        }

        // advance the search
        xt = xt + alp * ux;
        yt = yt + alp * uy;

        // Condition to reduce false detections
        if (std::hypot(xt - x1, yt - y1) >= std::hypot(x2 - x1, y2 - y1) || std::hypot(xt, yt) >= range_max_) {
          remove_detection = true;
          break;
        }

        // if there was a previous overlap continue to search
        if (overlap) {
          continue;
        }
        // else break search
        break;
      }

      // If the detection was flagged to remove, continue search and discard corners
      if (remove_detection) {
        continue;
      }

      // Point p = {pt.first, pt.second};
      // centers.push_back(p);
      // Now add the corners and invibsle humans
      // Corners
      auto corner_position = tf2::Vector3(x1, y1, 0.);
      corner_position = transform * corner_position;
      geometry_msgs::Pose corner_pose;
      corner_pose.position.x = corner_position.x();
      corner_pose.position.y = corner_position.y();
      corner_pose.orientation.w = 1;
      corner_array.poses.push_back(corner_pose);

      // Calculate velocity
      double vel_ux = robot_position.x() - in_pose_mid.x();
      double vel_uy = robot_position.y() - in_pose_mid.y();
      double vec_ang = atan2(vel_uy, vel_ux);

      // add inv human info for obstacle msg
      std::vector<double> info = {in_pose_mid.x(), in_pose_mid.y(), 1.5 * cos(vec_ang), 1.5 * sin(vec_ang)};
      inv_humans.push_back(info);

      // Fill the objects to publish the rviz markers
      double yaw = atan2(1.5 * sin(vec_ang), 1.5 * cos(vec_ang));
      tf2::Quaternion quaternion_tf2;
      quaternion_tf2.setRPY(0., 0., yaw);
      geometry_msgs::Quaternion q = tf2::toMsg(quaternion_tf2);

      geometry_msgs::PoseStamped inv_pose;
      inv_pose.pose.position.x = in_pose_mid.x();
      inv_pose.pose.position.y = in_pose_mid.y();
      inv_pose.pose.orientation = q;

      visualization_msgs::Marker arrow;
      arrow.header.frame_id = MAP_FRAME;
      arrow.id = m_id;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.pose.orientation = q;
      arrow.pose.position.x = in_pose_mid.x();
      arrow.pose.position.y = in_pose_mid.y();
      arrow.pose.position.z = 0.0;
      arrow.lifetime = ros::Duration(0.1);
      arrow.scale.x = 0.6;
      arrow.scale.y = 0.1;
      arrow.scale.z = 0.1;
      arrow.color.a = 1.0;
      arrow.color.b = 1.0;

      m_id += 1;

      visualization_msgs::Marker marker;
      marker.header.frame_id = MAP_FRAME;
      marker.id = m_id;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1;
      marker.pose.position.x = in_pose_mid.x();
      marker.pose.position.y = in_pose_mid.y();
      marker.pose.position.z = 0.6;
      marker.lifetime = ros::Duration(0.1);
      marker.scale.x = 0.6;
      marker.scale.y = 0.6;
      marker.scale.z = 1.2;
      marker.color.a = 1.0;
      marker.color.r = 1.0;

      m_id += 1;

      marker_array.markers.push_back(marker);
      marker_array.markers.push_back(arrow);
      // Publish the markers
      inv_array.poses.push_back(inv_pose.pose);
    }

    pub_invis_human_viz_.publish(marker_array);
    // Publish all data
    publishInvisibleHumans(corner_array, inv_array, inv_humans);
    // Detect the passages using these estimates for inv humans
    detectPassages(inv_array);
  }
  return true;
}

// TODO: Check this method: Make it configurable to different envs
void MapScanner::detectPassages(geometry_msgs::PoseArray detections) {
  cohan_msgs::PassageType psg_type;
  psg_type.type = cohan_msgs::PassageType::OPEN;

  if (!detections.poses.empty()) {
    std::map<double, int> dists;
    double dist = 999;
    double mid_scan = ranges_[ranges_.size() / 2];
    int i = 0;
    // Get the distances and indices of the inv humans from the robot
    for (auto &pose : detections.poses) {
      dist = std::hypot(pose.position.x - robot_pose_.pose.position.x, pose.position.y - robot_pose_.pose.position.y);
      dists[dist] = i;
      i++;
    }

    // Passage detection starts
    auto inv1 = dists.begin();
    if (dists.size() > 1) {
      auto inv2 = std::next(dists.begin(), 1);
      double seperation_dist =
          std::hypot(detections.poses[inv1->second].position.x - detections.poses[inv2->second].position.x, detections.poses[inv1->second].position.y - detections.poses[inv2->second].position.y);

      Eigen::Vector2d mid_point((detections.poses[inv1->second].position.x + detections.poses[inv2->second].position.x) / 2,
                                (detections.poses[inv1->second].position.y + detections.poses[inv2->second].position.y) / 2);

      Eigen::Vector2d robot_point(robot_pose_.pose.position.x, robot_pose_.pose.position.y);

      auto dectection_robot_dir = ((robot_point - mid_point).dot(robot_vec_)) / (robot_point - mid_point).norm();

      if (dectection_robot_dir < -0.9) {
        // Condition for door
        if (inv1->first < 2.0 && abs(inv1->first - inv2->first) < 0.1 && seperation_dist < 3.0 && seperation_dist > 0.6) {
          if (mid_scan > 1.33) {
            ROS_DEBUG("It's a door");
            psg_type.type = cohan_msgs::PassageType::DOOR;

          } else  // If there is not enough space to enter, it might be a pillar
          {
            ROS_DEBUG("It is a pillar");
            psg_type.type = cohan_msgs::PassageType::PILLAR;
          }
          // Neither, a possible passage (No need to switch to PASS THROUGH here)
          ROS_DEBUG("Possibility of door or narrow junction pass");
        }
      }
    }
    // Condition for a wall
    else if (inv1->first < 2.0 && ranges_[corner_ranges_[inv1->second]] < 3.0) {
      ROS_DEBUG("It is a wall");
      psg_type.type = cohan_msgs::PassageType::WALL;
    }
    passage_detect_pub_.publish(psg_type);
  } else {
    passage_detect_pub_.publish(psg_type);
    return;
  }
}

}  // namespace invisible_humans_detection

#if !defined(DOXYGEN_SHOULD_SKIP_THIS)
// ROS node for invisible humans detection
int main(int argc, char **argv) {
  ros::init(argc, argv, "map_scanner_node");
  invisible_humans_detection::MapScanner mp_scanner;
  return 0;
}
#endif