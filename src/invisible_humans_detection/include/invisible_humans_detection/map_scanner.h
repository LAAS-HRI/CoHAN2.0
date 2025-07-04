/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 LAAS/CNRS
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
#ifndef MAP_SCANNER_H
#define MAP_SCANNER_H
#include <cohan_msgs/PassageType.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cassert>
#include <cmath>

namespace invisible_humans_detection {
using Coordinates = std::vector<std::pair<double, double>>;
using Point = std::pair<double, double>;

/**
 * @brief Class for scanning maps to detect invisible humans
 */
class MapScanner {
 public:
  /**
   * @brief Default constructor for MapScanner class
   */
  MapScanner();

  /**
   * @brief Default destructor for MapScanner class
   */
  ~MapScanner();

  /**
   * @brief Initializes the MapScanner node and sets up ROS
   */
  void initialize();

 private:
  /**
   * @brief Loads ROS parameters from the node handle
   * @param private_nh Private node handle containing parameters
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& private_nh);

  /**
   * @brief Callback for map updates
   * @param grid The occupancy grid message containing map data
   */
  void mapCB(const nav_msgs::OccupancyGrid& grid);

  /**
   * @brief Locates invisible humans based on the detected corner sets
   * @param c1 First set of corners
   * @param c2 Second set of corners
   * @param direction Directions vector (positive or negative to select the right or left point)
   * @param footprint_transform Transform of the robot footprint to the world frame
   * @return True if invisible humans are located successfully, false otherwise
   */
  bool locateInvHumans(Coordinates c1, Coordinates c2, std::vector<char> direction, geometry_msgs::TransformStamped& footprint_transform);

  /**
   * @brief Detects occluded corners in the map where humans might be present
   * @param event Timer event triggering the detection
   */
  void detectOccludedCorners(const ros::TimerEvent& event);

  /**
   * @brief Publishes detected invisible humans (corners, poses and obstacles message)
   * @param corners Detected corners
   * @param poses Detected poses
   * @param inv_humans Invisible humans info for Obstacle Msg
   */
  void publishInvisibleHumans(const geometry_msgs::PoseArray& corners, const geometry_msgs::PoseArray& poses, std::vector<std::vector<double>>& inv_humans);

  /**
   * @brief Detects different kinds of passages in the map
   * @param detections Array of detected invisible humans
   */
  void detectPassages(geometry_msgs::PoseArray detections);

  /**
   * @brief Converts world coordinates to map coordinates
   * @param wx World x-coordinate
   * @param wy World y-coordinate
   * @param mx Map x-coordinate (output)
   * @param my Map y-coordinate (output)
   * @return True if conversion is successful, false otherwise
   */
  bool worldToMap(double wx, double wy, int& mx, int& my) const {
    if (wx < origin_x_ || wy < origin_y_) return false;

    mx = static_cast<int>((wx - origin_x_) / resolution_);
    my = static_cast<int>((wy - origin_y_) / resolution_);

    return mx < size_x_ && my < size_y_;
  }

  /**
   * @brief Gets the index of a map cell based on its coordinates
   * @param mx Map x-coordinate
   * @param my Map y-coordinate
   * @return Index of the map cell
   */
  unsigned int getIndex(unsigned int mx, unsigned int my) const { return (my * size_x_) + mx; }

  /**
   * @brief Calculates a point perpendicular to p1, p2 and passing through p3 on the left of \vec(p1p2)
   * @param p1 First point
   * @param p2 Second point
   * @param p3 Third point
   * @param dist Distance for calculation
   * @return Calculated point to the left of p3 at a given distance
   */
  static Point getLeftPoint(Point p1, Point p2, Point p3, double dist = 1) {
    double x = p2.first - p1.first;
    double y = p2.second - p1.second;
    double point_dist = std::hypot(x, y);

    Point p;
    p.first = p3.first - (dist * y / point_dist);
    p.second = p3.second + (dist * x / point_dist);
    return p;
  }

  /**
   * @brief Calculates a point perpendicular to p1, p2 and passing through p3 on the right of \vec(p1p2)
   * @param p1 First point
   * @param p2 Second point
   * @param p3 Third point
   * @param dist Distance for calculation
   * @return Calculated point to the right of p3 at a given distance
   */
  static Point getRightPoint(Point p1, Point p2, Point p3, double dist = 1) {
    double x = p2.first - p1.first;
    double y = p2.second - p1.second;
    double point_dist = std::hypot(x, y);

    Point p;
    p.first = p3.first + (dist * y / point_dist);
    p.second = p3.second - (dist * x / point_dist);
    return p;
  }

  /**
   * @brief Calculates points perpendicular to p1, p2 and passing through p2 on the right and left of \vec(p1p2)
   * @param p1 First point
   * @param p2 Second point
   * @param radius Radius of human for calculation
   * @return Vector containing the two calculated points
   */
  static std::vector<Point> getTwoPoints(Point p1, Point p2, double radius) {
    std::vector<Point> points;
    auto l_p = getLeftPoint(p1, p2, p2, radius);
    points.push_back(l_p);
    auto r_p = getRightPoint(p1, p2, p2, radius);
    points.push_back(r_p);
    return points;
  }

  ros::Timer get_robot_pose_;               //!< Timer for periodically updating the robot's pose
  geometry_msgs::PoseStamped robot_pose_;   //!< Current pose of the robot
  tf2_ros::Buffer tf_;                      //!< Buffer for storing TF2 transformations
  ros::Subscriber map_sub_;                 //!< Subscriber for receiving map updates
  ros::Publisher scan_pub_;                 //!< Publisher for scan data
  ros::Publisher pub_invis_human_viz_;      //!< Publisher for visualizing invisible humans
  ros::Publisher pub_invis_human_;          //!< Publisher for invisible human obstacle msg
  ros::Publisher pub_invis_human_corners_;  //!< Publisher for invisible human corner data
  ros::Publisher pub_invis_humans_pos_;     //!< Publisher for invisible human positions
  ros::Publisher passage_detect_pub_;       //!< Publisher for detected passages
  nav_msgs::OccupancyGrid map_;             //!< Occupancy grid map data
  std::vector<float> ranges_;               //!< Ranges from sensor data
  std::vector<double> corner_ranges_;       //!< Ranges for detected corners
  int samples_;                             //!< Number of samples for scanning
  int scan_resolution_;                     //!< Resolution of the scan
  int size_x_;                              //!< Size of the map in the x-direction
  int size_y_;                              //!< Size of the map in the y-direction
  double origin_x_;                         //!< Origin of the map in the x-direction
  double origin_y_;                         //!< Origin of the map in the y-direction
  double resolution_;                       //!< Resolution of the map
  double angle_min_;                        //!< Minimum angle for scanning
  double angle_max_;                        //!< Maximum angle for scanning
  double range_min_;                        //!< Minimum range for scanning
  double range_max_;                        //!< Maximum range for scanning
  sensor_msgs::LaserScan scan_msg_;         //!< Laser scan message
  bool publish_scan_;                       //!< Flag to indicate whether to publish scan data
  double human_radius_;                     //!< Radius of a human for detection
  std::string ns_;                          //!< Namespace of the node
  Eigen::Vector2d robot_vec_;               //!< Unit vector in the direction of the robot
};

}  // namespace invisible_humans_detection

#endif  // MAP_SCANNER_H
