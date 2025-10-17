/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2022-2025 LAAS-CNRS
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Author: Phani Teja Singamaneni
 *********************************************************************************/

#ifndef INVISIBLE_HUMANS_DETECTION_HPP
#define INVISIBLE_HUMANS_DETECTION_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <cohan_msgs/msg/passage_type.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_helpers/parameters.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int8.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Default parameter values
#define DEFAULT_SAMPLES 1081
#define DEFAULT_RANGE_MIN 0.05
#define DEFAULT_RANGE_MAX 7.0
#define DEFAULT_ANGLE_MIN -2.358
#define DEFAULT_ANGLE_MAX 2.358
#define DEFAULT_SCAN_RESOLUTION 700
#define DEFAULT_PUBLISH_SCAN true
#define DEFAULT_HUMAN_RADIUS 0.31

namespace invisible_humans_detection {
using Coordinates = std::vector<std::pair<double, double>>;
using Point = std::pair<double, double>;

// Pattern 1 of ROS2 Nodes --> Inherit from rclcpp::Node (Good for Standalone nodes)

/**
 * @brief Class for scanning maps to detect invisible humans
 */
class InvHumansDetection : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for InvHumansDetection class
   */
  InvHumansDetection() : Node("inv_humans_detection") {}

  /**
   * @brief Initializes the InvHumansDetection node, setting up subscribers, publishers, and timers
   */
  void initialize();

  /**
   * @brief Destructor for InvHumansDetection class
   */
  ~InvHumansDetection() = default;

 private:
  /**
   * @brief Sets up parameter declarations and callback for parameter updates
   */
  void setupParameterCallback() {
    // Declare parameters using the generic helper
    param_helper_.declareStringParam("ns", "", "Namespace for multiple robots");
    param_helper_.declareIntParam("samples", DEFAULT_SAMPLES, 1, 10000, "Number of samples for scanning");
    param_helper_.declareFloatParam("range_min", DEFAULT_RANGE_MIN, 0.0, 10.0, "Minimum range for scanning");
    param_helper_.declareFloatParam("range_max", DEFAULT_RANGE_MAX, 0.1, 100.0, "Maximum range for scanning");
    param_helper_.declareFloatParam("angle_min", DEFAULT_ANGLE_MIN, -M_PI, M_PI, "Minimum angle for scanning (radians)");
    param_helper_.declareFloatParam("angle_max", DEFAULT_ANGLE_MAX, -M_PI, M_PI, "Maximum angle for scanning (radians)");
    param_helper_.declareIntParam("scan_resolution", DEFAULT_SCAN_RESOLUTION, 1, 10000, "Resolution of the scan");
    param_helper_.declareBoolParam("publish_scan", DEFAULT_PUBLISH_SCAN, "Whether to publish scan data");
    param_helper_.declareFloatParam("human_radius", DEFAULT_HUMAN_RADIUS, 0.1, 1.0, "Radius of a human for detection");

    // Set up parameter change callback with custom validation
    param_helper_.setupParameterCallback([this](const std::vector<rclcpp::Parameter>& params) -> bool {
      // Custom parameter validation logic for this specific node
      for (const auto& param : params) {
        const std::string& name = param.get_name();

        // Update internal variables when parameters change
        if (name == "ns")
          ns_ = param.as_string();
        else if (name == "samples")
          samples_ = param.as_int();
        else if (name == "range_min")
          range_min_ = param.as_double();
        else if (name == "range_max")
          range_max_ = param.as_double();
        else if (name == "angle_min")
          angle_min_ = param.as_double();
        else if (name == "angle_max")
          angle_max_ = param.as_double();
        else if (name == "scan_resolution")
          scan_resolution_ = param.as_int();
        else if (name == "publish_scan")
          publish_scan_ = param.as_bool();
        else if (name == "human_radius")
          human_radius_ = param.as_double();
      }
      return true;
    });

    // Load initial parameter values
    loadParameters();
  }

  /**
   * @brief Loads and initializes all parameters from ROS2 parameter server
   */
  void loadParameters() {
    // Get parameter values and store them in member variables
    ns_ = param_helper_.getParam<std::string>("ns", "");
    samples_ = param_helper_.getParam<int>("samples", DEFAULT_SAMPLES);
    range_min_ = param_helper_.getParam<double>("range_min", DEFAULT_RANGE_MIN);
    range_max_ = param_helper_.getParam<double>("range_max", DEFAULT_RANGE_MAX);
    angle_min_ = param_helper_.getParam<double>("angle_min", DEFAULT_ANGLE_MIN);
    angle_max_ = param_helper_.getParam<double>("angle_max", DEFAULT_ANGLE_MAX);
    scan_resolution_ = param_helper_.getParam<int>("scan_resolution", DEFAULT_SCAN_RESOLUTION);
    publish_scan_ = param_helper_.getParam<bool>("publish_scan", DEFAULT_PUBLISH_SCAN);
    human_radius_ = param_helper_.getParam<double>("human_radius", DEFAULT_HUMAN_RADIUS);
  }

  /**
   * @brief Callback for map updates
   * @param grid The occupancy grid message containing map data
   */
  void mapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr grid);

  /**
   * @brief Locates invisible humans based on the detected corner sets
   * @param c1 First set of corners
   * @param c2 Second set of corners
   * @param direction Directions vector (positive or negative to select the right or left point)
   * @param footprint_transform Transform of the robot footprint to the world frame
   * @return True if invisible humans are located successfully, false otherwise
   */
  bool locateInvHumans(Coordinates c1, Coordinates c2, std::vector<char> direction, geometry_msgs::msg::TransformStamped& footprint_transform);

  /**
   * @brief Detects occluded corners in the map where humans might be present
   */
  void detectOccludedCorners();

  /**
   * @brief Publishes detected invisible humans (corners, poses and obstacles message)
   * @param corners Detected corners
   * @param poses Detected poses
   * @param inv_humans Invisible humans info for Obstacle Msg
   */
  void publishInvisibleHumans(const geometry_msgs::msg::PoseArray& corners, const geometry_msgs::msg::PoseArray& poses, std::vector<std::vector<double>>& inv_humans);

  /**
   * @brief Detects different kinds of passages in the map
   * @param detections Array of detected invisible humans
   */
  void detectPassages(geometry_msgs::msg::PoseArray detections);

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

  // ROS2 helpers
  parameters::ParameterHelper param_helper_;  //!< Helper for managing parameters

  rclcpp::TimerBase::SharedPtr timer_;                                                           //!< Timer for periodically updating detection
  geometry_msgs::msg::PoseStamped robot_pose_;                                                   //!< Current pose of the robot
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                                   //!< Buffer for storing TF2 transformations
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                                      //!< TF2 transform listener
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;                        //!< Subscriber for receiving map updates
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;                           //!< Publisher for scan data
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_invis_human_viz_;       //!< Publisher for visualizing invisible humans
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr pub_invis_human_;  //!< Publisher for invisible human obstacle msg
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_invis_human_corners_;          //!< Publisher for invisible human corner data
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_invis_humans_pos_;             //!< Publisher for invisible human positions
  rclcpp::Publisher<cohan_msgs::msg::PassageType>::SharedPtr passage_detect_pub_;                //!< Publisher for detected passages
  nav_msgs::msg::OccupancyGrid map_;                                                             //!< Occupancy grid map data
  std::vector<float> ranges_;                                                                    //!< Ranges from sensor data
  std::vector<double> corner_ranges_;                                                            //!< Ranges for detected corners
  int samples_;                                                                                  //!< Number of samples for scanning
  int scan_resolution_;                                                                          //!< Resolution of the scan
  int size_x_;                                                                                   //!< Size of the map in the x-direction
  int size_y_;                                                                                   //!< Size of the map in the y-direction
  double origin_x_;                                                                              //!< Origin of the map in the x-direction
  double origin_y_;                                                                              //!< Origin of the map in the y-direction
  double resolution_;                                                                            //!< Resolution of the map
  double angle_min_;                                                                             //!< Minimum angle for scanning
  double angle_max_;                                                                             //!< Maximum angle for scanning
  double range_min_;                                                                             //!< Minimum range for scanning
  double range_max_;                                                                             //!< Maximum range for scanning
  sensor_msgs::msg::LaserScan scan_msg_;                                                         //!< Laser scan message
  bool publish_scan_;                                                                            //!< Flag to indicate whether to publish scan data
  double human_radius_;                                                                          //!< Radius of a human for detection
  std::string ns_;                                                                               //!< Namespace of the node
  Eigen::Vector2d robot_vec_;                                                                    //!< Unit vector in the direction of the robot
};

}  // namespace invisible_humans_detection

#endif  // INVISIBLE_HUMANS_DETECTION_HPP
