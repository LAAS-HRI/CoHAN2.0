/*********************************************************************
 * Modified by Phani Teja Singamaneni in 2021
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

#ifndef ROBOT_FOOTPRINT_MODEL_H
#define ROBOT_FOOTPRINT_MODEL_H

#include <hateb_local_planner/obstacles.h>
#include <hateb_local_planner/pose_se2.h>

#include <algorithm>
#include <utility>
#include <visualization_msgs/msg/marker.hpp>

namespace hateb_local_planner {

/**
 * @class BaseFootprintModel
 * @brief Abstract class that defines the interface for footprint/contour models
 *
 * The robot/human model class is currently used in optimization only, since
 * taking the navigation stack footprint into account might be
 * inefficient. The footprint is only used for checking feasibility.
 */
class BaseFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   */
  BaseFootprintModel() = default;

  /**
   * @brief Virtual destructor.
   */
  virtual ~BaseFootprintModel() = default;

  /**
   * @brief Calculate the distance between the robot/human and an obstacle
   * @param current_pose Current robot/human pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot/human
   */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const = 0;

  /**
   * @brief Estimate the distance between the robot/human and the predicted location of an obstacle at time t
   * @param current_pose robot/human pose, from which the distance to the obstacle is estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
   * @param t time, for which the predicted distance to the obstacle is calculated
   * @return Euclidean distance to the robot/human
   */
  virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const = 0;

  /**
   * @brief Visualize the robot/human using a markers
   *
   * Fill a marker message with all necessary information (type, pose, scale and color).
   * The header, namespace, id and marker lifetime will be overwritten.
   * @param current_pose Current robot/human pose
   * @param[out] markers container of marker messages describing the robot/human shape
   * @param color Color of the footprint
   */
  virtual void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const {}

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double getInscribedRadius() = 0;

  /**
   * @brief Compute the circumscribed radius of the footprint model
   * @return circumscribed radius
   */
  virtual double getCircumscribedRadius() const = 0;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared obstacle pointers
using FootprintModelPtr = std::shared_ptr<BaseFootprintModel>;
//! Abbrev. for shared obstacle const pointers
using FootprintModelConstPtr = std::shared_ptr<const BaseFootprintModel>;

/**
 * @class PointShape
 * @brief Class that defines a point-robot
 *
 * Instead of using a CircularFootprint this class might
 * be utitilzed and the robot/human radius can be added to the mininum distance
 * parameter. This avoids a subtraction of zero each time a distance is calculated.
 */
class PointFootprint : public BaseFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   */
  PointFootprint() = default;

  /**
   * @brief Default constructor of the abstract obstacle class
   * @param min_obstacle_dist Minimum obstacle distance
   */
  explicit PointFootprint(const double min_obstacle_dist) : min_obstacle_dist_(min_obstacle_dist) {}

  /**
   * @brief Virtual destructor.
   */
  ~PointFootprint() override = default;

  /**
   * @brief Calculate the distance between the robot/human and an obstacle
   * @param current_pose Current robot/human pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot/human
   */
  double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const override { return obstacle->getMinimumDistance(current_pose.position()); }

  /**
   * @brief Estimate the distance between the robot/human and the predicted location of an obstacle at time t
   * @param current_pose robot/human pose, from which the distance to the obstacle is estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
   * @param t time, for which the predicted distance to the obstacle is calculated
   * @return Euclidean distance to the robot/human
   */
  double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const override {
    return obstacle->getMinimumSpatioTemporalDistance(current_pose.position(), t);
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  double getInscribedRadius() override { return 0.0; }

  /**
   * @brief Compute the circumscribed radius of the footprint model
   * @return circumscribed radius
   */
  double getCircumscribedRadius() const override { return 0.0; }

  /**
   * @brief Visualize the robot/human using a markers
   *
   * Fill a marker message with all necessary information (type, pose, scale and color).
   * The header, namespace, id and marker lifetime will be overwritten.
   * @param current_pose Current robot/human pose
   * @param[out] markers container of marker messages describing the robot/human shape
   * @param color Color of the footprint
   */
  void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const override {
    // point footprint
    markers.emplace_back();
    visualization_msgs::msg::Marker& marker = markers.back();
    marker.type = visualization_msgs::msg::Marker::POINTS;
    current_pose.toPoseMsg(marker.pose);  // all points are transformed into the robot/human frame!
    marker.points.emplace_back();
    marker.scale.x = 0.025;
    marker.color = color;

    if (min_obstacle_dist_ <= 0) {
      return;
    }

    // footprint with min_obstacle_dist
    markers.emplace_back();
    visualization_msgs::msg::Marker& marker2 = markers.back();
    marker2.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker2.scale.x = 0.025;
    marker2.color = color;
    current_pose.toPoseMsg(marker2.pose);  // all points are transformed into the robot/human frame!

    const double n = 9;
    const double r = min_obstacle_dist_;
    for (double theta = 0; theta <= 2 * M_PI; theta += M_PI / n) {
      geometry_msgs::msg::Point pt;
      pt.x = r * cos(theta);
      pt.y = r * sin(theta);
      marker2.points.push_back(pt);
    }
  }

 private:
  const double min_obstacle_dist_ = 0.0;
};

/**
 * @class CircularFootprint
 * @brief Class that defines the a robot/human of circular shape
 */
class CircularFootprint : public BaseFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   * @param radius radius of the robot/human
   */
  CircularFootprint() = default;
  explicit CircularFootprint(double radius) : radius_(radius) {}

  /**
   * @brief Virtual destructor.
   */
  ~CircularFootprint() override = default;

  /**
   * @brief Set radius of the circular robot/human
   * @param radius radius of the robot/human
   */
  void setRadius(double radius) { radius_ = radius; }

  /**
   * @brief Calculate the distance between the robot/human and an obstacle
   * @param current_pose Current robot/human pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot/human
   */
  double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const override { return obstacle->getMinimumDistance(current_pose.position()) - radius_; }

  /**
   * @brief Estimate the distance between the robot/human and the predicted location of an obstacle at time t
   * @param current_pose robot/human pose, from which the distance to the obstacle is estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
   * @param t time, for which the predicted distance to the obstacle is calculated
   * @return Euclidean distance to the robot/human
   */
  double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const override {
    return obstacle->getMinimumSpatioTemporalDistance(current_pose.position(), t) - radius_;
  }

  /**
   * @brief Visualize the robot/human using a markers
   *
   * Fill a marker message with all necessary information (type, pose, scale and color).
   * The header, namespace, id and marker lifetime will be overwritten.
   * @param current_pose Current robot/human pose
   * @param[out] markers container of marker messages describing the robot/human shape
   * @param color Color of the footprint
   */
  void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const override {
    markers.resize(1);
    visualization_msgs::msg::Marker& marker = markers.back();
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    current_pose.toPoseMsg(marker.pose);
    marker.scale.x = marker.scale.y = 2 * radius_;  // scale = diameter
    marker.scale.z = 0.05;
    marker.color = color;
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  double getInscribedRadius() override { return radius_; }

  double getCircumscribedRadius() const override { return radius_; }

 private:
  double radius_;
};
using CircularFootprintPtr = std::shared_ptr<CircularFootprint>;

/**
 * @class TwoCirclesFootprint
 * @brief Class that approximates the robot/human with two shifted circles
 */
class TwoCirclesFootprint : public BaseFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   * @param front_offset shift the center of the front circle along the robot/human orientation starting from the center at the rear axis (in meters)
   * @param front_radius radius of the front circle
   * @param rear_offset shift the center of the rear circle along the opposite robot/human orientation starting from the center at the rear axis (in meters)
   * @param rear_radius radius of the front circle
   */
  TwoCirclesFootprint(double front_offset, double front_radius, double rear_offset, double rear_radius)
      : front_offset_(front_offset), front_radius_(front_radius), rear_offset_(rear_offset), rear_radius_(rear_radius) {}

  /**
   * @brief Virtual destructor.
   */
  ~TwoCirclesFootprint() override = default;

  /**
   * @brief Set parameters of the contour/footprint
   * @param front_offset shift the center of the front circle along the robot/human orientation starting from the center at the rear axis (in meters)
   * @param front_radius radius of the front circle
   * @param rear_offset shift the center of the rear circle along the opposite robot/human orientation starting from the center at the rear axis (in meters)
   * @param rear_radius radius of the front circle
   */
  void setParameters(double front_offset, double front_radius, double rear_offset, double rear_radius) {
    front_offset_ = front_offset;
    front_radius_ = front_radius;
    rear_offset_ = rear_offset;
    rear_radius_ = rear_radius;
  }

  /**
   * @brief Calculate the distance between the robot/human and an obstacle
   * @param current_pose Current robot/human pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot/human
   */
  double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const override {
    Eigen::Vector2d dir = current_pose.orientationUnitVec();
    double dist_front = obstacle->getMinimumDistance(current_pose.position() + front_offset_ * dir) - front_radius_;
    double dist_rear = obstacle->getMinimumDistance(current_pose.position() - rear_offset_ * dir) - rear_radius_;
    return std::min(dist_front, dist_rear);
  }

  /**
   * @brief Estimate the distance between the robot/human and the predicted location of an obstacle at time t
   * @param current_pose robot/human pose, from which the distance to the obstacle is estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
   * @param t time, for which the predicted distance to the obstacle is calculated
   * @return Euclidean distance to the robot/human
   */
  double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const override {
    Eigen::Vector2d dir = current_pose.orientationUnitVec();
    double dist_front = obstacle->getMinimumSpatioTemporalDistance(current_pose.position() + front_offset_ * dir, t) - front_radius_;
    double dist_rear = obstacle->getMinimumSpatioTemporalDistance(current_pose.position() - rear_offset_ * dir, t) - rear_radius_;
    return std::min(dist_front, dist_rear);
  }

  /**
   * @brief Visualize the robot/human using a markers
   *
   * Fill a marker message with all necessary information (type, pose, scale and color).
   * The header, namespace, id and marker lifetime will be overwritten.
   * @param current_pose Current robot/human pose
   * @param[out] markers container of marker messages describing the robot/human shape
   * @param color Color of the footprint
   */
  void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const override {
    Eigen::Vector2d dir = current_pose.orientationUnitVec();
    if (front_radius_ > 0) {
      markers.emplace_back();
      visualization_msgs::msg::Marker& marker1 = markers.front();
      marker1.type = visualization_msgs::msg::Marker::CYLINDER;
      current_pose.toPoseMsg(marker1.pose);
      marker1.pose.position.x += front_offset_ * dir.x();
      marker1.pose.position.y += front_offset_ * dir.y();
      marker1.scale.x = marker1.scale.y = 2 * front_radius_;  // scale = diameter
                                                              //       marker1.scale.z = 0.05;
      marker1.color = color;
    }
    if (rear_radius_ > 0) {
      markers.emplace_back();
      visualization_msgs::msg::Marker& marker2 = markers.back();
      marker2.type = visualization_msgs::msg::Marker::CYLINDER;
      current_pose.toPoseMsg(marker2.pose);
      marker2.pose.position.x -= rear_offset_ * dir.x();
      marker2.pose.position.y -= rear_offset_ * dir.y();
      marker2.scale.x = marker2.scale.y = 2 * rear_radius_;  // scale = diameter
                                                             //       marker2.scale.z = 0.05;
      marker2.color = color;
    }
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  double getInscribedRadius() override {
    double min_longitudinal = std::min(rear_offset_ + rear_radius_, front_offset_ + front_radius_);
    double min_lateral = std::min(rear_radius_, front_radius_);
    return std::min(min_longitudinal, min_lateral);
  }

  /**
   * @brief Compute the circumscribed radius of the footprint model
   * @return circumscribed radius
   */
  double getCircumscribedRadius() const override { return std::max(front_offset_ + front_radius_, rear_offset_ + rear_radius_); }

 private:
  double front_offset_;
  double front_radius_;
  double rear_offset_;
  double rear_radius_;
};

/**
 * @class LineFootprint
 * @brief Class that approximates the robot/human with line segment (zero-width)
 */
class LineFootprint : public BaseFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   * @param line_start start coordinates (only x and y) of the line (w.r.t. robot/human center at (0,0))
   * @param line_end end coordinates (only x and y) of the line (w.r.t. robot/human center at (0,0))
   */
  LineFootprint(const geometry_msgs::msg::Point& line_start, const geometry_msgs::msg::Point& line_end) { setLine(line_start, line_end); }

  /**
   * @brief Default constructor of the abstract obstacle class (Eigen Version)
   * @param line_start start coordinates (only x and y) of the line (w.r.t. robot/human center at (0,0))
   * @param line_end end coordinates (only x and y) of the line (w.r.t. robot/human center at (0,0))
   */
  LineFootprint(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, const double min_obstacle_dist) : min_obstacle_dist_(min_obstacle_dist) { setLine(line_start, line_end); }

  /**
   * @brief Virtual destructor.
   */
  ~LineFootprint() override = default;

  /**
   * @brief Set vertices of the contour/footprint
   * @param vertices footprint vertices (only x and y) around the robot/human center (0,0) (do not repeat the first and last vertex at the end)
   */
  void setLine(const geometry_msgs::msg::Point& line_start, const geometry_msgs::msg::Point& line_end) {
    line_start_.x() = line_start.x;
    line_start_.y() = line_start.y;
    line_end_.x() = line_end.x;
    line_end_.y() = line_end.y;
  }

  /**
   * @brief Set vertices of the contour/footprint (Eigen version)
   * @param vertices footprint vertices (only x and y) around the robot/human center (0,0) (do not repeat the first and last vertex at the end)
   */
  void setLine(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) {
    line_start_ = line_start;
    line_end_ = line_end;
  }

  /**
   * @brief Calculate the distance between the robot/human and an obstacle
   * @param current_pose Current robot/human pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot/human
   */
  double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const override {
    Eigen::Vector2d line_start_world;
    Eigen::Vector2d line_end_world;
    transformToWorld(current_pose, line_start_world, line_end_world);
    return obstacle->getMinimumDistance(line_start_world, line_end_world);
  }

  /**
   * @brief Estimate the distance between the robot/human and the predicted location of an obstacle at time t
   * @param current_pose robot/human pose, from which the distance to the obstacle is estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
   * @param t time, for which the predicted distance to the obstacle is calculated
   * @return Euclidean distance to the robot/human
   */
  double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const override {
    Eigen::Vector2d line_start_world;
    Eigen::Vector2d line_end_world;
    transformToWorld(current_pose, line_start_world, line_end_world);
    return obstacle->getMinimumSpatioTemporalDistance(line_start_world, line_end_world, t);
  }

  /**
   * @brief Visualize the robot/human using a markers
   *
   * Fill a marker message with all necessary information (type, pose, scale and color).
   * The header, namespace, id and marker lifetime will be overwritten.
   * @param current_pose Current robot/human pose
   * @param[out] markers container of marker messages describing the robot/human shape
   * @param color Color of the footprint
   */
  void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const override {
    markers.emplace_back();
    visualization_msgs::msg::Marker& marker = markers.front();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    current_pose.toPoseMsg(marker.pose);  // all points are transformed into the robot/human frame!

    // line
    geometry_msgs::msg::Point line_start_world;
    line_start_world.x = line_start_.x();
    line_start_world.y = line_start_.y();
    line_start_world.z = 0;
    marker.points.push_back(line_start_world);

    geometry_msgs::msg::Point line_end_world;
    line_end_world.x = line_end_.x();
    line_end_world.y = line_end_.y();
    line_end_world.z = 0;
    marker.points.push_back(line_end_world);

    marker.scale.x = 0.05;
    marker.color = color;
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  double getInscribedRadius() override {
    return 0.0;  // lateral distance = 0.0
  }

  /**
   * @brief Compute the circumscribed radius of the footprint model
   * @return circumscribed radius
   */
  double getCircumscribedRadius() const override { return std::max(std::hypot(line_start_.x(), line_start_.y()), std::hypot(line_end_.x(), line_end_.y())); }

 private:
  /**
   * @brief Transforms a line to the world frame manually
   * @param current_pose Current robot/human pose
   * @param[out] line_start line_start_ in the world frame
   * @param[out] line_end line_end_ in the world frame
   */
  void transformToWorld(const PoseSE2& current_pose, Eigen::Vector2d& line_start_world, Eigen::Vector2d& line_end_world) const {
    double cos_th = std::cos(current_pose.theta());
    double sin_th = std::sin(current_pose.theta());
    line_start_world.x() = current_pose.x() + cos_th * line_start_.x() - sin_th * line_start_.y();
    line_start_world.y() = current_pose.y() + sin_th * line_start_.x() + cos_th * line_start_.y();
    line_end_world.x() = current_pose.x() + cos_th * line_end_.x() - sin_th * line_end_.y();
    line_end_world.y() = current_pose.y() + sin_th * line_end_.x() + cos_th * line_end_.y();
  }

  Eigen::Vector2d line_start_;
  Eigen::Vector2d line_end_;
  const double min_obstacle_dist_ = 0.0;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class PolygonFootprint
 * @brief Class that approximates the robot/human with a closed polygon
 */
class PolygonFootprint : public BaseFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   * @param vertices footprint vertices (only x and y) around the robot/human center (0,0) (do not repeat the first and last vertex at the end)
   */
  explicit PolygonFootprint(Point2dContainer vertices) : vertices_(std::move(vertices)) {}

  /**
   * @brief Virtual destructor.
   */
  ~PolygonFootprint() override = default;

  /**
   * @brief Set vertices of the contour/footprint
   * @param vertices footprint vertices (only x and y) around the robot/human center (0,0) (do not repeat the first and last vertex at the end)
   */
  void setVertices(const Point2dContainer& vertices) { vertices_ = vertices; }

  /**
   * @brief Calculate the distance between the robot/human and an obstacle
   * @param current_pose Current robot/human pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot/human
   */
  double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const override {
    Point2dContainer polygon_world(vertices_.size());
    transformToWorld(current_pose, polygon_world);
    return obstacle->getMinimumDistance(polygon_world);
  }

  /**
   * @brief Estimate the distance between the robot/human and the predicted location of an obstacle at time t
   * @param current_pose robot/human pose, from which the distance to the obstacle is estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
   * @param t time, for which the predicted distance to the obstacle is calculated
   * @return Euclidean distance to the robot/human
   */
  double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const override {
    Point2dContainer polygon_world(vertices_.size());
    transformToWorld(current_pose, polygon_world);
    return obstacle->getMinimumSpatioTemporalDistance(polygon_world, t);
  }

  /**
   * @brief Visualize the robot/human using a markers
   *
   * Fill a marker message with all necessary information (type, pose, scale and color).
   * The header, namespace, id and marker lifetime will be overwritten.
   * @param current_pose Current robot/human pose
   * @param[out] markers container of marker messages describing the robot/human shape
   * @param color Color of the footprint
   */
  void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const override {
    if (vertices_.empty()) return;

    markers.emplace_back();
    visualization_msgs::msg::Marker& marker = markers.front();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    current_pose.toPoseMsg(marker.pose);  // all points are transformed into the robot/human frame!

    for (const auto& vertice : vertices_) {
      geometry_msgs::msg::Point point;
      point.x = vertice.x();
      point.y = vertice.y();
      point.z = 0;
      marker.points.push_back(point);
    }
    // add first point again in order to close the polygon
    geometry_msgs::msg::Point point;
    point.x = vertices_.front().x();
    point.y = vertices_.front().y();
    point.z = 0;
    marker.points.push_back(point);

    marker.scale.x = 0.025;
    marker.color = color;
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  double getInscribedRadius() override {
    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector2d center(0.0, 0.0);

    if (vertices_.size() <= 2) {
      return 0.0;
    }

    for (int i = 0; i < static_cast<int>(vertices_.size()) - 1; ++i) {
      // compute distance from the robot/human center point to the first vertex
      double vertex_dist = vertices_[i].norm();
      double edge_dist = distance_point_to_segment_2d(center, vertices_[i], vertices_[i + 1]);
      min_dist = std::min({min_dist, vertex_dist, edge_dist});
    }

    // we also need to check the last vertex and the first vertex
    double vertex_dist = vertices_.back().norm();
    double edge_dist = distance_point_to_segment_2d(center, vertices_.back(), vertices_.front());
    return std::min({min_dist, vertex_dist, edge_dist});
  }

  /**
   * @brief Compute the circumscribed radius of the footprint model
   * @return circumscribed radius
   */
  double getCircumscribedRadius() const override {
    double radius = 0.0;
    for (const auto& vertex : vertices_) {
      double dist = std::hypot(vertex.x(), vertex.y());
      radius = std::max(radius, dist);
    }
    return radius;
  }

 private:
  /**
   * @brief Transforms a polygon to the world frame manually
   * @param current_pose Current robot/human pose
   * @param[out] polygon_world polygon in the world frame
   */
  void transformToWorld(const PoseSE2& current_pose, Point2dContainer& polygon_world) const {
    double cos_th = std::cos(current_pose.theta());
    double sin_th = std::sin(current_pose.theta());
    for (std::size_t i = 0; i < vertices_.size(); ++i) {
      polygon_world[i].x() = current_pose.x() + cos_th * vertices_[i].x() - sin_th * vertices_[i].y();
      polygon_world[i].y() = current_pose.y() + sin_th * vertices_[i].x() + cos_th * vertices_[i].y();
    }
  }

  Point2dContainer vertices_;
};

}  // namespace hateb_local_planner

#endif /* ROBOT_FOOTPRINT_MODEL_H */
