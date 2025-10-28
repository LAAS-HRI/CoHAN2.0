/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2020-2025 LAAS-CNRS
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
 *********************************************************************************/

#ifndef UTILS_HPP_
#define UTILS_HPP_
#include <tf2/LinearMath/Vector3.h>
//  TF2
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define RCLCPP_ASSERT(expr)                                                           \
  if (!(expr)) {                                                                      \
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp_assert"), "Assertion failed: %s", #expr); \
    throw std::runtime_error("Assertion failed: " #expr);                             \
  }

#define RCLCPP_ASSERT_MSG(expr, msg, ...)                                  \
  if (!(expr)) {                                                           \
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp_assert"), msg, ##__VA_ARGS__); \
    throw std::runtime_error("Assertion failed: " msg);                    \
  }

// Conditional logging macros (ROS2 doesn't have _COND variants)
#define RCLCPP_DEBUG_COND(cond, logger, ...) \
  if (cond) {                                \
    RCLCPP_DEBUG(logger, __VA_ARGS__);       \
  }

#define RCLCPP_INFO_COND(cond, logger, ...) \
  if (cond) {                               \
    RCLCPP_INFO(logger, __VA_ARGS__);       \
  }

#define RCLCPP_WARN_COND(cond, logger, ...) \
  if (cond) {                               \
    RCLCPP_WARN(logger, __VA_ARGS__);       \
  }

#define RCLCPP_ERROR_COND(cond, logger, ...) \
  if (cond) {                                \
    RCLCPP_ERROR(logger, __VA_ARGS__);       \
  }

// Stream-based conditional logging macros
#define RCLCPP_DEBUG_STREAM_COND(cond, logger, stream_arg) \
  if (cond) {                                              \
    RCLCPP_DEBUG_STREAM(logger, stream_arg);               \
  }

#define RCLCPP_INFO_STREAM_COND(cond, logger, stream_arg) \
  if (cond) {                                             \
    RCLCPP_INFO_STREAM(logger, stream_arg);               \
  }

#define RCLCPP_WARN_STREAM_COND(cond, logger, stream_arg) \
  if (cond) {                                             \
    RCLCPP_WARN_STREAM(logger, stream_arg);               \
  }

#define RCLCPP_ERROR_STREAM_COND(cond, logger, stream_arg) \
  if (cond) {                                              \
    RCLCPP_ERROR_STREAM(logger, stream_arg);               \
  }

// ONCE logging macros (print message only once)
#define RCLCPP_DEBUG_ONCE(logger, ...)   \
  do {                                   \
    static bool __once = false;          \
    if (!__once) {                       \
      __once = true;                     \
      RCLCPP_DEBUG(logger, __VA_ARGS__); \
    }                                    \
  } while (0)

#define RCLCPP_INFO_ONCE(logger, ...)   \
  do {                                  \
    static bool __once = false;         \
    if (!__once) {                      \
      __once = true;                    \
      RCLCPP_INFO(logger, __VA_ARGS__); \
    }                                   \
  } while (0)

#define RCLCPP_WARN_ONCE(logger, ...)   \
  do {                                  \
    static bool __once = false;         \
    if (!__once) {                      \
      __once = true;                    \
      RCLCPP_WARN(logger, __VA_ARGS__); \
    }                                   \
  } while (0)

#define RCLCPP_ERROR_ONCE(logger, ...)   \
  do {                                   \
    static bool __once = false;          \
    if (!__once) {                       \
      __once = true;                     \
      RCLCPP_ERROR(logger, __VA_ARGS__); \
    }                                    \
  } while (0)

/**
 * @brief Strips leading slash from frame ID if present
 * @param frame_id The frame ID string
 * @return Frame ID without leading slash
 */
inline std::string strip_leading_slash(const std::string& frame_id) {
  if (!frame_id.empty() && frame_id[0] == '/') {
    return frame_id.substr(1);
  }
  return frame_id;
}

/**
 * @brief Look up twist information between frames with custom reference point
 *
 * This method computes the twist (linear and angular velocity) of one frame
 * relative to another, allowing specification of a custom reference point and frame.
 * It performs the necessary coordinate transformations and averages the motion
 * over the specified time interval.
 *
 * @param tracking_frame The frame being tracked (whose motion we want to determine)
 * @param observation_frame The frame from which we're observing
 * @param reference_frame The frame in which the twist should be expressed
 * @param reference_point The point about which the twist should be computed
 * @param reference_point_frame The frame in which the reference point is expressed
 * @param time The time at which to get the twist
 * @param averaging_interval Time interval over which to average the twist
 * @param[out] twist The resulting twist message containing linear and angular velocities
 */

inline void lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame, const tf2::Vector3& reference_point,
                        const std::string& reference_point_frame, const rclcpp::Time& time, const rclcpp::Duration& averaging_interval, geometry_msgs::msg::Twist& twist,
                        std::shared_ptr<tf2_ros::Buffer> tf_) {
  // Use the same clock type as the provided time to avoid mixing clock sources
  rcl_clock_type_t clock_type = time.get_clock_type();

  rclcpp::Time latest_time(clock_type);
  rclcpp::Time target_time(clock_type);

  tf2::TimePoint latest_time_tf2;
  tf2::CompactFrameID target_id = tf_->_lookupFrameNumber(strip_leading_slash(tracking_frame));
  tf2::CompactFrameID source_id = tf_->_lookupFrameNumber(strip_leading_slash(observation_frame));
  tf_->_getLatestCommonTime(source_id, target_id, latest_time_tf2, nullptr);

  // Convert tf2::TimePoint to rclcpp::Time using the chosen clock type
  latest_time = rclcpp::Time(latest_time_tf2.time_since_epoch().count(), clock_type);
  rclcpp::Time zero_time(0, 0, clock_type);

  if (zero_time == time) {
    target_time = latest_time;
  } else {
    target_time = time;
  }

  rclcpp::Time end_time = std::min(target_time + averaging_interval * 0.5, latest_time);

  rclcpp::Duration small_duration = rclcpp::Duration::from_nanoseconds(10000);
  rclcpp::Time min_time = rclcpp::Time(small_duration.nanoseconds(), clock_type);
  rclcpp::Time start_time = std::max(min_time + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
  rclcpp::Duration corrected_averaging_interval = end_time - start_time;                             // correct for the possiblity that start time was
                                                                                                     // truncated above.
  geometry_msgs::msg::TransformStamped start_msg;
  geometry_msgs::msg::TransformStamped end_msg;
  start_msg = tf_->lookupTransform(observation_frame, tracking_frame, start_time);
  end_msg = tf_->lookupTransform(observation_frame, tracking_frame, end_time);

  tf2::Stamped<tf2::Transform> start;
  tf2::Stamped<tf2::Transform> end;
  tf2::fromMsg(start_msg, start);
  tf2::fromMsg(end_msg, end);

  tf2::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
  tf2::Quaternion quat_temp;
  temp.getRotation(quat_temp);
  tf2::Vector3 o = start.getBasis() * quat_temp.getAxis();
  double ang = quat_temp.getAngle();

  double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
  double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
  double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();

  tf2::Vector3 twist_vel((delta_x) / corrected_averaging_interval.seconds(), (delta_y) / corrected_averaging_interval.seconds(), (delta_z) / corrected_averaging_interval.seconds());
  tf2::Vector3 twist_rot = o * (ang / corrected_averaging_interval.seconds());

  // This is a twist w/ reference frame in observation_frame  and reference
  // point is in the tracking_frame at the origin (at start_time)

  // correct for the position of the reference frame
  tf2::Stamped<tf2::Transform> inverse;
  tf2::fromMsg(tf_->lookupTransform(reference_frame, tracking_frame, target_time), inverse);
  tf2::Vector3 out_rot = inverse.getBasis() * twist_rot;
  tf2::Vector3 out_vel = inverse.getBasis() * twist_vel + inverse.getOrigin().cross(out_rot);

  // Rereference the twist about a new reference point
  // Start by computing the original reference point in the reference frame:
  tf2::TimePoint target_time_tf2(std::chrono::nanoseconds(target_time.nanoseconds()));
  tf2::Stamped<tf2::Vector3> rp_orig(tf2::Vector3(0, 0, 0), target_time_tf2, tracking_frame);
  geometry_msgs::msg::TransformStamped reference_frame_trans;
  tf2::fromMsg(tf_->lookupTransform(reference_frame, rp_orig.frame_id_, rp_orig.stamp_), reference_frame_trans);

  geometry_msgs::msg::PointStamped rp_orig_msg;
  // rp_orig_msg.header.stamp = rclcpp::Time(std::chrono::nanoseconds(rp_orig.stamp_.time_since_epoch().count()).count(), clock_type);
  rp_orig_msg.header.stamp = rclcpp::Time(rp_orig.stamp_.time_since_epoch().count(), clock_type);
  rp_orig_msg.header.frame_id = rp_orig.frame_id_;
  rp_orig_msg.point.x = rp_orig.x();
  rp_orig_msg.point.y = rp_orig.y();
  rp_orig_msg.point.z = rp_orig.z();
  tf2::doTransform(rp_orig_msg, rp_orig_msg, reference_frame_trans);

  // convert the requrested reference point into the right frame
  tf2::Stamped<tf2::Vector3> rp_desired(reference_point, target_time_tf2, reference_point_frame);
  geometry_msgs::msg::PointStamped rp_desired_msg;
  // rp_desired_msg.header.stamp = rclcpp::Time(std::chrono::nanoseconds(rp_desired.stamp_.time_since_epoch().count()).count(), clock_type);
  rp_desired_msg.header.stamp = rclcpp::Time(rp_desired.stamp_.time_since_epoch().count(), clock_type);
  rp_desired_msg.header.frame_id = rp_desired.frame_id_;
  rp_desired_msg.point.x = rp_desired.x();
  rp_desired_msg.point.y = rp_desired.y();
  rp_desired_msg.point.z = rp_desired.z();
  tf2::doTransform(rp_desired_msg, rp_desired_msg, reference_frame_trans);
  // compute the delta
  tf2::Vector3 delta = rp_desired - rp_orig;
  // Correct for the change in reference point
  out_vel = out_vel + out_rot * delta;
  // out_rot unchanged

  twist.linear.x = out_vel.x();
  twist.linear.y = out_vel.y();
  twist.linear.z = out_vel.z();
  twist.angular.x = out_rot.x();
  twist.angular.y = out_rot.y();
  twist.angular.z = out_rot.z();
}

/**
 * @brief Look up twist information between frames with default reference point
 *
 * Simplified version that uses the origin of tracking_frame as the reference point
 * and observation_frame as the reference frame.
 * @param tracking_frame The frame being tracked
 * @param observation_frame The frame from which we're observing
 * @param time The time at which to get the twist
 * @param averaging_interval Time interval over which to average the twist
 * @param[out] twist The resulting twist message
 */
inline void lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const rclcpp::Time& time, const rclcpp::Duration& averaging_interval, geometry_msgs::msg::Twist& twist,
                        std::shared_ptr<tf2_ros::Buffer> tf_) {
  // ref point is origin of tracking_frame, ref_frame = obs_frame
  lookupTwist(tracking_frame, observation_frame, observation_frame, tf2::Vector3(0, 0, 0), tracking_frame, time, averaging_interval, twist, tf_);
}

#endif  // UTILS_HPP_