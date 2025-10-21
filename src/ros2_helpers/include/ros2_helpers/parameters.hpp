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

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include <functional>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <vector>

namespace parameters {

/**
 * @brief Generic parameter utility class for ROS2 nodes
 * This class provides a reusable way to declare parameters with validation
 * and set up parameter change callbacks
 */
class ParameterHelper {
 public:
  /**
   * @brief Constructor
   * @param node Shared pointer to the ROS2 node
   */
  explicit ParameterHelper() = default;

  void initialize(rclcpp::Node::SharedPtr node) {
    node_ = node;
    is_lifecycle_ = false;
  }

  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
    lifecycle_node_ = node;
    is_lifecycle_ = true;
  }

  /**
   * @brief Declare a floating point parameter with range validation
   * @param name Parameter name
   * @param default_value Default value
   * @param min Minimum allowed value
   * @param max Maximum allowed value
   * @param description Parameter description
   * @param step Step size for the parameter (default: 0.001)
   */
  void declareFloatParam(const std::string& name, double default_value, double min, double max, const std::string& description, double step = 0.001) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;

    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = min;
    range.to_value = max;
    range.step = step;
    descriptor.floating_point_range.push_back(range);

    if (is_lifecycle_) {
      lifecycle_node_->declare_parameter(name, default_value, descriptor);
    } else {
      node_->declare_parameter(name, default_value, descriptor);
    }
  }

  /**
   * @brief Declare an integer parameter with range validation
   * @param name Parameter name
   * @param default_value Default value
   * @param min Minimum allowed value
   * @param max Maximum allowed value
   * @param description Parameter description
   * @param step Step size for the parameter (default: 1)
   */
  void declareIntParam(const std::string& name, int64_t default_value, int64_t min, int64_t max, const std::string& description, int64_t step = 1) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;

    rcl_interfaces::msg::IntegerRange range;
    range.from_value = min;
    range.to_value = max;
    range.step = step;
    descriptor.integer_range.push_back(range);

    if (is_lifecycle_) {
      lifecycle_node_->declare_parameter(name, default_value, descriptor);
    } else {
      node_->declare_parameter(name, default_value, descriptor);
    }
  }

  /**
   * @brief Declare a boolean parameter
   * @param name Parameter name
   * @param default_value Default value
   * @param description Parameter description
   */
  void declareBoolParam(const std::string& name, bool default_value, const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;

    if (is_lifecycle_) {
      lifecycle_node_->declare_parameter(name, default_value, descriptor);
    } else {
      node_->declare_parameter(name, default_value, descriptor);
    }
  }

  /**
   * @brief Declare a string parameter
   * @param name Parameter name
   * @param default_value Default value
   * @param description Parameter description
   */
  void declareStringParam(const std::string& name, const std::string& default_value, const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;

    if (is_lifecycle_) {
      lifecycle_node_->declare_parameter(name, default_value, descriptor);
    } else {
      node_->declare_parameter(name, default_value, descriptor);
    }
  }

  /**
   * @brief Set up parameter change callback with optional custom validation
   * @param custom_callback Optional custom callback function for parameter validation/handling
   */
  void setupParameterCallback(std::function<bool(const std::vector<rclcpp::Parameter>&)> custom_callback = nullptr) {
    auto callback = [this, custom_callback](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      auto logger = is_lifecycle_ ? lifecycle_node_->get_logger() : node_->get_logger();

      for (const auto& param : params) {
        RCLCPP_DEBUG(logger, "Parameter '%s' changed to: %s", param.get_name().c_str(), param.value_to_string().c_str());
      }

      // If custom callback is provided, use it for additional validation
      if (custom_callback) {
        result.successful = custom_callback(params);
        if (!result.successful) {
          result.reason = "Custom parameter validation failed";
        }
      }

      return result;
    };

    // Store the callback handle to keep it alive
    if (is_lifecycle_) {
      param_callback_handle_lifecycle_ = lifecycle_node_->add_on_set_parameters_callback(callback);
    } else {
      param_callback_handle_ = node_->add_on_set_parameters_callback(callback);
    }
  }

  /**
   * @brief Get parameter value safely with type checking
   * @param name Parameter name
   * @param default_value Default value to return if parameter doesn't exist
   * @return Parameter value or default value
   */
  template <typename T>
  T getParam(const std::string& name, const T& default_value) const {
    try {
      if (is_lifecycle_) {
        return lifecycle_node_->get_parameter(name).get_value<T>();
      } else {
        return node_->get_parameter(name).get_value<T>();
      }
    } catch (const std::exception& e) {
      auto logger = is_lifecycle_ ? lifecycle_node_->get_logger() : node_->get_logger();
      RCLCPP_WARN(logger, "Failed to get parameter '%s': %s. Using default value.", name.c_str(), e.what());
      return default_value;
    }
  }

  /**
   * @brief Update parameter value programmatically
   * @param name Parameter name
   * @param value New parameter value
   * @return True if successful, false otherwise
   */
  template <typename T>
  bool setParam(const std::string& name, const T& value) {
    try {
      rcl_interfaces::msg::SetParametersResult result;
      if (is_lifecycle_) {
        result = lifecycle_node_->set_parameter(rclcpp::Parameter(name, value));
      } else {
        result = node_->set_parameter(rclcpp::Parameter(name, value));
      }
      return result.successful;
    } catch (const std::exception& e) {
      auto logger = is_lifecycle_ ? lifecycle_node_->get_logger() : node_->get_logger();
      RCLCPP_ERROR(logger, "Failed to set parameter '%s': %s", name.c_str(), e.what());
      return false;
    }
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
  bool is_lifecycle_ = false;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_lifecycle_;
};

}  // namespace parameters

#endif  // PARAMETERS_HPP_
