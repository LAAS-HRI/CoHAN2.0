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

#ifndef INVISIBLE_HUMANS_CONFIG_HPP_
#define INVISIBLE_HUMANS_CONFIG_HPP_

#include <cmath>
#include <ros2_helpers/parameters.hpp>

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

class InvisibleHumansConfig {
 public:
  InvisibleHumansConfig() = default;
  ~InvisibleHumansConfig() = default;

  /**
   * @brief Initializes the parameter helper for this config
   */
  void initialize(rclcpp::Node::SharedPtr node) {
    param_helper_.initialize(node);
    RCLCPP_INFO(node->get_logger(), "Initializing InvisibleHumansDetectionConfig parameters...");
  }

  /**
   * @brief Sets up parameter declarations and callback for parameter updates
   */
  void setupParameterCallback() {
    // Bind all parameters with automatic updates
    bindParameters();

    // Set up parameter change callback - parameters are auto-updated by bindings
    param_helper_.setupParameterCallback([this](const std::vector<rclcpp::Parameter>& params) -> bool {
      // Parameters are automatically updated by ParameterHelper bindings
      // Add any custom validation logic here if needed
      return true;
    });

    // Load initial parameter values
    param_helper_.loadBoundParameters();
  }

  // Public member variables for configuration
  std::string ns;       //!< Namespace for the node
  int samples;          //!< Number of samples for scanning
  double range_min;     //!< Minimum range for scanning
  double range_max;     //!< Maximum range for scanning
  double angle_min;     //!< Minimum angle for scanning (radians)
  double angle_max;     //!< Maximum angle for scanning (radians)
  int scan_resolution;  //!< Resolution of the scan
  bool publish_scan;    //!< Flag to indicate whether to publish scan data
  double human_radius;  //!< Radius of a human for detection

 private:
  /**
   * @brief Binds all configuration variables to parameters for auto-update
   */
  void bindParameters() {
    // Set default values for parameters BEFORE binding
    ns = "";
    samples = DEFAULT_SAMPLES;
    range_min = DEFAULT_RANGE_MIN;
    range_max = DEFAULT_RANGE_MAX;
    angle_min = DEFAULT_ANGLE_MIN;
    angle_max = DEFAULT_ANGLE_MAX;
    scan_resolution = DEFAULT_SCAN_RESOLUTION;
    publish_scan = DEFAULT_PUBLISH_SCAN;
    human_radius = DEFAULT_HUMAN_RADIUS;

    // Bind parameters with automatic updates
    param_helper_.bindStringParam("ns", ns, "Namespace for multiple robots");
    param_helper_.bindIntParam("samples", samples, 1, 10000, "Number of samples for scanning");
    param_helper_.bindFloatParam("range_min", range_min, 0.0, 10.0, "Minimum range for scanning");
    param_helper_.bindFloatParam("range_max", range_max, 0.1, 100.0, "Maximum range for scanning");
    param_helper_.bindFloatParam("angle_min", angle_min, -6.3, 6.3, "Minimum angle for scanning (radians)");
    param_helper_.bindFloatParam("angle_max", angle_max, -6.3, 6.3, "Maximum angle for scanning (radians)");
    param_helper_.bindIntParam("scan_resolution", scan_resolution, 1, 10000, "Resolution of the scan");
    param_helper_.bindBoolParam("publish_scan", publish_scan, "Whether to publish scan data");
    param_helper_.bindFloatParam("human_radius", human_radius, 0.1, 1.0, "Radius of a human for detection");
  }

  parameters::ParameterHelper param_helper_;  //!< Parameter helper for managing ROS2 parameters
};

}  // namespace invisible_humans_detection

#endif  // INVISIBLE_HUMANS_CONFIG_HPP_
