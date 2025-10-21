/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2025 LAAS-CNRS
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

#ifndef AGENT_LAYER_CONFIG_HPP_
#define AGENT_LAYER_CONFIG_HPP_

#include <cmath>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ros2_helpers/parameters.hpp>

namespace cohan_layers {
class AgentLayerConfig {
 public:
  AgentLayerConfig() = default;
  ~AgentLayerConfig() = default;

  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string layer_name) {
    layer_name_ = layer_name;
    param_helper_.initialize(node);
    RCLCPP_INFO(node->get_logger(), "Initializing parameters for layer: %s", layer_name.c_str());
  }

  void setupParameterCallback() {
    // Declare parameters using the generic helper with layer name prefix
    param_helper_.declareBoolParam(layer_name_ + ".enabled", true, "Whether to apply this plugin or not");
    param_helper_.declareFloatParam(layer_name_ + ".amplitude", 150., 0.0, 254.0, "Amplitude of adjustments at peak");
    param_helper_.declareFloatParam(layer_name_ + ".radius", 1.5, 0.0, 10.0, "Radius of the Gaussian");
    param_helper_.declareFloatParam(layer_name_ + ".agent_radius", 1.0, 0.0, 10.0, "Radius of the agent");
    param_helper_.declareStringParam(layer_name_ + ".ns", "", "ROS namespace for topics and services");

    // Set up parameter change callback with custom validation
    param_helper_.setupParameterCallback([this](const std::vector<rclcpp::Parameter>& params) -> bool {
      // Custom parameter validation logic for this specific node
      for (const auto& param : params) {
        const std::string& name = param.get_name();

        // Update internal variables when parameters change
        if (name == layer_name_ + ".enabled")
          enabled = param.as_bool();
        else if (name == layer_name_ + ".amplitude")
          amplitude = param.as_double();
        else if (name == layer_name_ + ".radius")
          radius = param.as_double();
        else if (name == layer_name_ + ".agent_radius")
          agent_radius = param.as_double();
        else if (name == layer_name_ + ".ns")
          ns = param.as_string();
      }
      return true;
    });

    // Load initial parameter values
    loadParameters();
  }

  bool enabled;         //!< Whether the plugin is enabled
  double amplitude;     //!< Amplitude of adjustments at peak
  double radius;        //!< Radius of the Gaussian
  double agent_radius;  //!< Radius of the agent
  std::string ns;       //!< ROS namespace

 private:
  /**
   * @brief Loads and initializes all parameters from ROS2 parameter server
   */
  void loadParameters() {
    // Get parameter values and store them in member variables
    enabled = param_helper_.getParam<bool>(layer_name_ + ".enabled", true);
    amplitude = param_helper_.getParam<double>(layer_name_ + ".amplitude", 150.0);
    radius = param_helper_.getParam<double>(layer_name_ + ".radius", 1.5);
    agent_radius = param_helper_.getParam<double>(layer_name_ + ".agent_radius", 1.0);
    ns = param_helper_.getParam<std::string>(layer_name_ + ".ns", "");
  }

  parameters::ParameterHelper param_helper_;  //!< Parameter helper for managing ROS2 parameters
  std::string layer_name_;                    //!< Name of the layer for parameter scoping
};

}  // namespace cohan_layers

#endif  // AGENT_LAYER_CONFIG_HPP_
