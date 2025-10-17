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
 *
 * Author: Phani Teja Singamaneni
 *********************************************************************************/

#ifndef STATIC_AGENT_LAYER_H
#define STATIC_AGENT_LAYER_H
#include <cohan_layers/agent_layer.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace cohan_layers {
class StaticAgentLayer : public AgentLayer {
 public:
  StaticAgentLayer() { layered_costmap_ = nullptr; }

  /**
   * @brief Initializes the StaticAgentLayer and sets up ROS communication
   */
  void onInitialize() override;

  /**
   * @brief Updates the bounds of the costmap based on static agent positions
   * @param min_x Minimum x-coordinate of the bounds
   * @param min_y Minimum y-coordinate of the bounds
   * @param max_x Maximum x-coordinate of the bounds
   * @param max_y Maximum y-coordinate of the bounds
   */
  void updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y) override;

  /**
   * @brief Updates the costmap with static agent information
   * @param master_grid Reference to the master costmap grid
   * @param min_i Minimum i-index of the grid
   * @param min_j Minimum j-index of the grid
   * @param max_i Maximum i-index of the grid
   * @param max_j Maximum j-index of the grid
   */
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Match the size of the master costmap
   */
  void matchSize() override {}

  /**
   * @brief Reset the layer
   */
  void reset() override {}

  /**
   * @brief Check if this layer can be cleared
   * @return true if clearable
   */
  bool isClearable() override { return false; }

 protected:
  /**
   * @brief Declares parameters and sets up dynamic reconfiguration
   */
  void declareParameters();

  /**
   * @brief Loads initial parameter values
   */
  void loadParameters();

  /**
   * @brief Callback for dynamic parameter changes
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Callback handle for dynamic parameters
   */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};
}  // namespace cohan_layers

#endif  // STATIC_AGENT_LAYER_H
