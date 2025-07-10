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

#ifndef AGENT_VISIBILITY_LAYER_H
#define AGENT_VISIBILITY_LAYER_H
#include <cohan_layers/AgentVisibilityLayerConfig.h>
#include <cohan_layers/agent_layer.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

namespace cohan_layers {
class AgentVisibilityLayer : public AgentLayer {
 public:
  AgentVisibilityLayer() { layered_costmap_ = nullptr; }

  /**
   * @brief Initializes the AgentVisibilityLayer and sets up ROS communication
   */
  void onInitialize() override;

  /**
   * @brief Updates the bounds of the costmap based on agent positions
   * @param min_x Minimum x-coordinate of the bounds
   * @param min_y Minimum y-coordinate of the bounds
   * @param max_x Maximum x-coordinate of the bounds
   * @param max_y Maximum y-coordinate of the bounds
   */
  void updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y) override;

  /**
   * @brief Updates the costmap with agent visibility information
   * @param master_grid Reference to the master costmap grid
   * @param min_i Minimum i-index of the grid
   * @param min_j Minimum j-index of the grid
   * @param max_i Maximum i-index of the grid
   * @param max_j Maximum j-index of the grid
   */
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

 protected:
  /**
   * @brief Configures the AgentVisibilityLayer using dynamic reconfigure
   * @param config Configuration parameters for the layer
   * @param level Reconfiguration level
   */
  void configure(AgentVisibilityLayerConfig& config, uint32_t level);

  /**
   * @brief Dynamic reconfigure server for AgentVisibilityLayer
   */
  dynamic_reconfigure::Server<AgentVisibilityLayerConfig>* server_;

  /**
   * @brief Callback type for dynamic reconfigure server
   */
  dynamic_reconfigure::Server<AgentVisibilityLayerConfig>::CallbackType f_;
};
}  // namespace cohan_layers

#endif  // AGENT_VISIBILITY_LAYER_H
