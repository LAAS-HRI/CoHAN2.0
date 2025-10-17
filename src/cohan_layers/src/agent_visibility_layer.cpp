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

#include <angles/angles.h>

#include <Eigen/Core>
#include <cohan_layers/agent_visibility_layer.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cohan_layers::AgentVisibilityLayer, nav2_costmap_2d::Layer)

namespace cohan_layers {
void AgentVisibilityLayer::onInitialize() {
  AgentLayer::onInitialize();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  declareParameters();
  loadParameters();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&AgentVisibilityLayer::dynamicParametersCallback, this, std::placeholders::_1));
}

void AgentVisibilityLayer::declareParameters() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  // Declare parameters with descriptions and ranges
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::FloatingPointRange range;

  // amplitude
  descriptor.description = "Amplitude of the Gaussian";
  range.from_value = 0.0;
  range.to_value = 255.0;
  range.step = 0.01;
  descriptor.floating_point_range = {range};
  declareParameter("amplitude", rclcpp::ParameterValue(255.0));

  // radius
  descriptor.description = "Radius of the effect";
  range.from_value = 0.0;
  range.to_value = 10.0;
  descriptor.floating_point_range = {range};
  declareParameter("radius", rclcpp::ParameterValue(2.0));
}

void AgentVisibilityLayer::loadParameters() {
  // Get parameter values and store them in member variables
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  node->get_parameter(name_ + ".amplitude", amplitude_);
  node->get_parameter(name_ + ".radius", radius_);
}

rcl_interfaces::msg::SetParametersResult AgentVisibilityLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (auto parameter : parameters) {
    const auto& param_type = parameter.get_type();
    const auto& param_name = parameter.get_name();

    if (param_type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + ".amplitude") {
        amplitude_ = parameter.as_double();
      } else if (param_name == name_ + ".radius") {
        radius_ = parameter.as_double();
      }
    } else if (param_type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + ".enabled") {
        enabled_ = parameter.as_bool();
      }
    }
  }

  return result;
}

void AgentVisibilityLayer::updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y) {
  for (const auto& agent : transformed_agents_) {
    *min_x = std::min(*min_x, agent.pose.position.x - radius_);
    *min_y = std::min(*min_y, agent.pose.position.y - radius_);
    *max_x = std::max(*max_x, agent.pose.position.x + radius_);
    *max_y = std::max(*max_y, agent.pose.position.y + radius_);
  }
}

void AgentVisibilityLayer::updateCosts(nav2_costmap_2d::Costmap2D& /*master_grid*/, int min_i, int min_j, int max_i, int max_j) {
  std::lock_guard<std::recursive_mutex> lock(lock_);
  if (!enabled_) {
    return;
  }

  if (agents_.agents.size() == 0) {
    return;
  }

  nav2_costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  for (uint i = 0; i < transformed_agents_.size(); i++) {
    auto agent = transformed_agents_[i];
    auto state = transformed_agents_[i].state;
    auto type = transformed_agents_[i].type;
    double theta = tf2::getYaw(agent.pose.orientation);
    Eigen::Vector2d orient_vec(std::cos(theta), std::sin(theta));

    if (type == 1) {
      if (!states_.empty()) {
        if (state > 1) {
          continue;
        }
      }
      unsigned int width = std::max(1, static_cast<int>((2 * radius_) / res));
      unsigned int height = std::max(1, static_cast<int>((2 * radius_) / res));

      double cx = agent.pose.position.x;
      double cy = agent.pose.position.y;
      double ox = cx - radius_;
      double oy = cy - radius_;

      int mx;
      int my;
      costmap->worldToMapNoBounds(ox, oy, mx, my);

      int start_x = 0;
      int start_y = 0;
      int end_x = width;
      int end_y = height;
      if (mx < 0) {
        start_x = -mx;
      } else if (mx + width > costmap->getSizeInCellsX()) {
        end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - mx);
      }

      if ((start_x + mx) < min_i) {
        start_x = min_i - mx;
      }

      if ((end_x + mx) > max_i) {
        end_x = max_i - mx;
      }

      if (my < 0) {
        start_y = -my;
      } else if (my + height > costmap->getSizeInCellsY()) {
        end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - my);
      }

      if ((start_y + my) < min_j) {
        start_y = min_j - my;
      }

      if ((end_y + my) > max_j) {
        end_y = max_j - my;
      }

      double bx = ox + (res / 2);
      double by = oy + (res / 2);

      double var = radius_;

      for (int i = start_x; i < end_x; i++) {
        for (int j = start_y; j < end_y; j++) {
          unsigned char old_cost = costmap->getCost(i + mx, j + my);
          if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
            continue;
          }

          double x = bx + (i * res);
          double y = by + (j * res);
          double val;

          val = Gaussian2D(x, y, cx, cy, amplitude_, var, var);

          double rad = sqrt(-2 * var * log(val / amplitude_));
          Eigen::Vector2d pt_vec(x - cx, y - cy);

          if (rad > radius_) {
            continue;
          }
          auto cvalue = static_cast<unsigned char>(val);  // std::min(5*val,254.0);
          if (orient_vec.dot(pt_vec) <= 0) costmap->setCost(i + mx, j + my, std::max(cvalue, old_cost));
        }
      }
    }
  }
}

};  // namespace cohan_layers
