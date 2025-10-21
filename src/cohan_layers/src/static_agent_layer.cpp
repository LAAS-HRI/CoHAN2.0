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

#include <cohan_layers/static_agent_layer.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cohan_layers::StaticAgentLayer, nav2_costmap_2d::Layer)

namespace cohan_layers {
void StaticAgentLayer::onInitialize() {
  AgentLayer::onInitialize();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }
}

void StaticAgentLayer::updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y) {
  for (const auto& agent : transformed_agents_) {
    *min_x = std::min(*min_x, agent.pose.position.x - cfg_->radius);
    *min_y = std::min(*min_y, agent.pose.position.y - cfg_->radius);
    *max_x = std::max(*max_x, agent.pose.position.x + cfg_->radius);
    *max_y = std::max(*max_y, agent.pose.position.y + cfg_->radius);
  }
}

void StaticAgentLayer::updateCosts(nav2_costmap_2d::Costmap2D& /*master_grid*/, int min_i, int min_j, int max_i, int max_j) {
  std::lock_guard<std::recursive_mutex> lock(lock_);

  nav2_costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();

  if (!cfg_->enabled) {
    reset();
    return;
  }

  if (agents_.agents.size() == 0) {
    return;
  }

  double res = costmap->getResolution();

  for (uint i = 0; i < transformed_agents_.size(); i++) {
    auto agent = transformed_agents_[i];
    auto state = transformed_agents_[i].state;
    auto type = transformed_agents_[i].type;

    // Check the condition to switch the radius
    bool is_human_still = states_.empty() ? (type == 1) : ((state != 2) && (type == 1));

    unsigned int width = std::max(1, static_cast<int>((2 * cfg_->radius) / res));
    unsigned int height = std::max(1, static_cast<int>((2 * cfg_->radius) / res));

    double cx = agent.pose.position.x;
    double cy = agent.pose.position.y;
    double ox = cx - cfg_->radius;
    double oy = cy - cfg_->radius;

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

    double bx = 0;
    double by = 0;
    double var = 0;
    double rad = 0;

    if (is_human_still) {
      bx = ox + (res / 2);
      by = oy + (res / 2);
      var = cfg_->radius;
      if (state > 2) {
        rad = cfg_->agent_radius;
      }
    }

    else {
      if (type == 1) {
        rad = cfg_->agent_radius;
      } else {
        rad = robot_radius_;
      }
    }

    for (int i = start_x; i < end_x; i++) {
      for (int j = start_y; j < end_y; j++) {
        unsigned char old_cost = costmap->getCost(i + mx, j + my);
        if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
          continue;
        }

        if (is_human_still && state < 2) {
          double x = bx + (i * res);
          double y = by + (j * res);
          double val;
          val = Gaussian2D(x, y, cx, cy, cfg_->amplitude, var, var);
          double rad = sqrt(-2 * var * log(val / cfg_->amplitude));

          if (rad > cfg_->radius) {
            continue;
          }
          auto cvalue = static_cast<unsigned char>(val);
          costmap->setCost(i + mx, j + my, std::max(cvalue, old_cost));

        } else if (is_human_still && state > 2) {  // This removes costmap while human is moving and could cause issues with planning
                                                   // else {
          double x;
          double y;
          costmap->mapToWorld(i + mx, j + my, x, y);
          x = x - cx, y = y - cy;
          double inrad = sqrt((x * x) + (y * y));

          if (inrad > rad) {
            continue;
          }
          auto cvalue = nav2_costmap_2d::LETHAL_OBSTACLE;
          costmap->setCost(i + mx, j + my, std::max(cvalue, old_cost));
        }
      }
    }
  }
}

};  // namespace cohan_layers
