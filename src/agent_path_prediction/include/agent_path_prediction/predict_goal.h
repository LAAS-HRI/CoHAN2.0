/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2025 LAAS/CNRS
 * All rights reserved.
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
 *  Author: Phani Teja Singamaneni
 *********************************************************************/

#ifndef PREDICT_GOAL_HH_
#define PREDICT_GOAL_HH_
#include <cmath>
#include <eigen3/Eigen/Core>
#include <map>
#include <string>
#include <utility>
#include <vector>
#define COV 0.2

namespace agents {

/**
 * @brief Class representing a 1D Gaussian distribution for probability calculations
 */
class Gaussian {
 public:
  /**
   * @brief Constructor for Gaussian distribution
   * @param mean Mean value of the distribution
   * @param cov Covariance value of the distribution
   */
  Gaussian(double mean, double cov) {
    this->mean_ = mean;
    this->cov_ = cov;
    this->inv_cov_ = 1.0 / cov;
    this->norm_const_ = 1.0 / std::sqrt(2 * M_PI * cov);  // Normalizing constant
  }

  /**
   * @brief Evaluates the probability density function at a point
   * @param x Point at which to evaluate the PDF
   * @return PDF value at point x
   */
  double pdf(double x) const {
    double diff = x - mean_;
    return norm_const_ * std::exp(-0.5 * diff * diff * inv_cov_);
  }

 private:
  double mean_;        // Mean of the Gaussian distribution
  double cov_;         // Covariance of the distribution
  double inv_cov_;     // Inverse of covariance for faster computation
  double norm_const_;  // Normalization constant for PDF
};

/**
 * @brief Class implementing Bayesian goal prediction for agents
 */
class BayesianGoalPrediction {
 public:
  using Trajectory = std::vector<Eigen::Vector2d>;

  /**
   * @brief Constructor initializing with default covariance
   */
  BayesianGoalPrediction() : nd_(0, COV) {}

  /**
   * @brief Default destructor
   */
  ~BayesianGoalPrediction() = default;

  /**
   * @brief Initialize the predictor with potential goals and window size
   * @param goals Map of goal names to their 2D positions
   * @param window_size Size of the sliding window for trajectory analysis
   */
  void initialize(const std::map<std::string, Eigen::Vector2d> &goals, int window_size) {
    for (const auto &goal : goals) {
      goal_names_.push_back(goal.first);
      goals_.push_back(goal.second);
    }
    window_size_ = window_size;
  }

  /**
   * @brief Predict the most likely goal for an agent
   * @param id Agent identifier
   * @param xy Current position of the agent
   * @return Name of the predicted goal, "None" if insufficient data
   */
  std::string predictGoal(int id, Eigen::Vector2d &xy) {
    std::string goal = "None";

    addPosition(id, xy);
    if (agents_trajs_[id].size() < window_size_) {
      return goal;
    }

    getProbabilities(id);
    int max_prob = std::max_element(agent_probs_[id].begin(), agent_probs_[id].end()) - agent_probs_[id].begin();
    goal = goal_names_[max_prob];
    return goal;
  }

 private:
  /**
   * @brief Add a new position to agent's trajectory. This method updates the trajectory of the agent by adding the current position. If the trajectory exceeds the window size, the oldest position is
   * removed.
   * @param id Agent ID
   * @param xy Current position of the agent as a 2D vector
   */
  void addPosition(int id, Eigen::Vector2d &xy) {
    if (agents_trajs_[id].size() > window_size_) {
      agents_trajs_[id].erase(agents_trajs_[id].begin());
    }
    agents_trajs_[id].push_back(xy);
  }

  /**
   * @brief Calculate probabilities for each goal based on the agent's trajectory. This method computes the likelihood of each goal being the agent's target based on the agent's motion history and
   * predefined goal priors.
   * @param id Agent ID
   */
  void getProbabilities(int id) {
    std::vector<double> probs;  // Temporary storage for probabilities
    std::vector<double> dists;  // Storage for distances to goals

    if (goal_priors_[id].empty()) {
      double prior = 1.0 / static_cast<double>(goals_.size());
      for (int i = 0; i < goals_.size(); i++) {
        goal_priors_[id].push_back(prior);
      }
    }

    auto trajectory = agents_trajs_[id];
    int goal_id = 0;
    double probs_sum = 0;
    bool still = false;

    for (auto &goal : goals_) {
      double probability = 1.0 / goals_.size();
      int n = trajectory.size();
      for (int i = 1; i < n; i++) {
        Eigen::Vector2d heading = trajectory[i] - trajectory[i - 1];
        if (heading.norm() == 0) {
          still = true;
          break;
        }

        Eigen::Vector2d goal_vec = goal - trajectory[i];
        if (i == n - 1) {
          dists.push_back(goal_vec.norm());
        }

        double phi = 0;
        if (heading.norm() != 0) {
          phi = std::acos(heading.dot(goal_vec) / (heading.norm() * goal_vec.norm()));
        }

        double g = std::exp((i - n) / 0.5);
        probability *= std::pow(nd_.pdf(phi), g);  // Combined proability for the entire trajectory
      }

      if (!still) {
        double goal_probability = goal_priors_[id][goal_id];
        goal_probability *= probability;
        probs_sum += goal_probability;
        probs.push_back(goal_probability);
      }

      goal_id++;
    }

    if (!still) {
      for (auto &prob : probs) {
        prob = prob / probs_sum;
        double prior = prob;
      }
      agent_probs_[id] = probs;
    }
  }

  Gaussian nd_;                                     //!< Gaussian distribution for probability calculations
  std::vector<Eigen::Vector2d> goals_;              //!< List of goal positions
  std::vector<std::string> goal_names_;             //!< List of goal names
  std::map<int, Trajectory> agents_trajs_;          //!< Map of agent trajectories
  std::map<int, std::vector<double>> goal_priors_;  //!< Map of goal priors for each agent
  std::map<int, std::vector<double>> agent_probs_;  //!< Map of probabilities for each agent
  int window_size_;                                 //!< Sliding window size for trajectory analysis
};
}  // namespace agents
#endif
