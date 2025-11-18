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

#ifndef SIM_HPP
#define SIM_HPP

#include <SDL2/SDL.h>
#include <yaml-cpp/yaml.h>
// Needs to be after yaml
#include <X11/Xlib.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>  // C++17
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

#define ROBOT_STATE_SIZE 6
#define COLOR_WHITE 255, 255, 255, 255
#define COLOR_BLACK 0, 0, 0, 255
#define MAX_ENTITIES 100
#define VELOCITY_TIMEOUT_MS 100  // Time in milliseconds before zeroing velocity
#define COLOR_ALPHA_OPAQUE 255
#define CIRCLE_BASE_SEGMENTS 32
#define CIRCLE_SEGMENTS_PER_RADIUS 1.5
#define TWO_PI 6.28318530717958647692
#define PI 3.14159265358979323846
#define DEG_360 360
#define DEG_30 30
#define DEG_180 180.0
#define LASER_DEFAULT_RANGE 10.0
#define ZOOM_STEP 1.1
#define MS_PER_SECOND 1000

// --- Texture-based occupancy grid rendering ---
#define K_OCCUPIED_COLOR 0x000000FF  // Black, opaque
#define K_FREE_COLOR 0xFFFFFFFF      // White, opaque

namespace cohan_sim {
/**
 * @brief Core entity class representing a movable object in the simulation
 */
class Entity {
 public:
  /**
   * @brief Constructs a new entity with position, orientation and appearance
   * @param x Initial X position in world coordinates
   * @param y Initial Y position in world coordinates
   * @param theta Initial orientation in radians
   * @param radius Collision radius for obstacle checking
   * @param color Visual appearance color
   */
  Entity(double x, double y, double theta, double radius, SDL_Color color) : x_(x), y_(y), theta_(theta), radius_(radius), color_(color) {}

  /**
   * @brief Get current X position
   * @return X coordinate in world frame
   */
  double x() const { return x_; }

  /**
   * @brief Get current Y position
   * @return Y coordinate in world frame
   */
  double y() const { return y_; }

  /**
   * @brief Get current orientation
   * @return Orientation in radians
   */
  double theta() const { return theta_; }

  /**
   * @brief Get entity's collision radius
   * @return Radius in meters
   */
  double radius() const { return radius_; }

  /**
   * @brief Get entity's visual color
   * @return SDL_Color structure
   */
  SDL_Color color() const { return color_; }

  /**
   * @brief Get X velocity
   * @return Linear velocity in x direction
   */
  double vx() const { return vx_; }

  /**
   * @brief Get Y velocity
   * @return Linear velocity in y direction
   */
  double vy() const { return vy_; }

  /**
   * @brief Get angular velocity
   * @return Angular velocity in radians/sec
   */
  double omega() const { return omega_; }

  /**
   * @brief Get X velocity in global frame
   * @return Linear velocity in x direction
   */
  double g_vx() const { return g_vx_; }

  /**
   * @brief Get Y velocity in global frame
   * @return Linear velocity in y direction
   */
  double g_vy() const { return g_vy_; }

  /**
   * @brief Get angular velocity in global frame
   * @return Angular velocity in radians/sec
   */
  double g_omega() const { return g_omega_; }

  /**
   * @brief Set entity's position and orientation
   * @param x New X position
   * @param y New Y position
   * @param theta New orientation in radians
   */
  void setPose(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
  }

  /**
   * @brief Set entity's velocities
   * @param vx Linear X velocity
   * @param vy Linear Y velocity
   * @param omega Angular velocity
   */

  void setVelocity(double vx, double vy, double omega) {
    vx_ = vx;
    vy_ = vy;
    omega_ = omega;
    updateGlobalVelocities();
  }

  /**
   * @brief Set entity's collision radius
   * @param r New radius in meters
   */
  void setRadius(double r) { radius_ = r; }

  /**
   * @brief Set entity's visual color
   * @param color New SDL_Color
   */
  void setColor(SDL_Color color) { color_ = color; }

  /**
   * @brief Process keyboard input for entity control
   * @param speed Maximum linear speed
   * @param angular_speed Maximum angular speed
   */
  void keyboardControl(double speed, double angular_speed) {
    const Uint8* state = SDL_GetKeyboardState(nullptr);
    vx_ = 0;
    vy_ = 0;
    omega_ = 0;
    if (state[SDL_SCANCODE_I]) {
      vx_ = speed;
    }
    if (state[SDL_SCANCODE_K]) {
      vx_ = -speed;
    }
    if (state[SDL_SCANCODE_J]) {
      vy_ = speed;
    }
    if (state[SDL_SCANCODE_L]) {
      vy_ = -speed;
    }
    if (state[SDL_SCANCODE_D] && !(SDL_GetModState() & KMOD_SHIFT)) {
      omega_ = -angular_speed;
    }
    if (state[SDL_SCANCODE_A] && !(SDL_GetModState() & KMOD_SHIFT)) {
      omega_ = angular_speed;
    }
    // Update global velocities (local to world frame transformation - used by Holonomic motion)
    updateGlobalVelocities();
  }

  /**
   * @brief Updates the entity's vx, vy from local (body) frame to world frame using current heading
   * @return None
   */
  void updateGlobalVelocities() {
    // Update global velocities based on current local velocities and orientation
    double cos_t = cos(theta_);
    double sin_t = sin(theta_);

    // Correct transformation from local to world frame
    double vx_world = (vx_ * cos_t) - (vy_ * sin_t);
    double vy_world = (vx_ * sin_t) + (vy_ * cos_t);

    // Store the global velocity in member variables
    g_vx_ = vx_world;
    g_vy_ = vy_world;
    g_omega_ = omega_;
  }

  // /**
  //  * @brief Updates the entity's vx, vy from local (body) frame to world frame using current heading
  //  * @return std::vector<double> containing world-frame vx and vy
  //  */
  // std::vector<double> getWorldVelocity() {
  //   std::vector<double> world_vel(2);

  //   double cos_t = cos(theta_);
  //   double sin_t = sin(theta_);

  //   // Correct transformation from local to world frame
  //   double vx_world = (vx_ * cos_t) - (vy_ * sin_t);
  //   double vy_world = (vx_ * sin_t) + (vy_ * cos_t);

  //   // Store the global velocity in member variables
  //   g_vx_ = vx_world;
  //   g_vy_ = vy_world;
  //   g_omega_ = omega_;

  //   world_vel[0] = vx_world;
  //   world_vel[1] = vy_world;
  //   return world_vel;
  // }

 private:
  double x_, y_, theta_;                                        //!< Position and orientation in world frame
  double radius_;                                               //!< Collision radius for entity interactions
  SDL_Color color_;                                             //!< Visual appearance color
  double vx_{}, vy_{}, omega_{}, g_vx_{}, g_vy_{}, g_omega_{};  //!< Current linear and angular velocities
};

/**
 * @brief Configuration structure for map properties loaded from YAML
 */
struct MapYamlConfig {
  std::string image;       //!< Path to map image file
  double resolution;       //!< Map resolution in meters/pixel
  double origin[3];        //!< Map origin [x, y, theta]
  int negate;              //!< Whether to negate the occupancy values
  double occupied_thresh;  //!< Threshold for considering cells occupied
  double free_thresh;      //!< Threshold for considering cells free
  double speed;            //!< Default linear speed for keyboard control
  double angular_speed;    //!< Default angular speed for keyboard control
};

/**
 * @brief Extended entity structure with simulation-specific properties
 */
struct SimEntity {
  std::string name = "unnamed";                                      //!< Unique name identifier
  int idx = 0.0;                                                     //!< Numeric index for referencing
  Entity entity = Entity(0.0, 0.0, 0.0, 0.5, {255, 255, 255, 255});  //!< Core entity instance
  double initial_x = 0.0;                                            //!< Initial X position for resets
  double initial_y = 0.0;                                            //!< Initial Y position for resets
  double initial_theta = 0.0;                                        //!< Initial orientation for resets
  bool use_keyboard = false;                                         //!< Whether entity accepts keyboard input
  bool use_differential = false;                                     //!< Whether to use differential drive motion
  bool use_laser = false;                                            //!< Whether entity has laser scanner
  double laser_range = 10.0;                                         //!< Maximum laser scan range
  int laser_resolution = 1081;                                       //!< Number of laser scan points
  double laser_angle = 6.283185307179586;                            //!< Total laser scan angle in radians
  std::vector<float> laser_data;                                     //!< Current laser scan readings
  double head_rotation = 0.0;                                        //!< Current head rotation angle
  double target_head_angle = 0.0;                                    //!< Target angle for head rotation
  bool head_at_target = true;                                        //!< Whether head reached target angle
};

/**
 * @brief Main 2D simulation class handling entities, visualization and physics
 */
class Simulator2D {
 public:
  /**
   * @brief Initialize simulator with world configuration
   * @param filename Path to world configuration YAML
   * @param render Whether to enable visualization
   */
  explicit Simulator2D(const char* filename, bool render = false);
  ~Simulator2D();

  /**
   * @brief Initialize the simulator with default properties
   */
  void initialize_properties();

  /**
   * @brief Run the main simulation loop
   */
  void runSimulation();

  /**
   * @brief Render the simulation
   */
  bool renderSimulation();

  /**
   * @brief Step the simulation forward by one frame
   * @return true if step was successful, false otherwise
   */
  bool stepSimulation();

  /**
   * @brief Set velocity command for a specific robot
   * @param idx Robot index
   * @param vx Linear X velocity
   * @param vy Linear Y velocity
   * @param omega Angular velocity
   */
  void setRobotVelocity(int idx, double vx, double vy, double omega);

  /**
   * @brief Set the head rotation for a specific agent
   * @param agent_idx Index of the agent
   * @param angle Rotation angle in radians
   */
  void setHeadRotation(int idx, double angle);

  /**
   * @brief Get list of all robots in simulation
   * @return Vector of SimEntity objects representing robots
   */
  std::vector<SimEntity> robots() const { return entities_; }

  /**
   * @brief Get current simulation time
   * @return Time in seconds since simulation start
   */
  double getSimTime() {
    Uint32 ticks = SDL_GetTicks();               // Get elapsed time in milliseconds
    return static_cast<double>(ticks / 1000.0);  // Convert to seconds
  }

 private:
  /**
   * @brief Signal handler for SIGINT
   * @param sig Signal number
   */
  static void handle_sigint(int sig);

  /**
   * @brief Helper function to read PGM file properly
   * @param f image file
   */
  static void skip_pgm_comments(FILE* f) {
    int c;
    while ((c = fgetc(f)) != EOF) {
      if (c == '#') {
        while ((c = fgetc(f)) != '\n' && c != EOF) {
        }
      } else if (!isspace(c)) {
        ungetc(c, f);
        break;
      }
    }
  }

  /**
   * @brief Converts world coordinates (meters) to map pixel coordinates
   * @param wx X coordinate in world frame (meters)
   * @param wy Y coordinate in world frame (meters)
   * @param mx Output map x-coordinate (pixels)
   * @param my Output map y-coordinate (pixels)
   */
  void world_to_map_coords(double wx, double wy, int* mx, int* my) {
    *mx = static_cast<int>(round((wx - config_.origin[0]) / config_.resolution));
    *my = map_height_ - 1 - static_cast<int>(round((wy - config_.origin[1]) / config_.resolution));
  }

  /**
   * @brief Converts map pixel coordinates to world coordinates (meters)
   * @param mx Map x-coordinate (pixels)
   * @param my Map y-coordinate (pixels)
   * @param wx Output x coordinate in world frame (meters)
   * @param wy Output y coordinate in world frame (meters)
   */
  void map_to_world_coords(int mx, int my, double* wx, double* wy) {
    *wx = mx * config_.resolution + config_.origin[0];
    *wy = (map_height_ - my) * config_.resolution + config_.origin[1];
  }

  /**
   * @brief Converts map pixel coordinates to screen coordinates with zoom and pan
   * @param mx Map x-coordinate (pixels)
   * @param my Map y-coordinate (pixels)
   * @param sx Output screen x-coordinate (pixels)
   * @param sy Output screen y-coordinate (pixels)
   */
  void map_to_screen(int mx, int my, int* sx, int* sy) const {
    *sx = static_cast<int>((zoom_ * static_cast<double>(mx)) + pan_x_);
    *sy = static_cast<int>((zoom_ * static_cast<double>(my)) + pan_y_);
  }

  /**
   * @brief Draws a visually correct circle outline at any zoom using parametric points
   * @param cx Center x-coordinate (screen px)
   * @param cy Center y-coordinate (screen px)
   * @param radius Circle radius in pixels (screen px)
   * @param color SDL_Color for the circle
   */
  void draw_circle(int cx, int cy, int radius, SDL_Color color);

  /**
   * @brief Renders an entity (robot or human) to the SDL window
   * @param sim_entity Simulation entity to draw
   */
  void draw_entity(SimEntity& sim_entity);

  /**
   * @brief Reset simulation to initial state
   */
  void resetSim();

  /**
   * @brief Update entity state and handle keyboard control
   * @param sim_entity Entity to update
   */
  void update_entity(SimEntity& sim_entity);

  /**
   * @brief Simulates 2D laser scan from robot's position
   * @param sim_entity SimEntity struct with per-entity laser params
   */
  void laser_scan(SimEntity& sim_entity);

  /**
   * @brief Render laser scan visualization
   * @param sim_entity Entity whose laser data to render
   */
  void render_laser(const SimEntity& sim_entity);

  /**
   * @brief Add a SimEntity from YAML
   * @param sim_entity SimEntity to add
   */
  void add_entity(SimEntity sim_entity);

  /**
   * @brief Load world configuration from YAML file
   * @param filename Path to YAML configuration file
   * @return true if loading successful, false otherwise
   */
  bool loadWorld(const char* filename);

  /**
   * @brief Load occupancy grid from PGM image file
   * @param filename Path to PGM image file
   */
  void loadMapPGM(const std::string& filename);

  /**
   * @brief Update texture used for rendering occupancy grid
   */
  void update_occupancy_grid_texture();

  /**
   * @brief Draw occupancy grid to screen
   */
  void draw_occupancy_grid();

  /**
   * @brief Check if entity can move to new position (collision checks with map)
   * @param e Entity to check
   * @param x Target X coordinate
   * @param y Target Y coordinate
   * @return true if movement possible, false if blocked
   */
  bool canMove(Entity& e, double x, double y);

  /**
   * @brief Move entity using holonomic motion model
   * @param e Entity to move
   * @param dt Time step in seconds
   */
  void moveHolonomic(Entity& e, double dt);

  /**
   * @brief Move entity using differential drive motion model
   * @param e Entity to move
   * @param dt Time step in seconds
   */
  void moveDifferential(Entity& e, double dt);

  /**
   * @brief Update entity's head rotation towards target angle
   * @param sim_entity Entity whose head rotation to update
   */
  void updateHeadRotation(SimEntity& sim_entity);

  MapYamlConfig config_;                 //!< Map configuration parameters
  SDL_Texture* occupancy_grid_texture_;  //!< Texture for map rendering
  int occupancy_grid_tex_w_;             //!< Map texture width
  int occupancy_grid_tex_h_;             //!< Map texture height
  int occupancy_grid_dirty_;             //!< Flag indicating if map needs update
  std::vector<SimEntity> entities_;      //!< List of all simulation entities
  int entity_count_;                     //!< Number of active entities

  // Pan and zoom properties
  double zoom_;                       //!< Current view zoom level
  double pan_x_;                      //!< Horizontal pan offset
  double pan_y_;                      //!< Vertical pan offset
  int dragging_;                      //!< Flag indicating mouse drag
  int drag_start_x_, drag_start_y_;   //!< Starting point of drag
  double pan_start_x_, pan_start_y_;  //!< Pan offset at drag start

  // Rendering properties
  pthread_t render_thread_;                         //!< Thread handle for rendering
  bool render_thread_running_;                      //!< Flag for render thread status
  SDL_Renderer* renderer_;                          //!< SDL rendering context
  SDL_Window* window_;                              //!< SDL window handle
  int map_width_;                                   //!< Map width in pixels
  int map_height_;                                  //!< Map height in pixels
  std::vector<std::vector<bool>> occupancy_grid_;   //!< 2D occupancy grid data
  bool occupancy_grid_initialized_;                 //!< Flag for grid initialization
  double last_update_time_;                         //!< Timestamp of last update
  double last_render_time_;                         //!< Timestamp of last render
  bool keyboard_in_use_;                            //!< Flag for keyboard control
  static volatile __sig_atomic_t sigint_received_;  //!< Signal handling flag
  Uint32 last_vel_command_;                         //!< Timestamp of last velocity command
  bool render_;                                     //!< Flag for rendering enabled
};

}  // namespace cohan_sim

#endif  // SIM_HPP