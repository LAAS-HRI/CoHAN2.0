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

#include <cohan_sim/sim.hpp>
#define FPS 60

namespace cohan_sim {
Simulator2D::Simulator2D(const char* filename, bool render) {
  // Initialize properties
  initialize_properties();

  // Set state variables
  render_ = render;

  // Initialize SDL
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    throw std::runtime_error("Failed to initialize SDL: " + std::string(SDL_GetError()));
  }

  if (loadWorld(filename)) {
    // Load the occupancy grid from the map image
    loadMapPGM(config_.image);
    occupancy_grid_initialized_ = true;
  } else {
    throw std::runtime_error("Failed to load world from file: " + std::string(filename));
  }

  SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");  // Enable linear filtering

  if (render_) {
    XInitThreads();  // Initialize X11 threading for SDL compatibility

    window_ = SDL_CreateWindow("Cohan Sim", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, map_width_, map_height_, SDL_WINDOW_SHOWN);
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  }
}
Simulator2D::~Simulator2D() {
  // Cleanup SDL resources
  if (occupancy_grid_texture_) {
    SDL_DestroyTexture(occupancy_grid_texture_);
    occupancy_grid_texture_ = nullptr;
  }
  if (renderer_) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }
  if (window_) {
    SDL_DestroyWindow(window_);
  }
  SDL_Quit();
}
void Simulator2D::initialize_properties() {
  // Pointers
  renderer_ = nullptr;
  window_ = nullptr;
  occupancy_grid_texture_ = nullptr;

  // Variables
  sigint_received_ = 0;
  occupancy_grid_tex_w_ = 0;
  occupancy_grid_tex_h_ = 0;
  occupancy_grid_dirty_ = 1;
  zoom_ = 1.0;
  pan_x_ = 0.0;
  pan_y_ = 0.0;
  dragging_ = 0;
  drag_start_x_ = 0;
  drag_start_y_ = 0;
  pan_start_x_ = 0.0;
  pan_start_y_ = 0.0;
  render_thread_running_ = false;
  map_width_ = 0;
  map_height_ = 0;
  occupancy_grid_initialized_ = false;
  last_update_time_ = 0.0;
  last_render_time_ = 0.0;
  keyboard_in_use_ = false;
  entity_count_ = 0;
}

bool Simulator2D::loadWorld(const char* filename) {
  try {
    // Load goals from the YAML file
    YAML::Node config = YAML::LoadFile(filename);

    auto image_file = config["image"].as<std::string>("map.pgm");
    fs::path image_path = fs::path(image_file);
    if (image_path.is_relative()) {
      image_path = fs::path(filename).parent_path() / image_path;
    }
    // Setup the map params in config
    config_.image = image_path.string();
    config_.resolution = config["resolution"].as<double>(1081);
    config_.origin[0] = config["origin"][0].as<double>(0);
    config_.origin[1] = config["origin"][1].as<double>(0);
    config_.origin[2] = config["origin"][2].as<double>(0);
    config_.negate = config["negate"].as<int>(0);
    config_.occupied_thresh = config["occupied_thresh"].as<double>(0.65);
    config_.free_thresh = config["free_thresh"].as<double>(0.196);

    // Setup the keyboard params from in config
    config_.speed = config["speed"].as<double>(2.0);
    config_.angular_speed = config["angular_speed"].as<double>(1.57);

    const YAML::Node& entities = config["entities"];

    // Iterate through each entity
    for (const auto& ent : entities) {
      SimEntity sim_ent;
      sim_ent.name = ent["name"].as<std::string>("unnamed");

      sim_ent.entity.setPose(ent["start_x"].as<double>(0.0), ent["start_y"].as<double>(0.0), ent["start_theta"].as<double>(0.0));
      sim_ent.entity.setRadius(ent["radius"].as<double>(0.5));
      SDL_Color color;
      color.r = ent["color"] && ent["color"].size() > 0 ? ent["color"][0].as<int>(255) : 255;
      color.g = ent["color"] && ent["color"].size() > 1 ? ent["color"][1].as<int>(255) : 255;
      color.b = ent["color"] && ent["color"].size() > 2 ? ent["color"][2].as<int>(255) : 255;
      color.a = ent["color"] && ent["color"].size() > 3 ? ent["color"][3].as<int>(255) : 255;
      sim_ent.entity.setColor(color);
      sim_ent.initial_x = ent["start_x"].as<double>(0.0);
      sim_ent.initial_y = ent["start_y"].as<double>(0.0);
      sim_ent.initial_theta = ent["start_theta"].as<double>(0.0);
      sim_ent.use_keyboard = ent["use_keyboard"].as<bool>(false);
      sim_ent.use_differential = ent["use_differential"].as<bool>(false);
      sim_ent.use_laser = ent["use_laser"].as<bool>(false);
      sim_ent.laser_range = ent["laser_range"].as<double>(10.0);
      sim_ent.laser_resolution = ent["laser_resolution"].as<int>(1081);
      sim_ent.laser_angle = ent["laser_angle"].as<double>(2 * M_PI);
      add_entity(sim_ent);
    }
    return true;

  } catch (const YAML::Exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return false;
  }
  return false;
}

void Simulator2D::loadMapPGM(const std::string& filename) {
  FILE* f = fopen(filename.c_str(), "rb");
  if (!f) {
    throw std::runtime_error("Failed to open PGM file: " + filename);
  }
  char header[3];
  if (fscanf(f, "%2s", header) != 1 || (strcmp(header, "P5") != 0 && strcmp(header, "P2") != 0)) {
    fclose(f);
    throw std::runtime_error("Unsupported PGM format (must be P2 or P5)");
  }
  skip_pgm_comments(f);
  if (fscanf(f, "%d", &map_width_) != 1) {
    fclose(f);
    throw std::runtime_error("Invalid PGM width");
  }
  skip_pgm_comments(f);
  if (fscanf(f, "%d", &map_height_) != 1) {
    fclose(f);
    throw std::runtime_error("Invalid PGM height");
  }
  skip_pgm_comments(f);
  int maxval;
  if (fscanf(f, "%d", &maxval) != 1) {
    fclose(f);
    throw std::runtime_error("Invalid PGM maxval");
  }
  fgetc(f);  // skip single whitespace after header

  std::vector<std::vector<bool>> grid(map_width_, std::vector<bool>(map_height_));
  if (strcmp(header, "P5") == 0) {
    // Binary
    for (int y = 0; y < map_height_; y++) {
      for (int x = 0; x < map_width_; x++) {
        int val = fgetc(f);
        grid[x][y] = (val < maxval / 2);  // black = occupied
      }
    }
  } else {
    // ASCII
    for (int y = 0; y < map_height_; y++) {
      for (int x = 0; x < map_width_; x++) {
        int val;
        if (fscanf(f, "%d", &val) != 1) {
          throw std::runtime_error("Failed to read value from PGM file");
        }
        grid[x][y] = (val < maxval / 2);
      }
    }
  }
  fclose(f);
  occupancy_grid_ = grid;
}

void Simulator2D::add_entity(SimEntity sim_entity) {
  if (sim_entity.use_keyboard) {
    if (keyboard_in_use_) {
      std::cout << "Warning: Only one entity can have use_keyboard=true. Entity '" << (!sim_entity.name.empty() ? sim_entity.name : "unknown") << "' will have keyboard disabled." << std::endl;
      sim_entity.use_keyboard = false;
    } else {
      keyboard_in_use_ = true;
    }
  }
  if (entity_count_ >= MAX_ENTITIES) {
    return;
  }
  if (sim_entity.use_laser && sim_entity.laser_resolution > 0) {
    sim_entity.laser_data.resize(sim_entity.laser_resolution, sim_entity.laser_range > 0.0 ? sim_entity.laser_range : LASER_DEFAULT_RANGE);
    if (sim_entity.laser_range <= 0.0) {
      sim_entity.laser_range = LASER_DEFAULT_RANGE;
      std::cout << "Warning: Invalid laser range provided, defaulting to " << LASER_DEFAULT_RANGE << " meters" << std::endl;
    }
  }
  last_vel_command_ = SDL_GetTicks();  // Initialize last velocity command timestamp
  sim_entity.idx = entity_count_;
  entities_.push_back(sim_entity);
  entity_count_++;
}

void Simulator2D::laser_scan(SimEntity& sim_entity) {
  int robot_px;
  int robot_py;
  world_to_map_coords(sim_entity.entity.x(), sim_entity.entity.y(), &robot_px, &robot_py);
  int robot_sx;
  int robot_sy;
  map_to_screen(robot_px, robot_py, &robot_sx, &robot_sy);
  double laser_angle = sim_entity.laser_angle;
  if (laser_angle > TWO_PI) {
    laser_angle = TWO_PI;
  }
  if (laser_angle < 0) {
    laser_angle = 0;
  }
  int n_rays = sim_entity.laser_resolution > 1 ? sim_entity.laser_resolution : 2;
  double angle_min = -laser_angle / 2.0;
  double angle_increment = laser_angle / (n_rays - 1);
  for (int i = 0; i < n_rays; i++) {
    sim_entity.laser_data[i] = sim_entity.laser_range;
    double angle = angle_min + (i * angle_increment) + sim_entity.entity.theta();
    // printf("angle = %f\n", angle);
    if (angle > TWO_PI) {
      angle -= TWO_PI;
    }
    if (angle < 0) {
      angle += TWO_PI;
    }
    double wx = sim_entity.entity.x() + (sim_entity.laser_range * cos(angle));
    double wy = sim_entity.entity.y() + (sim_entity.laser_range * sin(angle));
    int end_x;
    int end_y;
    world_to_map_coords(wx, wy, &end_x, &end_y);
    int x0 = robot_px;
    int y0 = robot_py;
    int x1 = end_x;
    int y1 = end_y;
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int e2;
    int x = x0;
    int y = y0;
    int prev_x = x0;
    int prev_y = y0;
    int hit = 0;
    int steps = 0;
    while (true) {
      if (x < 0 || x >= map_width_ || y < 0 || y >= map_height_) {
        break;
      }
      if (occupancy_grid_[x][y]) {
        double dist = sqrt(((prev_x - x0) * (prev_x - x0)) + ((prev_y - y0) * (prev_y - y0)));
        double range = dist * config_.resolution;

        if (range < 0.0) {
          range = 0.0;
        }
        if (range >= sim_entity.laser_range) {
          range = sim_entity.laser_range;
        }
        sim_entity.laser_data[i] = range;
        int prev_sx;
        int prev_sy;
        map_to_screen(prev_x, prev_y, &prev_sx, &prev_sy);
        hit = 1;
        break;
      }
      prev_x = x;
      prev_y = y;
      if (x == x1 && y == y1) {
        break;
      }
      e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y += sy;
      }
      steps++;
      if (steps > (int)(sim_entity.laser_range / config_.resolution)) {
        break;
      }
    }
    if (!hit) {
      int prev_sx;
      int prev_sy;
      map_to_screen(prev_x, prev_y, &prev_sx, &prev_sy);
    }
  }
}

void Simulator2D::render_laser(const SimEntity& sim_entity) {
  int robot_px, robot_py;
  world_to_map_coords(sim_entity.entity.x(), sim_entity.entity.y(), &robot_px, &robot_py);
  int robot_sx, robot_sy;
  map_to_screen(robot_px, robot_py, &robot_sx, &robot_sy);

  for (size_t i = 0; i < sim_entity.laser_data.size(); ++i) {
    double angle = -sim_entity.laser_angle / 2.0 + (i * sim_entity.laser_angle / (sim_entity.laser_data.size() - 1)) + sim_entity.entity.theta();
    double wx = sim_entity.entity.x() + (sim_entity.laser_data[i] * cos(angle));
    double wy = sim_entity.entity.y() + (sim_entity.laser_data[i] * sin(angle));
    int end_x, end_y;
    world_to_map_coords(wx, wy, &end_x, &end_y);
    int prev_sx, prev_sy;
    map_to_screen(end_x, end_y, &prev_sx, &prev_sy);

    SDL_SetRenderDrawColor(renderer_, 0, COLOR_ALPHA_OPAQUE, 0, COLOR_ALPHA_OPAQUE);
#ifdef UBUNTU_18
    SDL_RenderDrawLine(renderer_, robot_sx, robot_sy, prev_sx, prev_sy);
#else
    SDL_RenderDrawLineF(renderer_, robot_sx, robot_sy, prev_sx, prev_sy);
#endif
  }
}

bool Simulator2D::canMove(Entity& e, double x, double y) {
  int px;
  int py;
  world_to_map_coords(x, y, &px, &py);
  int r_px = (int)(e.radius() / config_.resolution);
  if (px < r_px || px >= map_width_ - r_px || py < r_px || py >= map_height_ - r_px) {
    return false;
  }
  for (int angle = 0; angle < DEG_360; angle += DEG_30) {
    double rad = angle * PI / DEG_180;
    int cx = px + (int)(r_px * cos(rad));
    int cy = py + (int)(r_px * sin(rad));
    if (cx < 0 || cx >= map_width_ || cy < 0 || cy >= map_height_) {
      return false;
    }
    if (occupancy_grid_[cx][cy]) {
      return false;
    }
  }
  return true;
}

void Simulator2D::moveHolonomic(Entity& e, double dt) {
  auto world_vel_x = e.g_vx();
  auto world_vel_y = e.g_vy();
  double new_x = e.x() + (world_vel_x * dt);
  double new_y = e.y() + (world_vel_y * dt);
  if (canMove(e, new_x, new_y)) {
    auto theta = e.theta();
    theta += e.omega() * dt;
    if (theta > 2 * M_PI) {
      theta -= 2 * M_PI;
    }
    if (theta < 0) {
      theta += 2 * M_PI;
    }
    e.setPose(new_x, new_y, theta);
  } else {
    e.setVelocity(0, 0, 0);
  }
}

void Simulator2D::moveDifferential(Entity& e, double dt) {
  double v = e.vx();  // Forward velocity in robot's frame
  double omega = e.omega();
  double theta = e.theta();
  double new_x, new_y, new_theta;
  // Threshold for considering omega as zero
  const double EPSILON = 1e-6;
  if (fabs(omega) < EPSILON) {
    // Straight motion
    new_x = e.x() + v * cos(theta) * dt;
    new_y = e.y() + v * sin(theta) * dt;
    new_theta = theta;
  } else {
    // Arc motion
    double r = v / omega;
    new_theta = theta + omega * dt;
    new_x = e.x() + r * (sin(new_theta) - sin(theta));
    new_y = e.y() - r * (cos(new_theta) - cos(theta));
  }
  // Normalize new_theta to [0, 2PI)
  if (new_theta >= TWO_PI) {
    new_theta -= TWO_PI;
  }
  if (new_theta < 0) {
    new_theta += TWO_PI;
  }
  if (canMove(e, new_x, new_y)) {
    e.setPose(new_x, new_y, new_theta);
  } else {
    // Stop the robot if blocked
    e.setVelocity(0, 0, 0);
  }
}

void Simulator2D::updateHeadRotation(SimEntity& sim_entity) {
  // Compute shortest angular difference
  double diff = sim_entity.target_head_angle - sim_entity.head_rotation;
  // Normalize diff to [-PI, PI]
  while (diff > PI) {
    diff -= TWO_PI;
  }
  while (diff < -PI) {
    diff += TWO_PI;
  }
  double max_step = 0.05;   // radians per frame (tune as needed)
  if (fabs(diff) > 1e-3) {  // Only update if not already close
    if (fabs(diff) < max_step) {
      sim_entity.head_rotation = sim_entity.target_head_angle;
    } else {
      sim_entity.head_rotation += (diff > 0 ? max_step : -max_step);
      // Normalize to [0, 2PI]
      if (sim_entity.head_rotation < 0) {
        sim_entity.head_rotation += TWO_PI;
      }
      if (sim_entity.head_rotation > TWO_PI) {
        sim_entity.head_rotation -= TWO_PI;
      }
    }
  } else {
    sim_entity.head_at_target = true;
  }  // If we reach the target angle, set flag
}

void Simulator2D::update_occupancy_grid_texture() {
  if (map_width_ <= 0 || map_height_ <= 0) {
    printf("Error: Invalid occupancy grid dimensions (width: %d, height: %d)\n", map_width_, map_height_);
    return;
  }

  if (!occupancy_grid_dirty_ && occupancy_grid_texture_ && occupancy_grid_tex_w_ == map_width_ && occupancy_grid_tex_h_ == map_height_) {
    return;
  }

  if (occupancy_grid_texture_) {
    SDL_DestroyTexture(occupancy_grid_texture_);
    occupancy_grid_texture_ = nullptr;
  }

  occupancy_grid_texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, map_width_, map_height_);
  if (!occupancy_grid_texture_) {
    printf("Failed to create occupancy grid texture: %s\n", SDL_GetError());
    return;
  }

  occupancy_grid_tex_w_ = map_width_;
  occupancy_grid_tex_h_ = map_height_;
  Uint32* pixels = (Uint32*)malloc(map_width_ * map_height_ * sizeof(Uint32));
  if (!pixels) {
    return;
  }

  for (int y = 0; y < map_height_; ++y) {
    for (int x = 0; x < map_width_; ++x) {
      pixels[(y * map_width_) + x] = occupancy_grid_[x][y] ? K_OCCUPIED_COLOR : K_FREE_COLOR;
    }
  }

  SDL_UpdateTexture(occupancy_grid_texture_, nullptr, pixels, map_width_ * sizeof(Uint32));
  free(pixels);
  occupancy_grid_dirty_ = 0;
}

void Simulator2D::draw_occupancy_grid() {
  // Fill the entire window with black before drawing the map
  int win_w = 0;
  int win_h = 0;
  SDL_GetRendererOutputSize(renderer_, &win_w, &win_h);
  SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);  // Black
  SDL_RenderClear(renderer_);
  update_occupancy_grid_texture();
  if (!occupancy_grid_texture_) {
    return;
  }
  SDL_Rect dst_rect;
  dst_rect.x = (int)pan_x_;
  dst_rect.y = (int)pan_y_;
  dst_rect.w = (int)(map_width_ * zoom_);
  dst_rect.h = (int)(map_height_ * zoom_);
  SDL_RenderCopy(renderer_, occupancy_grid_texture_, nullptr, &dst_rect);
}

void Simulator2D::update_entity(SimEntity& sim_entity) {
  if (sim_entity.use_keyboard && sim_entity.entity.vx() == 0 && sim_entity.entity.vy() == 0 && sim_entity.entity.omega() == 0) {
    // Update head_rotation for keyboard-controlled entity
    const Uint8* state = SDL_GetKeyboardState(NULL);
    if (state[SDL_SCANCODE_A] && (SDL_GetModState() & KMOD_SHIFT)) {
      sim_entity.head_rotation -= 0.05;
      if (sim_entity.head_rotation < 0) {
        sim_entity.head_rotation += TWO_PI;
      }
    }
    if (state[SDL_SCANCODE_D] && (SDL_GetModState() & KMOD_SHIFT)) {
      sim_entity.head_rotation += 0.05;
      if (sim_entity.head_rotation > TWO_PI) {
        sim_entity.head_rotation -= TWO_PI;
      }
    }
    sim_entity.entity.keyboardControl(config_.speed, config_.angular_speed);
  } else {
    Uint32 current_time = SDL_GetTicks();
    if (current_time - last_vel_command_ > VELOCITY_TIMEOUT_MS) {
      // Zero out velocities if no recent command
      sim_entity.entity.setVelocity(0, 0, 0);
    }
  }

  if (sim_entity.use_differential) {
    moveDifferential(sim_entity.entity, 1.0 / FPS);
  } else {
    moveHolonomic(sim_entity.entity, 1.0 / FPS);
  }

  // Update head rotation if not at target
  if (!sim_entity.head_at_target) {
    updateHeadRotation(sim_entity);
  }
}

void Simulator2D::draw_entity(SimEntity& sim_entity) {
  int px;
  int py;
  world_to_map_coords(sim_entity.entity.x(), sim_entity.entity.y(), &px, &py);
  int sx;
  int sy;
  map_to_screen(px, py, &sx, &sy);
  int radius_px = (int)(sim_entity.entity.radius() / config_.resolution * zoom_);
  draw_circle(sx, sy, radius_px, sim_entity.entity.color());
  // Draw small inner circle
  int small_radius = (int)(radius_px * 0.35);
  draw_circle(sx, sy, small_radius, sim_entity.entity.color());
  // Draw two dots inside the small circle, facing heading + head_rotation
  double heading_angle = -sim_entity.entity.theta() + sim_entity.head_rotation;
  double dot_dist = small_radius * 0.6;
  double dot_radius = fmax(2, small_radius * 0.18);
  for (int i = -1; i <= 1; i += 2) {
    double dot_angle = heading_angle + (i * 0.3);
    int dot_x = sx + (int)(dot_dist * cos(dot_angle));
    int dot_y = sy + (int)(dot_dist * sin(dot_angle));
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, COLOR_ALPHA_OPAQUE);
    for (int a = 0; a < 360; a += 20) {
      double th = a * PI / 180.0;
      int px = dot_x + (int)(dot_radius * cos(th));
      int py = dot_y + (int)(dot_radius * sin(th));
#ifdef UBUNTU_18
      SDL_RenderDrawPoint(renderer_, px, py);
#else
      SDL_RenderDrawPointF(renderer_, px, py);
#endif
    }
  }
  // Draw heading arrow (should not rotate with inner circle)
  int arrow_len = radius_px * 2;
  double arrow_angle = -sim_entity.entity.theta();  // Only use entity heading, not head_rotation
  int ax = sx + (int)(arrow_len * cos(arrow_angle));
  int ay = sy + (int)(arrow_len * sin(arrow_angle));
  auto color = sim_entity.entity.color();
  SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, COLOR_ALPHA_OPAQUE);
#ifdef UBUNTU_18
  SDL_RenderDrawLine(renderer_, sx, sy, ax, ay);
#else
  SDL_RenderDrawLineF(renderer_, sx, sy, ax, ay);
#endif
}

void Simulator2D::draw_circle(int cx, int cy, int radius, SDL_Color color) {
  SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, COLOR_ALPHA_OPAQUE);
  if (radius < 2) {
#ifdef UBUNTU_18
    SDL_RenderDrawPoint(renderer_, cx, cy);
#else
    SDL_RenderDrawPointF(renderer_, cx, cy);
#endif
    return;
  }
  int segments = CIRCLE_BASE_SEGMENTS + static_cast<int>(radius * CIRCLE_SEGMENTS_PER_RADIUS);
  double theta_step = TWO_PI / segments;
  int prev_x = cx + radius;
  int prev_y = cy;
  for (int i = 1; i <= segments; ++i) {
    double theta = i * theta_step;
    int x = cx + static_cast<int>(round(radius * cos(theta)));
    int y = cy + static_cast<int>(round(radius * sin(theta)));
#ifdef UBUNTU_18
    SDL_RenderDrawLine(renderer_, prev_x, prev_y, x, y);
#else
    SDL_RenderDrawLineF(renderer_, prev_x, prev_y, x, y);
#endif
    prev_x = x;
    prev_y = y;
  }
}

void Simulator2D::handle_sigint(int sig) {
  (void)sig;
  sigint_received_ = 1;
}

/***********************************************************************
 *                                                                     *
 * The simulator have the following GUI support.                       *
 *                                                                     *
 * Pressing r or R - Resets the Simulation                             *
 * Pressing z or Z - Resets the Zoom and Pan to default                *
 *                                                                     *
 * For moving the robot to which keyboard is attached,                 *
 *                                                                     *
 *                       Move Forward                                  *
 *                           ^                                         *
 *                          i/I                                        *
 *  Move Left        < j/J       l/L >    Move Right                   *
 * (only holonomic)                      (only holonomic)              *
 *                          k/K                                        *
 *                           v                                         *
 *                      sMove Backward                                 *
 *                                                                     *
 * For turnings,                                                       *
 *              Turn Left   << a/A   d/D >>   Turn right               *
 *                                                                     *
 * Turn Head Left   << Shift + a/A   Shift + d/D >>   Turn Head right  *
 *                                                                     *
 *                                                                     *
 * --> Same for the runSimulation and stepSimulation                   *
 ***********************************************************************/

void Simulator2D::runSimulation() {
  signal(SIGINT, handle_sigint);  // Register SIGINT handler for graceful shutdown

  if (occupancy_grid_.empty()) {
    return;
  }

  bool quit = false;
  SDL_Event e;
  Uint32 start_tick;

  while (!quit) {
    if (sigint_received_) {
      quit = true;
      break;
    }

    start_tick = SDL_GetTicks();

    while (SDL_PollEvent(&e) != 0) {
      if (e.type == SDL_QUIT) {
        quit = true;
      }
      if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_r) {
        resetSim();  // Define this
      }
      if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_z) {
        zoom_ = 1.0;
        pan_x_ = 0.0;
        pan_y_ = 0.0;
      }
      if (e.type == SDL_MOUSEWHEEL) {
        int mouse_x, mouse_y;
        SDL_GetMouseState(&mouse_x, &mouse_y);
        double prev_zoom = zoom_;
        zoom_ *= (e.wheel.y > 0) ? ZOOM_STEP : 1.0 / ZOOM_STEP;
        pan_x_ = mouse_x - ((mouse_x - pan_x_) * (zoom_ / prev_zoom));
        pan_y_ = mouse_y - ((mouse_y - pan_y_) * (zoom_ / prev_zoom));
      }
      if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
        dragging_ = true;
        drag_start_x_ = e.button.x;
        drag_start_y_ = e.button.y;
        pan_start_x_ = pan_x_;
        pan_start_y_ = pan_y_;
      }
      if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) {
        dragging_ = false;
      }
      if (e.type == SDL_MOUSEMOTION && dragging_) {
        pan_x_ = pan_start_x_ + (e.motion.x - drag_start_x_);
        pan_y_ = pan_start_y_ + (e.motion.y - drag_start_y_);
      }
    }

    for (auto& ent : entities_) {
      if (ent.use_laser) {
        laser_scan(ent);
      }
      update_entity(ent);
    }

    if (render_) {
      renderSimulation();
    }

    Uint32 elapsed = SDL_GetTicks() - start_tick;
    if (elapsed < MS_PER_SECOND / FPS) {
      SDL_Delay((MS_PER_SECOND / FPS) - elapsed);
    }
  }
}
volatile __sig_atomic_t Simulator2D::sigint_received_ = 0;

bool Simulator2D::renderSimulation() {
  draw_occupancy_grid();

  for (auto& ent : entities_) {
    if (ent.use_laser) {
      render_laser(ent);
    }
  }
  for (auto& ent : entities_) {
    draw_entity(ent);
  }
  SDL_RenderPresent(renderer_);
  return true;
}

bool Simulator2D::stepSimulation() {
  SDL_Event e;
  Uint32 start_tick;

  start_tick = SDL_GetTicks();

  signal(SIGINT, handle_sigint);  // Register SIGINT handler for graceful shutdown

  if (sigint_received_) {
    return false;
  }

  while (SDL_PollEvent(&e) != 0) {
    if (e.type == SDL_QUIT) {
      return false;
    }
    if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_r) {
      resetSim();  // Define this
    }
    if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_z) {
      zoom_ = 1.0;
      pan_x_ = 0.0;
      pan_y_ = 0.0;
    }
    if (e.type == SDL_MOUSEWHEEL) {
      int mouse_x, mouse_y;
      SDL_GetMouseState(&mouse_x, &mouse_y);
      double prev_zoom = zoom_;
      zoom_ *= (e.wheel.y > 0) ? ZOOM_STEP : 1.0 / ZOOM_STEP;
      pan_x_ = mouse_x - ((mouse_x - pan_x_) * (zoom_ / prev_zoom));
      pan_y_ = mouse_y - ((mouse_y - pan_y_) * (zoom_ / prev_zoom));
    }
    if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
      dragging_ = true;
      drag_start_x_ = e.button.x;
      drag_start_y_ = e.button.y;
      pan_start_x_ = pan_x_;
      pan_start_y_ = pan_y_;
    }
    if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) {
      dragging_ = false;
    }
    if (e.type == SDL_MOUSEMOTION && dragging_) {
      pan_x_ = pan_start_x_ + (e.motion.x - drag_start_x_);
      pan_y_ = pan_start_y_ + (e.motion.y - drag_start_y_);
    }
  }

  for (auto& ent : entities_) {
    if (ent.use_laser) {
      laser_scan(ent);
    }
    update_entity(ent);
  }

  if (render_) {
    while (!renderSimulation());

    Uint32 elapsed = SDL_GetTicks() - start_tick;
    if (elapsed < MS_PER_SECOND / FPS) {
      SDL_Delay((MS_PER_SECOND / FPS) - elapsed);
    }
  }

  return true;
}

void Simulator2D::resetSim() {
  for (auto& ent : entities_) {
    ent.entity.setPose(ent.initial_x, ent.initial_y, ent.initial_theta);
    ent.entity.setVelocity(0, 0, 0);
    ent.head_rotation = 0.0;
    last_vel_command_ = SDL_GetTicks();  // Reset last velocity command timestamp
  }
  // Reset zoom and pan
  zoom_ = 1.0;
  pan_x_ = 0.0;
  pan_y_ = 0.0;
}

void Simulator2D::setRobotVelocity(int idx, double vx, double vy, double omega) {
  if (idx >= 0 && idx < entity_count_) {
    last_vel_command_ = SDL_GetTicks();  // Update last velocity command timestamp
    auto* sim_ent = &entities_[idx];
    sim_ent->entity.setVelocity(vx, vy, omega);
  }
}

void Simulator2D::setHeadRotation(int idx, double angle) {
  if (idx >= 0 && idx < entity_count_) {
    auto* sim_ent = &entities_[idx];
    // Normalize angle to [0, 2π]
    if (angle < 0) {
      angle += TWO_PI;
    }
    if (angle > TWO_PI) {
      angle -= TWO_PI;
    }
    sim_ent->target_head_angle = angle;
    sim_ent->head_at_target = false;  // Reset flag since we are setting a new target
  }
}

}  // namespace cohan_sim
