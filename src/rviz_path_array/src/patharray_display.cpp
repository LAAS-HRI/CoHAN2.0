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

#include "patharray_display.hpp"

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMatrix4.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

namespace rviz_path_array {

// Helper function to validate quaternions
bool validateQuaternion(const geometry_msgs::msg::Quaternion& q) {
  double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::abs(norm - 1.0) < 0.01;  // Allow small tolerance
}

bool validateQuaternions(const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
  for (const auto& pose : poses) {
    if (!validateQuaternion(pose.pose.orientation)) {
      return false;
    }
  }
  return true;
}

AgentPathArrayDisplay::AgentPathArrayDisplay() {
  style_property_ = new rviz_common::properties::EnumProperty("Line Style", "Lines", "The rendering operation to use to draw the grid lines.", this, SLOT(updateStyle()));

  style_property_->addOption("Lines", LINES);
  style_property_->addOption("Billboards", BILLBOARDS);

  line_width_property_ = new rviz_common::properties::FloatProperty("Line Width", 0.03,
                                                                    "The width, in meters, of each path line."
                                                                    "Only works with the 'Billboards' style.",
                                                                    this, SLOT(updateLineWidth()), this);
  line_width_property_->setMin(0.001);
  line_width_property_->hide();

  color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(25, 255, 0), "Color to draw the path.", this);

  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 1.0, "Amount of transparency to apply to the path.", this);

  buffer_length_property_ = new rviz_common::properties::IntProperty("Buffer Length", 1, "Number of paths to display.", this, SLOT(updateBufferLength()));
  buffer_length_property_->setMin(1);

  offset_property_ =
      new rviz_common::properties::VectorProperty("Offset", Ogre::Vector3::ZERO, "Allows you to offset the path from the origin of the reference frame.  In meters.", this, SLOT(updateOffset()));

  pose_style_property_ = new rviz_common::properties::EnumProperty("Pose Style", "None", "Shape to display the pose as.", this, SLOT(updatePoseStyle()));
  pose_style_property_->addOption("None", NONE);
  pose_style_property_->addOption("Axes", AXES);
  pose_style_property_->addOption("Arrows", ARROWS);

  pose_axes_length_property_ = new rviz_common::properties::FloatProperty("Length", 0.3, "Length of the axes.", this, SLOT(updatePoseAxisGeometry()));
  pose_axes_radius_property_ = new rviz_common::properties::FloatProperty("Radius", 0.03, "Radius of the axes.", this, SLOT(updatePoseAxisGeometry()));

  pose_arrow_color_property_ = new rviz_common::properties::ColorProperty("Pose Color", QColor(255, 85, 255), "Color to draw the poses.", this, SLOT(updatePoseArrowColor()));
  pose_arrow_shaft_length_property_ = new rviz_common::properties::FloatProperty("Shaft Length", 0.1, "Length of the arrow shaft.", this, SLOT(updatePoseArrowGeometry()));
  pose_arrow_head_length_property_ = new rviz_common::properties::FloatProperty("Head Length", 0.2, "Length of the arrow head.", this, SLOT(updatePoseArrowGeometry()));
  pose_arrow_shaft_diameter_property_ = new rviz_common::properties::FloatProperty("Shaft Diameter", 0.1, "Diameter of the arrow shaft.", this, SLOT(updatePoseArrowGeometry()));
  pose_arrow_head_diameter_property_ = new rviz_common::properties::FloatProperty("Head Diameter", 0.3, "Diameter of the arrow head.", this, SLOT(updatePoseArrowGeometry()));
  pose_axes_length_property_->hide();
  pose_axes_radius_property_->hide();
  pose_arrow_color_property_->hide();
  pose_arrow_shaft_length_property_->hide();
  pose_arrow_head_length_property_->hide();
  pose_arrow_shaft_diameter_property_->hide();
  pose_arrow_head_diameter_property_->hide();
}

AgentPathArrayDisplay::~AgentPathArrayDisplay() {
  destroyObjects();
  destroyPoseAxesChain();
  destroyPoseArrowChain();
}

void AgentPathArrayDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateBufferLength();
}

void AgentPathArrayDisplay::reset() {
  MFDClass::reset();
  updateBufferLength();
}

void AgentPathArrayDisplay::allocateAxesVector(std::vector<rviz_rendering::Axes*>& axes_vect, int num) {
  if (num > static_cast<int>(axes_vect.size())) {
    for (size_t i = axes_vect.size(); static_cast<int>(i) < num; i++) {
      auto* axes = new rviz_rendering::Axes(scene_manager_, scene_node_, pose_axes_length_property_->getFloat(), pose_axes_radius_property_->getFloat());
      axes_vect.push_back(axes);
    }
  } else if (num < static_cast<int>(axes_vect.size())) {
    for (int i = axes_vect.size() - 1; num <= i; i--) {
      delete axes_vect[i];
    }
    axes_vect.resize(num);
  }
}

void AgentPathArrayDisplay::allocateArrowVector(std::vector<rviz_rendering::Arrow*>& arrow_vect, int num) {
  if (num > static_cast<int>(arrow_vect.size())) {
    for (size_t i = arrow_vect.size(); static_cast<int>(i) < num; i++) {
      auto* arrow = new rviz_rendering::Arrow(scene_manager_, scene_node_);
      arrow_vect.push_back(arrow);
    }
  } else if (num < static_cast<int>(arrow_vect.size())) {
    for (int i = arrow_vect.size() - 1; num <= i; i--) {
      delete arrow_vect[i];
    }
    arrow_vect.resize(num);
  }
}

void AgentPathArrayDisplay::destroyPoseAxesChain() {
  for (auto& i : axes_chain_) {
    allocateAxesVector(i, 0);
  }
  axes_chain_.resize(0);
}

void AgentPathArrayDisplay::destroyPoseArrowChain() {
  for (auto& i : arrow_chain_) {
    allocateArrowVector(i, 0);
  }
  arrow_chain_.resize(0);
}

void AgentPathArrayDisplay::updateStyle() {
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  switch (style) {
    case LINES:
    default:
      line_width_property_->hide();
      break;

    case BILLBOARDS:
      line_width_property_->show();
      break;
  }

  updateBufferLength();
}

void AgentPathArrayDisplay::updateLineWidth() {
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());
  float line_width = line_width_property_->getFloat();

  if (style == BILLBOARDS) {
    for (auto* billboard_line : billboard_lines_) {
      if (billboard_line) billboard_line->setLineWidth(line_width);
    }
  }
  context_->queueRender();
}

void AgentPathArrayDisplay::updateOffset() {
  scene_node_->setPosition(offset_property_->getVector());
  context_->queueRender();
}

void AgentPathArrayDisplay::updatePoseStyle() {
  auto pose_style = static_cast<PoseStyle>(pose_style_property_->getOptionInt());
  switch (pose_style) {
    case AXES:
      pose_axes_length_property_->show();
      pose_axes_radius_property_->show();
      pose_arrow_color_property_->hide();
      pose_arrow_shaft_length_property_->hide();
      pose_arrow_head_length_property_->hide();
      pose_arrow_shaft_diameter_property_->hide();
      pose_arrow_head_diameter_property_->hide();
      break;
    case ARROWS:
      pose_axes_length_property_->hide();
      pose_axes_radius_property_->hide();
      pose_arrow_color_property_->show();
      pose_arrow_shaft_length_property_->show();
      pose_arrow_head_length_property_->show();
      pose_arrow_shaft_diameter_property_->show();
      pose_arrow_head_diameter_property_->show();
      break;
    default:
      pose_axes_length_property_->hide();
      pose_axes_radius_property_->hide();
      pose_arrow_color_property_->hide();
      pose_arrow_shaft_length_property_->hide();
      pose_arrow_head_length_property_->hide();
      pose_arrow_shaft_diameter_property_->hide();
      pose_arrow_head_diameter_property_->hide();
  }
  updateBufferLength();
}

void AgentPathArrayDisplay::updatePoseAxisGeometry() {
  for (auto& axes_vect : axes_chain_) {
    for (auto& j : axes_vect) {
      j->set(pose_axes_length_property_->getFloat(), pose_axes_radius_property_->getFloat());
    }
  }
  context_->queueRender();
}

void AgentPathArrayDisplay::updatePoseArrowColor() {
  QColor color = pose_arrow_color_property_->getColor();

  for (auto& arrow_vect : arrow_chain_) {
    for (auto& j : arrow_vect) {
      j->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);
    }
  }
  context_->queueRender();
}

void AgentPathArrayDisplay::updatePoseArrowGeometry() {
  for (auto& arrow_vect : arrow_chain_) {
    for (auto& j : arrow_vect) {
      j->set(pose_arrow_shaft_length_property_->getFloat(), pose_arrow_shaft_diameter_property_->getFloat(), pose_arrow_head_length_property_->getFloat(),
             pose_arrow_head_diameter_property_->getFloat());
    }
  }
  context_->queueRender();
}

void AgentPathArrayDisplay::destroyObjects() {
  // Destroy all simple lines, if any
  for (auto& manual_object : manual_objects_) {
    if (manual_object) {
      manual_object->clear();
      scene_manager_->destroyManualObject(manual_object);
      manual_object = nullptr;  // ensure it doesn't get destroyed again
    }
  }

  // Destroy all billboards, if any
  for (auto& billboard_line : billboard_lines_) {
    if (billboard_line) {
      delete billboard_line;     // also destroys the corresponding scene node
      billboard_line = nullptr;  // ensure it doesn't get destroyed again
    }
  }
}

void AgentPathArrayDisplay::updateBufferLength() {
  // Delete old path objects
  destroyObjects();

  // Destroy all axes and arrows
  destroyPoseAxesChain();
  destroyPoseArrowChain();

  // Read options
  int buffer_length = buffer_length_property_->getInt();
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  // Create new path objects
  switch (style) {
    case LINES:  // simple lines with fixed width of 1px
      manual_objects_.resize(buffer_length);
      for (auto& i : manual_objects_) {
        Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
        manual_object->setDynamic(true);
        scene_node_->attachObject(manual_object);

        i = manual_object;
      }
      break;

    case BILLBOARDS:  // billboards with configurable width
      billboard_lines_.resize(buffer_length);
      for (auto& i : billboard_lines_) {
        auto* billboard_line = new rviz_rendering::BillboardLine(scene_manager_, scene_node_);
        i = billboard_line;
      }
      break;
  }
  axes_chain_.resize(buffer_length);
  arrow_chain_.resize(buffer_length);
}

bool validateFloats(const cohan_msgs::msg::AgentPathArray& msg) {
  bool valid = true;
  for (const auto& path : msg.paths) {
    valid = valid && rviz_common::validateFloats(path.path.poses);
  }
  return valid;
}

void AgentPathArrayDisplay::processMessage(cohan_msgs::msg::AgentPathArray::ConstSharedPtr msg) {
  // Calculate index of oldest element in cyclic buffer
  size_t buffer_index = messages_received_ % buffer_length_property_->getInt();

  auto style = static_cast<LineStyle>(style_property_->getOptionInt());
  Ogre::ManualObject* manual_object = nullptr;
  rviz_rendering::BillboardLine* billboard_line = nullptr;

  // Delete oldest element
  switch (style) {
    case LINES:
      manual_object = manual_objects_[buffer_index];
      manual_object->clear();
      break;

    case BILLBOARDS:
      billboard_line = billboard_lines_[buffer_index];
      billboard_line->clear();
      break;
  }

  // Check if path contains invalid coordinate values
  if (!validateFloats(*msg)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
    return;
  }

  bool val_quat = true;
  for (const auto& path : msg->paths) {
    val_quat = val_quat && validateQuaternions(path.path.poses);
  }

  if (!val_quat) {
    RCLCPP_WARN(rclcpp::get_logger("rviz_path_array"), "Path '%s' contains unnormalized quaternions.", qPrintable(getName()));
  }

  // Lookup transform into fixed frame
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, rclcpp::Time(msg->header.stamp), position, orientation)) {
    RCLCPP_DEBUG(rclcpp::get_logger("rviz_path_array"), "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
  }

  Ogre::Matrix4 transform(orientation);
  transform.setTrans(position);

  //  scene_node_->setPosition( position );
  //  scene_node_->setOrientation( orientation );

  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  float line_width = line_width_property_->getFloat();

  for (const auto& path : msg->paths) {
    uint32_t num_points = path.path.poses.size();

    switch (style) {
      case LINES:
        manual_object->estimateVertexCount(num_points);
        manual_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
        for (uint32_t i = 0; i < num_points; ++i) {
          const geometry_msgs::msg::Point& pos = path.path.poses[i].pose.position;
          Ogre::Vector3 xpos = transform * Ogre::Vector3(pos.x, pos.y, pos.z);
          manual_object->position(xpos.x, xpos.y, xpos.z);
          manual_object->colour(color);
        }

        manual_object->end();
        break;

      case BILLBOARDS:
        billboard_line->setNumLines(1);
        billboard_line->setMaxPointsPerLine(num_points);
        billboard_line->setLineWidth(line_width);

        for (uint32_t i = 0; i < num_points; ++i) {
          const geometry_msgs::msg::Point& pos = path.path.poses[i].pose.position;
          Ogre::Vector3 xpos = transform * Ogre::Vector3(pos.x, pos.y, pos.z);
          billboard_line->addPoint(xpos, color);
        }

        break;
    }

    // process pose markers
    auto pose_style = static_cast<PoseStyle>(pose_style_property_->getOptionInt());
    std::vector<rviz_rendering::Arrow*>& arrow_vect = arrow_chain_[buffer_index];
    std::vector<rviz_rendering::Axes*>& axes_vect = axes_chain_[buffer_index];

    switch (pose_style) {
      case AXES:
        allocateAxesVector(axes_vect, num_points);
        for (uint32_t i = 0; i < num_points; ++i) {
          const geometry_msgs::msg::Point& pos = path.path.poses[i].pose.position;
          const geometry_msgs::msg::Quaternion& quat = path.path.poses[i].pose.orientation;
          Ogre::Vector3 xpos = transform * Ogre::Vector3(pos.x, pos.y, pos.z);
          Ogre::Quaternion xquat = orientation * Ogre::Quaternion(quat.w, quat.x, quat.y, quat.z);
          axes_vect[i]->setPosition(xpos);
          axes_vect[i]->setOrientation(xquat);
        }
        break;

      case ARROWS:
        allocateArrowVector(arrow_vect, num_points);
        for (uint32_t i = 0; i < num_points; ++i) {
          const geometry_msgs::msg::Point& pos = path.path.poses[i].pose.position;
          const geometry_msgs::msg::Quaternion& quat = path.path.poses[i].pose.orientation;
          Ogre::Vector3 xpos = transform * Ogre::Vector3(pos.x, pos.y, pos.z);
          Ogre::Quaternion xquat = orientation * Ogre::Quaternion(quat.w, quat.x, quat.y, quat.z);

          QColor color = pose_arrow_color_property_->getColor();
          arrow_vect[i]->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);

          arrow_vect[i]->set(pose_arrow_shaft_length_property_->getFloat(), pose_arrow_shaft_diameter_property_->getFloat(), pose_arrow_head_length_property_->getFloat(),
                             pose_arrow_head_diameter_property_->getFloat());
          arrow_vect[i]->setPosition(xpos);
          arrow_vect[i]->setDirection(xquat * Ogre::Vector3(1, 0, 0));
        }
        break;

      default:
        break;
    }
    context_->queueRender();
  }
}

}  // namespace rviz_path_array

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_path_array::AgentPathArrayDisplay, rviz_common::Display)
