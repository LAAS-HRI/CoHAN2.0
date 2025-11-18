#!/usr/bin/env python3

"""
Software License Agreement (MIT License)

Copyright (c) 2020-2025 LAAS-CNRS

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Author: Phani Teja Singamaneni
"""

# Brief: This function publishes /joint_states for tucked_arm_pose of PR2, necessary for displaying robot robot_description in Rviz
# Author: Phani Teja Singamaneni

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('cohan_sim_joints')
        self.joint_states = JointState()
        self.joint_states.header.frame_id = "joint_states"
        self.joint_states.name = ["laser_tilt_mount_joint","fl_caster_rotation_joint","fl_caster_l_wheel_joint","fl_caster_r_wheel_joint","fr_caster_rotation_joint","fr_caster_l_wheel_joint","fr_caster_r_wheel_joint","bl_caster_rotation_joint","bl_caster_l_wheel_joint","bl_caster_r_wheel_joint","br_caster_rotation_joint","br_caster_l_wheel_joint","br_caster_r_wheel_joint","r_gripper_motor_slider_joint","r_gripper_motor_screw_joint","r_gripper_l_finger_joint","r_gripper_r_finger_joint","r_gripper_l_finger_tip_joint","r_gripper_r_finger_tip_joint","r_gripper_joint","l_gripper_motor_slider_joint","l_gripper_motor_screw_joint","l_gripper_l_finger_joint","l_gripper_r_finger_joint","l_gripper_l_finger_tip_joint","l_gripper_r_finger_tip_joint","l_gripper_joint","torso_lift_joint","torso_lift_motor_screw_joint","head_pan_joint","head_tilt_joint","l_shoulder_pan_joint","l_shoulder_lift_joint","l_upper_arm_roll_joint","l_elbow_flex_joint","l_forearm_roll_joint","l_wrist_flex_joint","l_wrist_roll_joint","r_shoulder_pan_joint","r_shoulder_lift_joint","r_upper_arm_roll_joint","r_elbow_flex_joint","r_forearm_roll_joint","r_wrist_flex_joint","r_wrist_roll_joint"]
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -2.9802322387695312e-08, 0.06914562731981277, 1.2399593591690063, 1.7890771627426147, -1.6932392120361328, -1.7343393564224243, -0.09060896933078766, -0.07657696306705475, -0.0138227678835392, 1.097334384918213, -1.5566887855529785, -2.114457845687866, -1.4083462953567505, -1.8511372804641724, 0.2240774929523468]
        
        # Create publisher
        self.joint_states_pub = self.create_publisher(JointState, "/cohan_sim_joint_states", 10)
        
        # Create timer to publish at 50 Hz
        self.timer = self.create_timer(1.0/50.0, self.publish_joint_states)
        
    def publish_joint_states(self):
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states_pub.publish(self.joint_states)

def main():
    rclpy.init()
    joint_publisher = JointStatePublisher()
    try:
        rclpy.spin(joint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
