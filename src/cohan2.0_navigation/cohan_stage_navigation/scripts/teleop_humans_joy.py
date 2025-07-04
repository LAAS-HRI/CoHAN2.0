#!/usr/bin/env python
# Brief: Node for controlling human avatars through joy stick
# Author: Phani Teja Singamaneni

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopHumans(object):
  def __init__(self, h_id):
    self.h_id = h_id
    self.angular_ = 3
    self.linear_x = 1
    self.linear_y = 0
    rospy.init_node("teleop_humans_joy")
    rospy.Subscriber("joy", Joy, self.JoyCB)
    name = '/human'+str(self.h_id)+'/cmd_vel'
    self.vel_pub = rospy.Publisher(name, Twist, queue_size = 1)

    rospy.spin()

  def JoyCB(self, msg):
    velocity = Twist()
    velocity.linear.x = 0.2*msg.axes[self.linear_x]
    velocity.linear.y = 0.2*msg.axes[self.linear_y]
    velocity.angular.z = 0.2*msg.axes[self.angular_]
    self.vel_pub.publish(velocity)


if __name__ == '__main__':
  teleop_joy = TeleopHumans(2)
