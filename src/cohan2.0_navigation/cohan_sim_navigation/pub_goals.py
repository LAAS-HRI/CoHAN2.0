#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def create_goal(x, y, frame="map"):
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = frame
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0
    goal.pose.orientation.w = 1.0  # Facing forward
    return goal

if __name__ == "__main__":
    rospy.init_node("goal_publisher")

    pub_robot = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    pub_human = rospy.Publisher("/human1/move_base_simple/goal", PoseStamped, queue_size=10)

    rospy.sleep(1.0)  # Wait for publishers to connect

    goal_human = create_goal(10.0, 2.0)
    goal_robot = create_goal(2.0, 2.1)

    pub_robot.publish(goal_robot)
    pub_human.publish(goal_human)

    rospy.loginfo("Goals published.")
