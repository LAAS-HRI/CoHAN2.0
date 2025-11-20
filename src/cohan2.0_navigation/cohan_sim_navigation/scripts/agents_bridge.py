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

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and robot, and publishes /tracked_agents required for CoHAN
# Author: Phani Teja Singamaneni

import sys
import rclpy
from rclpy.node import Node
import time
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from nav_msgs.msg import Odometry
from message_filters import Subscriber, TimeSynchronizer


class SimAgents(Node):
    """
    Bridge between CoHAN Sim agents and CoHAN tracked_agents message.

    Subscribes to simulated human and robot odometry topics, converts them to TrackedAgents messages,
    and publishes them for use by CoHAN Navigation. Handles both human and robot agents, synchronizes their states,
    and assigns appropriate segment and agent types.
    """

    def __init__(self, num_hum):
        """
        Initialize the SimAgents bridge.

        Args:
            num_hum (int): Number of human agents in the simulation.
        """
        super().__init__('sim_agents')
        self.num_hum = num_hum
        # self.ns = self.get_namespace().strip('/')
        self.ns = self.get_namespace()
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.robot = TrackedAgent()
        self.sig_1 = False
        self.sig_2 = False
        
        self.setup_subscribers_and_publisher()

    def setup_subscribers_and_publisher(self):
        """
        Set up subscribers and publisher for tracked agents.
        Synchronizes human agent odometry and robot odometry, and starts publishing tracked_agents messages.
        """
        agent_sub = []

        for agent_id in range(1, self.num_hum + 1):
            name = 'human' + str(agent_id)
            if self.ns != name:
                agent_sub.append(Subscriber(self, Odometry, "/" + name + "/base_pose_ground_truth"))
        
        # Subscribe to the robot
        if self.ns != "/":
            self.robot_sub = self.create_subscription(Odometry, "/base_pose_ground_truth", self.RobotCB, 10)
        else:
            self.sig_2 = True

        self.tracked_agents_pub = self.create_publisher(TrackedAgents, "tracked_agents", 10)
        
        # Set up message filter synchronization
        if agent_sub:
            self.pose_msg = TimeSynchronizer(agent_sub, 10)
            self.pose_msg.registerCallback(self.AgentsCB)
        
        # Create timer to publish at 50 Hz (0.02 seconds)
        self.publish_timer = self.create_timer(0.02, self.publishAgents)

    def AgentsCB(self,*msg):
        """
        Callback for synchronized human agent odometry messages.
        Converts odometry to TrackedAgent messages and updates the tracked_agents list.

        Args:
            *msg: Synchronized odometry messages for human agents.
        """
        # if len(msg) != self.num_hum:
        #     return
    
        tracked_agents = TrackedAgents()
        idx = 0
        for agent_id in range(1,self.num_hum+1):
            if self.ns == "human"+str(agent_id):
                continue
            agent_segment = TrackedSegment()
            agent_segment.type = self.Segment_Type
            # print(agent_id-1)
            agent_segment.pose.pose = msg[idx].pose.pose
            agent_segment.twist.twist = msg[idx].twist.twist
            tracked_agent = TrackedAgent()
            tracked_agent.type = AgentType.HUMAN
            tracked_agent.name = "human"+str(agent_id)
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)
            idx += 1
        if(tracked_agents.agents):
            self.agents = tracked_agents
            self.sig_1 = True

    def RobotCB(self, msg):
        """
        Callback for robot odometry messages.
        Converts odometry to a TrackedAgent message for the robot and updates the tracked_agents list.

        Args:
            msg (Odometry): Odometry message for the robot.
        """
        if self.num_hum < 2:
            self.agents = TrackedAgents()
        agent_segment = TrackedSegment()
        agent_segment.type = self.Segment_Type
        agent_segment.pose.pose = msg.pose.pose
        agent_segment.twist.twist = msg.twist.twist
        tracked_agent = TrackedAgent()
        tracked_agent.type = AgentType.ROBOT
        tracked_agent.name = "robot"
        tracked_agent.segments.append(agent_segment)
        self.robot = tracked_agent
        self.sig_2 = True

    def publishAgents(self):
        """
        Publishes the current tracked_agents message if both human and robot data are available.
        Assigns track IDs and sets the header fields.
        """
        if(self.sig_1 and self.sig_2):
            self.agents.header.stamp = self.get_clock().now().to_msg()
            self.agents.header.frame_id = "map"
            if(self.ns != "/"):
                self.agents.agents.append(self.robot)
            for agent_id in range(0, len(self.agents.agents)):
                self.agents.agents[agent_id].track_id = agent_id+1
            self.tracked_agents_pub.publish(self.agents)
            if self.num_hum >= 2:
                self.sig_1 = False
            if self.ns != "/":
                self.sig_2 = False


def main():
    rclpy.init()
    
    # Filter out ROS2 arguments to get only our custom arguments
    filtered_args = []
    skip_next = False
    for i, arg in enumerate(sys.argv[1:], 1):
        if skip_next:
            skip_next = False
            continue
        if arg.startswith('--ros-args'):
            # Skip all remaining ROS args
            break
        if arg.startswith('-r') or arg.startswith('--remap'):
            skip_next = True  # Skip the next argument too (remap value)
            continue
        filtered_args.append(arg)
    
    if len(filtered_args) < 1:
        print("Usage: ros2 run cohan_sim_navigation agents_bridge.py <num_humans>")
        return
    
    nh = filtered_args[0]

    agents = SimAgents(num_hum=int(nh))
    agents.get_logger().info("Starting agents_bridge with {} humans".format(nh))
    
    try:
        rclpy.spin(agents)
    except KeyboardInterrupt:
        pass
    finally:
        agents.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
