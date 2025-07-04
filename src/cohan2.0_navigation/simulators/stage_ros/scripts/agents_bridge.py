#!/usr/bin/env python3

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and robot, and publishes /tracked_agents required for CoHAN
# Author: Phani Teja Singamaneni

import sys
import rospy
import time
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from nav_msgs.msg import Odometry
import message_filters


class StageAgents(object):
    """
    Bridge between Stage simulator agents and CoHAN tracked_agents message.

    Subscribes to simulated human and robot odometry topics, converts them to TrackedAgents messages,
    and publishes them for use by CoHAN Navigation. Handles both human and robot agents, synchronizes their states,
    and assigns appropriate segment and agent types.
    """

    def __init__(self, num_hum, ns_):
        """
        Initialize the StageAgents bridge.

        Args:
            num_hum (int): Number of human agents in the simulation.
            ns_ (str): Namespace for the current agent (empty string for robot).
        """
        self.num_hum = num_hum
        self.ns = ns_
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.robot = TrackedAgent()
        self.sig_1 = False
        self.sig_2 = False

    def AgentsPub(self):
        """
        Set up ROS node, subscribers, and publisher for tracked agents.
        Synchronizes human agent odometry and robot odometry, and starts publishing tracked_agents messages.
        """
        rospy.init_node('Stage_Agents', anonymous=True)
        agent_sub = []

        # Subscibe to human agents
        if self.num_hum < 2:
            self.sig_1 = True

        for agent_id in range(1,self.num_hum+1):
            name = 'human'+str(agent_id)
            if self.ns != name:
                agent_sub.append(message_filters.Subscriber("/" + name + "/base_pose_ground_truth", Odometry))

        # Subscribe to the robot
        if self.ns != "":
            robot_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.RobotCB)
        else:
            self.sig_2 = True

        self.tracked_agents_pub = rospy.Publisher("tracked_agents", TrackedAgents, queue_size=1)
        pose_msg = message_filters.TimeSynchronizer(agent_sub, 10)
        pose_msg.registerCallback(self.AgentsCB)
        rospy.Timer(rospy.Duration(0.02), self.publishAgents)
        rospy.spin()

    def AgentsCB(self,*msg):
        """
        Callback for synchronized human agent odometry messages.
        Converts odometry to TrackedAgent messages and updates the tracked_agents list.

        Args:
            *msg: Synchronized odometry messages for human agents.
        """
        if len(msg) != self.num_hum:
            return
        
        tracked_agents = TrackedAgents()
        for agent_id in range(1,self.num_hum+1):
            if self.ns == "human"+str(agent_id):
                continue
            agent_segment = TrackedSegment()
            agent_segment.type = self.Segment_Type
            # print(agent_id-1)
            agent_segment.pose.pose = msg[agent_id-1].pose.pose
            agent_segment.twist.twist = msg[agent_id-1].twist.twist
            tracked_agent = TrackedAgent()
            tracked_agent.type = AgentType.HUMAN
            tracked_agent.name = "human"+str(agent_id)
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)
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

    def publishAgents(self, event):
        """
        Publishes the current tracked_agents message if both human and robot data are available.
        Assigns track IDs and sets the header fields.

        Args:
            event (rospy.TimerEvent): The timer event.
        """
        if(self.sig_1 and self.sig_2):
            self.agents.header.stamp = rospy.Time.now()
            self.agents.header.frame_id = "map"
            if(self.ns != ""):
                self.agents.agents.append(self.robot)
            for agent_id in range(0, len(self.agents.agents)):
                self.agents.agents[agent_id].track_id = agent_id+1
            self.tracked_agents_pub.publish(self.agents)
            if self.num_hum >= 2:
                self.sig_1 = False
            if self.ns != "":
                self.sig_2 = False


if __name__ == '__main__':
    nh = sys.argv[1]
    if(len(sys.argv)<5):
        ns=""
    else:
        ns = sys.argv[2]
    agents = StageAgents(num_hum=int(nh), ns_= ns)
    agents.AgentsPub()
