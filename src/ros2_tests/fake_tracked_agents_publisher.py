#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.parameter
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, AccelWithCovariance
from geometry_msgs.msg import Pose, Twist, Accel, Point, Quaternion, Vector3
from std_msgs.msg import Header
import math
import time
from rclpy.clock import Clock, ClockType

class FakeTrackedAgentsPublisher(Node):
    def __init__(self):
        super().__init__('fake_tracked_agents_publisher')        
        
        # Create publisher for tracked agents
        self.publisher = self.create_publisher(TrackedAgents, '/tracked_agents', 10)
        
        # Timer to publish at regular intervals
        self.timer = self.create_timer(0.1, self.publish_fake_agents)  # 10 Hz
        
        # Internal state for agent motion
        self.start_time = time.time()
        self.get_logger().info('Fake tracked agents publisher started on topic: /tracked_agents')
    
    def create_fake_agent(self, agent_id, x, y, vx, vy, yaw=0.0):
        """Create a fake tracked agent with given parameters"""
        agent = TrackedAgent()
        agent.track_id = agent_id
        agent.state = TrackedAgent.MOVING
        agent.type = 1  # Default type
        agent.name = f"human{agent_id}"
        
        # Create tracked segment (body part)
        segment = TrackedSegment()
        segment.type = 1  # Default agent part type
        
        # Set pose
        segment.pose = PoseWithCovariance()
        segment.pose.pose.position = Point(x=x, y=y, z=0.0)
        
        # Convert yaw to quaternion
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        segment.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
        # Set small covariance (confident in position)
        segment.pose.covariance = [0.1] * 36  # 6x6 matrix, all elements 0.1
        
        # Set twist (velocity)
        segment.twist = TwistWithCovariance()
        segment.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        segment.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        segment.twist.covariance = [0.01] * 36  # Small covariance for velocity
        
        # Set acceleration (zero for now)
        segment.accel = AccelWithCovariance()
        segment.accel.accel.linear = Vector3(x=0.0, y=0.0, z=0.0)
        segment.accel.accel.angular = Vector3(x=0.0, y=0.0, z=0.0)
        segment.accel.covariance = [0.01] * 36
        
        agent.segments = [segment]
        return agent
    
    def publish_fake_agents(self):
        """Publish fake tracked agents with dynamic motion"""
        msg = TrackedAgents()
        
        # Set header with simulation time
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Create dynamic agents with different motion patterns
        current_time = time.time() - self.start_time
        
        # Agent 1: Moving in a circle
        radius = 2.0
        angular_vel = 0.5  # rad/s
        angle = angular_vel * current_time
        x1 = radius * math.cos(angle)
        y1 = radius * math.sin(angle)
        vx1 = -radius * angular_vel * math.sin(angle)
        vy1 = radius * angular_vel * math.cos(angle)
        yaw1 = angle + math.pi/2
        
        agent1 = self.create_fake_agent(1, x1, y1, vx1, vy1, yaw1)
        
        # Agent 2: Moving back and forth on x-axis
        amplitude = 3.0
        freq = 0.3  # Hz
        x2 = amplitude * math.sin(2 * math.pi * freq * current_time)
        y2 = 2.0  # Fixed y position
        vx2 = amplitude * 2 * math.pi * freq * math.cos(2 * math.pi * freq * current_time)
        vy2 = 0.0
        yaw2 = 0.0 if vx2 >= 0 else math.pi
        
        agent2 = self.create_fake_agent(2, x2, y2, vx2, vy2, yaw2)
        
        # Agent 3: Moving diagonally
        speed = 0.8  # m/s
        x3 = speed * current_time * 0.7071  # cos(45°)
        y3 = -2.0 + speed * current_time * 0.7071  # sin(45°), starting from y=-2
        vx3 = speed * 0.7071
        vy3 = speed * 0.7071
        yaw3 = math.pi/4  # 45 degrees
        
        agent3 = self.create_fake_agent(3, x3, y3, vx3, vy3, yaw3)
        
        # Agent 4: Stationary agent
        agent4 = self.create_fake_agent(4, 1.5, -1.0, 0.0, 0.0, 0.0)
        agent4.state = TrackedAgent.STATIC
        
        # Add agents to message
        msg.agents = [agent1, agent2, agent3, agent4]
        
        # Publish
        self.publisher.publish(msg)
        
        # Log every 50 publications (5 seconds at 10Hz)
        if int(current_time * 10) % 50 == 0:
            self.get_logger().info(f'Published {len(msg.agents)} fake agents at time {current_time:.1f}s')
            self.get_logger().info(f'  Agent 1: pos=({x1:.2f}, {y1:.2f}), vel=({vx1:.2f}, {vy1:.2f})')
            self.get_logger().info(f'  Agent 2: pos=({x2:.2f}, {y2:.2f}), vel=({vx2:.2f}, {vy2:.2f})')
            self.get_logger().info(f'  Agent 3: pos=({x3:.2f}, {y3:.2f}), vel=({vx3:.2f}, {vy3:.2f})')

def main(args=None):
    rclpy.init(args=args)
    
    node = FakeTrackedAgentsPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down fake tracked agents publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()