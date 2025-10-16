#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import time

class SimulationClockPublisher(Node):
    def __init__(self):
        super().__init__('simulation_clock_publisher')
        
        self.get_logger().info('Simulation clock publisher initializing...')
        
        # Create clock publisher for simulation time
        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)
        
        # Create timer to publish clock at higher frequency (100 Hz) for simulation time
        self.clock_timer = self.create_timer(0.01, self.publish_clock)
        
        # Initialize simulation time
        self.sim_time = 0.0
        
        self.get_logger().info('Simulation clock publisher started on topic: /clock')
    
    def publish_clock(self):
        """Publish simulation time clock"""
        clock_msg = Clock()
        
        # Increment simulation time
        self.sim_time += 0.01  # 10ms increment for 100Hz
        
        # Convert to ROS time
        secs = int(self.sim_time)
        nsecs = int((self.sim_time - secs) * 1e9)
        clock_msg.clock.sec = secs
        clock_msg.clock.nanosec = nsecs
        
        self.clock_publisher.publish(clock_msg)
        
        # Log every 1000 publications (10 seconds at 100Hz)
        if int(self.sim_time * 100) % 1000 == 0:
            self.get_logger().info(f'Published simulation time: {self.sim_time:.1f}s')

def main(args=None):
    rclpy.init(args=args)
    
    node = SimulationClockPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down simulation clock publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()