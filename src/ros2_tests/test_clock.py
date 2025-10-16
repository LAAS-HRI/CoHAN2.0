#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import ClockType

class ClockTypeChecker(Node):
    def __init__(self):
        super().__init__('clock_type_checker')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Node started, checking clock type every second...')

    def timer_callback(self):
        clock_type = self.get_clock().clock_type
        print(clock_type.value)


def main(args=None):
    rclpy.init(args=args)
    node = ClockTypeChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
