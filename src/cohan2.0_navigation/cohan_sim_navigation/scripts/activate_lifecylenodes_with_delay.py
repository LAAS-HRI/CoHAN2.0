#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import time
import sys

def activate_nodes_after_delay(node_names, delay_s):
    rclpy.init()
    node = Node('delayed_activator')

    for node_name in node_names:
        client = node.create_client(ChangeState, f'/{node_name}/change_state')
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f'Waiting for {node_name}/change_state service...')
        
        # Configure
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

    # Wait before activating
    time.sleep(delay_s)

    for node_name in node_names:
        client = node.create_client(ChangeState, f'/{node_name}/change_state')
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        if future.result().success:
            node.get_logger().info(f'{node_name} activated successfully {delay_s}s after configure!')
        else:
            node.get_logger().error(f'Failed to activate {node_name}.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args') and not a.startswith('-r') and not a.startswith('__node')]

    if len(args) < 2:
        print("Usage: activate_lifecylenodes_with_delay.py <delay_s> <node_name1> [<node_name2> ...]")
        sys.exit(1)

    delay_s = float(args[0])
    node_names = args[1:]

    activate_nodes_after_delay(node_names, delay_s)
