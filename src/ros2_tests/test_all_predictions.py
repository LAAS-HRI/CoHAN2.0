#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.parameter
from agent_path_prediction.srv import AgentPosePredict, AgentGoal
from agent_path_prediction.msg import PredictedGoals, PredictedGoal
from cohan_msgs.msg import AgentPathArray, AgentPath
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from std_srvs.srv import Empty
import sys
import time
import math

class ComprehensivePredictionTester(Node):
    def __init__(self):
        # Add timestamp to make node name unique and avoid conflicts
        import random
        node_name = f'comprehensive_prediction_tester_{random.randint(1000, 9999)}'
        super().__init__(node_name)
        
        # Create service clients
        self.predict_client = self.create_client(AgentPosePredict, '/agent_path_prediction/predict_agent_poses')
        self.set_goal_client = self.create_client(AgentGoal, '/agent_path_prediction/set_agent_goal')
        # Reset predictions service client
        self.reset_predictions_client = self.create_client(Empty, '/agent_path_prediction/reset_prediction_services')
        
        # Create publishers for external data
        self.external_paths_pub = self.create_publisher(AgentPathArray, '/agent_path_prediction/external_agent_paths', 10)
        self.predicted_goals_pub = self.create_publisher(PredictedGoals, '/agent_path_prediction/predicted_goal', 10)
        
        # Create static transform broadcaster for map <-> base_footprint
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()
        
        # Wait for services
        self.get_logger().info('Waiting for services...')
        while not self.predict_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('predict_agent_poses service not available, waiting...')
        
        while not self.set_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_agent_goal service not available, waiting...')
        
        self.get_logger().info('All services available!')

    def publish_static_transform(self):
        """Publish static transform from map to base_footprint"""
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        
        # Robot at origin for simplicity
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(transform)
        self.get_logger().info('Published static transform: map -> base_footprint')

    @staticmethod
    def get_prediction_type_name(pred_type):
        """Get human-readable name for prediction type"""
        type_names = {
            AgentPosePredict.Request.VELOCITY_OBSTACLE: "VELOCITY_OBSTACLE",
            AgentPosePredict.Request.EXTERNAL: "EXTERNAL",
            AgentPosePredict.Request.BEHIND_ROBOT: "BEHIND_ROBOT",
            AgentPosePredict.Request.PREDICTED_GOAL: "PREDICTED_GOAL"
        }
        return type_names.get(pred_type, f"UNKNOWN({pred_type})")

    def call_predict_service(self, prediction_type, agent_ids, predict_times):
        """Call the prediction service and return results"""
        request = AgentPosePredict.Request()
        request.type = prediction_type
        request.ids = agent_ids
        request.predict_times = predict_times
        
        type_name = self.get_prediction_type_name(prediction_type)
        self.get_logger().info(f'\n--- Testing {type_name} prediction ---')
        self.get_logger().info(f'Agent IDs: {agent_ids}, Prediction times: {predict_times}')
        
        future = self.predict_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'✓ Success! Received {len(response.predicted_agents_poses)} predicted agent trajectories')
            
            for predicted_poses in response.predicted_agents_poses:
                self.get_logger().info(f'  Agent {predicted_poses.id}: {len(predicted_poses.poses)} poses')
                if predicted_poses.poses:
                    first = predicted_poses.poses[0].pose.pose.position
                    last = predicted_poses.poses[-1].pose.pose.position
                    self.get_logger().info(f'    From ({first.x:.2f}, {first.y:.2f}) to ({last.x:.2f}, {last.y:.2f})')
            
            return response
        else:
            self.get_logger().error(f'✗ Failed to call {type_name} prediction service!')
            return None

    def test_velocity_obstacle_prediction(self):
        """Test VELOCITY_OBSTACLE prediction (uses current agent velocities)"""
        agent_ids = [1, 2, 3]  # Test with multiple agents
        predict_times = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]  # Multiple time horizons
        
        return self.call_predict_service(
            AgentPosePredict.Request.VELOCITY_OBSTACLE,
            agent_ids,
            predict_times
        )

    def publish_external_paths(self):
        """Publish external paths for EXTERNAL prediction mode"""
        self.get_logger().info('Publishing external agent paths...')
        
        msg = AgentPathArray()
        msg.header = Header()
        msg.header.frame_id = "map"
        
        # Create path for agent 1 (straight line)
        agent1_path = AgentPath()
        agent1_path.id = 1
        agent1_path.path = Path()
        agent1_path.path.header = msg.header
        
        # Create waypoints for agent 1
        for i in range(10):
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            # Don't set timestamp - let C++ code handle timing
            pose.pose.position = Point(x=float(i * 0.5), y=0.0, z=0.0)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            agent1_path.path.poses.append(pose)
        
        # Create path for agent 2 (curved path)
        agent2_path = AgentPath()
        agent2_path.id = 2
        agent2_path.path = Path()
        agent2_path.path.header = msg.header
        
        # Create curved waypoints for agent 2
        for i in range(10):
            angle = i * 0.2  # radians
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            # Don't set timestamp - let C++ code handle timing
            pose.pose.position = Point(
                x=2.0 * math.cos(angle),
                y=2.0 * math.sin(angle),
                z=0.0
            )
            # Set orientation to face tangent direction
            yaw = angle + math.pi/2
            pose.pose.orientation = Quaternion(
                x=0.0, y=0.0,
                z=math.sin(yaw/2), w=math.cos(yaw/2)
            )
            agent2_path.path.poses.append(pose)
        
        msg.paths = [agent1_path, agent2_path]
        self.external_paths_pub.publish(msg)
        
        # Give some time for the message to be received
        time.sleep(0.5)

    def test_external_prediction(self):
        """Test EXTERNAL prediction (uses published external paths)"""
        self.publish_external_paths()
        
        agent_ids = [1, 2]  # Match the agents we published paths for
        predict_times = [1.0, 2.0, 3.0]
        
        return self.call_predict_service(
            AgentPosePredict.Request.EXTERNAL,
            agent_ids,
            predict_times
        )

    def set_external_goals(self):
        """Set external goals for agents"""
        self.get_logger().info('Setting external goals for agents...')
        
        request = AgentGoal.Request()
        
        # Set goals for multiple agents
        goal1 = PoseStamped()
        goal1.header = Header()
        goal1.header.frame_id = "map"
        goal1.pose.position = Point(x=5.0, y=3.0, z=0.0)
        goal1.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        goal2 = PoseStamped()
        goal2.header = Header()
        goal2.header.frame_id = "map"
        goal2.pose.position = Point(x=-2.0, y=4.0, z=0.0)
        goal2.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Create agent goals
        from agent_path_prediction.msg import AgentPose
        agent_goal1 = AgentPose()
        agent_goal1.id = 1
        agent_goal1.pose = goal1
        agent_goal1.type = 0  # Set a default type
        
        agent_goal2 = AgentPose()
        agent_goal2.id = 2
        agent_goal2.pose = goal2
        agent_goal2.type = 0  # Set a default type
        
        request.goals = [agent_goal1, agent_goal2]
        
        future = self.set_goal_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info('✓ Successfully set external goals')
            return True
        else:
            self.get_logger().error('✗ Failed to set external goals')
            return False

    def test_external_with_goals_prediction(self):
        """Test EXTERNAL prediction with set goals"""
        if not self.set_external_goals():
            return None
            
        agent_ids = [1, 2]
        predict_times = [1.0, 2.0, 4.0]
        
        return self.call_predict_service(
            AgentPosePredict.Request.EXTERNAL,
            agent_ids,
            predict_times
        )

    def test_behind_robot_prediction(self):
        """Test BEHIND_ROBOT prediction (agents move to position behind robot)"""
        agent_ids = [1, 2, 3]
        predict_times = [1.0, 2.0, 3.0, 5.0]
        
        return self.call_predict_service(
            AgentPosePredict.Request.BEHIND_ROBOT,
            agent_ids,
            predict_times
        )

    def publish_predicted_goals(self):
        """Publish predicted goals for PREDICTED_GOAL mode"""
        self.get_logger().info('Publishing predicted goals...')
        
        msg = PredictedGoals()
        msg.header = Header()
        msg.header.frame_id = "map"
        
        # Create predicted goals for agents
        goal1 = PredictedGoal()
        goal1.id = 1
        goal1.goal.position = Point(x=4.0, y=2.0, z=0.0)
        goal1.goal.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        goal2 = PredictedGoal()
        goal2.id = 2
        goal2.goal.position = Point(x=-3.0, y=1.5, z=0.0)
        goal2.goal.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        goal3 = PredictedGoal()
        goal3.id = 3
        goal3.goal.position = Point(x=0.0, y=-4.0, z=0.0)
        goal3.goal.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        msg.goals = [goal1, goal2, goal3]
        self.predicted_goals_pub.publish(msg)
        
        # Give some time for the message to be received
        time.sleep(0.5)

    def test_predicted_goal_prediction(self):
        """Test PREDICTED_GOAL prediction (uses published predicted goals)"""
        self.publish_predicted_goals()
        
        agent_ids = [1, 2, 3]
        predict_times = [1.0, 3.0, 5.0]
        
        return self.call_predict_service(
            AgentPosePredict.Request.PREDICTED_GOAL,
            agent_ids,
            predict_times
        )

    def run_all_tests(self):
        """Run all prediction tests"""
        self.get_logger().info('\n🧪 Starting comprehensive prediction tests...\n')
        
        tests = [
            ("VELOCITY_OBSTACLE", self.test_velocity_obstacle_prediction),
            ("EXTERNAL (with paths)", self.test_external_prediction),
            ("EXTERNAL (with goals)", self.test_external_with_goals_prediction),
            ("BEHIND_ROBOT", self.test_behind_robot_prediction),
            ("PREDICTED_GOAL", self.test_predicted_goal_prediction),
        ]
        
        results = {}
        for test_name, test_func in tests:
            self.get_logger().info(f'\n{"="*50}')
            self.get_logger().info(f'Running {test_name} test...')
            self.get_logger().info(f'{"="*50}')
            
            try:
                result = test_func()
                results[test_name] = result is not None
                if result:
                    self.get_logger().info(f'✅ {test_name} test completed successfully')
                else:
                    self.get_logger().warn(f'⚠️  {test_name} test failed or returned no results')
            except Exception as e:
                self.get_logger().error(f'❌ {test_name} test failed with exception: {e}')
                results[test_name] = False
            
            # Attempt to reset prediction services after each test to ensure isolation
            try:
                if self.reset_predictions_client.wait_for_service(timeout_sec=1.0):
                    reset_req = Empty.Request()
                    reset_future = self.reset_predictions_client.call_async(reset_req)
                    rclpy.spin_until_future_complete(self, reset_future, timeout_sec=2.0)
                    self.get_logger().info('Called reset_prediction_services')
                else:
                    self.get_logger().warn('reset_prediction_services not available to call')
            except Exception as e:
                self.get_logger().warn(f'Failed to call reset_prediction_services: {e}')

            time.sleep(0.5)  # Brief pause between tests
        
        # Print summary
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info('📊 TEST RESULTS SUMMARY')
        self.get_logger().info(f'{"="*50}')
        
        passed = sum(results.values())
        total = len(results)
        
        for test_name, success in results.items():
            status = "✅ PASS" if success else "❌ FAIL"
            self.get_logger().info(f'{status} - {test_name}')
        
        self.get_logger().info(f'\nOverall: {passed}/{total} tests passed')
        
        return results

def main():
    rclpy.init()
    
    tester = None
    try:
        tester = ComprehensivePredictionTester()
        
        if len(sys.argv) > 1 and sys.argv[1] == '--single':
            # Single test mode
            test_type = sys.argv[2].upper() if len(sys.argv) > 2 else 'VELOCITY_OBSTACLE'
            
            if test_type == 'VELOCITY_OBSTACLE':
                tester.test_velocity_obstacle_prediction()
            elif test_type == 'EXTERNAL':
                tester.test_external_prediction()
            elif test_type == 'EXTERNAL_GOALS':
                tester.test_external_with_goals_prediction()
            elif test_type == 'BEHIND_ROBOT':
                tester.test_behind_robot_prediction()
            elif test_type == 'PREDICTED_GOAL':
                tester.test_predicted_goal_prediction()
            else:
                tester.get_logger().error(f'Unknown test type: {test_type}')
        else:
            # Run all tests
            tester.run_all_tests()
            
    except Exception as e:
        print(f"Error during testing: {e}")
    finally:
        if tester is not None:
            tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()