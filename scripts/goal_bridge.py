#!/usr/bin/env python3
"""
Goal Bridge Node

Bridges the /goal_pose topic from RViz to the Nav2 /navigate_to_pose action.
This enables the standard RViz "2D Goal Pose" tool to work with Nav2.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class GoalBridge(Node):
    def __init__(self):
        super().__init__('goal_bridge')
        
        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Topic subscriber
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.get_logger().info('Goal Bridge started.')
        self.get_logger().info('Ready to receive goals from RViz "2D Goal Pose" tool.')
        
        self.current_goal_handle = None
    
    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f'Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        
        # Cancel previous goal if active
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling previous goal...')
            self.current_goal_handle.cancel_goal_async()
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available!')
            return
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        goal_msg.pose.header.frame_id = 'map'
        
        # Send goal
        self.get_logger().info('Sending goal to Nav2...')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server!')
            return
        
        self.get_logger().info('Goal accepted.')
        self.current_goal_handle = goal_handle
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist = feedback.distance_remaining
        # Log distance occasionally or just keep silent to reduce noise
        # self.get_logger().info(f'Distance remaining: {dist:.2f}m')
    
    def result_callback(self, future):
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal aborted by server.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal canceled.')
        
        self.current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = GoalBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
