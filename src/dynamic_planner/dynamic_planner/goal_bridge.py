#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from action_msgs.msg import GoalStatus


class GoalBridge(Node):
    """
    Subscribes to /goal_pose (PoseStamped) and forwards the goal to Nav2
    via the NavigateToPose action. If a new goal arrives while a previous
    one is active, it cancels it and sends the latest goal.
    """

    def __init__(self):
        super().__init__('goal_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.on_goal_pose,
            qos
        )

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.get_logger().info('GoalBridge started: subscribing /goal_pose and bridging to /navigate_to_pose')

    def on_goal_pose(self, msg: PoseStamped):
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available yet.')
            return

        # Cancel any previous goal if still active
        if self.current_goal_handle is not None:
            status = self.current_goal_handle.status
            if status in [
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING
            ]:
                self.get_logger().info('Cancelling previous goal before sending new one...')
                cancel_future = self.current_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)

        # Create and send new goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        self.get_logger().info(
            f"Forwarding new goal to Nav2: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception in goal_response: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2.')
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted by Nav2. Waiting for result...')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        current = fb.current_pose.pose.position
        self.get_logger().debug(f"Progress → x={current.x:.2f}, y={current.y:.2f}")

    def get_result_callback(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Get result exception: {e}')
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ Nav2 goal succeeded!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('⚠️ Nav2 goal aborted.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('⏹️ Nav2 goal canceled.')
        else:
            self.get_logger().info(f'Nav2 goal finished with status: {status}')


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
