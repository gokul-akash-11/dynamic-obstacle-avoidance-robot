#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json, math, time, random


def safe_parse_json(s: str):
    """Attempt to parse JSON robustly."""
    if not s or not s.strip():
        return None
    try:
        return json.loads(s)
    except Exception:
        try:
            return json.loads(s.replace("'", '"'))
        except Exception:
            return None


class DynamicPlanner(Node):
    """
    Subscribes to /predicted_paths and publishes /goal_pose only when
    a dynamic obstacle is predicted to come too close to the robot.
    """

    def __init__(self):
        super().__init__('dynamic_planner')
        self.sub = self.create_subscription(String, '/predicted_paths', self.on_predicted_paths, 10)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.safe_distance = 100.0       # pixels or scaled units from tracker
        self.cooldown = 5.0              # seconds between goal replans
        self.last_publish_time = 0.0

        self.get_logger().info("Dynamic planner started — waiting for predictions.")

    def on_predicted_paths(self, msg: String):
        parsed = safe_parse_json(msg.data)
        if not parsed:
            return

        # Extract first future coordinate
        fx, fy = self.extract_first_future(parsed)
        if fx is None:
            return

        distance = math.hypot(fx, fy)
        if distance < self.safe_distance:
            now = time.time()
            if now - self.last_publish_time > self.cooldown:
                self.get_logger().warn(f"⚠️ Predicted obstacle too close (dist={distance:.1f}) → replanning new goal...")
                self.publish_safe_goal()
                self.last_publish_time = now
        else:
            self.get_logger().debug(f"Predicted obstacle at dist={distance:.1f} → safe.")

    def extract_first_future(self, parsed):
        if isinstance(parsed, dict):
            for v in parsed.values():
                if isinstance(v, dict) and 'future' in v and v['future']:
                    pt = v['future'][0]
                    return float(pt[0]), float(pt[1])
        elif isinstance(parsed, list):
            for item in parsed:
                if 'future' in item and item['future']:
                    pt = item['future'][0]
                    return float(pt[0]), float(pt[1])
        return None, None

    def publish_safe_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        # Move slightly to a random offset (detour-like goal)
        goal.pose.position.x = random.uniform(-0.5, 0.5)
        goal.pose.position.y = random.uniform(-0.5, 0.5)
        goal.pose.orientation.w = 1.0

        self.pub.publish(goal)
        self.get_logger().info(f"🚩 Published dynamic goal → x:{goal.pose.position.x:.2f}, y:{goal.pose.position.y:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = DynamicPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
