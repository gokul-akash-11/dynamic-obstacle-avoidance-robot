#!/usr/bin/env python3
# ~/fake_mover_fixed.py
import rclpy, os, math, json
from rclpy.node import Node
from std_msgs.msg import String

class FakeMoverFixed(Node):
    def __init__(self):
        super().__init__('fake_mover_fixed')
        self.pub = self.create_publisher(String, '/predicted_paths', 10)
        self.t = 0.0
        # publish at 5 Hz
        self.timer = self.create_timer(0.2, self.tick)
        self.center_x = 20.0   # small coords to satisfy dynamic_planner dist < 100
        self.center_y = 20.0
        self.radius_x = 8.0
        self.radius_y = 5.0
        self.get_logger().info(f"fake_mover_fixed started. ROS_DOMAIN_ID={os.getenv('ROS_DOMAIN_ID')}")

    def tick(self):
        self.t += 0.2
        # create a short future trajectory (6 steps) around a small centre
        future = []
        for i in range(6):
            # a moving point on a small ellipse (keeps values small)
            angle = self.t + 0.25 * i
            x = self.center_x + self.radius_x * math.cos(angle + i * 0.05)
            y = self.center_y + self.radius_y * math.sin(angle + i * 0.05)
            future.append([round(x, 3), round(y, 3)])
        payload = {"1": {"label": "person", "future": future}}
        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)
        # small info log for debugging
        self.get_logger().debug("Published /predicted_paths: " + msg.data[:160])

def main():
    rclpy.init()
    node = FakeMoverFixed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
