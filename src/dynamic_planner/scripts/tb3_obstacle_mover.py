#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose   # <-- correct for Jazzy (Gazebo Garden)

class TB3Mover(Node):
    def __init__(self):
        super().__init__('tb3_mover')

        # Gazebo service (check your world name)
        self.world_name = "turtlebot3_world"
        self.entity_name = "tb3_mover"

        self.client = self.create_client(
            SetEntityPose,
            f'/world/{self.world_name}/set_entity_pose'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /world/.../set_entity_pose service...')

        self.t = 0.0
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info('tb3_mover: ready and moving.')

    def timer_cb(self):
        # Sine-wave path around (1.2, 0)
        self.t += 0.05
        x = 1.2 + 0.6 * math.cos(self.t)
        y = 0.6 * math.sin(self.t)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.01
        pose.orientation.w = 1.0

        req = SetEntityPose.Request()
        req.entity.name = self.entity_name
        req.pose = pose

        self.client.call_async(req)  # fire & forget

def main(args=None):
    rclpy.init(args=args)
    node = TB3Mover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
