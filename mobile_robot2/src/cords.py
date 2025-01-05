#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import tf_transformations

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            # Replace 'base_link' and 'odom' with your relevant frames
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', now)
            self.print_transform(trans)
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

    def print_transform(self, trans: TransformStamped):
        translation = trans.transform.translation
        rotation = trans.transform.rotation
        euler_angles = tf_transformations.euler_from_quaternion([
            rotation.x, rotation.y, rotation.z, rotation.w
        ])

        self.get_logger().info(f"Translation: x={translation.x}, y={translation.y}, z={translation.z}")
        self.get_logger().info(f"Rotation: roll={euler_angles[0]}, pitch={euler_angles[1]}, yaw={euler_angles[2]}")

def main(args=None):
    rclpy.init(args=args)
    node = TfListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()