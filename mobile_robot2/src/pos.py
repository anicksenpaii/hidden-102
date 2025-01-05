import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')

        # Create a TF2 Buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically check for transforms
        self.timer = self.create_timer(1.0, self.get_tf_transform)

    def get_tf_transform(self):
        try:
            # Look up the transform between 'world' and 'dummy'
            transform = self.tf_buffer.lookup_transform('odom', 'dummy', rclpy.time.Time())
            
            # Extract the position (x, y, z)
            position = transform.transform.translation
            self.get_logger().info(f"Position of 'dummy': x={position.x}, y={position.y}, z={position.z}")
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TfListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
