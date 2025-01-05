#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import std_msgs.msg

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to input point cloud
        self.pc_subscriber = self.create_subscription(
            PointCloud2, 
            '/velodyne/PointCloud2', 
            self.point_cloud_callback, 
            100
        )
        
        # Create publisher for transformed point cloud
        self.pc_publisher = self.create_publisher(
            PointCloud2,
            '/transformed_pointcloud',
            10
        )
        
        self.timer = self.create_timer(1.0, self.get_tf_transform)
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.point_cloud = []

    def get_tf_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            position = transform.transform.translation
            self.current_position = np.array([position.x, position.y, position.z])
            orientation = transform.transform.rotation
            self.current_orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            self.get_logger().info(f"Position of 'base_link': x={self.current_position[0]}, y={self.current_position[1]}, z={self.current_position[2]}")
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

    def point_cloud_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        transformed_points = []
        
        for point in points:
            point_np = np.array([point[0], point[1], point[2]])
            transformed_point = self.apply_transform_to_point(point_np)
            transformed_points.append(transformed_point)
            self.point_cloud.append(transformed_point)
            
        # Publish the transformed point cloud
        self.publish_point_cloud(msg.header)

    def apply_transform_to_point(self, point):
        rotation_matrix = self.get_rotation_matrix_from_quaternion(self.current_orientation)
        transformed_point = rotation_matrix.dot(point) + self.current_position
        #transformed_point =point 
        return transformed_point

    def get_rotation_matrix_from_quaternion(self, quaternion):
        x, y, z, w = quaternion
        R = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)]
        ])
        return R

    def publish_point_cloud(self, header):
        # Create point cloud message
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = "odom"  # Set appropriate frame_id
        
        # Define the fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create point cloud message using sensor_msgs_py helper function
        msg = pc2.create_cloud(header, fields, self.point_cloud)
        
        # Publish the message
        self.pc_publisher.publish(msg)
        self.get_logger().info("Published transformed point cloud")

def main(args=None):
    rclpy.init(args=args)
    node = TfListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
