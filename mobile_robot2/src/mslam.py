import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import message_filters

class SynchronizedTfListenerNode(Node):
    def __init__(self):
        super().__init__('synchronized_tf_listener_node')
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for transformed point cloud
        self.pc_publisher = self.create_publisher(
            PointCloud2,
            '/transformed_pointcloud',
            10
        )
        
        # Create subscribers
        self.point_cloud_sub = message_filters.Subscriber(
            self,
            PointCloud2,
            '/velodyne/PointCloud2'
        )
        
        # Synchronize the point cloud and transform messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.point_cloud_sub],
            queue_size=10,
            slop=1
        )
        
        # Register callback for synchronized messages
        self.ts.registerCallback(self.synchronized_callback)
        
        self.point_cloud = []

    def synchronized_callback(self, point_cloud_msg):
        try:
            # Get the transform at the time of the point cloud message
            transform = self.tf_buffer.lookup_transform(
                'odom',
                point_cloud_msg.header.frame_id,
                point_cloud_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Extract transform data
            position = transform.transform.translation
            current_position = np.array([position.x, position.y, position.z])
            orientation = transform.transform.rotation
            current_orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            
            # Process point cloud
            points = pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            transformed_points = []
            
            for point in points:
                point_np = np.array([point[0], point[1], point[2]])
                transformed_point = self.apply_transform_to_point(point_np, current_position, current_orientation)
                transformed_points.append(transformed_point)
            
            # Update point cloud and publish
            self.point_cloud = transformed_points
            self.publish_point_cloud(point_cloud_msg.header)
            
            self.get_logger().info(f"Processed synchronized data at time {point_cloud_msg.header.stamp.sec}.{point_cloud_msg.header.stamp.nanosec}")
            
        except Exception as ex:
            self.get_logger().warn(f"Could not transform point cloud: {ex}")

    def apply_transform_to_point(self, point, position, orientation):
        # Apply translation and rotation to the point cloud data
        rotation_matrix = self.get_rotation_matrix_from_quaternion(orientation)
        transformed_point = np.dot(rotation_matrix, point) + position
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
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = "odom"
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg = pc2.create_cloud_xyz32(header, self.point_cloud)
        self.pc_publisher.publish(msg)
        self.get_logger().info("Published synchronized transformed point cloud")


def main(args=None):
    rclpy.init(args=args)
    node = SynchronizedTfListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
