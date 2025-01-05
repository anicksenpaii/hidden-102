#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
import message_filters

class TimeSyncNode(Node):

    def __init__(self):
        super().__init__('sync_node')

        qos = QoSProfile(depth=10)

        # Subscribers for Lidar and TF messages using message_filters.Subscriber
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/velodyne/PointCloud2', qos_profile=qos)
        self.tf_sub = message_filters.Subscriber(self, TFMessage, '/tf', qos_profile=qos)

        # Approximate Time Synchronizer
        queue_size = 10
        max_delay = 0.2
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.tf_sub], queue_size, max_delay)
        self.time_sync.registerCallback(self.sync_callback)

    def sync_callback(self, lidar_msg, tf_msg):
        """
        Callback function to handle synchronized messages.
        """
        self.get_logger().info("Synchronizing messages...")

        # Log header information to verify
        self.get_logger().info(f"Lidar Message Header: {lidar_msg.header}")
        self.get_logger().info(f"TF Message Header: {tf_msg.transforms[0].header}")

        # Process synchronized lidar and tf messages
        self.get_logger().info(f"Lidar Message Timestamp: {lidar_msg.header.stamp}")
        self.get_logger().info(f"TF Message Timestamp: {tf_msg.transforms[0].header.stamp}")

def main(args=None):
    rclpy.init(args=args)
    node = TimeSyncNode()

    # Spin the node to keep it running
    rclpy.spin(node)

    # Shutdown when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
