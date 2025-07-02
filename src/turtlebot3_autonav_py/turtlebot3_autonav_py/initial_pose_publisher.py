#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Declare parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('pose_x', 0.0)
        self.declare_parameter('pose_y', 0.0)
        self.declare_parameter('pose_yaw', 0.0)

        # Get parameters
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.pose_x = self.get_parameter('pose_x').value
        self.pose_y = self.get_parameter('pose_y').value
        self.pose_yaw = self.get_parameter('pose_yaw').value

        # Publisher
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Set up timer - publish multiple times to ensure it's received
        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        self.publish_count = 0
        self.max_publish = 3  # Publish 3 times

        self.get_logger().info(f'Initial pose publisher starting (x: {self.pose_x}, y: {self.pose_y})')

    def publish_initial_pose(self):
        if self.publish_count >= self.max_publish:
            self.get_logger().info('Published initial pose, shutting down')
            self.timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Position
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.position.z = 0.0

        # Orientation (from yaw angle)
        yaw = self.pose_yaw
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Set reasonable covariance for AMCL
        msg.pose.covariance[0] = 0.25   # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = 0.068 # yaw

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published initial pose #{self.publish_count+1}: x={self.pose_x}, y={self.pose_y}')
        self.publish_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
