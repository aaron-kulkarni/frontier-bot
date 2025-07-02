#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Declare parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('pose_x', -2.0)
        self.declare_parameter('pose_y', -0.5)
        self.declare_parameter('pose_yaw', 0.0)

        # Get parameters
        self.pose_x = self.get_parameter('pose_x').value
        self.pose_y = self.get_parameter('pose_y').value
        self.pose_yaw = self.get_parameter('pose_yaw').value

        self.get_logger().info(f'Initial pose will be set to: x={self.pose_x}, y={self.pose_y}, yaw={self.pose_yaw}')

        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(2.0, self.publish_initial_pose)  # Wait 2 seconds
        self.count = 0
        self.max_attempts = 3  # Publish multiple times to ensure it's received

    def publish_initial_pose(self):
        if self.count >= self.max_attempts:
            self.get_logger().info('Published initial pose multiple times. Shutting down.')
            rclpy.shutdown()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set position from parameters
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.position.z = 0.0

        # Set orientation from yaw parameter
        yaw = self.pose_yaw
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Set reasonable covariance values for AMCL
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published initial pose #{self.count+1}: x={self.pose_x}, y={self.pose_y}, yaw={self.pose_yaw}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
