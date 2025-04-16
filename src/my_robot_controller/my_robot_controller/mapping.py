#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("Mapping Node has started.")

        # Publisher to send movement commands
        self._pose_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber to receive LiDAR data
        self._scan_listener = self.create_subscription(
            LaserScan, "/scan", self.robot_controller, 10
        )

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()
        safety_distance = 0.5  # Minimum distance to walls
        exploration_bias = 1.5  # Encourage movement toward open areas

        # Define angles to monitor
        a = 10  # Angle range for obstacle detection
        front = min(scan.ranges[:a+1] + scan.ranges[-a:])
        left = min(scan.ranges[90-a:90+a+1])
        right = min(scan.ranges[270-a:270+a+1])

        # Find the most open direction
        open_left = sum(scan.ranges[45:135]) / len(scan.ranges[45:135])
        open_right = sum(scan.ranges[225:315]) / len(scan.ranges[225:315])

        # Navigation logic
        if front < safety_distance:  # Obstacle ahead, turn
            if right > left:
                cmd.angular.z = -0.5  # Prefer right turn
            else:
                cmd.angular.z = 0.5  # Prefer left turn
            cmd.linear.x = 0.05
        else:
            # Encourage movement towards open space
            if open_right > open_left * exploration_bias:
                cmd.angular.z = -0.3  # Steer right
            elif open_left > open_right * exploration_bias:
                cmd.angular.z = 0.3  # Steer left
            else:
                cmd.angular.z = 0.0  # Go straight
            cmd.linear.x = 0.3  # Move forward

        # Publish the movement command
        self._pose_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()