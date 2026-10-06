#!/usr/bin/env python3
"""Publish rear-axle speed (m/s) and virtual front steering (radians)."""
import argparse
import math
import time
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, default=0)
    parser.add_argument('--steer', type=float, default=0)
    parser.add_argument('--seconds', type=float, default=5)
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.speed, args.steer, args.seconds)) or args.seconds <= 0:
        parser.error('arguments must be finite, seconds > 0')
    rclpy.init()
    node = rclpy.create_node('golf_drive_command')
    pub = node.create_publisher(AckermannDriveStamped, '/golf/drive', 1)
    msg = AckermannDriveStamped()
    deadline = time.monotonic() + 5
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.destroy_node(); rclpy.shutdown()
        raise SystemExit('No vehicle subscriber. Start the simulation with the same ROS_DOMAIN_ID.')
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.drive.speed = args.speed; msg.drive.steering_angle = args.steer
            pub.publish(msg); rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        msg.drive.speed = msg.drive.steering_angle = 0.0
        for _ in range(5):
            pub.publish(msg); time.sleep(0.05)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
