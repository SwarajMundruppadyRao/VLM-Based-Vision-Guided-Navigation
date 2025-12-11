#!/usr/bin/env python3
"""
Monitor script to display navigation commands and velocity outputs.

This script subscribes to navigation commands and velocity commands
to monitor system behavior.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SystemMonitor(Node):
    """Node to monitor system outputs."""

    def __init__(self):
        super().__init__('system_monitor')
        
        # Subscribe to navigation commands
        self.nav_sub = self.create_subscription(
            String,
            '/vlm/navigation_command',
            self.nav_callback,
            10
        )
        
        # Subscribe to velocity commands
        self.vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.vel_callback,
            10
        )
        
        # Subscribe to detected text
        self.text_sub = self.create_subscription(
            String,
            '/ocr/detected_text',
            self.text_callback,
            10
        )
        
        self.get_logger().info('System Monitor started')
        self.get_logger().info('Monitoring:')
        self.get_logger().info('  - /ocr/detected_text')
        self.get_logger().info('  - /vlm/navigation_command')
        self.get_logger().info('  - /cmd_vel')
        self.get_logger().info('-' * 50)

    def text_callback(self, msg):
        """Display detected text."""
        self.get_logger().info(f'[OCR] Detected: "{msg.data}"')

    def nav_callback(self, msg):
        """Display navigation command."""
        self.get_logger().info(f'[VLM] Command: {msg.data}')

    def vel_callback(self, msg):
        """Display velocity command."""
        linear = msg.linear.x
        angular = msg.angular.z
        
        if linear == 0.0 and angular == 0.0:
            status = "STOPPED"
        elif linear > 0:
            status = f"FORWARD (v={linear:.2f})"
        elif linear < 0:
            status = f"BACKWARD (v={linear:.2f})"
        elif angular > 0:
            status = f"TURN_LEFT (w={angular:.2f})"
        elif angular < 0:
            status = f"TURN_RIGHT (w={angular:.2f})"
        else:
            status = "IDLE"
            
        self.get_logger().info(f'[CTRL] Motion: {status}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    monitor = SystemMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
