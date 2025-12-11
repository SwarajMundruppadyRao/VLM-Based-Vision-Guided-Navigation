#!/usr/bin/env python3
"""
Test script to publish sample text to the OCR topic.

This script simulates OCR output for testing the VLM and Control nodes
without needing a camera or OCR system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class TestTextPublisher(Node):
    """Node to publish test text messages."""

    def __init__(self):
        super().__init__('test_text_publisher')
        self.publisher = self.create_publisher(String, '/ocr/detected_text', 10)
        self.get_logger().info('Test text publisher started')

    def publish_test_sequence(self):
        """Publish a sequence of test commands."""
        test_messages = [
            "Go Forward",
            "Turn Left",
            "Stop",
            "Turn Right",
            "Exit",
            "Room 101",
            "Move Ahead"
        ]
        
        for msg_text in test_messages:
            msg = String()
            msg.data = msg_text
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {msg_text}')
            time.sleep(3)  # Wait 3 seconds between messages


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    publisher = TestTextPublisher()
    
    try:
        publisher.get_logger().info('Starting test sequence...')
        publisher.publish_test_sequence()
        publisher.get_logger().info('Test sequence complete')
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
