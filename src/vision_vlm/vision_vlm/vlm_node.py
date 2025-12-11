#!/usr/bin/env python3
"""
VLM Node for Vision-Guided Navigation

This node processes detected text and visual information using a
Vision-Language Model to generate navigation commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
from transformers import AutoProcessor, AutoModelForCausalLM
from PIL import Image as PILImage
import io
import numpy as np


class VLMNode(Node):
    """ROS 2 node for vision-language model processing."""

    def __init__(self):
        super().__init__('vlm_node')
        
        # Declare parameters
        self.declare_parameter('text_topic', '/ocr/detected_text')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/vlm/navigation_command')
        self.declare_parameter('model_name', 'microsoft/git-base')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('max_response_length', 50)
        
        # Get parameters
        text_topic = self.get_parameter('text_topic').value
        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        model_name = self.get_parameter('model_name').value
        device = self.get_parameter('device').value
        self.max_length = self.get_parameter('max_response_length').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Store latest data
        self.latest_text = None
        self.latest_image = None
        
        # Initialize model (simplified for demonstration)
        self.get_logger().info(f'Loading VLM model: {model_name}')
        try:
            self.device = torch.device(device if torch.cuda.is_available() and device == 'cuda' else 'cpu')
            self.get_logger().info(f'Using device: {self.device}')
            
            # Note: In production, you would load an actual VLM model here
            # For now, we'll use rule-based logic as a placeholder
            self.model_loaded = False
            self.get_logger().warn('Using rule-based navigation logic (VLM model placeholder)')
            
        except Exception as e:
            self.get_logger().error(f'Error loading model: {str(e)}')
            self.model_loaded = False
        
        # Create subscribers
        self.text_sub = self.create_subscription(
            String,
            text_topic,
            self.text_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        # Create publisher for navigation commands
        self.cmd_pub = self.create_publisher(
            String,
            output_topic,
            10
        )
        
        self.get_logger().info(f'VLM Node started')
        self.get_logger().info(f'Subscribing to text: {text_topic}')
        self.get_logger().info(f'Subscribing to images: {image_topic}')
        self.get_logger().info(f'Publishing commands to: {output_topic}')

    def text_callback(self, msg):
        """Process detected text."""
        self.latest_text = msg.data
        self.get_logger().info(f'Received text: {self.latest_text}')
        self.process_navigation()

    def image_callback(self, msg):
        """Store the latest image."""
        self.latest_image = msg

    def interpret_text_to_command(self, text):
        """
        Interpret detected text to generate navigation command.
        This is a rule-based approach as a placeholder for VLM.
        
        Args:
            text: Detected text from OCR
            
        Returns:
            Navigation command string
        """
        text_lower = text.lower()
        
        # Define navigation keywords and their commands
        if any(word in text_lower for word in ['stop', 'halt', 'wait']):
            return 'STOP'
        elif any(word in text_lower for word in ['left', 'turn left']):
            return 'TURN_LEFT'
        elif any(word in text_lower for word in ['right', 'turn right']):
            return 'TURN_RIGHT'
        elif any(word in text_lower for word in ['forward', 'ahead', 'go', 'straight']):
            return 'MOVE_FORWARD'
        elif any(word in text_lower for word in ['back', 'reverse', 'backward']):
            return 'MOVE_BACKWARD'
        elif any(word in text_lower for word in ['exit', 'door', 'entrance']):
            return 'MOVE_TO_EXIT'
        elif any(word in text_lower for word in ['room', 'office']):
            # Extract room number if present
            import re
            room_match = re.search(r'\d+', text)
            if room_match:
                return f'MOVE_TO_ROOM_{room_match.group()}'
            return 'MOVE_FORWARD'
        else:
            return 'ANALYZE'

    def process_navigation(self):
        """Process text and image data to generate navigation command."""
        if self.latest_text is None:
            return
        
        try:
            # Generate navigation command based on text
            command = self.interpret_text_to_command(self.latest_text)
            
            # Publish command
            cmd_msg = String()
            cmd_msg.data = command
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f'Generated command: {command}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing navigation: {str(e)}')


def main(args=None):
    """Main entry point for the VLM node."""
    rclpy.init(args=args)
    
    try:
        vlm_node = VLMNode()
        rclpy.spin(vlm_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
