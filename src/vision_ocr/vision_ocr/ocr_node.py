#!/usr/bin/env python3
"""
OCR Node for Vision-Guided Navigation

This node subscribes to camera images, performs OCR to detect text,
and publishes the detected text for further processing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import pytesseract
import numpy as np


class OCRNode(Node):
    """ROS 2 node for performing OCR on camera images."""

    def __init__(self):
        super().__init__('ocr_node')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/ocr/detected_text')
        self.declare_parameter('confidence_threshold', 60.0)
        self.declare_parameter('processing_rate', 2.0)  # Hz
        self.declare_parameter('preprocess_image', True)
        
        # Get parameters
        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        processing_rate = self.get_parameter('processing_rate').value
        self.preprocess = self.get_parameter('preprocess_image').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Create subscriber and publisher
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        self.text_pub = self.create_publisher(
            String,
            output_topic,
            10
        )
        
        # Processing control
        self.latest_image = None
        self.processing_timer = self.create_timer(
            1.0 / processing_rate,
            self.process_image
        )
        
        self.get_logger().info(f'OCR Node started, subscribing to {image_topic}')
        self.get_logger().info(f'Publishing detected text to {output_topic}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')

    def image_callback(self, msg):
        """Store the latest image for processing."""
        self.latest_image = msg

    def preprocess_image(self, image):
        """
        Preprocess image for better OCR results.
        
        Args:
            image: Input image in BGR format
            
        Returns:
            Preprocessed image ready for OCR
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply adaptive thresholding
        processed = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            11,
            2
        )
        
        # Denoise
        processed = cv2.fastNlMeansDenoising(processed)
        
        return processed

    def process_image(self):
        """Process the latest image with OCR."""
        if self.latest_image is None:
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            
            # Preprocess if enabled
            if self.preprocess:
                ocr_image = self.preprocess_image(cv_image)
            else:
                ocr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Perform OCR with detailed data
            ocr_data = pytesseract.image_to_data(
                ocr_image,
                output_type=pytesseract.Output.DICT
            )
            
            # Extract text with confidence filtering
            detected_texts = []
            for i, text in enumerate(ocr_data['text']):
                confidence = float(ocr_data['conf'][i])
                if confidence > self.confidence_threshold and text.strip():
                    detected_texts.append(text.strip())
            
            # Publish detected text
            if detected_texts:
                text_msg = String()
                text_msg.data = ' '.join(detected_texts)
                self.text_pub.publish(text_msg)
                self.get_logger().info(f'Detected text: {text_msg.data}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    """Main entry point for the OCR node."""
    rclpy.init(args=args)
    
    try:
        ocr_node = OCRNode()
        rclpy.spin(ocr_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
